#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import rosparam
import time
import math
import numpy
import threading
import actionlib
from std_msgs.msg import Bool, Float64, String
from dynamixel_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
# -- Custom Message --
from mimi_manipulation_pkg.srv import ManipulateSrv
from mimi_manipulation_pkg.msg import *

from motor_controller import ArmPoseChanger

class ObjectGrasper(ArmPoseChanger):
    def __init__(self):
        super(ObjectGrasper,self).__init__()
        rospy.Subscriber('/current_location',String,self.navigationPlaceCB)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size = 1)
        self.act = actionlib.SimpleActionServer('/manipulation/grasp',
                                                ObjectGrasperAction,
                                                execute_cb = self.actionMain,
                                                auto_start = False)
        self.act.register_preempt_callback(self.actionPreempt)

        self.navigation_place = 'Null'
        self.target_place = rosparam.get_param('/location_dict')

        self.act.start()

    def placeMode(self):#override
        self.moveBase(-0.6)
        y = self.target_place[self.navigation_place] + 0.14
        #x = (y-0.78)/10+0.5
        x = 0.5
        joint_angle = self.inverseKinematics([x, y])
        if numpy.nan in joint_angle:
            return False
        
        self.armController(joint_angle)
        rospy.sleep(2.0)
        self.moveBase(0.6)
        rospy.sleep(2.0)
        self.moveBase(0.4)

        joint_angle = self.inverseKinematics([x, y-0.03])
        if not (numpy.nan in joint_angle):
            self.armController(joint_angle)
        rospy.sleep(2.0)
        self.controlEndeffector(False)
        rospy.sleep(2.0)
        self.moveBase(-0.9)
        self.changeArmPose('carry')
        self.navigation_place = 'Null'
        rospy.loginfo('Finish place command\n')
        return True
                        
    def moveBase(self,rad_speed):
        cmd = Twist()
        for speed_i in range(10):
            cmd.linear.x = rad_speed*0.05*speed_i
            cmd.angular.z = 0
            self.cmd_vel_pub.publish(cmd)
            rospy.sleep(0.1)
        for speed_i in range(10):
            cmd.linear.x = rad_speed*0.05*(10-speed_i)
            cmd.angular.z = 0
            self.cmd_vel_pub.publish(cmd)
            rospy.sleep(0.1)
        cmd.linear.x = 0
        cmd.angular.z = 0
        self.cmd_vel_pub.publish(cmd)

    def approachObject(self,object_centroid):
        if object_centroid.x > 1.5:
            return False
        elif object_centroid.x < 0.5 or object_centroid.x > 0.8:
            move_range = (object_centroid.x-0.65)*2.0
            if abs(move_range) < 0.3:
                move_range = int(move_range/abs(move_range))*0.3
            self.moveBase(move_range)
            rospy.sleep(4.0)
            return False
        else :
            return True

    def graspObject(self, object_centroid):
        rospy.loginfo('\n----- Grasp Object -----')
        self.moveBase(-0.5)
        if self.navigation_place == 'Null':
            y = object_centroid.z + 0.06
        else:
            y = self.target_place[self.navigation_place] + 0.10
        #x = (y-0.75)/10+0.5
        x = 0.475
        joint_angle = self.inverseKinematics([x, y])
        if numpy.nan in joint_angle:
            return False
        self.armController(joint_angle)
        rospy.sleep(2.5)
        move_range = 0.5 + (object_centroid.x + 0.05 - x)*3.0
        # 0.5:後退量, 0.05:realsenseからshoulderまでのx軸の距離, 3.0:moveBaseの数値に変換(おおよそ)
        self.moveBase(move_range*0.7)
        rospy.sleep(0.3)
        self.moveBase(move_range*0.4)
        grasp_flg = self.controlEndeffector(True)
        rospy.sleep(1.0)
        self.controlShoulder(joint_angle[0]+5.0)
        self.moveBase(-0.9)
        self.changeArmPose('carry')
        rospy.sleep(4.0)
        if grasp_flg :
            grasp_flg = abs(self.torque_error[4]) > 30
        if grasp_flg :
            rospy.loginfo('Successfully grasped the object!')
        else:
            self.callMotorService(4, self.origin_angle[4])
            rospy.loginfo('Failed to grasp the object.')
        rospy.loginfo('Finish grasp.')
        return grasp_flg
    
    def navigationPlaceCB(self,res):
        self.navigation_place = res.data

    def startUp(self):
        _ = self.controlEndeffector(False)
        self.changeArmPose('carry')
        self.controlHead(0.0)

    def actionPreempt(self):
        rospy.loginfo('Preempt callback')
        self.act.set_preempted(text = 'message for preempt')
        self.preempt_flg = True

    def actionMain(self,object_centroid):
        target_centroid = object_centroid.grasp_goal
        grasp_result = ObjectGrasperResult()
        grasp_flg = False
        approach_flg = self.approachObject(target_centroid)
        if approach_flg:
            grasp_flg = self.graspObject(target_centroid)
        grasp_result.grasp_result = grasp_flg
        self.act.set_succeeded(grasp_result)


if __name__ == '__main__':
    rospy.init_node('object_grasper')
    grasper = ObjectGrasper()
    grasper.startUp()
    rospy.spin()
