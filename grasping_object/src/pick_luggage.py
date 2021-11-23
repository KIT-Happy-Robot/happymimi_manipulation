#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import rosparam
import roslib.packages
import os
import sys
import time
import math
import numpy
import threading
import actionlib
from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

from happymimi_msgs.srv import StrTrg
from happymimi_manipulation_msgs.msg import *
from happymimi_recognition_msgs.srv import RecognitionCount, RecognitionCountRequest

motor_controller_path = roslib.packages.get_pkg_dir('dynamixel_controller')
sys.path.insert(0, os.path.join(motor_controller_path, 'src/'))
from motor_controller import ManipulateArm

teleop_path = roslib.packages.get_pkg_dir('happymimi_teleop')
sys.path.insert(0, os.path.join(teleop_path, 'src/'))
from base_control import BaseControl

class GraspingActionServer(ManipulateArm):
    def __init__(self):
        super(GraspingActionServer,self).__init__()
        rospy.Subscriber('/current_location',String,self.navigationPlaceCB)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size = 1)
        self.navigation_place = 'Null'
        self.target_place = rosparam.get_param('/location_dict')

        self.base_control = BaseControl()

        self.act = actionlib.SimpleActionServer('/manipulation/grasp',
                                                GraspingObjectAction,
                                                execute_cb = self.actionMain,
                                                auto_start = False)
        self.act.register_preempt_callback(self.actionPreempt)

        self.act.start()

    def placeMode(self):#override
        self.base_control.translateDist(-0.15)
        rospy.sleep(1.0)
        y = self.target_place[self.navigation_place] + 0.14
        #x = (y-0.78)/10+0.5
        x = 0.5
        joint_angle = self.inverseKinematics([x, y])
        if numpy.nan in joint_angle:
            return False

        self.armControllerByTopic(joint_angle)
        rospy.sleep(2.5)
        self.base_control.translateDist(0.3)
        rospy.sleep(1.0)
        self.base_control.translateDist(0.1, 0.1)
        rospy.sleep(1.0)

        joint_angle = self.inverseKinematics([x, y-0.03])
        if not (numpy.nan in joint_angle):
            self.armControllerByTopic(joint_angle)
        rospy.sleep(2.5)
        self.controlEndeffector(False)
        rospy.sleep(2.0)
        self.base_control.translateDist(-0.25)
        self.changeArmPose('carry')
        self.navigation_place = 'Null'
        rospy.loginfo('Finish place command\n')
        return True

    def approachObject(self,object_centroid):
        if object_centroid.x > 1.5:
            return False
        elif object_centroid.x < 0.5 or object_centroid.x > 0.8:
            move_range = (object_centroid.x-0.65)
            if abs(move_range) < 0.2: move_range = int(move_range/abs(move_range))*0.2
            self.base_control.translateDist(move_range)
            rospy.sleep(4.0)
            return False
        else :
            return True

    def pickLuggage(self, object_centroid):
        rospy.loginfo('\n----- Pick Luggage-----')

        self.controlEndeffector(True)

        x = 0.45
        #x = (y-0.75)/10+0.5
        y = object_centroid.z + 0.15 + 0.03
        joint_angle = self.inverseKinematics([x, y])
        if numpy.nan in joint_angle:
            return False
        self.armControllerByTopic(joint_angle)
        rospy.sleep(2.5)

        move_range = object_centroid.x + 0.07 - x + 0.05
        self.base_control.translateDist(move_range*0.8, 0.15)
        rospy.sleep(0.5)
        self.base_control.translateDist(move_range*0.2, 0.1)
        rospy.sleep(0.5)

        self.controlWrist(joint_angle[2]+45.0)
        self.base_control.translateDist(-0.4)

        self.changeArmPose('carry')
        rospy.sleep(4.0)
        return

    def checkExistence(self):
        countObject = rospy.ServiceProxy('/recognition/count', RecognitionCount)
        rospy.wait_for_service('/recognition/count')
        exist_flg = bool(countObject(RecognitionCountRequest('luggage')).num)
        return not exist_flg

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
        target_centroid = object_centroid.goal
        grasp_result = GraspingObjectResult()
        grasp_flg = False
        approach_flg = self.approachObject(target_centroid)
        if approach_flg:
            self.pickLuggage(target_centroid)
            grasp_flg = checkExistence()
        grasp_result.result = grasp_flg
        self.act.set_succeeded(grasp_result)


if __name__ == '__main__':
    rospy.init_node('pick_luggage')
    grasping_action_server= GraspingActionServer()
    grasping_action_server.startUp()
    rospy.spin()
