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
from sensor_msgs.msg import LaserScan

from happymimi_msgs.srv import StrTrg
from happymimi_manipulation_msgs.msg import *
from happymimi_recognition_msgs.srv import RecognitionCount, RecognitionCountRequest

motor_controller_path = roslib.packages.get_pkg_dir('dynamixel_controller')
sys.path.insert(0, os.path.join(motor_controller_path, 'src/'))
from motor_controller import ManipulateArm
from grasping_action_server import GraspingActionServer

teleop_path = roslib.packages.get_pkg_dir('happymimi_teleop')
sys.path.insert(0, os.path.join(teleop_path, 'src/'))
from base_control import BaseControl

class PickLuggageActionServer(GraspingActionServer):
    def __init__(self):
        super(PickLuggageActionServer,self).__init__()
        rospy.Subscriber('/scan', LaserScan, self.laserCB)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist,queue_size = 1)
        self.laser = []
        self.front_laser = 999.9

        self.base_control = BaseControl()

        self.act = actionlib.SimpleActionServer('/manipulation/pick_luggage',
                                                PickLuggageAction,
                                                execute_cb = self.actionMain,
                                                auto_start = False)
        self.act.register_preempt_callback(self.actionPreempt)

        self.act.start()

    def turnToLuggage(self, req):
        if req == 'left':
            # 90〜150
            laser_left = self.laser[359:599]
            width_min = 0
            width_max = 0
            for i, dist in enumerate(laser_left):
                if 0.5 < dist:
                    if width_min == 0:
                        width_min = i
                    if width_max < i:
                        width_max = i
            width_center = (width_min+width_max)/2
            angle = width_center/4
        elif req == 'right':
            # 30〜90
            laser_right = self.laser[119:359]
            width_min = 0
            width_max = 0
            for i, dist in enumerate(laser_left):
                if 0.5 < dist:
                    if width_min == 0:
                        width_min = i
                    if width_max < i:
                        width_max = i
            width_center = (width_min+width_max)/2
            angle = -(240-width_center)/4
        if angle == 0: return
        self.base_control.rotateAngle(angle, 0.5)
        rospy.sleep(3.0)

    def apploachLuggage(self):
        self.base_control.translateDist(self.front_laser, 0.1)
        rospy.sleep(2.0)
        return self.front_laser

    def pickLuggage(self, req):
        rospy.loginfo('\n----- Pick Luggage-----')

        x = 0.35
        y = 0.45
        joint_angle = self.inverseKinematics([x, y])
        if numpy.nan in joint_angle:
            return False
        self.armControllerByTopic(joint_angle)
        rospy.sleep(2.5)

        print '1'
        self.turnToLuggage(req)
        print '2'
        if front_laser > 1.5: return False
        past_front_laser = self.apploachLuggage()
        print '3'

        self.controlEndeffector(True)
        rospy.sleep(1.0)
        self.changeArmPose('carry')
        rospy.sleep(4.0)
        grasp_flg = self.checkExistence(past_front_laser)
        return grasp_flg

    def laserCB(self, msg):
        self.laser = msg.ranges
        self.front_laser = msg.ranges[359]

    def checkExistence(self, past_value):
        return self.front_laser - past_value > 0.10

    def startUp(self):
        pass
        '''
        _ = self.controlEndeffector(False)
        self.changeArmPose('carry')
        self.controlHead(0.0)
        '''

    def actionPreempt(self):
        rospy.loginfo('Preempt callback')
        self.act.set_preempted(text = 'message for preempt')
        self.preempt_flg = True

    def actionMain(self,req):
        grasp_result = GraspingObjectResult()
        grasp_flg = False
        grasp_flg = self.pickLuggage(req.goal)
        grasp_result.result = grasp_flg
        self.act.set_succeeded(grasp_result)


if __name__ == '__main__':
    rospy.init_node('pick_luggage')
    p_l_action_server = PickLuggageActionServer()
    #p_l_action_server.startUp()
    rospy.spin()
