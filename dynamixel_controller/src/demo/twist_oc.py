#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
import roslib.packages
import os
import sys
motor_controller_path = roslib.packages.get_pkg_dir('dynamixel_controller')
sys.path.insert(0, os.path.join(motor_controller_path, 'src/'))
from motor_controller import ManipulateArm


class TwistOpenClose(ManipulateArm):
    def __init__(self):
        super(TwistOpenClose,self).__init__()
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