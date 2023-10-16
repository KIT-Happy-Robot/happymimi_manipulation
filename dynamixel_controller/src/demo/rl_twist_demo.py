#!/usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
import roslib.packages
import os
import sys
motor_controller_path = roslib.packages.get_pkg_dir('dynamixel_controller')
sys.path.insert(0, os.path.join(motor_controller_path, 'src/'))
from motor_controller import ManipulateArm