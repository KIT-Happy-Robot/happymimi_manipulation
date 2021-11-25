#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import actionlib
from enum import Enum
from std_msgs.msg import Bool, String, Float64

from happymimi_manipulation_msgs.msg import PickLuggageAction, PickLuggageGoal
from happymimi_msgs.srv import StrTrg

class GraspingAction(object):
    def __init__(self):
        pass

    def graspingFeedback(self,msg):
        rospy.loginfo('feedback %s'%(msg))

    def pickLuggage(self, req):
        act = actionlib.SimpleActionClient('/manipulation/pick_luggage', PickLuggageAction)
        rospy.loginfo('waiting for grasping server')
        act.wait_for_server(rospy.Duration(5))
        rospy.loginfo('send goal')
        goal = PickLuggageGoal()
        goal.goal = req
        act.send_goal(goal, feedback_cb = self.graspingFeedback)
        act.wait_for_result()
        result = act.get_result()

        return result.result

def actionMain(request):
    endeffector_pub = rospy.Publisher('/servo/endeffector',Bool,queue_size=1)
    head_pub = rospy.Publisher('/servo/head',Float64,queue_size=1)
    rospy.sleep(0.2)

    '''
    endeffector_pub.publish(False)
    head_pub.publish(30.0)
    rospy.sleep(2.0)
    '''

    grasp_flg = False

    GA = GraspingAction()

    grasp_flg = GA.pickLuggage(request.data)
    return grasp_flg


if __name__ == '__main__':
    rospy.init_node('pick_luggage_master')
    rospy.Service('/pick_luggage', StrTrg, actionMain)
    rospy.spin()
