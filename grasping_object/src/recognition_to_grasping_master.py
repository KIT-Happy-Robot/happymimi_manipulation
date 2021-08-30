#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import actionlib
from enum import Enum
from std_msgs.msg import Bool, String, Float64
# -- Custom Message --
from happymimi_msgs.srv import StrTrg
from happymimi_manipulation_msgs.msg import *
from happymimi_recognition_msgs.msg import *

class ResultState(Enum):
    success = 1
    wait = 2
    failure = 3

class RecognitionAction(object):
    def __init__(self):
        self.recognize_feedback = ResultState.wait

    def recognitionFeedback(self,msg):
        rospy.loginfo('feedback : %s'%(msg))
        self.recognize_feedback = ResultState.success if msg.recognize_feedback else ResultState.failure

    def recognizeObject(self,target_name):
        act = actionlib.SimpleActionClient('/manipulation/localize', RecognitionProcessingAction)
        act.wait_for_server(rospy.Duration(5))
        goal = RecognitionProcessingGoal()
        goal.recognize_goal = target_name
        act.send_goal(goal, feedback_cb = self.recognitionFeedback)
        loop_count = 0
        limit_count = 3.0
        result = ResultState.wait
        while result == ResultState.wait and not rospy.is_shutdown():
            print result
            result = act.get_result()
            if not result: result = ResultState.wait
            print result
            if self.recognize_feedback == ResultState.success:
                loop_count = 0
                limit_count -= 0.5
                self.recognize_feedback = ResultState.wait
            elif self.recognize_feedback == ResultState.failure:
                loop_count += 2
                self.recognize_feedback = ResultState.wait
            if loop_count > limit_count:
                act._set_simple_state(actionlib.SimpleGoalState.PENDING)
                act.cancel_goal()
                rospy.sleep(1.0)
                break
            rospy.Rate(3.0).sleep()
        result = act.get_result()
        recognize_flg = limit_count > loop_count
        print result
        return recognize_flg, result.recognize_result

class GraspingAction(object):
    def __init__(self):
        pass

    def graspingFeedback(self,msg):
        rospy.loginfo('feedback %s'%(msg))

    def graspObject(self, target_centroid):
        act = actionlib.SimpleActionClient('/manipulation/grasp', GraspingObjectAction)
        rospy.loginfo('waiting for grasping server')
        act.wait_for_server(rospy.Duration(5))
        rospy.loginfo('send goal')
        goal = GraspingObjectGoal()
        goal.grasp_goal = target_centroid
        act.send_goal(goal, feedback_cb = self.graspingFeedback)
        act.wait_for_result()
        result = act.get_result()

        return result.grasp_result

def actionMain(req):
    endeffector_pub = rospy.Publisher('/servo/endeffector',Bool,queue_size=1)
    head_pub = rospy.Publisher('/servo/head',Float64,queue_size=1)
    rospy.sleep(0.2)
    endeffector_pub.publish(False)
    head_pub.publish(25.0)
    rospy.sleep(2.0)
    recognize_flg = True
    grasp_flg = False
    grasp_count = 0
    RA = RecognitionAction()
    GA = GraspingAction()
    while recognize_flg and not grasp_flg and grasp_count < 3 and not rospy.is_shutdown():
        rospy.loginfo('\n----- Recognition Action -----')
        recognize_flg, object_centroid = RA.recognizeObject(req.target_name)
        if recognize_flg:
            rospy.loginfo('\n-----  Grasping Action   -----')
            grasp_flg = GA.graspObject(object_centroid)
            grasp_count += 1
    manipulation_flg = recognize_flg and grasp_flg
    return manipulation_flg


if __name__ == '__main__':
    rospy.init_node('manipulation_master')
    rospy.Service('/recognition_to_grasping',StrTrg, actionMain)
    rospy.spin()
