#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import actionlib
from enum import Enum
from std_msgs.msg import Bool, String, Float64

from happymimi_recognition_msgs.msg import RecognitionProcessingAction, RecognitionProcessingGoal
from happymimi_manipulation_msgs.msg import GraspingObjectAction, GraspingObjectGoal
from happymimi_manipulation_msgs.srv import RecognitionToGrasping

class ResultState(Enum):
    success = 1
    wait = 2
    failure = 3

class RecognitionAction(object):
    def __init__(self):
        self.recognition_feedback = ResultState.wait

    def recognitionFeedback(self,msg):
        rospy.loginfo('feedback : %s'%(msg))
        self.recognition_feedback = ResultState.success if msg.recognize_feedback else ResultState.failure

    def recognizeObject(self, request):
        act = actionlib.SimpleActionClient('/recognition/action', RecognitionProcessingAction)
        act.wait_for_server(rospy.Duration(5))

        goal = RecognitionProcessingGoal()
        goal.target_name = request.target_name
        goal.sort_option = request.sort_option
        act.send_goal(goal, feedback_cb = self.recognitionFeedback)

        act.wait_for_result()
        result = act.get_result()
        print result.result_flg

        return result.result_flg, result.centroid_point

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
        goal.goal = target_centroid
        act.send_goal(goal, feedback_cb = self.graspingFeedback)
        act.wait_for_result()
        result = act.get_result()

        return result.result

def actionMain(request):
    endeffector_pub = rospy.Publisher('/servo/endeffector',Bool,queue_size=1)
    head_pub = rospy.Publisher('/servo/head',Float64,queue_size=1)
    rospy.sleep(0.2)

    endeffector_pub.publish(False)
    head_pub.publish(25.0)
    rospy.sleep(2.0)

    recognition_flg = True
    grasp_flg = False
    grasp_count = 0

    RA = RecognitionAction()
    GA = GraspingAction()

    while recognition_flg and not grasp_flg and grasp_count < 2 and not rospy.is_shutdown():
        rospy.loginfo('\n----- Recognition Action -----')
        recognition_flg, object_centroid = RA.recognizeObject(request)
        if recognition_flg:
            rospy.loginfo('\n-----  Grasping Action   -----')
            grasp_flg = GA.graspObject(object_centroid)
            grasp_count += 1
    manipulation_flg = recognition_flg and grasp_flg
    return manipulation_flg


if __name__ == '__main__':
    rospy.init_node('manipulation_master')
    rospy.Service('/recognition_to_grasping', RecognitionToGrasping, actionMain)
    rospy.spin()
