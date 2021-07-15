#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
import rosparam
import numpy
import math
import threading
import time
from std_msgs.msg import Bool, Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_workbench_msgs.msg import DynamixelStateList
from dynamixel_workbench_msgs.srv import DynamixelCommand
# -- Custom Message -- 
from mimi_manipulation_pkg.srv import ManipulateSrv, DetectDepth

class MotorController(object):
    def __init__(self):
        rospy.Subscriber('/dynamixel_workbench/dynamixel_state',DynamixelStateList,self.getMotorStateCB)
        self.motor_pub = rospy.Publisher('/dynamixel_workbench/joint_trajectory',JointTrajectory,queue_size=10)
        self.motor_angle_pub = rospy.Publisher('/servo/angle_list',Float64MultiArray,queue_size=10)
        self.motor_client = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command',DynamixelCommand)
        # -- Motor Parameters --
        self.origin_angle = rosparam.get_param('/mimi_specification/Origin_Angle')
        self.gear_ratio = rosparam.get_param('/mimi_specification/Gear_Ratio')
        self.current_pose = [0]*6
        self.torque_error = [0]*6
        self.rotation_velocity = [0]*6

        rospy.Timer(rospy.Duration(0.5), self.motorAnglePub)
        
    def getMotorStateCB(self, state):
        for i in range(6):
            self.current_pose[i] = state.dynamixel_state[i].present_position
            self.rotation_velocity[i] = abs(state.dynamixel_state[i].present_velocity)
            self.torque_error[i] = state.dynamixel_state[i].present_current

    def motorAnglePub(self, event):
        deg_origin_angle = map(self.stepToDeg, self.origin_angle)
        deg_current_pose = map(self.stepToDeg, self.current_pose)
        current_deg_list = [x-y for (x,y) in zip(deg_current_pose, deg_origin_angle)]
        current_deg_list = [round(x, 1) for x in current_deg_list]
        current_deg_list[2] *= -1
        current_deg_list[5] *= -1
        pub_deg_list = Float64MultiArray(data=current_deg_list)
        self.motor_angle_pub.publish(pub_deg_list)

    def callMotorService(self, motor_id, rotate_value):
        if type(rotate_value) == type(float()):
            rotate_value = self.degToStep(rotate_value)
        res = self.motor_client('', motor_id, 'Goal_Position', rotate_value)

    def degToStep(self,deg):
        return int((deg+180)/360.0*4095)

    def stepToDeg(self,step):
        return round(step/4095.0*360.0-180, 1)

    '''
    def radToStep(self,rad):
        return int((rad + math.pi) / (2*math.pi) * 4095)

    def stepToRad(self,step):
        return step / 4095.0 * 2*math.pi - math.pi
    '''
    

class JointController(MotorController):
    def __init__(self):
        super(JointController,self).__init__()
        rospy.Subscriber('/servo/shoulder',Float64,self.controlShoulder)
        rospy.Subscriber('/servo/elbow',Float64,self.controlElbow)
        rospy.Subscriber('/servo/wrist',Float64,self.controlWrist)
        rospy.Subscriber('/servo/endeffector',Bool,self.controlEndeffector)
        rospy.Subscriber('/servo/head',Float64,self.controlHead)

    def controlShoulder(self,deg):
        if type(deg) == type(Float64()):
            deg = deg.data
        deg *= self.gear_ratio[0]
        step = self.degToStep(deg)
        step0 = 4095 - step + (self.origin_angle[0]-2048)
        step1 = step + (self.origin_angle[1]-2048)
        thread_m0 = threading.Thread(target=self.callMotorService, args=(0, step0,))
        thread_m1 = threading.Thread(target=self.callMotorService, args=(1, step1,))
        thread_m1.start()
        thread_m0.start()
        rospy.sleep(0.2)
        while (self.rotation_velocity[0] > 0 or self.rotation_velocity[1] > 0) and not rospy.is_shutdown():
            pass
        rospy.sleep(0.5)
        if abs(self.torque_error[0]) > 100 or abs(self.torque_error[1] > 100):
            thread_m0 = threading.Thread(target=self.callMotorService, args=(0, self.current_pose[0],))
            thread_m1 = threading.Thread(target=self.callMotorService, args=(1, self.current_pose[1],))
            thread_m0.start()
            thread_m1.start()

    def controlElbow(self,deg):
        if type(deg) == type(Float64()):
            deg = deg.data
        deg *= self.gear_ratio[2]
        deg *= -1
        step = self.degToStep(deg) + (self.origin_angle[2]-2048)
        self.callMotorService(2, step)
        rospy.sleep(0.2)
        while self.rotation_velocity[2] > 0 and not rospy.is_shutdown():
            pass
        rospy.sleep(0.5)
        if abs(self.torque_error[2]) > 100:
            self.callMotorService(2, self.current_pose[2])

    def controlWrist(self,deg):
        if type(deg) == type(Float64()):
            deg = deg.data
        deg *= self.gear_ratio[3]
        step = self.degToStep(deg) + (self.origin_angle[3]-2048)
        self.callMotorService(3, step)
        rospy.sleep(0.2)
        while self.rotation_velocity[3] > 0 and not rospy.is_shutdown():
            pass
        rospy.sleep(0.5)
        if abs(self.torque_error[3]) > 100:
            self.callMotorService(3, self.current_pose[3])

    def controlEndeffector(self,req):
        if type(req) == type(Bool()):
            req = req.data
        if not req:
            self.callMotorService(4, self.origin_angle[4])
            rospy.loginfo("ok")
            return True
        angle = self.origin_angle[4]
        grasp_flg = True
        while abs(self.torque_error[4]) <= 50 and not rospy.is_shutdown():
            angle -= 10
            self.callMotorService(4, angle)
            while self.rotation_velocity[4] > 15.0 and not rospy.is_shutdown():
                pass
            if angle < 2950:
                grasp_flg = False
                break;
        rospy.sleep(0.1)
        self.callMotorService(4, self.current_pose[4]-80)
        return grasp_flg

    def controlHead(self,deg):
        if type(deg) == type(Float64()):
            deg = deg.data
        deg *= self.gear_ratio[5]
        deg *= -1
        step = self.degToStep(deg) + (self.origin_angle[5]-2048)
        self.callMotorService(5, step)
    
    
class ArmPoseChanger(JointController):
    def __init__(self):
        super(ArmPoseChanger,self).__init__()
        #rospy.Service('/servo/debug_arm', , self.manipulateByInverseKinematics)
        rospy.Service('/servo/arm', ManipulateSrv, self.changeArmPose)
        self.detect_depth = rospy.ServiceProxy('/detect/depth', DetectDepth)
        self.arm_specification = rosparam.get_param('/mimi_specification')

    def inverseKinematics(self, coordinate):
        x = coordinate[0]
        y = coordinate[1]
        l0 = self.arm_specification['Ground_Arm_Height']
        l1 = self.arm_specification['Shoulder_Elbow_Length']
        l2 = self.arm_specification['Elbow_Wrist_Length']
        l3 = self.arm_specification['Wrist_Endeffector_Length']
        x -= l3
        y -= l0
        rospy.loginfo(x)
        data1 =  x*x+y*y+l1*l1-l2*l2
        data2 =  2*l1*math.sqrt(x*x+y*y)
        try:
            shoulder_angle = -1*math.acos((x*x+y*y+l1*l1-l2*l2) / (2*l1*math.sqrt(x*x+y*y))) + math.atan(y/x)# -1倍の有無で別解
            elbow_angle = math.atan((y-l1*math.sin(shoulder_angle))/(x-l1*math.cos(shoulder_angle)))-shoulder_angle
            wrist_angle = -1*(shoulder_angle + elbow_angle)
            angle_list = [shoulder_angle, elbow_angle, wrist_angle]
            angle_list = map(math.degrees, angle_list)
            rospy.loginfo(angle_list)
            return angle_list
        except ValueError:
            rospy.loginfo('I can not move arm.')
            return [numpy.nan]*3
        
    def armController(self, joint_angle):
        if type(joint_angle) != list:
            joint_angle = joint_angle.data
        thread_shoulder = threading.Thread(target=self.controlShoulder, args=(joint_angle[0],))
        thread_elbow = threading.Thread(target=self.controlElbow, args=(joint_angle[1],))
        thread_wrist = threading.Thread(target=self.controlWrist, args=(joint_angle[2],))
        thread_wrist.start()
        rospy.sleep(0.5)
        thread_elbow.start()
        rospy.sleep(0.5)
        thread_shoulder.start()

    def manipulateByInverseKinematics(coordinate):
        if type(coordinate) != float:
            coordinate = coordinate.coordinate
        joint_angle = self.inverseKinematics(coordinate)
        if numpy.nan in joint_angle:
            return False
        self.armController()
        return True

    def changeArmPose(self, cmd):
        if type(cmd) != str:
            cmd = cmd.target_name
        rospy.loginfo('Chagnge arm command : %s'%cmd)
        if cmd == 'origin':
            self.originMode()
            return True
        elif cmd == 'carry':
            self.carryMode()
            return True
        elif cmd == 'receive':
            res = self.receiveMode()
            return res
        elif cmd == 'give':
            self.giveMode()
            return True
        elif cmd == 'place':
            res = self.placeMode()
            return res
        else :
            rospy.loginfo('No such change arm command.')
            return False

    def originMode(self):
        shoulder_param = 0
        elbow_param = 0
        wrist_param = 0
        self.armController([shoulder_param, elbow_param, wrist_param])
        
    def carryMode(self):
        shoulder_param = -85
        elbow_param = 75
        wrist_param = 85
        self.armController([shoulder_param, elbow_param, wrist_param])

    def receiveMode(self):
        self.controlHead(25)
        rospy.sleep(0.5)
        shoulder_param = -40
        elbow_param = 70
        wrist_param = -30
        self.armController([shoulder_param, elbow_param, wrist_param])
        rospy.sleep(0.5)
        self.controlEndeffector(False)

        rospy.wait_for_service('/detect/depth')
        endeffector_res = False
        count = 0
        while not endeffector_res and count<2 and not rospy.is_shutdown():
            self.controlEndeffector(False)
            rospy.sleep(2.0)
            count += 1
            start_time = time.time()
            straight_line_distance = 9.99
            while time.time()-start_time<5.0 and straight_line_distance>0.42 and not rospy.is_shutdown():
                depth_res = self.detect_depth(280, 360)
                straight_line_distance = depth_res.centroid_point.x
            rospy.sleep(0.5)
            endeffector_res = self.controlEndeffector(True)
            rospy.sleep(1.5)
            
        self.carryMode()
        self.controlHead(0)
        return endeffector_res

    def giveMode(self):
        shoulder_param = -35
        elbow_param = 75
        wrist_param = -35
        self.armController([shoulder_param, elbow_param, wrist_param])
        rospy.sleep(1.0)
        while self.rotation_velocity[3] > 0 and not rospy.is_shutdown():
            pass
        rospy.sleep(1.0)
        wrist_error = abs(self.torque_error[3])
        give_time = time.time()
        while abs(wrist_error - abs(self.torque_error[3])) < 10 and time.time() - give_time < 5.0 and not rospy.is_shutdown():
            pass
        self.callMotorService(4, self.origin_angle[4])
        self.carryMode()

    def placeMode(self):
        #現時点では家具の高さを予めプログラムに打ち込む必要があり、
        #その情報をobject_grasperに格納しているのでそちらでplaceの関数をオーバーライドしています。
        pass

        
if __name__ == '__main__':
    rospy.init_node('motor_controller')
    experiment = ArmPoseChanger()
    rospy.spin()
