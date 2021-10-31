cmake_minimum_required(VERSION 3.0.2)
project(happymimi_manipulation_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  happymimi_msgs
  message_generation
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  RecognitionToGrasping.srv
  ArmControl.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY
  action
  FILES
  GraspingObject.action
)

##Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
  happymimi_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES happymimi_manipulation_msgs
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
