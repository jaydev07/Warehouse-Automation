cmake_minimum_required(VERSION 3.0.2)
project(pkg_ros_iot_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs 
  actionlib_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  msgRosIot.action
)

add_message_files(
  FILES
  msgMqttSub.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  actionlib_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pkg_ros_actions
 CATKIN_DEPENDS geometry_msgs actionlib_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
