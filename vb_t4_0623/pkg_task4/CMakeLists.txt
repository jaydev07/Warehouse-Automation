cmake_minimum_required(VERSION 3.0.2)
project(pkg_task4)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  myMessage.msg
  myModel.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control_turtle
 CATKIN_DEPENDS geometry_msgs rospy message_runtime

)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
