cmake_minimum_required(VERSION 2.8.3)
project(touch_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation geometry_msgs sensor_msgs std_msgs)

## Generate message in the 'msg' folder
add_message_files(
	FILES 
	TouchFeedback.msg 
	TouchButtonEvent.msg 
	TouchState.msg
	TouchPoseTwist.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  TouchIK.srv
)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs std_msgs)

include_directories(${catkin_INCLUDE_DIRS})
