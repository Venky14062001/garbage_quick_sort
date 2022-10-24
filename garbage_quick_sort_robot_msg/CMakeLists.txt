cmake_minimum_required(VERSION 2.8.3)

project(garbage_quick_sort_robot_msg)

find_package(catkin REQUIRED COMPONENTS
	message_generation std_msgs
)

set(MSG_FILES
  EffectorPose.msg
  JointGoal.msg
  RobotState.msg
)

set(SRV_FILES
  RobotStateFbk.srv
  EffectorPoseFbk.srv
  UltrasonicMeanFbk.srv
)

add_message_files(
  DIRECTORY msg FILES ${MSG_FILES}
)

add_service_files(
  DIRECTORY srv FILES ${SRV_FILES}
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

install(
  DIRECTORY msg/ srv/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
