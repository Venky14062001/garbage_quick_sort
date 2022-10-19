cmake_minimum_required(VERSION 3.0.2)
project(detection_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  BoundingBox.msg
  BoundingBoxes.msg
  TargetCenter.msg
  TargetsInfoPerFrame.msg
  TargetsInfo.msg
)

add_service_files(
  FILES
  FramesInfo.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs 
)

catkin_package(
 CATKIN_DEPENDS std_msgs message_runtime
)

install(
  DIRECTORY msg/ srv/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

