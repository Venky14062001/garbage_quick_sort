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
)

add_service_files(
  FILES
  yolo.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs 
)

catkin_package(
 CATKIN_DEPENDS std_msgs message_generation message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY msg/ srv/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

