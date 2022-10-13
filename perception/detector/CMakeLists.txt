cmake_minimum_required(VERSION 3.0.2)

project(detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  detection_msgs
  garbage_quick_sort_robot_msg
)

catkin_python_setup()
catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS scripts/gqsMainDetect.py scripts/pub_image.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY weights/ launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

