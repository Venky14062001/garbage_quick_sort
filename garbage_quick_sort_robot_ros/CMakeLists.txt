cmake_minimum_required(VERSION 2.8.3)

project(garbage_quick_sort_robot_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  moveit_commander
  moveit_msgs
  trajectory_msgs
  sensor_msgs
  std_msgs
  garbage_quick_sort_robot_msg
  garbage_quick_sort_robot_ik
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS scripts/gqsMainMoveIt scripts/gqsMainRoboticsToolbox 
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})