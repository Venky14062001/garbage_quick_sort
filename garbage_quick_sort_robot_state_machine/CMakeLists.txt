cmake_minimum_required(VERSION 2.8.3)

project(garbage_quick_sort_robot_state_machine)

find_package(catkin REQUIRED COMPONENTS
  rospy
  garbage_quick_sort_robot_msg
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS scripts/gqsMainStateMachine
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})