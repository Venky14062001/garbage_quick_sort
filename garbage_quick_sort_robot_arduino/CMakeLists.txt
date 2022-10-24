cmake_minimum_required(VERSION 3.0.2)
project(garbage_quick_sort_robot_arduino)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  garbage_quick_sort_robot_msg
)

catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS scripts/gqsMainArduinoHandler 
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



