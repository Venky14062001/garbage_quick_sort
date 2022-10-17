cmake_minimum_required(VERSION 3.0.2)
project(garbage_quick_sort_robot_arduino)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  CATKIN_DEPENDS roscpp
)

## Mark cpp header files for installation
install(DIRECTORY scripts/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


