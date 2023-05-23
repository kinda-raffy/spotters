cmake_minimum_required(VERSION 3.0.2)
project(movement)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/move
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
