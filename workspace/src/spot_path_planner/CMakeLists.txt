cmake_minimum_required(VERSION 3.0.2)
project(spot_path_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/run
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)