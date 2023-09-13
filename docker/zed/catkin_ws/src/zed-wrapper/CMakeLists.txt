cmake_minimum_required(VERSION 3.0.2)
project(zed-wrapper)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/zed_depth_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)