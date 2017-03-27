cmake_minimum_required(VERSION 2.8.3)
project(uwb_localization)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    rospy
)

install(
  PROGRAMS
    src/ros_uwb_odom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
