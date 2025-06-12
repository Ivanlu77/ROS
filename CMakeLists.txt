cmake_minimum_required(VERSION 3.0.2)
project(network_odom)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  std_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/network_odom_to_ros.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
