cmake_minimum_required(VERSION 2.8.3)
project(youibot_bms)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  rospy
  std_msgs
  sensor_msgs
  ecl_mobile_robot
  ecl_converters
  ecl_devices
  ecl_geometry
  ecl_sigslots
  ecl_time
  ecl_command_line
  youibot_serial
  youibot_msgs
  youibot_math_toolkit
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES youibot_bms
  CATKIN_DEPENDS nodelet roscpp rospy youibot_serial std_msgs ecl_mobile_robot ecl_converters ecl_devices ecl_geometry ecl_sigslots ecl_time ecl_command_line youibot_msgs youibot_math_toolkit
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)
