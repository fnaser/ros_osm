cmake_minimum_required(VERSION 2.8.3)
project(ros_osm)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-write-strings -g ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  dynamic_reconfigure
  tf
  tf2
  message_generation
)

add_service_files(
  FILES
  TransformMapToGps.srv
  GetInformation.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_generation roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  src
)
