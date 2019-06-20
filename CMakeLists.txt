cmake_minimum_required(VERSION 2.8.3)
project(navigation_exercise)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
)



add_service_files(
  FILES
  FreeSpace.srv
  SetAMCLPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES imes_navigation test_recovery
)

INCLUDE_DIRECTORIES(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_remapper src/amcl_scan_remapper.cpp include/navigation_exercise/amcl_scan_remapper.h)
target_link_libraries(scan_remapper ${catkin_LIBRARIES})
