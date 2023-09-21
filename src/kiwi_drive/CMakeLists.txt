cmake_minimum_required(VERSION 3.0.2)
project(kiwi_drive)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  roslaunch
  roslint
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_ros
)

find_package(cmake_modules REQUIRED)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Velocity.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  FrameToFrame.srv
  KinematicsForward.srv
  KinematicsInverse.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp roslaunch roslint rospy sensor_msgs std_msgs tf2 tf2_ros tf
  DEPENDS Eigen3 orocos_kdl kdl_parser # system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(kiwi_drive
  src/kiwi_drive/kiwi.cpp
)
target_link_libraries(kiwi_drive ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_controller_node src/kiwi_controller_node.cpp)
target_link_libraries(${PROJECT_NAME}_controller_node 
  kiwi_drive
  ${catkin_LIBRARIES}
  ${kdl_parser_LIBRARIES}
)

add_executable(${PROJECT_NAME}_encoder_node src/sensor/encoder.cpp)
target_link_libraries(
  ${PROJECT_NAME}_encoder_node 
  ${catkin_LIBRARIES}
  ${kdl_parser_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_encoder_node kiwi_drive_generate_messages_cpp)


add_executable(${PROJECT_NAME}_kinematics_node src/kiwi_drive/kinematics.cpp)
target_link_libraries(
  ${PROJECT_NAME}_kinematics_node
  ${catkin_LIBRARIES}
  ${orocos-kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_kinematics_node kiwi_drive_generate_messages_cpp)

add_executable(${PROJECT_NAME}_odometry_node src/kiwi_drive/odometry.cpp)
target_link_libraries(
  ${PROJECT_NAME}_odometry_node
  ${catkin_LIBRARIES}
  ${kdl_parser_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_odometry_node kiwi_drive_generate_messages_cpp)


roslaunch_add_file_check(launch)
install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
