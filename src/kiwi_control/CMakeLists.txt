cmake_minimum_required(VERSION 3.0.2)
project(kiwi_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(cfg/pid.cfg)


catkin_package(
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(kiwi_move2pose_node src/kiwi_move2pose_node.cpp)
add_dependencies(kiwi_move2pose_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(kiwi_move2pose_node 
  ${catkin_LIBRARIES}
)


add_executable(kiwi_pid_traj_tracker_node src/kiwi_pid_traj_track_node.cpp)
target_link_libraries(kiwi_pid_traj_tracker_node 
  ${catkin_LIBRARIES}
)