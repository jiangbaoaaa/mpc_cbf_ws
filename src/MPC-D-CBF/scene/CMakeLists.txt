cmake_minimum_required(VERSION 3.0.2)
project(scene)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS gazebo_msgs geometry_msgs nav_msgs roscpp rospy std_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/gazebo_path_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(movetest_node src/movetest.cpp)
target_link_libraries(movetest_node ${catkin_LIBRARIES})

add_executable(movetest_side_static_node src/movetest_side_static.cpp)
target_link_libraries(movetest_side_static_node ${catkin_LIBRARIES})

add_executable(getrobot_pose src/getrobot_pose.cpp)
target_link_libraries(getrobot_pose ${catkin_LIBRARIES})
