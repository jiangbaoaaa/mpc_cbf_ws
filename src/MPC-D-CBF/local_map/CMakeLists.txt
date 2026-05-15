cmake_minimum_required(VERSION 3.0.2)
project(local_map)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  grid_map_ros
  nav_msgs
  pcl_conversions
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  visualization_msgs
)
find_package(PCL REQUIRED)
find_package(CGAL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs grid_map_ros nav_msgs pcl_conversions roscpp sensor_msgs std_msgs tf tf_conversions visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(local_map_pub src/local_map_pub.cpp)

target_link_libraries(local_map_pub
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CGAL_LIB}
  ${GMP_LIBRARIES}
)
