cmake_minimum_required(VERSION 2.8.3)
project(exploration_preprocessing)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w")


find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  pathfinding
  voxel_mapping
  exploration_utils
  message_generation
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES exploration_preprocessing
  CATKIN_DEPENDS pathfinding voxel_mapping exploration_utils
  DEPENDS system_lib
)

include_directories( 
  SYSTEM 
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_library(exploration_preprocessing
  src/frontier_finder.cpp
  src/hierarchical_grid.cpp
  src/connectivity_graph.cpp
)
target_link_libraries(exploration_preprocessing
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)
