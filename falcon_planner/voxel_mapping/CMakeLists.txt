cmake_minimum_required(VERSION 2.8.3)
project(voxel_mapping)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
  eigen_conversions
  exploration_utils
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES voxel_mapping
  CATKIN_DEPENDS roscpp std_msgs exploration_utils
  # DEPENDS system_lib
)

include_directories( 
  SYSTEM 
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(voxel_mapping 
  src/map_server.cpp 
  src/tsdf.cpp
  src/esdf.cpp
  src/occupancy_grid.cpp
)
target_link_libraries(voxel_mapping
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
  glog
  -ldw
)