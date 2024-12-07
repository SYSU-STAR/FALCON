cmake_minimum_required(VERSION 2.8.3)
project(fast_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -w")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  cv_bridge
  pathfinding
  trajectory
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES fast_planner
 CATKIN_DEPENDS pathfinding trajectory
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(fast_planner 
  src/planner_manager.cpp
)
target_link_libraries(fast_planner ${catkin_LIBRARIES})

add_executable(traj_server 
  src/traj_server.cpp
)
target_link_libraries(traj_server 
  ${catkin_LIBRARIES} 
  -ldw 
  -lglog
)
