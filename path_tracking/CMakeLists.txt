cmake_minimum_required(VERSION 2.8.3)
project(path_tracking)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(read_traj src/read_traj.cpp)
target_link_libraries(read_traj ${catkin_LIBRARIES})

