cmake_minimum_required(VERSION 3.0.2)
project(my_teleop)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


