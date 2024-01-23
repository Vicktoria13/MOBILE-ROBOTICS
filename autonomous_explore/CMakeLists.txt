cmake_minimum_required(VERSION 2.8.3)
project(autonomous_explore)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  path_planning_map
)

find_package(OpenCV)
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS  roscpp rospy sensor_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}

)




add_library(${PROJECT_NAME}
  src/UnicycleAutonome.cpp
)


target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp 
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)



add_executable(explore src/explore.cpp)
target_link_libraries(explore ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(save_map_in_pgm src/save_map_in_pgm.cpp)
target_link_libraries(save_map_in_pgm ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})



