cmake_minimum_required(VERSION 2.8.3)
project(path_planning_map)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  map_server
  message_generation
  std_srvs
  nav_msgs
)

find_package(OpenCV)
find_package(yaml-cpp REQUIRED)



add_service_files(
  FILES
  getUnicyclePos.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS other non-ROS libs
std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
)








add_library(${PROJECT_NAME}
  src/Divide.cpp
  src/Subcell.cpp
  src/Dijsktra.cpp
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


# CODES AJOUTES

# - offline pour les test
# - map_process_node pour l'affichage de la map via opencv

add_executable(launch_path_planning src/launch_path_planning.cpp)
target_link_libraries(launch_path_planning ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} )


add_executable(test_offline src/test_offline.cpp)
target_link_libraries(test_offline ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} )


add_executable(map_process_node src/map_process.cpp)
target_link_libraries(map_process_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(occupancy_grid_loader src/occupancy_grid_loader.cpp)
target_link_libraries(occupancy_grid_loader ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} )


#service

add_executable(getPosition_server src/getPosition_server.cpp)
target_link_libraries(getPosition_server ${catkin_LIBRARIES})
