cmake_minimum_required(VERSION 2.8.3)
project(path_planning_map)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
)

find_package(OpenCV)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS other non-ROS libs
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

add_executable(offline src/offline.cpp)
target_link_libraries(offline ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} )


add_executable(map_process_node src/map_process.cpp)
target_link_libraries(map_process_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})


