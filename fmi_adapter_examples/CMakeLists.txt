cmake_minimum_required(VERSION 2.8.3)
project(fmi_adapter_examples)

find_package(catkin REQUIRED COMPONENTS
  fmi_adapter
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS fmi_adapter roscpp std_msgs
)

include_directories(
  SYSTEM ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_integrated_node src/${PROJECT_NAME}_integrated_node.cpp)
add_dependencies(${PROJECT_NAME}_integrated_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_integrated_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_integrated_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY share/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
