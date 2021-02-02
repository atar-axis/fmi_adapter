cmake_minimum_required(VERSION 3.14)
project(fmi_adapter)

include(ExternalProject)
include(FetchContent)

# Add FMI Library as an external project
externalproject_add(FMILibraryProject
  GIT_REPOSITORY https://github.com/modelon-community/fmi-library.git
  GIT_TAG 2.1
  TIMEOUT 60
)
externalproject_get_property(FMILibraryProject INSTALL_DIR)
set(FMILibraryProject_INCLUDE_DIR "${INSTALL_DIR}/src/install/include")
set(FMILibraryProject_LIB_DIR "${INSTALL_DIR}/src/install/lib")

#Add JSON Library as an external project
FetchContent_Declare(nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.9.1) # cmake >= 3.11
FetchContent_MakeAvailable(nlohmann_json) # cmake >= 3.14
set(nlohmann_json_INCLUDE_DIR "${nlohmann_json_SOURCE_DIR}/single_include")



## Compile as C++17
add_compile_options(-std=c++17 -Wfatal-errors -Wall -Wextra)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin
  REQUIRED COMPONENTS
    roscpp
    std_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
    LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
  # DEPENDS system_lib
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#catkin_lint: ignore_once missing_directory
#catkin_lint: ignore_once external_directory
include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${FMILibraryProject_INCLUDE_DIR}
  ${nlohmann_json_INCLUDE_DIR})

## Declare library from FMILibraryProject. Ensure globally unique name by prepending project name.
add_library(${PROJECT_NAME}_fmilib
  STATIC
    IMPORTED)

set_target_properties(${PROJECT_NAME}_fmilib
  PROPERTIES
    IMPORTED_LOCATION ${FMILibraryProject_LIB_DIR}/libfmilib.a)

### Set strict compiler settings for all source files, but not for test files due to warnings raised by GTest headers.
set_source_files_properties(
  include/${PROJECT_NAME}/FMIMaster.h
  src/${PROJECT_NAME}/FMIMaster.cpp
  include/${PROJECT_NAME}/FMUVariable.h
  src/${PROJECT_NAME}/FMUVariable.cpp
  include/${PROJECT_NAME}/FMU.h
  src/${PROJECT_NAME}/FMU.cpp
  src/${PROJECT_NAME}_node.cpp
  PROPERTIES
    COMPILE_FLAGS "-Wpedantic -Weffc++ -Werror -std=c++17")


## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/FMIMaster.cpp
  src/${PROJECT_NAME}/FMU.cpp
  src/${PROJECT_NAME}/FMUVariable.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  FMILibraryProject
  # JSONLibraryProject
  )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_fmilib
  nlohmann_json::nlohmann_json
  dl
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(
  ${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node
  PROPERTIES
    OUTPUT_NAME node
    PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  FMILibraryProject
  # JSONLibraryProject
  )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_fmilib
  dl
  ${PROJECT_NAME})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# if(CATKIN_ENABLE_TESTING)
#   file(GLOB_RECURSE myTestCppFiles "test/*.cpp")
#   set(GTEST_LIBRARY gtest)
#   find_package(rostest REQUIRED)
#   add_rostest_gtest("${PROJECT_NAME}_test" test/${PROJECT_NAME}.test ${myTestCppFiles})
#   target_include_directories("${PROJECT_NAME}_test" PRIVATE "test" SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
#   target_link_libraries("${PROJECT_NAME}_test" ${PROJECT_NAME} ${catkin_LIBRARIES} stdc++ pthread m)
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)