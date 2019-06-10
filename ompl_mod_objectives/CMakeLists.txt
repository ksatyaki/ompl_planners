cmake_minimum_required(VERSION 2.8.3)
project(ompl_mod_objectives)

set(CMAKE_BUILD_TYPE Debug)
set(MRPT_DONT_USE_DBG_LIBS 1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-m64")
set(CMAKE_C_FLAGS "-m64")
set(CMAKE_EXE_LINKER_FLAGS "-m64")
set(CMAKE_MODULE_LINKER_FLAGS "-m64")
set(CMAKE_SHARED_LINKER_FLAGS "-m64")

add_compile_options("-Wno-ignored-attributes")

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ompl REQUIRED)
if(OMPL_FOUND)
  MESSAGE(STATUS "Found ompl ${OMPL_VERSION} at ${OMPL_INCLUDE_DIRS}")
endif(OMPL_FOUND)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED 
  COMPONENTS
  roscpp
  cliffmap_ros
  stefmap_ros
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ompl_mod_objectives
  CATKIN_DEPENDS roscpp cliffmap_ros stefmap_ros
  DEPENDS OMPL Eigen3
)

add_library(ompl_mod_objectives
  SHARED
  src/ompl/mod/objectives/DTCOptimizationObjective.cpp
  src/ompl/mod/objectives/UpstreamCriterionOptimizationObjective.cpp
)

add_dependencies(ompl_mod_objectives ${catkin_EXPORTED_TARGETS})

target_link_libraries(ompl_mod_objectives
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS ompl_mod_objectives
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

