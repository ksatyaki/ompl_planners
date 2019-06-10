cmake_minimum_required(VERSION 2.8.9)
project (ompl_planners_ros)

set(CMAKE_BUILD_TYPE Debug)
set(MRPT_DONT_USE_DBG_LIBS 1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-m64")
set(CMAKE_C_FLAGS "-m64")
set(CMAKE_EXE_LINKER_FLAGS "-m64")
set(CMAKE_MODULE_LINKER_FLAGS "-m64")
set(CMAKE_SHARED_LINKER_FLAGS "-m64")

add_compile_options(-Wno-ignored-attributes -fext-numeric-literals)

#Depends on ompl and MRPT

find_package(ompl REQUIRED)
if(OMPL_FOUND)
  MESSAGE(STATUS "Found ompl ${OMPL_VERSION} at ${OMPL_INCLUDE_DIRS}")
endif(OMPL_FOUND)

find_package(MRPT REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  nav_msgs
  costmap_2d
  visualization_msgs
  ompl_mod_objectives
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${MRPT_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ompl_planners_ros
  CATKIN_DEPENDS roscpp nav_msgs cliffmap_ros costmap_2d visualization_msgs ompl_mod_objectives
  DEPENDS OMPL MRPT
)

add_library(ompl_planners_ros
  SHARED
  src/ompl_planners_ros/mc_reeds_shepp_car_planner.cpp
  src/ompl_planners_ros/mc_state_validity_checker.cpp
  src/ompl_planners_ros/visualization.cpp
)

target_link_libraries(ompl_planners_ros
  ${OMPL_LIBRARIES}
  ${MRPT_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(ompl_planners_ros ${catkin_EXPORTED_TARGETS})

add_executable(car_planner_ros_node src/car_planner_ros_node.cpp)
target_link_libraries(car_planner_ros_node ompl_planners_ros ${catkin_LIBRARIES})

install(TARGETS ompl_planners_ros car_planner_ros_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY cliffmaps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cliffmaps
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
  PATTERN ".svn" EXCLUDE)

