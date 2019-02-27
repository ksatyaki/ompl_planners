cmake_minimum_required(VERSION 2.8.9)
project (ompl_planners_ros)

set(MRPT_DONT_USE_DBG_LIBS 1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-m64")
set(CMAKE_C_FLAGS "-m64")
set(CMAKE_EXE_LINKER_FLAGS "-m64")
set(CMAKE_MODULE_LINKER_FLAGS "-m64")
set(CMAKE_SHARED_LINKER_FLAGS "-m64")

#Depends on OMPL and MRPT
find_package(OMPL REQUIRED)
find_package(MRPT REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS 
  roscpp 
  nav_msgs 
  cliffmap_ros 
  costmap_2d
  visualization_msgs
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
  CATKIN_DEPENDS roscpp nav_msgs cliffmap_ros costmap_2d visualization_msgs
  DEPENDS OMPL MRPT
)

add_library(ompl_planners_ros 
  SHARED 
  src/ompl_planners_ros/mc_reeds_shepp_car_planner.cpp 
  src/ompl_planners_ros/mc_state_validity_checker.cpp
  src/ompl/mod/objectives/DTCOptimizationObjective.cpp
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

# #Use "sudo make install" to apply
# install(TARGETS simplereedssheppcarplanner DESTINATION /usr/local/lib)

# install(TARGETS wts_driver wts_driver_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# PATTERN ".svn" EXCLUDE)
