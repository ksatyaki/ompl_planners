cmake_minimum_required(VERSION 2.8.9)
project (srscp)

set(MRPT_DONT_USE_DBG_LIBS 1)

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "-m64")
SET(CMAKE_C_FLAGS "-m64")
SET(CMAKE_EXE_LINKER_FLAGS "-m64")
SET(CMAKE_MODULE_LINKER_FLAGS "-m64")
SET(CMAKE_SHARED_LINKER_FLAGS "-m64")

#Depends on OMPL and MRPT
find_package(OMPL REQUIRED)
find_package(MRPT REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS roscpp nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${MRPT_INCLUDE_DIRS}
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES srscp
  CATKIN_DEPENDS roscpp nav_msgs 
  DEPENDS OMPL MRPT
)

add_library(srscp SHARED src/mc_reeds_shepp_car_planner.cpp src/mc_state_validity_checker.cpp)
target_link_libraries(srscp ${OMPL_LIBRARIES} ${MRPT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(srscp ${catkin_EXPORTED_TARGETS})

add_executable(test_rscp tests/test_rscp.cpp)
target_link_libraries(test_rscp srscp ${catkin_LIBRARIES})

add_executable(test_rscp_ros tests/test_rscp_ros.cpp)
target_link_libraries(test_rscp_ros srscp ${catkin_LIBRARIES})

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
