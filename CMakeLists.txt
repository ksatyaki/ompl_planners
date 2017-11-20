cmake_minimum_required(VERSION 2.8.9)
project (simplereedssheppcarplanner)

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "-m64")
SET(CMAKE_C_FLAGS "-m64")
SET(CMAKE_EXE_LINKER_FLAGS "-m64")
SET(CMAKE_MODULE_LINKER_FLAGS "-m64")
SET(CMAKE_SHARED_LINKER_FLAGS "-m64")

#Depends on OMPL and MRPT
find_package(OMPL REQUIRED)
find_package(MRPT REQUIRED) 

include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
  ${MRPT_INCLUDE_DIRS}
)

file(GLOB SOURCES "src/*.cpp")

add_library(simplereedssheppcarplanner SHARED ${SOURCES})
target_link_libraries(simplereedssheppcarplanner ${OMPL_LIBRARIES} ${MRPT_LIBRARIES})

add_executable(test_rscp tests/test_rscp.cpp)
target_link_libraries(test_rscp simplereedssheppcarplanner)

#Use "sudo make install" to apply
install(TARGETS simplereedssheppcarplanner DESTINATION /usr/local/lib)
