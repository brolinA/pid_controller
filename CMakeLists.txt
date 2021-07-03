cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(pid)


include_directories(include)
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})
# list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (pid_smd src/main.cpp src/pid_smd.cpp)
# target_link_libraries (lidar_obstacle_detection ${PCL_LIBRARIES})