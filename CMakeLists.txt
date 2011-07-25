cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cluster_subtraction)

find_package(PCL 1.0.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (i2ss i2ss.cpp)
target_link_libraries (i2ss ${PCL_LIBRARIES})

add_executable (scale scale.cpp)
target_link_libraries (scale ${PCL_LIBRARIES})
