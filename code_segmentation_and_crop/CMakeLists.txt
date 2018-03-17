cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(min_cut_segmentation)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (min_cut_segmentation code.cpp)
target_link_libraries (min_cut_segmentation ${PCL_LIBRARIES})
add_executable (segmentation segmentation.cpp)
target_link_libraries (segmentation ${PCL_LIBRARIES})
