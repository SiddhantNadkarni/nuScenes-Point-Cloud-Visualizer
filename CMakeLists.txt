cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(PCL_READER)

find_package(PCL 1.9 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
file(GLOB SOURCES eigenTransformation.cpp)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV 4.2.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

add_executable (pcl_reader ${SOURCES})
target_compile_options(pcl_reader PUBLIC -O3 -fconcepts -march=native -fno-omit-frame-pointer)
set_property(TARGET pcl_reader PROPERTY CXX_STANDARD 17)
target_link_libraries (pcl_reader ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
