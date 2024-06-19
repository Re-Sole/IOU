cmake_minimum_required(VERSION 2.8)
project(IntersectionOverUnion)

find_package(PCL 1.12 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(IntersectionOverUnion IntersectionOverUnion.cpp)
target_link_libraries(IntersectionOverUnion ${PCL_LIBRARIES})

add_executable(CountLabels CountLabels.cpp)
target_link_libraries(CountLabels ${PCL_LIBRARIES})

add_executable(Relabel Relabel.cpp)
target_link_libraries(Relabel ${PCL_LIBRARIES})