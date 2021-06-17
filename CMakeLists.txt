cmake_minimum_required(VERSION 3.16.2 FATAL_ERROR)
project(PointCloudSegmentation)
find_package(PCL 1.10 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ocs_exe Unorganized_pcd/3DObjectSegmentation.cpp)
target_link_libraries(ocs_exe ${PCL_LIBRARIES})

add_executable(3di_exe Unorganized_pcd/3dImageRotation.cpp)
target_link_libraries(3di_exe ${PCL_LIBRARIES})