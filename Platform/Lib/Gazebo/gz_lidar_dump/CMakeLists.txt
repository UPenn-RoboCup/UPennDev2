cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories("/usr/local/lib")

add_library(gz_lidar_dump SHARED lidar_dump.cc)
target_link_libraries(gz_lidar_dump ${GAZEBO_libraries} CameraPlugin zmq jpeg)
