cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}")

find_package(Boost COMPONENTS system REQUIRED)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories("/usr/local/lib")

add_library(gz_camera_dump SHARED camera_dump.cc)
target_link_libraries(gz_camera_dump ${GAZEBO_libraries} 
  CameraPlugin zmq jpeg
  ${Boost_LIBRARIES}
)

# Gazebo is looking for a .so file, so make sure that all shared
# libraries end in .so even if they wouldn't normally on some platforms.
set_target_properties(
  gz_camera_dump
  PROPERTIES SUFFIX .so
)

INSTALL(
  TARGETS gz_camera_dump
  DESTINATION "."
)
