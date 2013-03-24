cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ../../../Lib/Util
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(
  gz_comms_manager SHARED
  gz_comms_manager.cpp
  dcm.cpp
  ../../../Lib/Util/pid.c
  ../../../Lib/Util/filter.c
)

target_link_libraries(
  gz_comms_manager
  ${GAZEBO_LIBRARIES}
)
