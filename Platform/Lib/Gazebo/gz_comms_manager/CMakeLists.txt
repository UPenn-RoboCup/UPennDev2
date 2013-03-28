cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(LUA lua5.1)
endif()

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${LUA_INCLUDE_DIRS}
  ../../Comms
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${LUA_LIBRARY_DIRS}
)

add_library(
  gz_comms_manager SHARED
  gz_comms_manager.cpp
  ../../Comms/dcm.cpp
  ../../Comms/config.cpp
  ../../Comms/pid.c
  ../../Comms/filter.c
)

target_link_libraries(
  gz_comms_manager
  ${GAZEBO_LIBRARIES}
  ${LUA_LIBRARIES}
)
