cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}")

find_package(Boost COMPONENTS system REQUIRED)
find_package(Protobuf REQUIRED)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  if (APPLE)
    pkg_check_modules(LUA lua)
  else()
    pkg_check_modules(LUA lua5.1)
  endif()
endif()

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${LUA_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
  ../../Comms
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${LUA_LIBRARY_DIRS}
  ${PROTOBUF_LIBRARY_DIRS}
)

add_library(
  gz_comms_manager SHARED
  gz_comms_manager.cpp
  ../../Comms/dcm.cpp
  ../../Comms/config.cpp
  ../../Comms/pid.c
  ../../Comms/filter.c
)

message("${PROTOBUF_LIBRARIES}")
target_link_libraries(
  gz_comms_manager
  ${GAZEBO_LIBRARIES}
  ${LUA_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

# Gazebo is looking for a .so file, so make sure that all shared
# libraries end in .so even if they wouldn't normally on some platforms.
set_target_properties(
  gz_comms_manager
  PROPERTIES SUFFIX .so
)

INSTALL(
  TARGETS gz_comms_manager
  DESTINATION "."
)
