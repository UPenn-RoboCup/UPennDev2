cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}")

find_package(Boost COMPONENTS system REQUIRED)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(ZMQ libzmq>=3.2 REQUIRED)
  pkg_check_modules(MSGPACK libmsgpack-dev)
  if (APPLE)
    pkg_check_modules(LUA lua)
  else()
    pkg_check_modules(LUA lua5.1)
  endif()
endif()

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
  ${MSGPACK_INCLUDE_DIRS}
  ${LUA_INCLUDE_DIRS}
  ../../Comms
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${ZMQ_LIBRARY_DIRS}
  ${MSGPACK_LIBRARY_DIRS}
  ${LUA_LIBRARY_DIRS}
)

add_library(
  gz_world_manager SHARED
  gz_world_manager.cpp
  ../../Comms/config.cpp
)

target_link_libraries(
  gz_world_manager
  ${GAZEBO_LIBRARIES}
  ${ZMQ_LIBRARIES}
  ${MSGPACK_LIBRARIES}
  ${LUA_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Gazebo is looking for a .so file, so make sure that all shared
# libraries end in .so even if they wouldn't normally on some platforms.
set_target_properties(
  gz_world_manager
  PROPERTIES SUFFIX .so
)

INSTALL(
  TARGETS gz_world_manager
  DESTINATION "."
)
