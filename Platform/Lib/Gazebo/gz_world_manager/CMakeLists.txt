cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(ZMQ libzmq>=3.2 REQUIRED)
  pkg_check_modules(MSGPACK libmsgpack-dev)
  pkg_check_modules(LUA lua5.1)
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
)
