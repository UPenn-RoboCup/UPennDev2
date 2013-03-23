cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(ZMQ libzmq>=3.2 REQUIRED)
  pkg_check_modules(MSGPACK libmsgpack-dev)
endif()

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
  ${MSGPACK_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${ZMQ_LIBRARY_DIRS}
  ${MSGPACK_LIBRARY_DIRS}
)

add_library(
  gz_time_publisher SHARED
  gz_time_publisher.cpp
)

target_link_libraries(
  gz_time_publisher
  ${GAZEBO_LIBRARIES}
  ${ZMQ_LIBRARIES}
  ${MSGPACK_LIBRARIES}
)
