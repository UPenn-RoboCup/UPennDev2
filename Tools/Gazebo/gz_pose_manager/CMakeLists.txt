cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}")

find_package(Boost COMPONENTS system REQUIRED)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
	pkg_check_modules(GAZEBO gazebo)
	pkg_check_modules(ZMQ libzmq>=3.2 REQUIRED)
endif()

include_directories(
  /usr/local/include
	${GAZEBO_INCLUDE_DIRS}
	${ZMQ_INCLUDE_DIRS}
	../../Comms
	)

link_directories(
  /usr/local/lib
	${GAZEBO_LIBRARY_DIRS}
	${ZMQ_LIBRARY_DIRS}
	)

add_library(
	gz_pose_manager SHARED
	gz_pose_manager.cpp
	)

target_link_libraries(
	gz_pose_manager
	${GAZEBO_LIBRARIES}
	${ZMQ_LIBRARIES}
	${MSGPACK_LIBRARIES}
	${Boost_LIBRARIES}
	)

# Gazebo is looking for a .so file, so make sure that all shared
# libraries end in .so even if they wouldn't normally on some platforms.
set_target_properties(
	gz_pose_manager
	PROPERTIES SUFFIX .so
	)

INSTALL(
	TARGETS gz_pose_manager
	DESTINATION "."
	)
