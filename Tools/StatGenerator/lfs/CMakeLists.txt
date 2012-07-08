cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(LuaComm)

# include for lua headers
include_directories(../../../Lib/Lua/src/)

# Comm main library definition
add_library(lfs SHARED lfs.c lfs.h)

# set optimization level
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# set library prefix to "" otherwise it will create libComm.so
set_target_properties(lfs PROPERTIES PREFIX "")

target_link_libraries(lfs m)
