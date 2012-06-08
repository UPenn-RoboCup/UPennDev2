cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(LuaShm)

# include for lua headers
include_directories(/usr/local/include)
include_directories(/usr/include/lua5.1)

# TODO: compiling should set an environment variable for the platform
include_directories(/$(NaoqiCTC)/sysroot/usr/include/boost-1_45)

# unix main library definition
add_library(shm SHARED luashm.cpp)

# set optimization level (shm cannot be higher than 1)
set(CMAKE_CXX_FLAGS "-O1")

# set library prefix to "" otherwise it will create libNaoCam.so
set_target_properties(shm PROPERTIES PREFIX "")

# link to rt for shared memory ipc
target_link_libraries(shm rt)

