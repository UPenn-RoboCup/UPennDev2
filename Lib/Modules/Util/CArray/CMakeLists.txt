cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(CArray)

# include for lua headers
include_directories(/usr/local/include)
include_directories(/usr/include/lua5.1)

# unix main library definition
add_library(carray SHARED luacarray.cpp)

# set optimization level
#   TODO: can we now use level 2? it would not work for NaoV3
set(CMAKE_CXX_FLAGS "-O2")

# set library prefix to "" otherwise it will create libNaoCam.so
set_target_properties(carray PROPERTIES PREFIX "")

