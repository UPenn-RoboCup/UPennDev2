cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(LuaOccMap)

# include for lua headers
include_directories(/usr/local/include)
include_directories(/usr/include/lua5.1)

# unix main library definition
add_library(OccMap MODULE luaOccMap.cc OccMap.cpp KmeanCluster.cpp)

# set optimization level
#   TODO: can we now use level 2? it would not work for NaoV3
set(CMAKE_CXX_FLAGS "-O2")

# set library prefix to "" otherwise it will create libNaoCam.so
set_target_properties(OccMap PROPERTIES PREFIX "")
target_link_libraries(OccMap m stdc++)
