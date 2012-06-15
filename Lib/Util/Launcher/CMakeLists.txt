cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(rocket)

# include for lua headers
include_directories(../../Lua/src)

# unix main library definition
add_library(launcher SHARED lualauncher.c)

# set optimization level
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")

# set library prefix to "" otherwise it will create liblauncher.so
set_target_properties(launcher PROPERTIES PREFIX "")

#target_link_libraries(test usb)
target_link_libraries(launcher usb)

