cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(rocket)

# include for lua headers
include_directories(/usr/local/include)

# unix main library definition
#add_library(cutil SHARED luacutil.cpp)
add_library(launcher SHARED lualauncher.c)
#add_executable(test test.c)

# set optimization level
#   TODO: can we now use level 2? it would not work for NaoV3
set(CMAKE_CXX_FLAGS "-O2")

# set library prefix to "" otherwise it will create libNaoCam.so
#set_target_properties(cutil PROPERTIES PREFIX "")
#set_target_properties(test PROPERTIES PREFIX "")
set_target_properties(launcher PROPERTIES PREFIX "")

#target_link_libraries(test usb)
target_link_libraries(launcher usb)

