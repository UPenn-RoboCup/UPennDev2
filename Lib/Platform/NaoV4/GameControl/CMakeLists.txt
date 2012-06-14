cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(NaoGameControlReceiver)

# include for lua headers
include_directories(/usr/local/include)
include_directories(/usr/include/lua5.1)

# main library definition
add_library(NaoGameControlReceiver SHARED luaNaoGameControlReceiver.cc timeScalar.cc)

# set optimization level
#   TODO: can we now use level 2? it would not work for NaoV3
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# set library prefix to "" otherwise it will create libNaoCam.so
set_target_properties(NaoGameControlReceiver PROPERTIES PREFIX "")

# link to pthread library
target_link_libraries(NaoGameControlReceiver m)

