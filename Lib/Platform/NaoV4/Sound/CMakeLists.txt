cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(sound)

# include for lua headers
include_directories(/usr/local/include)
include_directories(/usr/include/lua5.1)

# play and record binaries
add_executable(play play.cc alsa_util.cc)
add_executable(record record.cc alsa_util.cc)

# unix main library definition
add_library(SoundComm SHARED SoundComm.cc sound_comm_thread.cc alsa_util.cc dtmf.cc)

# set optimization level
#   TODO: can we now use level 2? it would not work for NaoV3
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# set library prefix to "" otherwise it will create libNaoCam.so
set_target_properties(SoundComm PROPERTIES PREFIX "")

# link to alsa library
target_link_libraries(SoundComm asound)
target_link_libraries(play asound)
target_link_libraries(record asound)

