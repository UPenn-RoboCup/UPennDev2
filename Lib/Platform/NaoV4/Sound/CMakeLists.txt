cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(SoundComm)

# include for lua headers
#include_directories(/usr/local/include)
#include_directories(/usr/include/lua5.1)

#include_directories(/home/brindza/upenn/robocup/aldebaran/ctc/nao-atom-cross-toolchain-1.12.3/sysroot/usr/include)
#link_directories(/home/brindza/upenn/robocup/aldebaran/ctc/nao-atom-cross-toolchain-1.12.3/cross/i686-pc-linux-gnu/usr/lib)
#link_directories(/home/brindza/upenn/robocup/aldebaran/ctc/nao-atom-cross-toolchain-1.12.3/sysroot/lib)
#link_directories(/home/brindza/upenn/robocup/aldebaran/ctc/nao-atom-cross-toolchain-1.12.3/sysroot/usr/lib)

#include_directories(/$(NaoqiCTC)/sysroot/usr/include)
#link_directories(/$(NaoqiCTC)/sysroot/usr)
#link_directories(/$(NaoqiCTC)/sysroot/usr/lib)

# unix main library definition
#add_library(unix SHARED luaunix.c)

add_executable(sound_comm sound_comm.cc alsa_util.cc dtmf.cc)
#add_executable(sound_comm sound_comm.c alsa_util.c)
add_executable(fttf tfft.cc)


# set optimization level
#   TODO: can we now use level 2? it would not work for NaoV3
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# set library prefix to "" otherwise it will create libNaoCam.so
#set_target_properties(unix PROPERTIES PREFIX "")

# link to alsa library
target_link_libraries(sound_comm asound)

