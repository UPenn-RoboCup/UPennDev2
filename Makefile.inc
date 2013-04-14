TARGETS = all clean
.PHONY: $(TARGETS)

# Compiler Definition
# Default values here are for Linux
CXX=g++
CC=gcc
#LD=ld
LD=g++
LDFLAGS=-lstdc++
CFLAGS=-O2 -fpic
CXXFLAGS=$(CFLAGS)
SHLIBEXT=so
LIBOPTS=-shared -fpic
LIBRT=-lrt

# Library and Include Paths

ifeq ($(shell pkg-config --exists orocos-kdl && echo 0),0)
  LIB_KDL= `pkg-config --libs orocos-kdl`
else
  LIB_KDL= -L/usr/local/lib/ -lorocos-kdl
endif

ifeq ($(shell pkg-config --exists luajit && echo 0),0)
  LUAJIT_INC=`pkg-config luajit --cflags`
else 
ifeq ($(shell pkg-config --exists lua5.1 && echo 0),0)
  LUA_INC=`pkg-config lua5.1 --cflags`
else
  LUA_INC=`pkg-config lua --cflags`
endif
endif

INCLUDES=-I/usr/local/include $(LUA_INC) $(LUAJIT_INC)
LIB_DIR=-L/usr/local/lib -L/usr/lib -L/usr/lib/x86_64-linux-gnu/

ifndef OSTYPE
  OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
endif

# Mac specific settings
ifeq ($(OSTYPE),darwin)
  # Universal Binaries
  #CC=gcc -arch x86_64 -arch i386
  #CXX=g++ -arch x86_64 -arch i386
  #LD=g++ -arch x86_64 -arch i386
	
	# 64 bit only for OSX
  #CC=gcc -arch x86_64
  #CXX=g++ -arch x86_64
  #LD=g++ -arch x86_64

	# 64 bit only for OSX
	# clang!
  CC=cc
  CXX=c++
  LD=ld
	
	CFLAGS=-Os -fpic -march=native
	CXXFLAGS=$(CFLAGS)
	LDFLAGS+=-macosx_version_min 10.7
	
  SHLIBEXT=dylib
  LIBOPTS= -bundle -undefined dynamic_lookup
  LIBRT=
  MODE=$(shell getconf LONG_BIT)
  LIB_DIR=-L/usr/local/lib -L/usr/lib
endif
