TARGETS = all clean
.PHONY: $(TARGETS)

# Compiler Definition
# Default values here are for Linux
CXX=g++
CC=gcc
#LD=ld
LD=g++
LDFLAGS=
CFLAGS=-O2 -fpic
CXXFLAGS=-O2 -fpic
SHLIBEXT=so
LIBOPTS=-shared -fpic
LIBRT=-lrt

# Library and Include Paths

ifeq ($(shell pkg-config --exists lua && echo 0),0)
  LUA_INC=`pkg-config lua --cflags`
else
  LUA_INC=`pkg-config lua5.1 --cflags`
endif
INCLUDES=-I/usr/local/include -I/usr/include $(LUA_INC)
LIB_DIR=-L/usr/local/lib -L/usr/lib

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
  CC=gcc -arch x86_64
  CXX=g++ -arch x86_64
  LD=g++ -arch x86_64
	
  SHLIBEXT=dylib
  LIBOPTS= -bundle -undefined dynamic_lookup
  LIBRT=
  MODE=$(shell getconf LONG_BIT)
endif
