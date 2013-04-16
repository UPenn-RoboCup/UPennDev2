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
INCLUDES=-I/usr/local/include
LIB_DIR=-L/usr/local/lib -L/usr/lib/x86_64-linux-gnu/

# Mac specific settings
ifndef OSTYPE
  OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
endif
ifeq ($(OSTYPE),darwin)
	# clang!
  CC=cc
  CXX=c++
  LD=ld
	CFLAGS=-Os -fpic -march=native
	CXXFLAGS=$(CFLAGS)
	LDFLAGS+=-macosx_version_min 10.7
  SHLIBEXT=dylib
  LIBOPTS=-bundle -undefined dynamic_lookup
  LIBRT=
  LIB_DIR=-L/usr/local/lib
endif

ifeq ($(shell pkg-config --exists orocos-kdl && echo 0),0)
  LIB_KDL= `pkg-config --libs orocos-kdl`
else
  LIB_KDL= -L/usr/local/lib/ -lorocos-kdl
endif

# Find LuaJIT
ifeq ($(shell pkg-config --exists luajit && echo 0),0)
  LUAJIT_INC=`pkg-config luajit --cflags`
  LUAJIT_LIB=`pkg-config luajit --libs`
  #USE_LUAJIT=yes
endif
# Find Pure Lua
LUA_VERSION=5.2
ifeq ($(shell pkg-config --exists lua$(LUA_VERSION) && echo 0),0)
  LUA_INC=`pkg-config lua$(LUA_VERSION) --cflags`
  LUA_LIB=`pkg-config lua$(LUA_VERSION) --libs`
else
  LUA_INC=`pkg-config lua --cflags`
  LUA_LIB=`pkg-config lua --libs`
endif

ifdef USE_LUAJIT
INCLUDES+=$(LUAJIT_INC)
LIBOPTS+=$(LUAJIT_LIB)
else
ifdef LUA_INC
INCLUDES+=$(LUA_INC)
LIBOPTS+=$(LUA_LIB)
else
#BAD
endif
endif
