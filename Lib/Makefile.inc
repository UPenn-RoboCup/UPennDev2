# Makefile Includes
# http://owen.sj.ca.us/~rk/howto/slides/make/slides/makerecurs.html
TOP := $(dir $(lastword $(MAKEFILE_LIST)))
TARGETS = all clean
.PHONY: $(TARGETS)

## Path Definition

NAODIR= Platform/Nao
NAOV4DIR = Platform/NaoV4
WEBOTSNAODIR= Platform/WebotsNao
WEBOTSOPDIR= Platform/WebotsOP
WEBOTSGENERICDIR= Platform/WebotsGeneric
WEBOTSHUBODIR= Platform/WebotsHubo
WEBOTSCHARLIDIR= Platform/WebotsCharli
WEBOTSSAFFIRDIR= Platform/WebotsSaffir
VISIONTESTDIR = Platform/VisionTest
HPDIR= Platform/HP2
OPDIR= Platform/OP
XOSDIR= Platform/XOS
CHARLIDIR = Platform/Charli
MODULEDIR = Modules
NAOQIDIR= $(NAOV4DIR)/NaoQi
WEBOTSDIR = $(MODULEDIR)/Webots
IMAGEDIR= $(MODULEDIR)/ImageProc
COMMDIR= $(MODULEDIR)/Comm
OCCMAPDIR = $(MODULEDIR)/OccMap
UTILDIR= $(MODULEDIR)/Util
MATLABDIR= ../Tools/Matlab
COLORTABLEDIR= $(MATLABDIR)/Colortable
VELODIR= $(MODULEDIR)/Velocity
PRIMEDIR= $(MODULEDIR)/PrimeSense
HANDSDIR= $(MODULEDIR)/Hands
PLATFORMS = nao naov4 op xos webots_nao webots_op webots_generic webots_hubo visiontest
INSTDIR= ../Player/Lib
WEBOTSCONTDIR= ../WebotsController
PLAYERDIR= ../Player
NAODEPDIR= ../Install/dependencies
NAODEPLUA51DIR= $(NAODEPDIR)/usr/local/lib/lua/5.1

WEBOTSTHOROPDIR= Platform/WebotsTHOROP
WEBOTSATLASDIR= Platform/WebotsAtlas
HOKUYODIR= $(MODULEDIR)/Hokuyo
TORCHDIR= ../Tools/torch
SERIALDIR=$(MODULEDIR)/Serial

## Compiler Definition
CXX=g++
CC=gcc
#LD=ld
LD=g++
LDFLAGS=
STRIP=strip
CFLAGS=-O2 -fpic
CXXFLAGS= -O2 -fpic

MEXEXT=$(shell mexext)
SHLIBEXT= so
LIBOPTS= -shared -fpic
LIBRT= -lrt

# Library and Include Paths
TORCHLIB=torch_jit
ifeq ($(shell pkg-config --exists luajit && echo 0),0)
  LUAJIT_INC=`pkg-config luajit --cflags`
else 
# No LuaJIT found, falling back to regular Lua
ifeq ($(shell pkg-config --exists lua5.1 && echo 0),0)
  LUA_INC=`pkg-config lua5.1 --cflags`
  TORCHLIB=torch
else
  LUA_INC=`pkg-config lua --cflags`
  TORCHLIB=torch
endif
endif

#This does not work 
LUA_INC=-I/usr/include/lua -I/usr/local/include/lua
LUA_VERSION_SPECIFIC=-I/usr/include/lua5.1 -I/usr/local/include/lua5.1 -I/usr/local/include/luajit-2.0 -I/usr/include/luajit-2.0
INCLUDES= -I/usr/local/include -I/usr/include $(LUA_INC) $(LUA_VERSION_SPECIFIC)
#INCLUDES= -I/usr/local/include -I/usr/include $(LUA_INC)
LIB_DIR= -L/usr/local/lib -L/usr/lib

INCLUDES= -I$(TOP)/Common -I/usr/local/include -I/usr/include $(LUA_INC) $(LUAJIT_INC)
LIB_DIR= -L/usr/local/lib -L/usr/lib

ifndef OSTYPE
  OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
endif

ifndef ARCH
  ARCH= $(shell uname -m)
endif

ifndef USER
  USER=$(shell whoami)
endif

ifeq ($(USER),darwin)
  CXXFLAGS= -O3 -fomit-frame-pointer -mtune=native -march=native -pipe -funroll-loops
endif

ifeq ($(OSTYPE),linux)
  SHLIBEXT= so
  LIBOPTS= -shared -fpic -lluajit-5.1
  LIBRT= -lrt
ifeq ($(USER),darwin)
  CXXFLAGS= -O3 -fomit-frame-pointer -mtune=native -march=native -pipe -funroll-loops
endif
ifeq ($(USER),nao)
  CXXFLAGS= -O2 -fomit-frame-pointer -mtune=native -march=native -pipe -funroll-loops
endif
endif

ifeq ($(OSTYPE),darwin)
#  SHLIBEXT= dylib
  SHLIBEXT= so
  LIBOPTS= -bundle -undefined dynamic_lookup
  CXXFLAGS= -O2 -fomit-frame-pointer -mtune=native
  LIBRT=
  # 64 bit only for OSX using clang
  CC=cc
  CXX=c++
  LD=ld
endif
