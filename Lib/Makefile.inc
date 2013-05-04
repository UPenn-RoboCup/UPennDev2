# Makefile Includes (http://owen.sj.ca.us/~rk/howto/slides/make/slides/makerecurs.html)
TARGETS = all clean
.PHONY: $(TARGETS)

## Path Definition

NAODIR= Platform/Nao
NAOV4DIR = Platform/NaoV4
WEBOTSNAODIR= Platform/WebotsNao
WEBOTSOPDIR= Platform/WebotsOP
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
PLATFORMS = nao naov4 op xos webots_nao webots_op webots_generic webots_hubo visiontest
INSTDIR= ../Player/Lib
WEBOTSCONTDIR= ../WebotsController
PLAYERDIR= ../Player
NAODEPDIR= ../Install/dependencies
NAODEPLUA51DIR= $(NAODEPDIR)/usr/local/lib/lua/5.1


## Compiler Definition
MODE=$(shell getconf LONG_BIT)
CXX=g++
CC=gcc
#LD=ld
LD=g++
LDFLAGS=
STRIP=strip
CXXFLAGS= -O2 -fpic

MEXEXT=mexglx
SHLIBEXT= so
LIBOPTS= -shared -fpic
LIBRT= -lrt

#This does not work 
LUA_INC=-I/usr/include/lua -I/usr/local/include/lua
LUA_VERSION_SPECIFIC=-I/usr/include/lua5.1 -I/usr/local/include/lua5.1 -I/usr/local/include/luajit-2.0 -I/usr/include/luajit-2.0
INCLUDES= -I/usr/local/include -I/usr/include $(LUA_INC) $(LUA_VERSION_SPECIFIC)
#INCLUDES= -I/usr/local/include -I/usr/include $(LUA_INC)
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


# OS Specific overrides
ifeq ($(OSTYPE),darwin)
  LIBOPTS=-bundle -undefined dynamic_lookup
  CXXFLAGS=-O2 -fomit-frame-pointer -mtune=native
  LIBRT=
  # 64 bit only for OSX using clang
  CC=cc
  CXX=c++
  LD=ld -macosx_version_min 10.7
  LDFLAGS=-lstdc++ -bundle -undefined dynamic_lookup
  MEXEXT=mexmaci64
endif
