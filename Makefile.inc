# Makefile Includes
# http://owen.sj.ca.us/~rk/howto/slides/make/slides/makerecurs.html
TOP := $(dir $(lastword $(MAKEFILE_LIST)))
TARGETS = all clean
.PHONY: $(TARGETS)

## Path Definition

NAODIR= Platforms/Nao
NAOV4DIR = Platforms/NaoV4
WEBOTSNAODIR= Platforms/WebotsNao
WEBOTSOPDIR= Platforms/WebotsOP
WEBOTSCHARLIDIR= Platforms/WebotsCharli
WEBOTSGENERICDIR= Platforms/WebotsGeneric
WEBOTSSAFFIRDIR= Platforms/WebotsSaffir
VISIONTESTDIR = Platforms/VisionTest
OPDIR= Platforms/OP
XOSDIR= Platforms/XOS
CHARLIDIR = Platforms/Charli
FRAMEWORKSDIR = Frameworks
NAOQIDIR= $(NAOV4DIR)/NaoQi
WEBOTSDIR = $(FRAMEWORKSDIR)/Webots
IMAGEDIR= $(FRAMEWORKSDIR)/ImageProc
COMMDIR= $(FRAMEWORKSDIR)/Comm
OCCMAPDIR = $(FRAMEWORKSDIR)/OccMap
UTILDIR= $(FRAMEWORKSDIR)/Util
MATLABDIR= Tools/Matlab
COLORTABLEDIR= $(MATLABDIR)/Colortable
VELODIR= $(FRAMEWORKSDIR)/Velocity
PRIMEDIR= $(FRAMEWORKSDIR)/PrimeSense
HANDSDIR= $(FRAMEWORKSDIR)/Hands
PLATFORMS = nao naov4 op xos webots_nao webots_op visiontest
INSTDIR= Run/Lib
WEBOTSCONTDIR= WebotsController
PLAYERDIR= Run
NAODEPDIR= Install/dependencies
NAODEPLUA51DIR= $(NAODEPDIR)/usr/local/lib/lua/5.1

WEBOTSTHOROPDIR= Platforms/WebotsTHOROP
WEBOTSATLASDIR= Platforms/WebotsAtlas
HOKUYODIR= $(FRAMEWORKSDIR)/Hokuyo
TORCHDIR= ../Tools/torch
SERIALDIR=$(FRAMEWORKSDIR)/Serial

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

INCLUDES= -I$(TOP)/$(FRAMEWORKSDIR)/Common -I/usr/local/include -I/usr/include $(LUA_INC) $(LUAJIT_INC)
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
  CXXFLAGS= -O3 -fpic -fomit-frame-pointer -mtune=native -march=native -pipe -funroll-loops
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

LUAEXT= lua
ifeq ($(OSTYPE),darwin)
REGEX=\( -regex '.*.$(LUAEXT)' -or -regex '.*.$(SHLIBEXT)' -or -regex '.*.sh' \)
else
REGEX=-regex '.*\($(SHLIBEXT)\|$(LUAEXT)\|sh\)'
endif

ifeq ($(CROSS),true)
  CTCDIR = $(NaoqiCTC)
else
  CTCDIR = 
endif

ifeq ($(CTCDIR),)
  ifeq ($(OSTYPE),linux)
    SHLIBEXT= so
  endif

  #ifeq ($(OSTYPE),darwin)
  #  SHLIBEXT= dylib
  #  REGEX=\( -regex '.*.$(LUAEXT)' -or -regex '.*.$(SHLIBEXT)' -or -regex '.*.sh' \)
  #endif
  else
    SHLIBEXT= so
  #ifeq ($(OSTYPE),darwin)
  #  REGEX=\( -regex '.*.$(LUAEXT)' -or -regex '.*.$(SHLIBEXT)' -or -regex '.*.sh' \)
  #endif
endif

export CTCDIR

%.o: %.cpp
	@echo "\t" CXX $@
	@$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	@echo "\t" CXX $@
	@$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	@echo "\t" CC $@
	@$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<


# For Intel Atom Z530 on the Darwin-OP
# http://jim-dev.blogspot.com/2010/04/gentoo-linux-on-dell-mini-1012.html
#CFLAGS="-O2 -march=core2 -mtune=generic -mssse3 -mfpmath=sse -pipe -fomit-frame-pointer"
#CXXFLAGS="${CFLAGS}"
# From Gentoo wiki (http://en.gentoo-wiki.com/wiki/Intel_D945GCLF2)
#CHOST="i686-pc-linux-gnu"
#CFLAGS="-O2 -march=core2 -mtune=generic -fomit-frame-pointer -pipe -mssse3 -mfpmath=sse"
# Possible?
#CXXFLAGS= -O3 -funroll-loops -fpic
#-mtune=atom
# ANother guide: http://linuxreviews.org/howtos/compiling/#toc1
# -march=native could help...?
#  Look at the warning here: http://en.gentoo-wiki.com/wiki/Safe_Cflags/Intel
#  http://tombarta.wordpress.com/2008/05/25/gcc-flags/
#  Should be using arch...? http://www.implbits.com/About/Blog/tabid/78/post/building-with-gcc-4-6-on-osx/Default.aspx
# http://stackoverflow.com/questions/1754460/apples-gcc-whats-the-difference-between-arch-i386-and-m32
#  Really, should use native unless on the Nao
# OSX Specific: http://programmers.stackexchange.com/questions/51670/why-do-programmers-use-or-recommend-mac-os-x
