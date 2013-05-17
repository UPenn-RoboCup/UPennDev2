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

# Independent useful options
ifndef OSTYPE
  OSTYPE=$(shell uname -s|awk '{print tolower($$0)}')
endif
ifndef ARCH
  ARCH=$(shell uname -m)
endif
ifndef USER
  USER=$(shell whoami)
endif

## Compiler Definition
# Defaults to Linux
CXX=g++
CC=gcc
LD=g++
LDFLAGS=-shared -fpic
STRIP=strip
CXXFLAGS=-O3 -mtune=native -march=native -fpic
CFLAGS=$(CXXFLAGS)
LIBOPTS=
LIBRT=-lrt
SHLIBEXT=so
# OS Specific overrides
ifeq ($(OSTYPE),darwin)
  LIBRT=
  CC=cc
  CXX=c++
  LD=ld -macosx_version_min 10.7
  LDFLAGS=-lstdc++ -bundle -undefined dynamic_lookup
endif

# Includes and library directories
INCLUDES=
LIB_DIR=

# Find LuaJIT
ifeq ($(shell pkg-config --exists luajit && echo 0),0)
	LUAJIT_INC=`pkg-config luajit --cflags`
	LUAJIT_LIB=`pkg-config luajit --libs`
	#USE_LUAJIT=yes
endif
# Find Pure Lua
LUA_VERSION=5.1
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

# REGEX to find lua files
LUAEXT=lua
ifeq ($(OSTYPE),darwin)
REGEX=\( -regex '.*.$(LUAEXT)' -or -regex '.*.$(SHLIBEXT)' -or -regex '.*.sh' \)
else
REGEX=-regex '.*\($(SHLIBEXT)\|$(LUAEXT)\|sh\)'
endif

# Verbose mode
VERBOSE?=no
V=@
ifeq ($(VERBOSE),yes)
V=
endif

# Default compiling
%.o: %.cpp
	$(V)echo "\t" CXX $@
	$(V)$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(V)echo "\t" CXX $@
	$(V)$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(V)echo "\t" CC $@
	$(V)$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
$(LIBNAME): lua_$(LIBNAME).o $(EXTRA_OBJ)
	@echo "\t" LINK $@
	$(V)$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $^ $(LIB_DIR) $(LIBOPTS)
	@rm $^
