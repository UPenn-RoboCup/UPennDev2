# Makefile Includes
# http://owen.sj.ca.us/~rk/howto/slides/make/slides/makerecurs.html
TOP := $(dir $(lastword $(MAKEFILE_LIST)))
TARGETS = all clean
.PHONY: $(TARGETS)

# Find the Operating system
ifndef OSTYPE
  OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
endif
ifndef ARCH
  ARCH= $(shell uname -m)
endif
# Cross compiling
ifeq ($(CROSS),true)
  CTCDIR = $(NaoqiCTC)
else
  CTCDIR = 
endif
export CTCDIR

######################
## Path Definitions ##
######################
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

##########################
## Compiler Definitions ##
##########################
# Defaults to Linux
CXX=g++
CC=gcc
LD=ld
LDFLAGS=-lstdc++ -shared -fpic
STRIP=strip
#CXXFLAGS=-O2 -fpic
# Hard core optimizations
CXXFLAGS= -O3 -fomit-frame-pointer -mtune=native -march=native -pipe -funroll-loops
CFLAGS=$(CXXFLAGS)
LIBOPTS=
LIBRT=-lrt
SHLIBEXT=so
INCLUDES=
LIB_DIR=
# OS Specific overrides
ifeq ($(OSTYPE),darwin)
  LIBRT=
  CC=cc
  CXX=c++
  LD=ld -macosx_version_min 10.7
  LDFLAGS=-lstdc++ -bundle -undefined dynamic_lookup
endif

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

# Add Lua to the includes
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

##########################
## Makefile Definitions ##
##########################
# Verbose mode
VERBOSE?=no
V=@
ifeq ($(VERBOSE),yes)
V=
endif
# General Targets
%.o: %.cpp
	@echo "\t" CXX $@
	$(V)$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	@echo "\t" CXX $@
	$(V)$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	@echo "\t" CC $@
	$(V)$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<
$(LIBNAME): lua_$(LIBNAME).o $(EXTRA_OBJ)
	@echo "\t" LINK $@
	$(V)$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $^ $(LIB_DIR) $(LIBOPTS)
	@rm $^
	
###################
## Installation  ##
###################
LUAEXT=lua
ifeq ($(OSTYPE),darwin)
REGEX=\( -regex '.*.$(LUAEXT)' -or -regex '.*.$(SHLIBEXT)' -or -regex '.*.sh' \)
else
REGEX=-regex '.*\($(SHLIBEXT)\|$(LUAEXT)\|sh\)'
endif