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
CFLAGS=-O2 -fpic
CXXFLAGS=-O2 -fpic
LIBOPTS=-shared -fpic
LIBRT=-lrt
SHLIBEXT=so

# If we are on OP/Nao
ifeq ($(USER),darwin)
  CXXFLAGS= -O3 -fomit-frame-pointer -mtune=native -march=native -pipe -funroll-loops
endif
ifeq ($(USER),nao)
  CXXFLAGS= -O2 -fomit-frame-pointer -mtune=native -march=native -pipe -funroll-loops
endif

# Includes and library directories
INCLUDES=-I/usr/local/include -I/usr/include
LIB_DIR=-L/usr/local/lib -L/usr/lib

# Find both Lua and LuaJit
ifeq ($(shell pkg-config --exists luajit && echo 0),0)
  LUAJIT_INC=`pkg-config luajit --cflags`
  LUAJIT_LIB=`pkg-config luajit --libs`
endif
ifeq ($(shell pkg-config --exists lua5.1 && echo 0),0)
  LUA_INC=`pkg-config lua5.1 --cflags`
  LUA_LIB=`pkg-config lua5.1 --libs`
else
  LUA_INC=`pkg-config lua --cflags`
  LUA_LIB=`pkg-config lua --libs`
endif

ifdef LUAJIT_INC
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

# OS Specific overrides
ifeq ($(OSTYPE),darwin)
  LIBOPTS= 
  CXXFLAGS= -O2 -fomit-frame-pointer -mtune=native
  LIBRT=
  # 64 bit only for OSX using clang
  CC=cc
  CXX=c++
  LD=ld -macosx_version_min 10.7
  LDFLAGS=-lstdc++ -bundle -undefined dynamic_lookup
endif

# REGEX to find lua files
LUAEXT=lua
ifeq ($(OSTYPE),darwin)
REGEX=\( -regex '.*.$(LUAEXT)' -or -regex '.*.$(SHLIBEXT)' -or -regex '.*.sh' \)
else
REGEX=-regex '.*\($(SHLIBEXT)\|$(LUAEXT)\|sh\)'
endif

# Default compiling
%.o: %.cpp
	@echo "\t" CXX $@
	@$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	@echo "\t" CXX $@
	@$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	@echo "\t" CC $@
	@$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

#LIBNAME?=badlibname
#LUA?=lua

$(LIBNAME): lua_$(LIBNAME).o $(EXTRA_OBJ)
	@echo "\t" LINK $@
	@$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $^ $(LIB_DIR) $(LIBOPTS)
	@rm $^

  #test: $(LIBNAME)
  #	@test ! -f $^.so && mv $^.$(SHLIBEXT) $^.so ; echo "Testing..."
  #	$(LUA) test_$^.lua ; echo "done test"
  #	@test ! -f $^.$(SHLIBEXT) && mv $^.so $^.$(SHLIBEXT) ; echo "move back"
