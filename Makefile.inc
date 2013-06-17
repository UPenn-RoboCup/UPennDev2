# Makefile Includes
# http://owen.sj.ca.us/~rk/howto/slides/make/slides/makerecurs.html
TOP := $(dir $(lastword $(MAKEFILE_LIST)))
TARGETS = all clean
.PHONY: $(TARGETS)
	
# From Julia Make.inc
CCCOLOR="\033[34m"
LINKCOLOR="\033[34;1m"
PERLCOLOR="\033[35m"
INFOCOLOR="\033[32m"
JULIACOLOR="\033[32;1m"
SRCCOLOR="\033[33m"
BINCOLOR="\033[37;1m"
JULCOLOR="\033[34;1m"
ENDCOLOR="\033[0m"

## Path Definition
INSTDIR= Run/Lib
WEBOTSCONTDIR= WebotsController
PLAYERDIR= Run
NAODEPDIR= Install/dependencies
NAODEPLUA51DIR= $(NAODEPDIR)/usr/local/lib/lua/5.1

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
QUIET_CC = @printf '    %b %b\n' $(CCCOLOR)CC$(ENDCOLOR) $(SRCCOLOR)$(GOAL)$(ENDCOLOR);
%.o: %.cpp
	$(V)printf '\t%b %b\n' $(CCCOLOR)CXX$(ENDCOLOR) $(SRCCOLOR)$@$(ENDCOLOR);
	$(V)$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(V)printf '\t%b %b\n' $(CCCOLOR)CXX$(ENDCOLOR) $(SRCCOLOR)$@$(ENDCOLOR);
	$(V)$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(V)printf '\t%b %b\n' $(CCCOLOR)CC$(ENDCOLOR) $(SRCCOLOR)$@$(ENDCOLOR);
	$(V)$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
$(LIBNAME): lua_$(LIBNAME).o $(EXTRA_OBJ)
	$(V)printf '\t%b %b\n' $(LINKCOLOR)LINK$(ENDCOLOR) $(BINCOLOR)$@$(ENDCOLOR);
	$(V)$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $^ $(EXTRA_LINK) $(LIB_DIR) $(LIBOPTS)

clean:
	$(V)printf '\t%b %b\n' $(PERLCOLOR)Cleaning$(ENDCOLOR) $(shell pwd);
	$(V)rm -f *.so *.o