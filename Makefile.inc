# Makefile Includes (http://owen.sj.ca.us/~rk/howto/slides/make/slides/makerecurs.html)
TARGETS = all clean
.PHONY: $(TARGETS)

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
  LIBOPTS= -shared -fpic
  LIBRT= -lrt
ifeq ($(USER),darwin)
  CXXFLAGS= -O3 -fomit-frame-pointer -mtune=native -march=native -pipe -funroll-loops
endif
endif

ifeq ($(OSTYPE),darwin)
#  SHLIBEXT= dylib
  SHLIBEXT= so
  LIBOPTS= -bundle -undefined dynamic_lookup
  # -march=native is not supported, so tune as your system demand
  CXXFLAGS= -O2 -fomit-frame-pointer -mtune=native
  LIBRT=
# The -arch option is only available on Mac systems to create universal binaries
ifeq ($(MODE),64)
  CC=gcc -arch x86_64
  CXX=g++ -arch x86_64
  LD=g++ -arch x86_64
  MEXEXT=mexmaci64
else
  CC=gcc -arch i386
  CXX=g++ -arch i386
  LD=g++ -arch i386
  MEXEXT=mexmaci
endif
endif