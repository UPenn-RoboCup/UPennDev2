TARGETS = all clean
.PHONY: $(TARGETS)

## Compiler Definition
# Default values here are for Linux
CXX=g++
CC=gcc
#LD=ld
LD=g++
LDFLAGS=
CFLAGS=-O2 -fpic
CXXFLAGS=-O2 -fpic
MEXEXT=mexglx
SHLIBEXT=so
LIBOPTS=-shared -fpic
LIBRT=-lrt

# Library and Include Paths
LUA_INC=`pkg-config lua --cflags`
INCLUDES=-I/usr/local/include -I/usr/include $(LUA_INC)
LIB_DIR=-L/usr/local/lib -L/usr/lib

ifndef OSTYPE
  OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
endif

ifeq ($(OSTYPE),darwin)
	SHLIBEXT=dylib
  #SHLIBEXT=so
  LIBOPTS= -bundle -undefined dynamic_lookup
  LIBRT=
# The -arch option is only available on Mac systems to create universal binaries
MODE=$(shell getconf LONG_BIT)
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