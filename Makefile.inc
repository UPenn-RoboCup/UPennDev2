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

# Mac specific settings
ifeq ($(OSTYPE),darwin)
  # Universal Binaries
  CC=gcc -arch x86_64 -arch i386
  CXX=g++ -arch x86_64 -arch i386
  LD=g++ -arch x86_64 -arch i386
  SHLIBEXT=dylib
  LIBOPTS= -bundle -undefined dynamic_lookup
  LIBRT=
  MODE=$(shell getconf LONG_BIT)
ifeq ($(MODE),64)
  MEXEXT=mexmaci64
else
  MEXEXT=mexmaci
endif

endif