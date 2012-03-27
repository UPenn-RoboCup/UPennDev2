# Makefile Includes (http://owen.sj.ca.us/~rk/howto/slides/make/slides/makerecurs.html)
TARGETS = all clean
.PHONY: $(TARGETS)

MODE=64
CXX=g++
CC=gcc
LD=ld
STRIP=strip
CXXFLAGS= -O2 -fpic

MEXEXT=mexglx
SHLIBEXT= so
LIBOPTS= -shared -fpic
LIBRT= -lrt

INCLUDES= -I/usr/local/include -I/usr/include/lua -I/usr/include/lua5.1
LIB_DIR= -L/usr/local/lib -L/usr/lib


ifndef OSTYPE
  OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
endif

ifeq ($(OSTYPE),linux)
  SHLIBEXT= so
  LIBOPTS= -shared -fpic
  LIBRT= -lrt
endif

ifeq ($(OSTYPE),darwin)
  SHLIBEXT= dylib
  LIBOPTS= -bundle -undefined dynamic_lookup
  CXXFLAGS= -O2
  LIBRT=
endif

ifeq ($(MODE),64)
  CC=gcc -arch x86_64
  CXX=g++ -arch x86_64
  LD=g++ -arch x86_64
else
  CC=gcc -arch i386
  CXX=g++ -arch i386
  LD=g++ -arch i386
endif

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
