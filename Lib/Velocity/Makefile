# Makefile used to build Webots files

TARGETS = all clean
.PHONY: $(TARGETS)

CXX=g++
CC=gcc
LD=ld
STRIP=strip
CXXFLAGS= -O2 -fpic

SHLIBEXT= so
LIBOPTS= -shared -fpic

ifndef OSTYPE
  OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
  #export OSTYPE
endif

ifeq ($(OSTYPE),linux)
  SHLIBEXT= so
  MEXEXT=mexglx
  LIBOPTS= -shared -fpic
  LIBRT= -lrt
endif
ifeq ($(OSTYPE),darwin)
  SHLIBEXT= dylib
  MEXEXT=mexmaci
  LIBOPTS= -bundle -undefined dynamic_lookup
ifeq ($(MODE),32)
  CC=gcc -arch i386
  CXX=g++ -arch i386
  LD=g++ -arch i386
endif
  CXXFLAGS= -O2
  LIBRT=
endif

INCLUDES = -I/usr/local/include -I/usr/include -I/usr/include/lua `pkg-config --cflags opencv`
LIBS = -L/usr/local/lib -L/usr/lib `pkg-config --libs opencv`

#INCLUDES = -I/usr/local/include -I/usr/include -I/usr/include/lua
#LIBS = -L/usr/local/lib -L/usr/lib

all: ballfilter

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

ballfilter: ballfilter.o BallModel.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $(LIBS) $^

mex:
	mex -O mexBall.cpp BallModel.cc $(INCLUDES) $(LIBRT)

clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)

