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
  LIBOPTS= -shared -fpic
  LIBRT= -lrt
endif
ifeq ($(OSTYPE),darwin)
  SHLIBEXT= dylib
  LIBOPTS= -bundle -undefined dynamic_lookup
ifeq ($(MODE),32)
  CC=gcc -arch i386
  CXX=g++ -arch i386
  LD=g++ -arch i386
endif
  CXXFLAGS= -O2
  LIBRT=
endif

INCLUDES = -I/usr/local/include -I/usr/include/lua -I/usr/include/lua5.1 -I/usr/include/ni
LIB_DIR = -L/usr/local/lib -L/usr/lib

#swig -lua -I/usr/include/ni openni.

all: primesense

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

primesense: luaprimesense.o
	$(CC) -o $@.$(SHLIBEXT) $(LIBOPTS) $(LIB_DIR) -lOpenNI $^

#controller: openni_wrap.o
#	$(CC) -o $@.$(SHLIBEXT) $(LIBOPTS) $(LIB_DIR) -lOpenNI $^

clean:
	rm -f *.o *.$(SHLIBEXT)
