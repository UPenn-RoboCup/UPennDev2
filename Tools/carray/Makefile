# Makefile used to build Webots files

TARGETS = all clean
.PHONY: $(TARGETS)

CC=gcc
CXX=g++
LD=ld
STRIP=strip
CXXFLAGS= -O2 -fpic
ARCH = i386

SHLIBEXT= so
LIBOPTS= -shared -fpic
MEXEXT=mexglx

ifndef OSTYPE
  OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
  #export OSTYPE
endif

ifeq ($(OSTYPE),linux)
  SHLIBEXT= so
  LIBOPTS= -shared -fpic
  MEXEXT=mexglx
  LIBRT= -lrt
endif
ifeq ($(OSTYPE),darwin)
#  SHLIBEXT= dylib
  SHLIBEXT= so
  LIBOPTS= -bundle -undefined dynamic_lookup
  MEXEXT=mexmaci
#  ARCH = i386
  ARCH = x86_64
  CXXFLAGS= -O2
  LD=g++
  LIBRT=
endif

INCLUDE_DIRS = -I/usr/local/include
LIB_DIR = /usr/local/lib

all: carray

%.o: %.cc
	$(CXX) -arch $(ARCH) $(CXXFLAGS) $(INCLUDE_DIRS) -o $@ -c $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -o $@ -c $<
#	$(CXX) -arch $(ARCH) $(CXXFLAGS) $(INCLUDE_DIRS) -o $@ -c $<
%.o: %.c
	$(CC) -arch $(ARCH) $(CXXFLAGS) $(INCLUDE_DIRS) -o $@ -c $<

carray: luacarray.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIBRT)

clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)
