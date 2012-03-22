# Makefile used to build libraries

TARGETS = all clean
.PHONY: $(TARGETS)

CXX=g++
CC=gcc
LD=g++
STRIP=strip
CXXFLAGS= -O2 -fpic
LDFLAGS=

SHLIBEXT= so
LIBOPTS= -shared

ifndef OSTYPE
  OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
  #export OSTYPE
endif

ARCH=$(shell uname -m)

ifeq ($(OSTYPE),linux)
  SHLIBEXT= so
  LIBOPTS= -shared
ifeq ($(ARCH),x86_64)
  CXXFLAGS= -m64 -O2 -fpic
  LDFLAGS= -m64
endif
endif
ifeq ($(OSTYPE),darwin)
  SHLIBEXT= dylib
  LIBOPTS= -bundle -undefined dynamic_lookup
ifeq ($(ARCH),x86_64)
  CXXFLAGS= -O2 -arch x86_64
  LDFLAGS= -arch x86_64
else
  CXXFLAGS= -O2 -arch i386
  LDFLAGS= -arch i386
endif
endif

INCLUDE_DIRS = -I/usr/local/include -I/usr/include -I/usr/include/lua -I/usr/include/lua5.1
LIB_DIR = /usr/local/lib -L/usr/lib

all: comm 

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDE_DIRS) -o $@ -c $<

comm: luaComm.o 
	$(LD) $(LDFLAGS) -o Comm.$(SHLIBEXT) $(LIBOPTS) $^ -L$(LIB_DIR) -lm

clean:
	rm -f *.$(SHLIBEXT) *.o
