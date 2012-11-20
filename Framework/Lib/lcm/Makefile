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
  CC=gcc -arch i386
  CXX=g++ -arch i386
  CXXFLAGS= -O2
  LD=g++ -arch i386
  LIBRT=
endif

INCLUDES = -I/usr/local/include -I/usr/include -I/usr/include/lua5.1 -I./types

LIB_DIRS = -L/usr/local/lib -L/usr/lib

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

lcm_%.c lcm_%.h: types/%.lcm
	lcm-gen -c $<

all: lcm lcm_rpc_request_t lcm_rpc_response_t

lcm: lualcm.o
	$(CC) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ `pkg-config --libs lcm` 

lcm_rpc_request_t: lcm_rpc_request_t.o lualcm_rpc_request_t.o
	$(CC) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ `pkg-config --libs lcm` 

lcm_rpc_response_t: lcm_rpc_response_t.o lualcm_rpc_response_t.o
	$(CC) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ `pkg-config --libs lcm` 

clean:
	rm -f *.o *.$(SHLIBEXT) lcm_*_t.h
