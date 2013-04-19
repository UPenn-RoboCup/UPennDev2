TARGETS = all clean
.PHONY: $(TARGETS)

CXX=g++
CC=gcc
AR=ar
LD=ld
STRIP=strip
CXXFLAGS= -O2 -fpic
ARFLAGS= rcs 

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

INCLUDES = -I/usr/local/include -I/usr/include -I../CANopen

LIB_DIRS = -L/usr/local/lib -L/usr/lib -L../CANopen

all: libepos.a test_epos

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

libepos.a: epos_slave.o
	$(AR) $(ARFLAGS) $@ $^

test_epos: test_epos.o libepos.a
	$(CXX) $(CXXFLAGS) -o $@ $^ $(INCLUDES) $(LIB_DIRS) -lcanopen -lrt

clean:
	rm -f *.o *.$(SHLIBEXT) libepos.a test_epos
