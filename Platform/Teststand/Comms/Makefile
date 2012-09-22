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

INCLUDES  = -I/usr/local/include -I/usr/include -I/usr/local/include/boost
INCLUDES += -I/usr/include/lua5.1 -I../../Lib/Epos -I../../Lib/CANopen
INCLUDES += -I../Mechanics

LIB_DIRS  = -L/usr/local/lib -L/usr/lib
LIB_DIRS += -L../../Lib/CANopen -L../../Lib/Epos -L../Mechanics

LIBS = -lMechanics -lepos -lcanopen -lpthread -lcurses -lrt -llua5.1

all: comms_manager

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

comms_manager: comms_manager.o epos_thread.o comms_thread.o shared_data.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(INCLUDES) $(LIB_DIRS) $(LIBS)

clean:
	rm -f *.o *.$(SHLIBEXT) comms_manager
