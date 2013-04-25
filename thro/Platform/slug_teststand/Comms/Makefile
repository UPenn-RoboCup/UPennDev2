TARGETS = all clean CANopen SLUG
.PHONY: $(TARGETS)

CWD=$(shell pwd)
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
INCLUDES += -I/usr/include/lua5.1 -I../../Lib/CANopen
INCLUDES += -I../../Lib/SLUG -I../../Lib/Comms

LIB_DIRS  = -L/usr/local/lib -L/usr/lib -L../../Lib/CANopen -L../../Lib/SLUG

LIBS  = -lslug -lncurses -lcanopen -lpthread -lcurses -lrt -llua5.1 -ldl -lm

OBJECTS  = comms_manager.o slug_thread.o \
           ../../Lib/Comms/comms_thread.o \
           ../../Lib/Comms/config.o \
           ../../Lib/Comms/dcm.o \

all: CANopen SLUG comms_manager

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

comms_manager: $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIB_DIRS) $(LIBS)

CANopen:
	cd ../../Lib/CANopen && make && cd $(CWD)

SLUG:
	cd ../../Lib/SLUG && make && cd $(CWD)

clean:
	rm -f *.o *.$(SHLIBEXT) comms_manager
