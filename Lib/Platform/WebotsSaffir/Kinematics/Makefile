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

ifeq ($(OSTYPE),linux)
  SHLIBEXT= so
  LIBOPTS= -shared
endif
ifeq ($(OSTYPE),darwin)
  SHLIBEXT= dylib
  LIBOPTS= -bundle -undefined dynamic_lookup
  CXXFLAGS= -O2 -arch i386
  LDFLAGS= -arch i386
endif

INCLUDE_DIR = /usr/local/include -I/usr/include -I/usr/include/lua -I/usr/include/lua5.1
LIB_DIR = -L/usr/local/lib -L/usr/lib

LIBNAME= SaffirKinematics

all:: kinematics #test_kinematics

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) -I$(INCLUDE_DIR) -o $@ -c $<

kinematics:: fk_tables.h luaSaffirKinematics.o SaffirKinematics.o Transform.o Lut.o
	$(LD) $(LDFLAGS) -o $(LIBNAME).$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm

test_kinematics:: fk_tables.h test_kinematics.o SaffirKinematics.o Transform.o Lut.o
	$(CXX) -o $@ $^

clean:
	rm -f *.$(SHLIBEXT) test_kinematics *.o
