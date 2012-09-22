# Makefile used to build libraries

TARGETS = all Kinematics Statics clean
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

all: Kinematics Statics test_kinematics test_statics

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) -I$(INCLUDE_DIR) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) -I$(INCLUDE_DIR) -o $@ -c $<

Kinematics: luaKinematics.o Kinematics.o Transform.o Lut.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm

Statics: luaStatics.o Statics.o Transform.o Lut.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm

test_kinematics: test_kinematics.o Kinematics.o Transform.o Lut.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIB_DIR) -lm

test_statics: test_statics.o Statics.o Kinematics.o Transform.o Lut.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIB_DIR) -lm

clean:
	rm -f *.$(SHLIBEXT) *.o test_kinematics test_statics
