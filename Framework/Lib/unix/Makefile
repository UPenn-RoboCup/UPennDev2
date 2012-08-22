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

BOOST_HOME = /usr/local/boost

INCLUDE_DIRS = -I/usr/local/include -I/usr/include/lua -I/usr/include/lua5.1
LIB_DIRS = -L/usr/local/lib -L/usr/lib

all: unix stty getch curses

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDE_DIRS) -o $@ -c $<

unix: luaunix.o
	$(CC) -o $@.$(SHLIBEXT) $(LIBOPTS) $^

stty: luastty.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $^

getch: luagetch.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $^

curses: luacurses.o
	$(CC) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIRS) -lncurses

clean:
	rm -f *.o *.$(SHLIBEXT)
