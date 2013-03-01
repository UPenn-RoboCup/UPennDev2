# Makefile used to build libraries
include ../../Lib/Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: vcglib

INCLUDE=-Ivcglib -I/usr/include/qt4 -I/opt/X11/include $(INCLUDES)

ifndef OSTYPE
  OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
endif

ifeq ($(OSTYPE),darwin)
LIB_DIR=-F/usr/local/lib -framework QtCore -framework QtXml \
					-macosx_version_min 10.6 \
					-lm
else
LIB_DIR=-lQtCore -lQtXml -lm
endif


%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDE) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDE) -o $@ -c $<
%.o: vcglib/wrap/ply/%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE) -o $@ -c $<

vcglib: lua_vcglib.o plylib.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR)

clean:
	rm -f *.$(SHLIBEXT) *.o
