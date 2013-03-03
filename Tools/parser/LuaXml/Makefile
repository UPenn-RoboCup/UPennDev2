# Makefile used to build libraries
include ../../../Lib/Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: LuaXML_lib

ifndef OSTYPE
  OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
endif

ifeq ($(OSTYPE),darwin)
LIB_DIR=-F/usr/local/lib -framework QtCore -framework QtXml \
					-macosx_version_min 10.6 \
					-lm
else
LIB_DIR=-lm
endif


%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDE) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDE) -o $@ -c $<

LuaXML_lib: LuaXML_lib.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ -lm

clean:
	rm -f *.$(SHLIBEXT) *.o
