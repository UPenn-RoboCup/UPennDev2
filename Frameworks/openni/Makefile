# Makefile used to build libraries
include ../../Makefile.inc
INCLUDES+=-I/usr/local/include/OpenNI2 -I/usr/local/include/NiTE
LIBOPTS+=-lOpenNI2 -lNiTE2
CXXFLAGS+=-Wunused-variable
PWD=`pwd`
BASE=$(shell basename `pwd`)

ifdef DEBUG
CXXFLAGS+=-g -Wall
endif

all: $(BASE)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC)  $(CFLAGS)   $(INCLUDES) -o $@ -c $<

%: lua_%.o $(EXTRA_OBJ)
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR)

clean:
	rm -f *.$(SHLIBEXT) *.o
