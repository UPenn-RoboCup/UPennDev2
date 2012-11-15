include ../../../Makefile.inc

WEBOTS_HOME=/usr/local/webots
SYS_INC=
ifeq ($(OSTYPE),darwin)
  WEBOTS_HOME=/Applications/Webots
  SYS_INC=-I/usr/include/sys
endif

WEBOTS_INC=-I$(WEBOTS_HOME)/include
WEBOTS_C_INC=-I$(WEBOTS_HOME)/include/controller/c
LIB_DIR=-L$(WEBOTS_HOME)/lib

%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SYS_INC) $(WEBOTS_INC) -o $@ -c $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SYS_INC) $(WEBOTS_INC) -o $@ -c $<
%.o: %.c
	$(CC)  $(CXXFLAGS) $(INCLUDES) $(SYS_INC) $(WEBOTS_INC) $(WEBOTS_C_INC)  -o $@ -c $<

all: webots

webots_wrap.c: webots.i
	swig -lua $(WEBOTS_C_INC) webots.i

webots: webots_wrap.o
	$(LD) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lController
#ifeq ($(OSTYPE),darwin)
#	ln -s -f webots.dylib webots.so
#endif

clean:
	rm -f webots.$(SHLIBEXT) webots_wrap.c *.o *.so *.dylib 