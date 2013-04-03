# Makefile used to build Webots files

TARGETS = all clean
.PHONY: $(TARGETS)

include ../../../Makefile.inc

ifeq ($(OSTYPE),darwin)
WEBOTS_HOME = /Applications/Webots
else
WEBOTS_HOME = /usr/local/webots
endif
WEBOTS_INCLUDE_DIR = $(WEBOTS_HOME)/include
WEBOTS_LIB_DIR = $(WEBOTS_HOME)/lib

INCLUDES+= -I$(WEBOTS_INCLUDE_DIR)/controller/c

all: controller

controller_wrap.c: controller.i
	swig -lua -I$(WEBOTS_HOME)/include/controller/c controller.i

controller: controller_wrap.o
	$(LD) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ -L$(WEBOTS_LIB_DIR) -lController 
	
openrave:
	swig -lua -I/usr/local/include/openrave-0.9 $(INCLUDES) openrave.i
	c++ $(CXXFLAGS) $(LUAINCLUDE) $(INCLUDES) -I/usr/local/include/openrave-0.9 -o openrave_wrap.o -c openrave_wrap.c
	$(LD) -o $@.$(SHLIBEXT) $(LIBOPTS) openrave_wrap.o -lopenrave0.9-core -lopenrave0.9

clean:
	rm -f controller.$(SHLIBEXT) *.o *.so controller_wrap.c
