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

clean:
	rm -f controller.$(SHLIBEXT) *.o *.so
