# Makefile used to build Webots files

TARGETS = all clean
.PHONY: $(TARGETS)
EXTRA_OBJ=lua_controller.o
LIBNAME=controller
include ../../../Makefile.inc

lua_controller.c: lua_controller.i
	@echo "\t" SWIG $@
	$(V)swig -lua -o $@ -I$(WEBOTS_HOME)/include/controller/c lua_controller.i

ifeq ($(OSTYPE),darwin)
WEBOTS_HOME = /Applications/Webots
else
WEBOTS_HOME = /usr/local/webots
endif
WEBOTS_INCLUDE_DIR = $(WEBOTS_HOME)/include
WEBOTS_LIB_DIR = $(WEBOTS_HOME)/lib

INCLUDES+=-I$(WEBOTS_INCLUDE_DIR)/controller/c
LIB_DIR+=-L$(WEBOTS_LIB_DIR)
LIBOPTS+=-lController

#all: lua_controller
#lua_controller: lua_controller_wrap.o
#	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIB_DIR) $(LIBOPTS) $^

clean:
	rm -f $(LIBNAME).$(SHLIBEXT) *.o *.so lua_controller.c