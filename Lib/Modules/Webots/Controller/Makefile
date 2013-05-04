# Makefile used to build Webots files
EXTRA_OBJ=lua_controller.o
LIBNAME=controller
include ../../../Makefile.inc

lua_controller.c: lua_controller.i
	@echo "\t" SWIG $@
	$(V)swig -lua -o $@ -I$(WEBOTS_HOME)/include/controller/c lua_controller.i

WEBOTS_HOME=/usr/local/webots
ifeq ($(OSTYPE),darwin)
WEBOTS_HOME=/Applications/Webots
endif
INCLUDES+=-I$(WEBOTS_HOME)/include/controller/c
LIB_DIR+=-L$(WEBOTS_HOME)/lib
LIBOPTS+=-lController

clean:
	rm -f $(LIBNAME).$(SHLIBEXT) *.o *.so lua_controller.c