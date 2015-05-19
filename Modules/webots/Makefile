# Makefile used to build Webots files
EXTRA_OBJ=lua_webots.o
LIBNAME=webots
EXTRA_CLEAN=lua_webots.c
include ../../Makefile.inc

lua_webots.c: lua_webots.i
	$(V)printf "\t%b %b\n" $(CCCOLOR)SWIG$(ENDCOLOR) $(SRCCOLOR)$@$(ENDCOLOR);
	$(V)swig -lua -o $@ -I$(WEBOTS_HOME)/include/controller/c $^

WEBOTS_HOME=/usr/local/webots
#WEBOTS_HOME=/home/thor/Downloads/webots
ifeq ($(OSTYPE),darwin)
WEBOTS_HOME=/Applications/Webots
endif
INCLUDES+=-I$(WEBOTS_HOME)/include/controller/c
LDFLAGS+=-L$(WEBOTS_HOME)/lib -lController
