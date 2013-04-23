LIBNAME=msgpack
include ../../Makefile.inc
LIBOPTS=-lmsgpack
torch=true
ifeq ($(torch),true)
  CXXFLAGS+=-DTORCH=1
  INCLUDES+=-I/usr/local/include/torch
  LIBOPTS+=-ltorch
endif

CXXFLAGS+=-g
