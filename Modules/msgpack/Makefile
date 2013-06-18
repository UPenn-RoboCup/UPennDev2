LIBNAME=msgpack
include ../Makefile.lib
LIBOPTS+=-lmsgpack
torch=true
ifeq ($(torch),true)
  CXXFLAGS+=-DTORCH=1
  LIBOPTS+=-ltorch
endif

