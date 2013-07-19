LIBNAME=msgpack
include ../../Makefile.inc
LIBOPTS+=-lmsgpack
ifdef USE_TORCH
CXXFLAGS+=-DTORCH=1
LIBOPTS+=-ltorch
endif
