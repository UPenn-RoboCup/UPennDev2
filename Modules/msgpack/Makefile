LIBNAME=msgpack
include ../../Makefile.inc
LDFLAGS+=-lmsgpack
ifdef USE_TORCH
CXXFLAGS+=-DTORCH=1
LDFLAGS+=-ltorch
endif
