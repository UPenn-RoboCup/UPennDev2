LIBNAME=rrts
EXTRA_OBJ=kdtree.o system_single_integrator.o Transform.o
include ../../Makefile.inc
ifdef USE_TORCH
CXXFLAGS+=-DTORCH=1
LDFLAGS+=-ltorch
endif