LIBNAME = THOROPKinematics
EXTRA_OBJ = THOROPKinematics.o
EXTRA_LINK = ../Transform/Transform.o
include ../../Makefile.inc
LDFLAGS += -lm
INCLUDES +=-I../Transform/
ifdef USE_TORCH
	CXXFLAGS+=-DTORCH=1
	LDFLAGS+=-ltorch
endif
