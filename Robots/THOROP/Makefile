LIBNAME = THOROPKinematics
EXTRA_OBJ = THOROPLegKinematics.o THOROPArmKinematics.o THOROPMassProps.o
EXTRA_LINK = ../Transform/Transform.o
include ../../Makefile.inc
LDFLAGS += -lm
INCLUDES +=-I../Transform/
ifdef USE_TORCH
	CXXFLAGS+=-DTORCH=1
	LDFLAGS+=-ltorch
endif
