LIBNAME = DARWINKinematics
EXTRA_OBJ = DARWINArmKinematics.o DARWINLegKinematics.o DARWINMassProps.o
EXTRA_LINK = ../Transform/Transform.o ../Transform/Jacobian.o
include ../../Makefile.inc
LDFLAGS += -lm
INCLUDES +=-I../Transform/
ifdef USE_TORCH
	CXXFLAGS+=-DTORCH=1
	LDFLAGS+=-ltorch
endif
