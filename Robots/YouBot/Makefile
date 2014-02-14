LIBNAME = YouBotKinematics
EXTRA_OBJ = YouBotKinematics.o
EXTRA_LINK = ../Transform/Transform.o
include ../../Makefile.inc
LDFLAGS += -lm
INCLUDES +=-I../Transform
#CXXFLAGS+=-DDEBUG
ifdef USE_TORCH
	CXXFLAGS+=-DTORCH=1
	LDFLAGS+=-ltorch
endif
