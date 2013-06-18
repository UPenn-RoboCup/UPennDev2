ifeq ($(shell uname),Darwin)
all:
	@echo No Mac uvc
else
LIBNAME=uvc
EXTRA_OBJ=v4l2.o timeScalar.o
include ../../Makefile.inc
LIBOPTS+=-lm $(LIBRT)
endif

