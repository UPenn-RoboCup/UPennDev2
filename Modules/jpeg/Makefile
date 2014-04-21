LIBNAME=jpeg
#EXTRA_OBJ=jpeg_utils.o
include ../../Makefile.inc
LDFLAGS+=-ljpeg
#CFLAGS+=-DDEBUG
ifdef USE_TORCH
	CFLAGS+=-DTORCH=1
	LDFLAGS+=-ltorch
endif
