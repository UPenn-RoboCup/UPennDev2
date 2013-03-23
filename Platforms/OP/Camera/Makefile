# Makefile used to build libraries
include ../../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

LIBNAME= OPCam

ifeq ($(OSTYPE),darwin)
all:
endif

ifeq ($(OSTYPE),linux)
all: camera
endif

camera: timeScalar.o OPCam.o v4l2.o
	$(LD) $(LDFLAGS) -o $(LIBNAME).$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm $(LIBRT)

clean:
	rm -f *.$(SHLIBEXT) *.o
