# Makefile used to build libraries
include ../../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

LIBNAME= NaoCam

ifeq ($(OSTYPE),darwin)
all:
endif

ifeq ($(OSTYPE),linux)
all: camera
endif

camera: naoCam.o naoCamThread.o timeScalar.o i2cBus.o
	$(LD) $(LDFLAGS) $(BOOST) -o $(LIBNAME).$(SHLIBEXT) $(LIBOPTS) $^ -L$(LIB_DIR) -lm 

clean:
	rm -f *.$(SHLIBEXT) *.o
