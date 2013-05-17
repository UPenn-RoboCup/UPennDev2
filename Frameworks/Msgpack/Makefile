LIBNAME=msgpack
EXTRA_OBJ= msgpack-c/src/objectc.o \
						msgpack-c/src/unpack.o \
						msgpack-c/src/version.o \
						msgpack-c/src/vrefbuffer.o \
						msgpack-c/src/zone.o
include ../../Makefile.inc
INCLUDES+=-Imsgpack-c/src
CXXFLAGS+=-g
torch=false
ifeq ($(torch),true)
  CXXFLAGS+=-DTORCH=1
  INCLUDES+=-I/usr/local/include/torch
  LIBOPTS+=-ltorch
endif

clean:
	rm -rf *.so *.o



