#USE_NITE_SKELETON=1
LIBNAME=openni
include ../../Makefile.inc
INCLUDES+=-I/usr/local/include/OpenNI2
LDFLAGS+=-lOpenNI2
ifdef USE_NITE_SKELETON
INCLUDES+=-I/usr/local/include/NiTE -DUSE_NITE_SKELETON
LDFLAGS+=-lNiTE2
endif
