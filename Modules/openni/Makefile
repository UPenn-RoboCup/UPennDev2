#USE_NITE_SKELETON=1
LIBNAME=openni
NOTUNE=1
include ../../Makefile.inc
INCLUDES+=-I/usr/local/include/OpenNI2 \
					-I/usr/local/include/ni2
LDFLAGS+=-L/usr/local/lib/ni2 \
				 -lOpenNI2
ifdef USE_NITE_SKELETON
INCLUDES+=-I/usr/local/include/NiTE -DUSE_NITE_SKELETON
LDFLAGS+=-lNiTE2
endif
