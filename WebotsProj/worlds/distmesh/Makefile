TARGETS = all clean
.PHONY: $(TARGETS)

#BOOST_INCLUDE_DIRS=-I/usr/local/boost_1_43_0 -I/opt/local/include -I/usr/local/include
BOOST_INCLUDE_DIRS=
INCLUDES=

ifndef OSTYPE
  OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
  #export OSTYPE
endif

ARCH= $(shell uname -m)
ifeq ($(OSTYPE),linux)
ifeq ($(ARCH),x86_64)
  MEXEXT=mexa64
else
  MEXEXT=mexglx
endif
  SHLIBEXT=so
  LIBRT=-lrt
endif
ifeq ($(OSTYPE),darwin)
  SHLIBEXT=dylib
ifeq ($(ARCH),x86_64)
  MEXEXT=mexmaci64
else
  MEXEXT=mexmaci
endif
  INCLUDES=-I/usr/include -I/usr/local/include
endif

all: dellipse dellipsoid dsegment trisurfupd

dellipse: dellipse.cpp
	mex -O $^ $(BOOST_INCLUDE_DIRS) $(INCLUDES) $(LIBRT)

dellipsoid: dellipsoid.cpp
	mex -O $^ $(BOOST_INCLUDE_DIRS) $(INCLUDES) $(LIBRT)

dsegment: dsegment.cpp
	mex -O $^ $(BOOST_INCLUDE_DIRS) $(INCLUDES) $(LIBRT)

trisurfupd: trisurfupd.cpp
	mex -O $^ $(BOOST_INCLUDE_DIRS) $(INCLUDES) $(LIBRT)

clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)*


