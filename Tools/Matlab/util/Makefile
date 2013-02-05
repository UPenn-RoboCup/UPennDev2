TARGETS = all clean
.PHONY: $(TARGETS)

BOOST_INCLUDE_DIRS=-I/opt/boost/include
INCLUDES=

ifndef OSTYPE
  OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
endif

ifeq ($(OSTYPE),linux)
  SHLIBEXT=so
  LIBRT=-lrt
endif
ifeq ($(OSTYPE),darwin)
  SHLIBEXT=dylib
  INCLUDES=-I/usr/include -I/usr/local/include
endif

MEXEXT=$(shell mexext)

all: time mexshm getch udpcomm

time: time.cc timeScalar.cc
	mex -O $^ $(BOOST_INCLUDE_DIRS) $(INCLUDES) $(LIBRT)

mexshm: mexshm.cpp
	mex -O $^ $(BOOST_INCLUDE_DIRS) $(INCLUDES) $(LIBRT)

getch: getch.c
	mex -O $^ $(BOOST_INCLUDE_DIRS) $(INCLUDES) $(LIBRT)

udpcomm: UDPComm.cc
	mex -O $^ $(BOOST_INCLUDE_DIRS) $(INCLUDES) $(LIBRT)


clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)

