# Makefile used to build mex files

TARGETS = all clean
.PHONY: $(TARGETS)

ifndef OSTYPE
  OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
endif

MEX = mex
CXXFLAGS = -O

LIBRT=

MEXSUFFIX=$(shell mexext)

all: rgbselect.$(MEXSUFFIX) 

%.$(MEXSUFFIX): %.cc
	$(MEX) $(CXXFLAGS) $<

%.$(MEXSUFFIX): %.c
	$(MEX) $(CXXFLAGS) $<

rgbselect.$(MEXSUFFIX): rgbselect.cc 
	$(MEX) $(CXXFLAGS) $^ $(LIBRT)

clean:
	rm -f *.$(MEXSUFFIX) *.o
