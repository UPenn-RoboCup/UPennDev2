include ../../Makefile.inc
MEXEXT=$(shell mexext)

all: monitorComm time mexshm

monitorComm: monitorComm.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o libMonitor.o -c libMonitor.cc
	mex -O $^ libMonitor.o $(INCLUDES)

time: time.cc timeScalar.cc
	mex -O $^ $(INCLUDES) $(LIBRT)

mexshm: mexshm.cpp
	mex -O $^ $(INCLUDES) $(LIBRT)

clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)
