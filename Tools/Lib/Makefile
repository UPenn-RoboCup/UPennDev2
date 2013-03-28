include ../../Makefile.inc
#MEX=mex -v
MEX=mex
MEXEXT=$(shell mexext)
LCMDIR=../../Framework/Lib/lcm

all: monitorComm time mexshm udpComm #mexlcm

monitorComm: monitorComm.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o libMonitor.o -c libMonitor.cc
	$(MEX) -O $^ libMonitor.o $(INCLUDES)

time: time.cc timeScalar.cc
	$(MEX) -O $^ $(INCLUDES) $(LIBRT)

cjpeg: cjpeg.cpp
	$(MEX) -O $^ $(INCLUDES) -ljpeg

djpeg: djpeg.cpp
	$(MEX) -O $^ $(INCLUDES) -ljpeg

mexshm: mexshm.cpp
	$(MEX) -O $^ $(INCLUDES) $(LIBRT)
	
udpComm: UDPComm.cc
	$(MEX) -O $^ $(INCLUDES) $(LIBRT)

mexlcm:
	lcm-gen -j $(LCMDIR)/types/*.lcm
	javac -cp `pkg-config --variable=classpath lcm-java` thor/*.java
	jar cf thor_matlab.jar thor/*.class

clean:
	rm -rf thor
	rm -f *.jar *.o *.$(SHLIBEXT) *.$(MEXEXT)
