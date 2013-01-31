include ../../Makefile.inc
MEXEXT=$(shell mexext)
LCMDIR=../../Framework/Lib/lcm

all: monitorComm time mexshm udpComm mexlcm

monitorComm: monitorComm.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o libMonitor.o -c libMonitor.cc
	mex -O $^ libMonitor.o $(INCLUDES)

time: time.cc timeScalar.cc
	mex -O $^ $(INCLUDES) $(LIBRT)

mexshm: mexshm.cpp
	mex -O $^ $(INCLUDES) $(LIBRT)
	
udpComm: UDPComm.cc
	mex -O $^ $(INCLUDES) $(LIBRT)

mexlcm:
	lcm-gen -j $(LCMDIR)/types/*.lcm
	javac -cp `pkg-config --variable=classpath lcm-java` thor/*.java
	jar cf thor_matlab.jar thor/*.class

clean:
	rm -rf thor
	rm -f *.jar *.o *.$(SHLIBEXT) *.$(MEXEXT)
