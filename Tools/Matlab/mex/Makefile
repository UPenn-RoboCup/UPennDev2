include Makefile.mexlib
BOOST_INCLUDE_DIR=-I/usr/local/include/boost
MEXEXT=$(shell mexext)

all: time shm getch udp cjpeg djpeg

time: time.cc timeScalar.cc
	$(MEX) $^

shm:
	$(MEX) $(BOOST_INCLUDE_DIR) $@.cc

getch:
	$(MEX) $@.c

udp:
	$(MEX) $@.cc

uvc:
	$(MEX) $@.cc
	
cjpeg:
	$(MEX) $@.cc -ljpeg

djpeg:
	$(MEX) $@.cc -ljpeg

zmq:	
	$(MEX) $@.cc -lzmq

clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)