include Makefile.mexlib
BOOST_INCLUDE_DIR=-I/usr/local/include/boost
MEXEXT=$(shell mexext)
TARGETS=time shm getch udp cjpeg djpeg msgpack zmq

all: $(TARGETS)

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
	
msgpack:
	$(MEX) $@.cc -lmsgpack
	
test:
	@cd tests
	@echo "Testing matlab scripts!"

clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)