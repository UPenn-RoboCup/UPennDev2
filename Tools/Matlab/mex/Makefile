include Makefile.mexlib
BOOST_INCLUDE_DIR=-I/usr/local/include/boost
MEXEXT=$(shell mexext)

all: time shm getch udp cjpeg djpeg

time: time.cc timeScalar.cc
	$(MEX) $^

shm: shm.cc
	$(MEX) $^ $(BOOST_INCLUDE_DIR)

getch: getch.c
	$(MEX) $^

udp: udp.cc
	$(MEX) $^

uvc: uvc.cc
	$(MEX) $^
	
cjpeg: cjpeg.cc
	$(MEX) $^ -ljpeg -L/usr/local/lib

djpeg: djpeg.cc
	$(MEX) $^ -ljpeg -L/usr/local/lib
	
clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)