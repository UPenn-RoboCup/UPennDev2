MATLAB=matlab -nosplash -nodesktop -nojvm -nodisplay
MEX=mex -O -I/usr/local/include -L/usr/local/lib
MEXEXT=$(shell mexext)

TARGETS=time shm getch udp cjpeg djpeg msgpack zmq

all: $(TARGETS)

time: time.cc timeScalar.cc
	$(MEX) $^

shm:
	$(MEX) $@.cc

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
	@echo "Testing matlab scripts!"
	$(MATLAB) -r "run_tests"
	@echo "Done!"

clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)