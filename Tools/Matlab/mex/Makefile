MATLAB=matlab -nosplash -nodesktop -nojvm -nodisplay
MEX=mex -O -I/usr/local/include -L/usr/local/lib
MEXEXT=$(shell mexext)
PWD=$(shell pwd)
LIBRT= $(shell [ `uname` != "Darwin" ] && echo "-lrt")

TARGETS=time mexshm getch udp_recv udp_send cjpeg djpeg msgpack zmq ImageProc Planner tcpopen
.PHONY: all clean $(TARGETS)
	
all: $(TARGETS)

time: time.cc timeScalar.cc
	$(MEX) $^

mexshm:
	$(MEX) $@.cc $(LIBRT)

getch:
	$(MEX) $@.c

udp_send:
	$(MEX) $@.cc
udp_recv:
	$(MEX) $@.cc
	
cjpeg:
	$(MEX) $@.cc -ljpeg

djpeg:
	$(MEX) $@.cc -ljpeg

zmq:	
	$(MEX) $@.cc -lzmq
	
ImageProc:
	@echo "Building ImageProc"
	cd ImageProc && make && cd $(PWD)

Planner:
	@echo "Building Planner"
	cd Planner && make && cd $(PWD)
	
msgpack:
	$(MEX) $@.cc -lmsgpack
	
tcpopen:
	$(MEX) $@.c
	
# Platform dependent modules
serialopen:
	$(MEX) $@.c	
uvc:
	$(MEX) $@.cc
	
test:
	@echo "Testing matlab scripts!"
	$(MATLAB) -r "run_tests"
	@echo "Done!"

clean:
	rm -f *.$(MEXEXT) *.o
	@cd ImageProc && make clean && cd $(PWD)
	@cd Planner && make clean && cd $(PWD)
