MATLAB=matlab -nosplash -nodesktop -nojvm -nodisplay
MEX=mex -O -I/usr/local/include -L/usr/local/lib
MEXEXT=$(shell mexext)
PWD=$(shell pwd)
LIBRT= $(shell [ `uname` != "Darwin" ] && echo "-lrt")

TARGETS= \
	time mexshm getch \
	udp_recv udp_send cjpeg djpeg \
	msgpack zmq \
	tcpopen lidartrans \
	serialize deserialize \
	zlibCompress zlibUncompress \
	astar_graph \
	astar_nonholonomic16 \
	dijkstra_graph \
	dijkstra_matrix \
	dijkstra_nonholonomic16 \
	subs_interp \
	subs_interp3_circular \
	ImageProc

.PHONY: all clean
	
all: $(TARGETS)

time:
	$(MEX) $@.cc

mexshm:
	$(MEX) $@.cc $(LIBRT)

getch:
	$(MEX) $@.c

udp_send:
	$(MEX) $@.cc

udp_recv:
	$(MEX) $@.cc

lidartrans:
	$(MEX) $@.cc
	
cjpeg:
	$(MEX) $@.cc -ljpeg

djpeg:
	$(MEX) $@.cc -ljpeg

zmq:	
	$(MEX) $@.cc -lzmq
	
msgpack:
	$(MEX) $@.cc -lmsgpack
	
tcpopen:
	$(MEX) $@.c

serialize:
	$(MEX) $@.c

deserialize:
	$(MEX) $@.c
	
zlibCompress:
	$(MEX) $@.cc -lz

zlibUncompress:
	$(MEX) $@.cc -lz
	
astar_graph:
	$(MEX) $@.cc

astar_nonholonomic16:
	$(MEX) $@.cc

dijkstra_graph:
	$(MEX) $@.cc

dijkstra_matrix:
	$(MEX) $@.cc

dijkstra_nonholonomic16:
	$(MEX) $@.cc

subs_interp:
	$(MEX) $@.cc

subs_interp3_circular:
	$(MEX) $@.cc

# Platform dependent modules
serialopen:
	$(MEX) $@.c	
uvc:
	$(MEX) $@.cc
	
ImageProc:
	@echo "Building ImageProc"
	cd ImageProc && make && cd $(PWD)
	
test:
	@echo "Testing matlab scripts!"
	$(MATLAB) -r "run_tests"
	@echo "Done!"

clean:
	rm -f *.$(MEXEXT) *.o
	@cd ImageProc && make clean && cd $(PWD)
