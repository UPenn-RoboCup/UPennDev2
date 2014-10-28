MATLAB=matlab -nosplash -nodesktop -nojvm -nodisplay
MEX=mex -O -I/usr/local/include -L/usr/local/lib
MEXEXT=$(shell mexext)
PWD=$(shell pwd)
LIBRT= $(shell [ `uname` != "Darwin" ] && echo "-lrt")

TARGETS=\
	time.$(MEXEXT) \
	mexshm.$(MEXEXT) \
	getch.$(MEXEXT) \
	udp_recv.$(MEXEXT) \
	udp_send.$(MEXEXT) \
	cjpeg.$(MEXEXT) \
	djpeg.$(MEXEXT) \
	msgpack.$(MEXEXT) \
	zmq.$(MEXEXT) \
	tcpopen.$(MEXEXT) \
	lidartrans.$(MEXEXT) \
	zlibCompress.$(MEXEXT) \
	zlibUncompress.$(MEXEXT) \
	astar_graph.$(MEXEXT) \
	astar_nonholonomic16.$(MEXEXT) \
	dijkstra_graph.$(MEXEXT) \
	dijkstra_matrix.$(MEXEXT) \
	dijkstra_nonholonomic16.$(MEXEXT) \
	subs_interp.$(MEXEXT) \
	subs_interp3_circular.$(MEXEXT) \
	yuyv2label.$(MEXEXT) \
	block_bitor.$(MEXEXT) \
	color_count.$(MEXEXT) \
	color_stats.$(MEXEXT) \
	connected_regions.$(MEXEXT) \
	goal_posts.$(MEXEXT) \
	field_lines.$(MEXEXT) \
	field_occupancy.$(MEXEXT) \
	field_spots.$(MEXEXT) \
	label_to_mask.$(MEXEXT) \
#	test \
#	serialize.$(MEXEXT) \
#	deserialize.$(MEXEXT) \

.PHONY: all clean
	
all: $(TARGETS)

%.o: %.cc
	$(CXX) -fpic -O3 -o $@ -c $<

%.$(MEXEXT): %.cc
	$(MEX) $<

mexshm.$(MEXEXT):
	$(MEX) mexshm.cc $(LIBRT)
	
cjpeg.$(MEXEXT):
	$(MEX) cjpeg.cc -ljpeg

djpeg.$(MEXEXT):
	$(MEX) djpeg.cc -ljpeg

zmq.$(MEXEXT):
	$(MEX) zmq.cc -lzmq
	
msgpack.$(MEXEXT):
	$(MEX) msgpack.cc -lmsgpack
	
zlibCompress.$(MEXEXT):
	$(MEX) zlibCompress.cc -lz

zlibUncompress.$(MEXEXT):
	$(MEX) zlibUncompress.cc -lz
	
connected_regions.$(MEXEXT): connected_regions.cc RegionProps.o ConnectRegions.o
	$(MEX) $^

goal_posts.$(MEXEXT): goal_posts.cc RegionProps.o
	$(MEX) $^

goal_crossbar.$(MEXEXT): goal_crossbar.cc RegionProps.o
	$(MEX) $^

field_lines.$(MEXEXT): field_lines.cc RadonTransform.o
	$(MEX) $^

field_spots.$(MEXEXT): field_spots.cc RegionProps.o ConnectRegions.o
	$(MEX) $^

kinematics.$(MEXEXT): kinematics.cc
	$(MEX) $^ ../../../Robots/THOROP/THOROPKinematics.cpp ../../../Robots/Transform/Transform.cpp  -I../../../Robots/THOROP -I../../../Robots/Transform
	
test:
	@echo "Testing matlab scripts!"
	$(MATLAB) -r "run_tests"
	@echo "Done!"

clean:
	rm -f *.$(MEXEXT) *.o
	@cd ImageProc && make clean && cd $(PWD)
