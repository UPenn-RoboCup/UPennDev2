MEX=mex -O -I/usr/local/include -L/usr/local/lib
MEXEXT=$(shell mexext)
PWD=$(shell pwd)

TARGETS= astar_graph \
				 astar_nonholonomic16 \
				 dijkstra_graph \
				 dijkstra_matrix \
				 dijkstra_nonholonomic16 \
				 subs_interp \
				 subs_interp3_circular

.PHONY: all clean $(TARGETS)
	
all: $(TARGETS)

astar_graph:


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

clean:
	rm -f *.$(MEXEXT) *.o
