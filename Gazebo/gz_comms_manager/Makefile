CWD = $(shell pwd)

all: libgz_comms_manager

libgz_comms_manager:
	rm -rf build
	mkdir build && cd build && cmake .. && make && make install && cd $(CWD)

clean:
	rm -rf build
	rm -f *.o *.so *.dylib

.PHONY: all clean libgz_comms_manager
