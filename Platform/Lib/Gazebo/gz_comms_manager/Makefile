CWD = $(shell pwd)

all: libgz_comms_manager.so

libgz_comms_manager.so:
	rm -rf build
	mkdir build && cd build && cmake .. && make && make install && cd $(CWD)

clean:
	rm -rf build
	rm -f *.o *.so

.PHONY: all clean libgz_comms_manager.so
