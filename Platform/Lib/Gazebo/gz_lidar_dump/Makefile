CWD = $(shell pwd)

all: libgz_camera_dump

libgz_camera_dump:
	rm -rf build
	mkdir build && cd build && cmake .. && make && make install && cd $(CWD)

clean:
	rm -rf build
	rm -f *.o *.so *.dylib

.PHONY: all clean libgz_camera_dump
