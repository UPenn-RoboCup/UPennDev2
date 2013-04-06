CWD = $(shell pwd)

all: libgz_camera_dump.so

libgz_camera_dump.so:
	rm -rf build
	mkdir build && cd build && cmake .. && make && cp $@ .. && cd $(CWD)

clean:
	rm -rf build
	rm -f *.o *.so

.PHONY: all clean libgz_camera_dump.so
