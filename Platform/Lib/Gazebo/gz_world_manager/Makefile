CWD = $(shell pwd)

libgz_world_manager.so:
	rm -rf build
	mkdir build && cd build && cmake .. && make && make install && cd $(CWD)

clean:
	rm -rf build
	rm -f *.o *.so

.PHONY: libgz_world_manager.so clean
