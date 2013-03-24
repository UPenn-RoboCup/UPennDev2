CWD = $(shell pwd)

libgz_time_publisher.so:
	rm -rf build
	mkdir build && cd build && cmake .. && make && cp $@ .. && cd $(CWD)

clean:
	rm -rf build
	rm -f *.o *.so

.PHONY: libgz_time_publisher.so clean
