.PHONY: all clean
CWD = $(shell pwd)

all:
	cd carray && make && cd $(CWD)
	cd cbuffer && make && cd $(CWD)
	cd cjpeg && make && cd $(CWD)
	cd cutil && make && cd $(CWD)
	cd shm && make && cd $(CWD)
	cd udp && make && cd $(CWD)
	cd unix && make && cd $(CWD)

clean:
	cd carray && make clean && cd $(CWD)
	cd cbuffer && make clean && cd $(CWD)
	cd cjpeg && make clean && cd $(CWD)
	cd cutil && make clean && cd $(CWD)
	cd shm && make clean && cd $(CWD)
	cd udp && make clean && cd $(CWD)
	cd unix && make clean && cd $(CWD)

