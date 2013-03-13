CWD = $(shell pwd)

all:
	cd shm && make && cd $(CWD)
	cd carray && make && cd $(CWD)
	cd cutil && make && cd $(CWD)
	cd cbuffer && make && cd $(CWD)
	cd unix && make && cd $(CWD)
#	cd lcm && make && cd $(CWD)
	cd jpeg && make && cd $(CWD)

clean:
	cd shm && make clean && cd $(CWD)
	cd carray && make clean && cd $(CWD)
	cd cutil && make clean && cd $(CWD)
	cd cbuffer && make clean && cd $(CWD)
	cd unix && make clean && cd $(CWD)
#	cd lcm && make clean && cd $(CWD)
	cd jpeg && make clean && cd $(CWD)

.PHONY: all clean
