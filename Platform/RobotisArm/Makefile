CWD = $(shell pwd)

all:
	cd Mechanics && make && cd $(CWD)
	cd Comms && make && cd $(CWD)

clean:
	cd Mechanics && make clean && cd $(CWD)
	cd Comms && make clean && cd $(CWD)

.PHONY: all clean
