CWD = $(shell pwd)

all:
	cd Comms && make && cd $(CWD)
	cd Mechanics && make && cd $(CWD)

clean:
	cd Comms && make clean && cd $(CWD)
	cd Mechanics && make clean && cd $(CWD)

.PHONY: all clean
