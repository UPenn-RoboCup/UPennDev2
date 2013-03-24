CWD = $(shell pwd)

all:
	cd Plugins && make && cd $(CWD)
	cd Mechanics && make && cd $(CWD)

clean:
	cd Plugins && make clean && cd $(CWD)
	cd Mechanics && make clean && cd $(CWD)

.PHONY: all clean
