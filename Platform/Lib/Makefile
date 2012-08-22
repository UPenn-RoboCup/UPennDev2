CWD = $(shell pwd)

all:
	cd CANopen && make && cd $(CWD)
	cd Epos && make && cd $(CWD)
	cd Microstrain && make && cd $(CWD)
	cd NetFT && make && cd $(CWD)

clean:
	cd CANopen && make clean && cd $(CWD)
	cd Epos && make clean && cd $(CWD)
	cd Microstrain && make clean && cd $(CWD)
	cd NetFT && make clean && cd $(CWD)
