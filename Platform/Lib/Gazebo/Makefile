CWD = $(shell pwd)

all:
	cd gz_comms_manager && make && cd $(CWD)
	cd gz_time_publisher && make && cd $(CWD)

clean:
	cd gz_comms_manager && make clean && cd $(CWD)
	cd gz_time_publisher && make clean && cd $(CWD)

.PHONY: all clean
