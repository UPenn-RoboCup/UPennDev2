CWD = $(shell pwd)

all:
	cd gz_comms_manager && make && cd $(CWD)
	cd gz_world_manager && make && cd $(CWD)
	cd gz_camera_dump && make && cd $(CWD)

clean:
	cd gz_comms_manager && make clean && cd $(CWD)
	cd gz_world_manager && make clean && cd $(CWD)
	cd gz_camera_dump && make clean && cd $(CWD)

.PHONY: all clean
