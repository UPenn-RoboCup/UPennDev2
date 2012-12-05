CWD = $(shell pwd)

all:
	cd plugins/physics/ash_physics && make && cd $(CWD)
	cd plugins/physics/ash_basic_physics && make && cd $(CWD)
	cd plugins/physics/ash_lowerbody_physics && make && cd $(CWD)

clean:
	cd plugins/physics/ash_physics && make clean && cd $(CWD)
	cd plugins/physics/ash_basic_physics && make clean && cd $(CWD)
	cd plugins/physics/ash_lowerbody_physics && make clean && cd $(CWD)

.PHONY: all clean
