CWD = $(shell pwd)

all:
	cd plugins/physics/saffir_physics && make && cd $(CWD)
	cd plugins/physics/saffir_physics_no_actuators && make && cd $(CWD)

clean:
	cd plugins/physics/saffir_physics && make clean && cd $(CWD)
	cd plugins/physics/saffir_physics_no_actuators && make clean && cd $(CWD)

.PHONY: all clean
