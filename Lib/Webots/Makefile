CWD= $(shell pwd)

all: controller comm

controller::
	cd Controller && MODE=$(MODE) make && cd $(CWD)

comm::
	cd Comm && MODE=$(MODE) make && cd $(CWD)

clean::
	cd Controller && make clean && cd $(CWD)
	cd Comm && make clean && cd $(CWD)


