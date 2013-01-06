CWD= $(shell pwd)

all:: body camera kinematics gamecontrol

body::
	cd Body && MODE=$(MODE) make && cd $(CWD)

camera::
	cd Camera && make && cd $(CWD)

kinematics::
	cd Kinematics && MODE=$(MODE) make && cd $(CWD)

gamecontrol::
	cd GameControl && make && cd $(CWD)

clean:
	cd Body && make clean && cd $(CWD)
	cd Camera && make clean && cd $(CWD)
	cd Kinematics && make clean && cd $(CWD)
	cd GameControl && make clean && cd $(CWD)


