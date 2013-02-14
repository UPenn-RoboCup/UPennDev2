CWD = $(shell pwd)

all:
	cd Mechanics && make && cd $(CWD)
	cd Project && make && cd $(CWD)
	cd Project/controllers \
	&& rm -f webots_controller \
	&& ln -s ../../../../Run webots_controller \
	&& cd $(CWD)

clean:
	cd Mechanics && make clean && cd $(CWD)
	cd Project && make clean && cd $(CWD)
	rm -f Project/controllers/webots_controller

.PHONY: all clean
