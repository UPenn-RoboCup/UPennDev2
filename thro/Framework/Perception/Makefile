.PHONY: all clean
CWD = $(shell pwd)

all:
	cd Slam && make && cd $(CWD)
	cd Octomap && make && cd $(CWD)

clean:
	cd Slam && make clean && cd $(CWD)
	cd Octomap && make clean && cd $(CWD)
