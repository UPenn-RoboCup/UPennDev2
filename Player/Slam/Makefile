TARGETS = all clean
.PHONY: $(TARGETS)

PWD=$(shell pwd)

all: slam

slam:: 
	echo $(PWD)
	cd slam && make && cp Slam.so $(PWD)/
	cd $(PWD)

clean:
	cd slam && make clean
	cd $(PWD) && rm -f *.$(SHLIBEXT)

