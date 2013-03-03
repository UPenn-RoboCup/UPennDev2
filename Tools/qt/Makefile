TARGETS = all clean
.PHONY: $(TARGETS)

PWD=$(shell pwd)

all: lqt rgbselect

lqt:: 
	echo $(PWD)
	cd ../lqt && mkdir bd && cd bd && cmake .. && make
	cd ../lqt/bd/lib && cp *.so $(PWD)/lib/qt
	cd $(PWD)

rgbselect::
	echo $(PWD)
	cd lib/util && make
	cd $(PWD)


clean:
	cd ../lqt/ && rm -rf bd
	cd lib/qt && rm -rf *.so
	cd lib/util && make clean

