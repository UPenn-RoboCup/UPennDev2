CWD= $(shell pwd)
CTC= $(NaoqiCTC)

all: naoqi 

naoqi:
ifeq ($(CTC),)
	@echo Cross compilitation tool not specified. \
				Please download the ctc tool from \
				the aldebaran website and follow the \
				instructions to configure it for your system
else
	rm -rf build
	mkdir build
	cd build && cmake -DCMAKE_TOOLCHAIN_FILE="$(CTC)/toolchain-atom.cmake" .. \
		&& make && cd $(CWD)
endif

mex:
	mex -O myfft.cc
	mex -O mydtmf.cc dtmf.cc
	mex -O mycrosscorr.cc dtmf.cc
	mex -O find_first_max.cc dtmf.cc

clean:
ifeq ($(CTC),)
	make -f Makefile.local clean	
else
	rm -rf build
endif
	rm -f *mexglx *mexa64
