MEXEXT= $(shell mexext)

all: cjpeg djpeg

include ../../../Lib/Makefile.inc

cjpeg: cjpeg.cpp
	mex -O $^ $(INCLUDES) -ljpeg -L/usr/local/lib -I/usr/local/include

djpeg: djpeg.cpp
	mex -O $^ $(INCLUDES) -ljpeg -L/usr/local/lib -I/usr/local/include

clean:
	rm -f *.$(MEXEXT) *.$(SHLIBEXT) *.o
