LIBNAME=kalman
EXTRA_OBJ=BallModel.o
include ../../Makefile.inc

# Is -lrt needed?
MEXEXT=$(shell mexext)
mex:
	mex -O mexKalman.cpp BallModel.cpp $(LIBRT)

clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)