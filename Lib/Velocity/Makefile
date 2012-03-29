# Makefile used to build Webots files
include ../Makefile.inc
TARGETS = all clean
.PHONY: $(TARGETS)

#INCLUDES = -I/usr/local/include -I/usr/include -I/usr/include/lua `pkg-config --cflags opencv`
#LIBS = -L/usr/local/lib -L/usr/lib `pkg-config --libs opencv`

all: ballfilter

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

ballfilter: ballfilter.o BallModel.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $(LIBS) $^

mex:
	mex -O mexBall.cpp BallModel.cc $(INCLUDES) $(LIBRT)

clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)

