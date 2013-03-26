MEXEXT= $(shell mexext)
OBJECTS  = 

all: msgpack

include ../../Lib/Makefile.inc

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

msgpack:
	mex -O $@.cc -lmsgpack -I/usr/local/include -L/usr/local/lib

clean:
	rm -f *.$(mexext) *.$(SHLIBEXT) *.o cpng
