# Makefile used to build Webots files
include ../../Makefile.inc
TARGETS = all clean
.PHONY: $(TARGETS)

all: handfilter

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

handfilter: luahandfilter.o BallModel.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $(LIBS) $^

mex:
	mex -O mexBall.cpp BallModel.cpp $(LIBRT)

clean:
	rm -f *.so *.o *.$(SHLIBEXT) *.$(MEXEXT)

