# Makefile used to build Webots files
include ../Makefile.inc
TARGETS = all clean
.PHONY: $(TARGETS)

all: handfilter handfilter2

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

handfilter: luahandfilter.o BallModel.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $(LIBS) $^

handfilter2: luahandfilter2.o BallModel.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $(LIBS) $^

mex:
	mex -O mexBall.cpp BallModel.cpp $(LIBRT)
	cp mexBall.$(MEXEXT) track_right_hand.$(MEXEXT)
	cp mexBall.$(MEXEXT) track_left_hand.$(MEXEXT)

clean:
	rm -f *.so *.o *.$(SHLIBEXT) *.$(MEXEXT)

