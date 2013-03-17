# Makefile used to build libraries
include ../../Makefile.inc

INCLUDES+=-I$(OPENNI2_INCLUDE)
LIB_DIR+=-L$(OPENNI2_REDIST)

all: kinect

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC)  $(CFLAGS)   $(INCLUDES) -o $@ -c $<

kinect: luaKinect.o 
	$(LD) $(LDFLAGS) -o Kinect.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm -lOpenNI2

standalone:
	$(CXX) $(INCLUDES) -c standalone.cpp -o run_kinect

clean:
	rm -f *.$(SHLIBEXT) *.o
