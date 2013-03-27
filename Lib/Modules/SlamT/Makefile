# Makefile used to build libraries
include ../../../Lib/Makefile.inc
#INCLUDES+=-I../../../Tools/torch/include/
#LIB_DIR+=-L../../../Tools/torch/
INCLUDES+=-I/usr/local/include/torch

all: Slam

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

Slam: lua_Slam.o lua_binStats.o lua_getMapCellsFromRay.o lua_HoughTransform.o \
				HoughTransform.o lua_ScanMatch2D.o lua_SubsambleDistance.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm -ltorch

clean:
	rm -f *.$(SHLIBEXT) *.o
