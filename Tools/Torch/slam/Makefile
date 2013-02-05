# Makefile used to build libraries
include ../../../Lib/Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

LIB_DIR+=-L/usr/local/lib/torch

INCLUDES=

all: Slam

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -I/usr/local/include/torch/ -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

Slam: lua_Slam.o lua_binStats.o lua_getMapCellsFromRay.o lua_HoughTransform.o \
				HoughTransform.o lua_ScanMatch2D.o lua_SubsambleDistance.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm -ltorch

clean:
	rm -f *.$(SHLIBEXT) *.o
