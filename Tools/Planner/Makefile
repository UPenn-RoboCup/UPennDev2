# Makefile used to build libraries
include ../../Makefile.inc

all: dijkstra astar

torch=true
ifeq ($(torch),true)
  CXXFLAGS+=-DTORCH=1
  INCLUDES+=-I/usr/local/include/torch
  LIBOPTS+=-ltorch
endif
CXXFLAGS+=-DDEBUG=1

dijkstra: lua_dijkstra.o
	$(CXX) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) $(LIB_DIRS) $(LIBS) -ljpeg

astar: lua_astar.o
	$(CXX) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) $(LIB_DIRS) $(LIBS) -ljpeg

clean:
	rm -f *.$(SHLIBEXT) *.o
