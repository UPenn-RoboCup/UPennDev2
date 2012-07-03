# Makefile used to build libraries
include ../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: OccMap

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

OccMap: luaOccMap.o OccMap.o KmeanCluster.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm

clean:
	rm -f *.$(SHLIBEXT) *.o
