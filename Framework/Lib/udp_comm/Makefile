# Makefile used to build libraries
include ../../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: udp

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

udp: lua_udp.o 
	$(LD) $(LDFLAGS) -o udp.$(SHLIBEXT) $^ $(LIBOPTS) $(LIB_DIR) -lm -lstdc++

clean:
	rm -f *.$(SHLIBEXT) *.o
