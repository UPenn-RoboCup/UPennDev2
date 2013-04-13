# Makefile used to build libraries
include ../../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: Udp

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

Udp: luaUdp.o 
	$(LD) $(LDFLAGS) -o Udp.$(SHLIBEXT) $^ $(LIBOPTS) $(LIB_DIR) -lm -lstdc++

clean:
	rm -f *.$(SHLIBEXT) *.o
