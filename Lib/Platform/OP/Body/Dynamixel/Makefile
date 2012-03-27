# Makefile used to build Webots files
include ../../../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: DynamixelPacket

%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -o $@ -c $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDE_DIRS) -o $@ -c $<

DynamixelPacket: luadynamixelpacket.o dynamixel.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIBRT)

clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)
