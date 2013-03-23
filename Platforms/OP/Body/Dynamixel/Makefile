# Makefile used to build Webots files
include ../../../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: DynamixelPacket

DynamixelPacket: luadynamixelpacket.o dynamixel.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIBRT)

clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)
