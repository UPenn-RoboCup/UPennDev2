# Makefile used to build Webots files

TARGETS = all clean
.PHONY: $(TARGETS)

include ../../../Makefile.inc

all: GameControlPacket

GameControlPacket: luagamecontrolpacket.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIBRT)

clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)
