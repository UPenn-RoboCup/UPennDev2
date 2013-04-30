# Makefile used to build libraries
include ../../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: SoundComm

SoundComm: SoundComm.o sound_comm_thread.o alsa_util.o dtmf.o 
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIBRT) -lasound

clean:
	rm -f *.$(SHLIBEXT) *.o
