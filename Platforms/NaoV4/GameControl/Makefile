# Makefile used to build libraries
include ../../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: NaoGameControlReceiver

NaoGameControlReceiver: luaNaoGameControlReceiver.o timeScalar.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIBRT)

clean:
	rm -f *.$(SHLIBEXT) *.o
