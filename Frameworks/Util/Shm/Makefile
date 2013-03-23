# Makefile used to build Webots files
include ../../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: shm 

shm: luashm.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIBRT)

clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)
