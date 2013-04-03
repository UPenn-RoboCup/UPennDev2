# Makefile used to build libraries
include ../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: comm 

comm: luaComm.o 
	$(LD) $(LDFLAGS) -o UDPComm.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm

clean:
	rm -f *.$(SHLIBEXT) *.o
