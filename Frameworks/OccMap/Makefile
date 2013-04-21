# Makefile used to build libraries
include ../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: OccMap

OccMap: lua_occmap.o map_update.o map_retrieve.o pfield.o kmean_cluster.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm

clean:
	rm -f *.$(SHLIBEXT) *.o
