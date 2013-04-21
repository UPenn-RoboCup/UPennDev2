include ../../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: carray

carray: lua_carray.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIBRT)

clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)

