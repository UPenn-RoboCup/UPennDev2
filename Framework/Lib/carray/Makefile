# Makefile used to build Webots files
include ../../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: carray

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

carray: luacarray.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $^ $(LIBOPTS) $(LIBRT)

clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)
