# Makefile used to build libraries
include ../../../Makefile.inc

INCLUDES += -I/usr/include/ -I../CANopen

TARGETS = all clean
.PHONY: $(TARGETS)

OBJECTS = slug_slave.o

all: libslug.a

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

libslug.a: $(OBJECTS)
	$(AR) $(ARFLAGS) $@ $^

clean:
	rm -f *.o *.$(SHLIBEXT) libslug.a
