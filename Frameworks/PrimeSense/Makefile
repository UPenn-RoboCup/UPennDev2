# Makefile used to build Webots files
include ../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

#swig -lua -I/usr/include/ni openni.

all: primesense

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -I/usr/include/ni -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -I/usr/include/ni -o $@ -c $<

primesense: luaprimesense.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $(LIB_DIR) $^ -lOpenNI -lstdc++

#controller: openni_wrap.o
#	$(CC) -o $@.$(SHLIBEXT) $(LIBOPTS) $(LIB_DIR) -lOpenNI $^

clean:
	rm -f *.o *.$(SHLIBEXT)
