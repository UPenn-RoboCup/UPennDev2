# Makefile used to build libraries
include ../../../Makefile.inc

all: cjpeg

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

cjpeg: cjpeg.o
	$(LD) $(LDFLAGS) -o cjpeg.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -ljpeg 

clean:
	rm -f *.$(SHLIBEXT) *.o
