# Makefile used to build libraries
include ../../../Makefile.inc

all: cjpeg

%.o: %.cpp
	g++ $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	g++ $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	gcc $(CFLAGS) $(INCLUDES) -o $@ -c $<

cjpeg: cjpeg.o
	g++ $(LDFLAGS) -o cjpeg.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -ljpeg 

clean:
	rm -f *.$(SHLIBEXT) *.o
