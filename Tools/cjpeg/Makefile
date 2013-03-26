# Makefile used to build libraries
include ../../Lib/Makefile.inc

all: cjpeg

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

cjpeg: cjpeg.o
	$(CXX) $(LDFLAGS) -o cjpeg.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) $(LIB_DIRS) $(LIBS) -ljpeg


clean:
	rm -f *.$(SHLIBEXT) *.o
