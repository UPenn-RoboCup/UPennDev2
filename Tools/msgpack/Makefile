OBJECTS  = 

all: cmsgpack msgpack

include ../../Lib/Makefile.inc

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

msgpack:
	mex -O $@.cc -lmsgpack -I/usr/local/include -L/usr/local/lib

cmsgpack: cmsgpack.o
	$(CXX) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) $(LIB_DIRS) $(LIBS) -lmsgpack

clean:
	rm -f *.$(SHLIBEXT) *.o cpng
