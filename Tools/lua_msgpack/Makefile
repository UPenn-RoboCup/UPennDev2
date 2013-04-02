MEXEXT= $(shell mexext)
OBJECTS  = 

all: lua_msgpack

include ../../Lib/Makefile.inc

CXXFLAGS+=

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

lua_msgpack: lua_msgpack.o
	$(CXX) $(LDFLAGS) -o msgpack.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) $(LIB_DIRS) $(LIBS) -lmsgpack

clean:
	rm -f *.$(mexext) *.$(SHLIBEXT) *.o
