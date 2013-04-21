OBJECTS=
all: libspnav spacenavd lua_spacenav lua_signal

include ../../Lib/Makefile.inc

INC= -Ilibspnav $(INCLUDES)
LIB=-Llibspnav -lspnav -lX11

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INC) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INC) -o $@ -c $<
%.o: %.c
	$(CC) $(CFLAGS) $(INC) -o $@ -c $<

libspnav::
	cd libspnav && make && cd $(CWD)

spacenavd::
	cd spacenavd && make && cd $(CWD)

lua_spacenav: lua_spacenav.o $(OBJECTS) 
	$(CXX) $(LDFLAGS) -o Spacenav.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) $(LIB_DIRS) $(LIBS) $(LIB)

lua_signal: lua_signal.o
	$(CXX) $(LDFLAGS) -o signal.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR)

clean:
	rm -f *.$(SHLIBEXT) *.o
	cd libspnav && make clean && cd $(CWD)
	cd spacenavd && make clean && cd $(CWD)
