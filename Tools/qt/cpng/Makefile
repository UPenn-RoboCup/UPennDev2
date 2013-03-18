OBJECTS  = 

all: lua_cpng

include ../../../Lib/Makefile.inc

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<


cpng: cpng.cpp
	$(CXX) $(INCLUDES) $(LIB_DIRS) $(CPP_FLAGS) -o $@ $^ $(LIBS) -lpng

lua_cpng: lua_cpng.o
	$(CXX) $(LDFLAGS) -o cpng.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) $(LIB_DIRS) $(LIBS) -lpng

#lua_signal: lua_signal.o
#	$(CXX) $(LDFLAGS) -o signal.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR)

clean:
	rm -f *.$(SHLIBEXT) *.o cpng
