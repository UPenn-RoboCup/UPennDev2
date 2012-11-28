# Makefile used to build libraries
include ../../Makefile.inc

MEX_EXT= $(shell mexext)
TARGETS_LUA= 
TARGETS_MEX= ScanMatch2D.$(MEX_EXT) getMapCellsFromRay.$(MEX_EXT) binStats.$(MEX_EXT)

all: $(TARGETS_LUA) $(TARGETS_MEX)

# C-files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

# MEX files
%.$(MEX_EXT): %.cc
	mex -O $^
%.$(MEX_EXT): %.cpp
	mex -O $^

# Lua files
%.$(SHLIBEXT): lua_%.o 
	$(LD) $(LDFLAGS) -o $@ $(LIBOPTS) $^ $(LIB_DIR) -lm
	
clean:
	rm -rf $(TARGETS_MEX) $(TARGETS_LUA) *.o *~