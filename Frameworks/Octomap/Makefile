# Makefile used to build libraries
include ../../Makefile.inc

#INCLUDES+=-I$(OPENNI2_INCLUDE)
#LIB_DIR+=-L$(OPENNI2_REDIST)

all: octomap

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC)  $(CFLAGS)   $(INCLUDES) -o $@ -c $<

octomap: lua_octomap.o 
	$(LD) $(LDFLAGS) -o Octomap.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -loctomap -loctomath

clean:
	rm -f *.$(SHLIBEXT) *.o
