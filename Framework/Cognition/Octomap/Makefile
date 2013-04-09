# Makefile used to build libraries
include ../../../Makefile.inc

all: octomap

%.o: %.cpp
	  $(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	  $(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c 
	  $(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

octomap: lua_octomap.o 
	$(LD) $(LDFLAGS) -o Octomap.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -loctomap -loctomath -ltorch

clean:
	rm -f *.$(SHLIBEXT) *.o
