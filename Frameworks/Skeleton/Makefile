# Makefile used to build libraries
include ../../Makefile.inc
INCLUDES+=-I/usr/local/include/OpenNI2 -I/usr/local/include/NiTE
LIBOPTS+=-lOpenNI2 -lNiTE2

all: skeleton

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC)  $(CFLAGS)   $(INCLUDES) -o $@ -c $<

skeleton: lua_skeleton.o 
	$(LD) $(LDFLAGS) -o skeleton.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR)

clean:
	rm -f *.$(SHLIBEXT) *.o
