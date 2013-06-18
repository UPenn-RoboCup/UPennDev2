# Makefile used to build libraries
include ../../../../Makefile.inc


TARGETS = lcmgen
OBJECTS = emit_c.o \
					emit_lua.o \
					getopt.o \
					lcmgen.o \
					main.o \
					sprintfalloc.o \
					tokenize.o

.PHONY: $(TARGETS)

INCLUDES +=-I/usr/include/glib-2.0 -I/usr/lib/i386-linux-gnu/glib-2.0/include  
 
CXXFLAGS += -std=c99
GLIB_LIBS = -pthread -lgthread-2.0 -lrt -lglib-2.0  

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<


lcmgen: $(OBJECTS)
	g++ $(INCLUDES) $(LIB_DIRS) $(CPP_FLAGS) -o $@ $^ $(GLIB_LIBS)

clean:
	rm -f lcmgen *.o
