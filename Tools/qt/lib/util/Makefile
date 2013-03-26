include ../../../../Lib/Makefile.inc

SOURCES=lua_rgbselect.cpp 

OBJECTS=$(SOURCES:.c=.o)

all: $(SOURCES) rgbselect

%.o: %.cpp
	g++ -c -o $@ $^ $(CPP_FLAGS) $(INCLUDES)

rgbselect: $(OBJECTS)
	g++ $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR)

clean:
	rm -f *.so *.o 
