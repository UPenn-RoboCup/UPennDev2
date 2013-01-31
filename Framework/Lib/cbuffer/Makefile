include ../../../Makefile.inc

%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

all: cbuffer 

cbuffer: luacbuffer.o
	$(CC) -o $@.$(SHLIBEXT) $^ $(LIB_DIR) $(LIBOPTS)

clean:
	rm -f *.o *.$(SHLIBEXT)
