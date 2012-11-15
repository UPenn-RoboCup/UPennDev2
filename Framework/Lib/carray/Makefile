include ../../../Makefile.inc

%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

all: carray

carray: luacarray.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIB_DIR) $(LIBOPTS) $(LIBRT) $^

clean:
	rm -f *.o *.$(SHLIBEXT)