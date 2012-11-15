include ../../../Makefile.inc

BOOST_INCLUDE_DIRS=-I/usr/local/include/boost

%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(BOOST_INCLUDE_DIRS) -o $@ -c $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(BOOST_INCLUDE_DIRS) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) $(BOOST_INCLUDE_DIRS) -o $@ -c $<

all: shm

shm: luashm.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $(LIBRT) $^

clean:
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)