# Makefile used to build libraries
include ../../Lib/Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

all: vcglib

INCLUDE=-Ivcglib -I/usr/include/qt4 $(INCLUDES)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDE) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDE) -o $@ -c $<

vcglib: lua_vcglib.o 
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm -lQtCore -lQtXml

clean:
	rm -f *.$(SHLIBEXT) *.o
