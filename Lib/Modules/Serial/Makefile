include ../../../Lib/Makefile.inc

all: lua_Serial

SerialDevice.o : SerialDevice.cc
	$(CXX) -c -o $@ $^ $(CXXFLAGS) $(INCLUDES)

%.o: %.cc
	$(CXX) -c -o $@ $^ $(CXXFLAGS) $(INCLUDES)

lua_Serial: lua_Serial.o SerialDevice.o
	$(LD) $(LDFLAGS) -o Serial.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lpthread

clean:
	rm -f *.so *.o 
