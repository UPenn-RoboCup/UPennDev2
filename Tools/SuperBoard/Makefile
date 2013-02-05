all: lua_Serial lua_kBPacket

include ../../Lib/Makefile.inc

kBotPacket2.o : kBotPacket2.c
	g++ -c -o $@ $^ $(CPP_FLAGS) $(INCLUDES)

crc32.o : crc32.c
	g++ -c -o $@ $^ $(CPP_FLAGS) $(INCLUDES)

SerialDevice.o : SerialDevice.cc
	g++ -c -o $@ $^ $(CPP_FLAGS) $(INCLUDES)

%.o: %.cc
	g++ -c -o $@ $^ $(CPP_FLAGS) $(INCLUDES)


lua_Serial: lua_Serial.o SerialDevice.o
	$(LD) $(LDFLAGS) -o Serial.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lpthread

lua_kBPacket: lua_kBPacket.o kBotPacket2.o crc32.o
	$(LD) $(LDFLAGS) -o kBPacket.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lpthread



clean:
	rm -f *.so *.o 
