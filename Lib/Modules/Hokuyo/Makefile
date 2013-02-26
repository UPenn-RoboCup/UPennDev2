OBJECTS  = Hokuyo.o HokuyoCircular.o HokuyoCircularHardware.o \
					DeviceInterface.o SerialDevice.o ErrorMessage.o CircularBuffer.o
#all: runHokuyo luaHokuyo lua_signal
all: luaHokuyo lua_signal

include ../../Makefile.inc

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<


runHokuyo: runHokuyo.cc $(OBJECTS)
	$(CXX) $(INCLUDES) $(LIB_DIRS) $(CPP_FLAGS) -o $@ $^ $(LIBS)

luaHokuyo: luaHokuyo.o $(OBJECTS)
	$(CXX) $(LDFLAGS) -o Hokuyo.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) $(LIB_DIRS) $(LIBS) -lpthread

lua_signal: lua_signal.o
	$(CXX) $(LDFLAGS) -o signal.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR)

clean:
	rm -f *.$(SHLIBEXT) *.o runHokuyo
