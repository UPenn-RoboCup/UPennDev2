OBJECTS  = Hokuyo.o HokuyoCircular.o HokuyoCircularHardware.o \
					DeviceInterface.o SerialDevice.o ErrorMessage.o CircularBuffer.o
#all: runHokuyo luaHokuyo lua_signal
all: luaHokuyo lua_signal

#include $(MAGIC_DIR)/Makefile.mk
include ../../Makefile.inc

#MAGIC_LIB_DIR    = $(MAGIC_DIR)/lib
#MAGIC_INCLUDE_DIR = $(MAGIC_DIR)/include

#INCLUDES += -I$(MAGIC_INCLUDE_DIR)
#LIB_DIRS += -L$(MAGIC_LIB_DIR)
#LIBS     += -lpthread 
# -lSerialDevice -lgz -lipc 

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<


runHokuyo: runHokuyo.cc $(OBJECTS)
	g++ $(INCLUDES) $(LIB_DIRS) $(CPP_FLAGS) -o $@ $^ $(LIBS)

luaHokuyo: luaHokuyo.o $(OBJECTS)
	$(LD) $(LDFLAGS) -o Hokuyo.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) $(LIB_DIRS) $(LIBS) -lpthread

lua_signal: lua_signal.o
	$(LD) $(LDFLAGS) -o signal.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR)

clean:
	rm -f *.$(SHLIBEXT) *.o runHokuyo
