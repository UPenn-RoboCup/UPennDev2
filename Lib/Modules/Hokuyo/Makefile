OBJECTS  = Hokuyo.o HokuyoCircular.o HokuyoCircularHardware.o 
all: runHokuyo luaHokuyo

#include $(MAGIC_DIR)/Makefile.mk
include ../../Makefile.inc

MAGIC_LIB_DIR    = $(MAGIC_DIR)/lib
MAGIC_INCLUDE_DIR = $(MAGIC_DIR)/include

INCLUDES += -I$(MAGIC_INCLUDE_DIR)
LIB_DIRS += -L$(MAGIC_LIB_DIR)
LIBS     += -lpthread -lSerialDevice -lgz -lipc -ldb_cxx  

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<


runHokuyo: runHokuyo.cc $(OBJECTS)
	g++ $(INCLUDES) $(LIB_DIRS) $(CPP_FLAGS) -o $@ $^ $(LIBS)

luaHokuyo: luaHokuyo.o
	$(LD) $(LDFLAGS) -o Hokuyo.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR)

clean:
	rm -f *.$(SHLIBEXT) *.o runHokuyo
