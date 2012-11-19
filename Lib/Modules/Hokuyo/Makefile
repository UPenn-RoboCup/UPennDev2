OBJECTS  = Hokuyo.o HokuyoCircular.o HokuyoCircularHardware.o 
TARGETS = runHokuyo

include $(MAGIC_DIR)/Makefile.mk

runHokuyo: runHokuyo.cc $(OBJECTS)
	g++ $(INCLUDES) $(LIB_DIRS) $(CPP_FLAGS) -o $@ $^ $(LIBS)


