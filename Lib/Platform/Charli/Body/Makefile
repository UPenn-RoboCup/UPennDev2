# Makefile used to build libraries
include ../../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

LIBNAME= microstrain

all: microstrain dynamixel

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

microstrain: luamicrostrain.o Microstrain.o
	$(LD) $(LDFLAGS) -o $(LIBNAME).$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm
dynamixel:
	cd Dynamixel && make && cd $(CWD)
clean:
	rm -f *.$(SHLIBEXT) *.o
	cd Dynamixel && make clean && cd $(CWD)
