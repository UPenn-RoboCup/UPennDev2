include ../../../Makefile.inc

INCLUDES += -I/usr/include/ -I/usr/local/include/eigen3
INCLUDES += -I../../Lib/KDL

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC)  $(CFLAGS) $(INCLUDES) -o $@ -c $<

all: Kinematics Dynamics

Kinematics: luaKinematics.o Mechanics.o ../../Lib/KDL/libKDLcustom.a
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) $(LIB_KDL)

Dynamics: luaDynamics.o Mechanics.o ../../Lib/KDL/libKDLcustom.a
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) $(LIB_KDL)
	
clean:
	rm -f *.$(SHLIBEXT) *.o
