include ../../../Makefile.inc

INCLUDES += -I/usr/include/ -I/usr/local/include/eigen3

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC)  $(CFLAGS) $(INCLUDES) -o $@ -c $<

all: Kinematics Dynamics

Kinematics: luaKinematics.o Mechanics.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) $(LIB_KDL)

Dynamics: luaDynamics.o Mechanics.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) $(LIB_KDL)
	
clean:
	rm -f *.$(SHLIBEXT) *.o
