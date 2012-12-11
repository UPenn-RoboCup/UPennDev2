include ../../../Makefile.inc

INCLUDES += -I/usr/include/
LIB_KDL   = `pkg-config --libs orocos-kdl`

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC)  $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

all: Kinematics Statics Dynamics

Kinematics: luaKinematics.o Mechanics.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) $(LIB_KDL)

Statics: luaStatics.o Mechanics.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) $(LIB_KDL)

Dynamics: luaDynamics.o Mechanics.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) $(LIB_KDL)
clean:
	rm -f *.$(SHLIBEXT) *.o
