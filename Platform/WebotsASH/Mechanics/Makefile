include ../../../Makefile.inc

#TARGETS = all Kinematics Statics clean
#.PHONY: $(TARGETS)
#AR=ar
#ARFLAGS= rcs

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -I$(INCLUDES) -o $@ -c $<
%.o: %.cc
	$(CXX) $(CXXFLAGS) -I$(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC)  $(CXXFLAGS) -I$(INCLUDES) -o $@ -c $<

all: Kinematics

Kinematics: luaKinematics.o Kinematics.o Transform.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $(LIB_DIR) -lm $^

clean:
	rm -f *.$(SHLIBEXT) *.o