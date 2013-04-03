# Makefile used to build libraries
include ../../../Makefile.inc

TARGETS = all clean
.PHONY: $(TARGETS)

LIBNAME= OPKinematics

all:: kinematics

kinematics:: luaOPKinematics.o OPKinematics.o Transform.o
	$(LD) $(LDFLAGS) -o $(LIBNAME).$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm

clean:
	rm -f *.$(SHLIBEXT) *.o
