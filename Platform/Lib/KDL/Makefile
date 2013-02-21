# Makefile used to build libraries
include ../../../Makefile.inc

INCLUDES += -I/usr/include/ -I/usr/local/include/eigen3
OBJECTS  = treecogsolver_recursive.o treeidsolver_recursive_newton_euler.o
OBJECTS += treeidfbsolver_recursive_newton_euler.o pluckermatrix.o spatialvector.o

TARGETS = all clean
.PHONY: $(TARGETS)

all: libKDLcustom.a

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

libKDLcustom.a: $(OBJECTS)
	$(AR) $(ARFLAGS) $@ $^

clean:
	rm -f *.o *.$(SHLIBEXT) libKDLcustom.a
