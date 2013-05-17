LIBNAME=spacenav
EXTRA_OBJ=spnav.o
#EXTRA_OBJ=spnav.o spnav_magellan.o
include ../../Makefile.inc
INCLUDES+=-I/opt/X11/include
LIB_DIR+=-L/opt/X11/lib
LIBOPTS+=-lX11
CWD=`pwd`
spacenavd:
	cd spacenavd && make && cd $(CWD)
clean:
	rm -f *.o *.so *.$(SHLIBEXT)
