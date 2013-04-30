# Makefile used to build libraries
include ../../Makefile.inc

#INCLUDES+=-I$(OPENNI2_INCLUDE)
#LIB_DIR+=-L$(OPENNI2_REDIST)

all: octomap

octomap: lua_octomap.o 
	$(LD) $(LDFLAGS) -o Octomap.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -loctomap -loctomath -ltorch

clean:
	rm -f *.$(SHLIBEXT) *.o
