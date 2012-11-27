# Makefile used to build Lcm files

TARGETS = all clean
.PHONY: $(TARGETS)

include ../../../Makefile.inc

LCMINCLUDE = /usr/local/include

NAME = lcm

LIBLCM = `pkg-config --libs lcm`

all: lcm
#lcm_rpc_t

%.o: %.cc
	$(CXX) $(CXXFLAGS) $(LUAINCLUDE) -I$(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(LUAINCLUDE) -I$(INCLUDES) -o $@ -c $<

lualcm_wrap.c: lualcm.i
	swig -lua -I$(LCMINCLUDE) $^

lcm: lualcm_wrap.o
	$(LD) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ -pthread -L/usr/local/lib -llcm -lgthread-2.0 -lrt -lglib-2.0

lcm_rpc_t.c: rpc_t.lcm
	lcm-gen -c $^

lcm_rpc_t_wrap.c: lcm_rpc_t.i
	swig -lua -I$(LCMINCLUDE) $^

lcm_rpc_t: lcm_rpc_t_wrap.o lcm_rpc_t.o
	$(LD) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ `pkg-config --libs lcm`



clean:
	rm -f $(NAME).$(SHLIBEXT) *.o *.so *_wrap.c lcm_*_t.c lcm_*_t.h
