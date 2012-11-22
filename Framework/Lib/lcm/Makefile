# Makefile used to build Lcm files

TARGETS = all clean
.PHONY: $(TARGETS)

include Makefile.inc

LCMINCLUDE = /usr/local/include

LUALCM = lualcm

all: $(LUALCM) lcm_rpc_t

%.o: %.cc
	$(CXX) $(CXXFLAGS) $(LUAINCLUDE) -I$(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(LUAINCLUDE) -I$(INCLUDES) -o $@ -c $<

$(LUALCM)_wrap.c: $(LUALCM).i
	swig -lua -I$(LCMINCLUDE) $(LUALCM).i

$(LUALCM): $(LUALCM)_wrap.o
	$(LD) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ `pkg-config --libs lcm`

lcm_%.c: types/*.lcm
	lcm-gen -c $<

lcm_rpc_t: lcm_rpc_t.o lualcm_rpc_t.o
	$(LD) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ `pkg-config --libs lcm` 


clean:
	rm -f $(LUALCM).$(SHLIBEXT) *.o *.so *_wrap.c lcm_*_t.h
