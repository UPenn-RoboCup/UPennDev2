# Makefile used to build Lcm files

TARGETS = all clean
.PHONY: $(TARGETS)

include ../../../Makefile.inc

LCMINCLUDE = /usr/local/include

NAME = lcm

all: lcm lcm_rpc_request_t lcm_rpc_response_t 
#lcm_rpc_t

%.o: %.cc
	$(CXX) $(CXXFLAGS) $(LUAINCLUDE) -I$(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(LUAINCLUDE) -I$(INCLUDES) -o $@ -c $<

lcm: lualcm.o
	$(LD) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ `pkg-config --libs lcm`

lcm_%.c lcm_%.h: types/%.lcm
	lcm-gen -c $<

lcm_%: lcm_%.o lualcm_%.o 
	$(LD) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ `pkg-config --libs lcm`

clean:
	rm -f $(NAME).$(SHLIBEXT) *.o *.so 
