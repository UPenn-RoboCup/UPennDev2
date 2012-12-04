# Makefile used to build Lcm files
include ../../../Makefile.inc
CWD = $(shell pwd)
NAME = lcm 

all:: lcmgen lcm lcm_rpc_request_t lcm_rpc_response_t 
#lcm_rpc_t

%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
	
lcm: lualcm.o
	$(LD) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ `pkg-config --libs lcm`

lcmgen::
	cd lcmgen && make && cd $(CWD)

lcm_%.c lcm_%.h: types/%.lcm
	./lcmgen/lcmgen -c $<
	./lcmgen/lcmgen --lua $<

lcm_%: lcm_%.o lualcm_%.o 
	$(LD) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ `pkg-config --libs lcm`

clean:
	rm -f $(NAME).$(SHLIBEXT) *.o *.so lcm_*_t.* lualcm_*_t.*
	cd lcmgen && make clean && cd $(CWD)
