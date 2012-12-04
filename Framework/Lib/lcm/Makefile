# Makefile used to build Lcm files
include ../../../Makefile.inc
CWD = $(shell pwd)
NAME = lcm 

TYPE_TARGETS=lcm_rpc_request_t lcm_rpc_response_t 

all:: lcmgen lcm $(TYPE_TARGETS)
#lcm_rpc_t

%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<
	
lcm: lualcm.o
	@echo
	@echo "Linking Lua LCM"
	$(LD) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ `pkg-config --libs lcm`

lcmgen::
	@echo
	@echo "Building the LCM generator..."
	cd lcmgen && make && cd $(CWD)

#lcm_%.c lcm_%.h: types/%.lcm
lcm_%: types/%.lcm
	@echo
	@echo "Building the LCM types..."
	./lcmgen/lcmgen -c $<
	./lcmgen/lcmgen --lua $<

lcm_%: lcm_%.o lualcm_%.o 
	@echo
	@echo "Linking the LCM module..."
	$(LD) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ `pkg-config --libs lcm`

clean:
	@echo
	@echo "Cleaning LCM support"
	rm -f $(NAME).$(SHLIBEXT) *.o *.so lcm_*_t.* lualcm_*_t.*
	cd lcmgen && make clean && cd $(CWD)
