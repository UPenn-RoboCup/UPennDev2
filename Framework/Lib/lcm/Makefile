# Makefile used to build Lcm files
include ../../../Makefile.inc
CWD = $(shell pwd)
NAME = lcm

TYPE_TARGETS=thor_rpc_request_t thor_rpc_response_t

all:: lcmgen lcm $(TYPE_TARGETS)

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

thor_%.c: types/%.lcm
	@echo
	@echo "Building the LCM types for c..."
	./lcmgen/lcmgen -c $<

luathor_%.c: types/%.lcm
	@echo
	@echo "Building the LCM types for lua..."
	./lcmgen/lcmgen --lua $<

thor_%.o: thorlcm_%.c
	@echo
	@echo "Building the LCM modules for types..."
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

lualcm_%.o: lualcm_%.c
	@echo
	@echo "Building the LCM modules for types..."
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -c $<

thor_%: thor_%.o luathor_%.o 
	@echo
	@echo "Linking the LCM module..."
	$(LD) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ `pkg-config --libs lcm`

matlab:
	lcm-gen -j types/*.lcm
	javac -cp `pkg-config --variable=classpath lcm-java` thor/*.java
	jar cf thor_matlab.jar thor/*.class

clean:
	@echo
	@echo "Cleaning LCM support"
	rm -rf thor
	rm -f $(NAME).$(SHLIBEXT) *.o *.so thor_*_t.* luathor_*_t.* *.jar
	cd lcmgen && make clean && cd $(CWD)
