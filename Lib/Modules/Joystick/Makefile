include ../../Makefile.inc

all: joystick

CWD= $(shell pwd)
CHAIN= $(CTCDIR)

CXXFLAGS=-O2 -fpic

%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<
%.o: %.c
	$(CC) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

luaxbox360.o: luaxbox360.cc
	$(CC) $(CXXFLAGS) $(INCLUDES) -I/usr/local/include/libusb-1.0 -o $@ -c $<

joystick: luajoystick.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIBRT)

xbox360: luaxbox360.o
	$(CXX) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIBRT) -lusb-1.0

cross:
ifeq ($(CHAIN),)
	@echo Cross compilitation tool not specified. \
				Please download the ctc tool from \
				the aldebaran website and follow the \
				instructions to configure it for your system.
else
	rm -rf build
	mkdir build
	cd build && cmake -DCMAKE_TOOLCHAIN_FILE="$(CHAIN)/toolchain-atom.cmake" .. \
		&& make && cd $(CWD)
endif

clean:
	rm -rf build 
	rm -f *.o *.$(SHLIBEXT) *.$(MEXEXT)
