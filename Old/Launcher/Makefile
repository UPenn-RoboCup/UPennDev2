ifeq ($(shell uname),Darwin)
all:
	@echo "No Mac USB API"
clean:
	@echo "No cleaning needed"
else
LIBNAME=launcher
include ../../Makefile.inc
LIBOPTS+=`pkg-config libusb-1.0 --libs`
INCLUDES+=`pkg-config libusb-1.0 --cflags`
endif
