# Linux has a joystick API
ifeq ($(shell uname),Darwin)
all:
	@echo "No Mac Joystick API"
else
LIBNAME=joystick
include ../../Makefile.inc
LIBOPTS+=-lusb-1.0
INCLUDES+=`pkg-config libusb-1.0 --cflags`
endif
