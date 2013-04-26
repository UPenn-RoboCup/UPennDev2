# Linux has a joystick API
#LIBNAME=joystick
# Mac does not, so we use libusb
LIBNAME=xbox360
include ../../Makefile.inc
LIBOPTS+=-lusb-1.0
INCLUDES+=`pkg-config libusb-1.0 --cflags`