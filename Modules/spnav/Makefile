LIBNAME=spnav
include ../../Makefile.inc
ifeq ($(OSTYPE),darwin)
EXTRA_OBJ += hid.o spnav.o
LIBOPTS += -framework IOKit -framework CoreFoundation
else
EXTRA_OBJ += hid-libusb.o spnav.o
LIBOPTS += `pkg-config libusb-1.0 --libs`
INCLUDES += `pkg-config libusb-1.0 --cflags`
endif
