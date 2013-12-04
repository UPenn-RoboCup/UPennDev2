LIBNAME=spacemouse
ifeq ($(OSTYPE),darwin)
EXTRA_OBJ=hid.o spnav.o
CFLAGS += -framework IOKit -framework CoreFoundation
else
EXTRA_OBJ=hid-libusb.o spnav.o
endif
include ../../Makefile.inc

#ifneq ($(OSTYPE),darwin)
INCLUDES += `pkg-config libusb-1.0 --cflags`
CFLAGS += `pkg-config libusb-1.0 --libs`
LDFLAGS+=-lpthread `pkg-config libusb-1.0 --libs`
#endif

LD=ld
