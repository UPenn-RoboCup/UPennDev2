LIBNAME=spacemouse

#ifeq ($(OSTYPE),darwin)
EXTRA_OBJ=hid-mac.o spnav.o
CFLAGS += -framework IOKit -framework CoreFoundation
#else
#EXTRA_OBJ=hid-libusb.o spnav.o
#endif

include ../../Makefile.inc

INCLUDES += `pkg-config libusb-1.0 --cflags`

#ifneq ($(OSTYPE),darwin)
#LDFLAGS+=-lpthread `pkg-config libusb-1.0 --libs`
#CFLAGS += `pkg-config libusb-1.0 --libs`
#LD=ld
#endif
