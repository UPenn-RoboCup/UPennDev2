LIBNAME=freenect2
USE_OPENCL = 1

EXTRA_OBJ=\
command_transaction.o \
depth_packet_processor.o \
depth_packet_stream_parser.o \
double_buffer.o \
event_loop.o \
frame_listener_impl.o \
packet_pipeline.o \
registration.o \
resource.o \
rgb_packet_stream_parser.o \
transfer_pool.o \
usb_control.o \
libfreenect2.o \
rgb_packet_processor.o \
cpu_depth_packet_processor.o

#tinythread.o

#EXTRA_OBJ+=turbo_jpeg_rgb_packet_processor.o

ifdef USE_OPENCL
EXTRA_OBJ+=opencl_depth_packet_processor.o
endif

#flextGL.o
#opengl_depth_packet_processor.o
#generate_resources.o \
#test_opengl_depth_packet_processor.o

include ../../Makefile.inc

CXXFLAGS+=-std=c++11

INCLUDES+=\
-I. \
-DRESOURCES_INC \
-DREG_OPENCL_FILE=\"opencl_depth_packet_processor.cl\"

LDFLAGS+=-lusb-1.0

#INCLUDES+=-DUSE_JPEG
#LDFLAGS+=-lturbojpeg

ifdef USE_OPENCL
INCLUDES+=-DLIBFREENECT2_WITH_OPENCL_SUPPORT
endif

ifeq ($(shell uname), Darwin)

INCLUDES+=`pkg-config libusb-1.0 --cflags`

ifdef USE_OPENCL
EXTRA_LINK+=-framework OpenCL
endif
#EXTRA_LINK+=-L/usr/local/Cellar/jpeg-turbo/1.3.1/lib/ -L/usr/local/opt/jpeg-turbo/lib \

else

INCLUDES+=`pkg-config libusb-1.0 --cflags`
EXTRA_LINK+=`pkg-config libusb-1.0 --libs`
ifdef USE_OPENCL
EXTRA_LINK+=-lOpenCL
endif

endif
