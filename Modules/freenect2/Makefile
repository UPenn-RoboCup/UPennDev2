LIBNAME=freenect2

EXTRA_OBJ=\
command_transaction.o \
depth_packet_processor.o \
depth_packet_stream_parser.o \
double_buffer.o \
event_loop.o \
frame_listener_impl.o \
registration.o \
resource.o \
rgb_packet_stream_parser.o \
tinythread.o \
transfer_pool.o \
usb_control.o \
libfreenect2.o \
rgb_packet_processor.o \
cpu_depth_packet_processor.o

#EXTRA_OBJ+=turbo_jpeg_rgb_packet_processor.o

ifdef USE_OPENCL
EXTRA_OBJ+=opencl_depth_packet_processor.o
endif

#opengl.o
#opengl_depth_packet_processor.o
#generate_resources.o \
#test_opengl_depth_packet_processor.o

include ../../Makefile.inc

CXXFLAGS+=-std=c++11

INCLUDES+=\
-I. \
-DLIBFREENECT2_THREADING_TINYTHREAD \
-DRESOURCES_INC \
-DREG_OPENCL_FILE=\"opencl_depth_packet_processor.cl\"

LDFLAGS+=-lusb-1.0

#INCLUDES+=-DUSE_JPEG
#LDFLAGS+=-lturbojpeg

ifdef USE_OPENCL
INCLUDES+=-DGLEW_MX -DGLEW_STATIC -DUSE_OPENCL
LDFLAGS+=-lglfw3 -lGLEW -lGLEWmx
endif

ifeq ($(shell uname),Darwin)

INCLUDES+=`pkg-config libusb-1.0 --cflags`

#INCLUDES+=-I/usr/local/Cellar/jpeg-turbo/1.3.1/include -I/usr/local/opt/jpeg-turbo/include

ifdef USE_OPENCL
EXTRA_LINK+=-framework OpenCL
endif
#EXTRA_LINK+=-L/usr/local/Cellar/jpeg-turbo/1.3.1/lib/ -L/usr/local/opt/jpeg-turbo/lib \

else

# Use: https://github.com/wiedemeyer/libfreenect2
#LIBFREENECT2_DIR=/home/thor/Desktop/libfreenect2-cl
#INCLUDES+=-I$(LIBFREENECT2_DIR)/depends/libusb/include/libusb-1.0
#EXTRA_LINK+=-L$(LIBFREENECT2_DIR)/depends/libusb/lib
INCLUDES+=`pkg-config libusb-1.0 --cflags`
EXTRA_LINK+=`pkg-config libusb-1.0 --libs`
ifdef USE_OPENCL
EXTRA_LINK+=-lOpenCL
endif

endif
