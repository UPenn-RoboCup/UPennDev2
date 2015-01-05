LIBNAME=freenect2

EXTRA_OBJ=\
command_transaction.o \
depth_packet_processor.o \
depth_packet_stream_parser.o \
double_buffer.o \
event_loop.o \
frame_listener_impl.o \
resource.o \
rgb_packet_processor.o \
rgb_packet_stream_parser.o \
tinythread.o \
transfer_pool.o \
usb_control.o \
libfreenect2.o \
turbo_jpeg_rgb_packet_processor.o \
cpu_depth_packet_processor.o
#opencl_depth_packet_processor.o

#opengl.o
#opengl_depth_packet_processor.o
#generate_resources.o \
#test_opengl_depth_packet_processor.o

include ../../Makefile.inc
INCLUDES+=\
-I. \
-DLIBFREENECT2_THREADING_TINYTHREAD \
-DRESOURCES_INC \
-DREG_OPENCL_FILE=\"opencl_depth_packet_processor.cl\" \
-g \
-DDEBUG
#-DGLEW_MX
#-DGLEW_STATIC

LDFLAGS+=\
-lturbojpeg -lusb-1.0
#-lglfw3 -lGLEW -lGLEWmx \

ifeq ($(shell uname),Darwin)

INCLUDES+=\
`pkg-config libusb-1.0 --cflags` \
-I/usr/local/Cellar/jpeg-turbo/1.3.1/include \
-I/usr/local/opt/jpeg-turbo/include

EXTRA_LINK+=\
-L/usr/local/Cellar/jpeg-turbo/1.3.1/lib/ \
-L/usr/local/opt/jpeg-turbo/lib \
-framework OpenCL

else

# Use: https://github.com/wiedemeyer/libfreenect2
LIBFREENECT2_DIR=/home/thor/Desktop/libfreenect2-cl

INCLUDES+=-I$(LIBFREENECT2_DIR)/depends/libusb/include/libusb-1.0
EXTRA_LINK+=-L$(LIBFREENECT2_DIR)/depends/libusb/lib
#						-lOpenCL

endif
