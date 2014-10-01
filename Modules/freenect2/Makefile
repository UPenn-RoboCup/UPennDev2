LIBNAME=freenect2
EXTRA_OBJ=\
command_transaction.o \
cpu_depth_packet_processor.o \
depth_packet_processor.o \
depth_packet_stream_parser.o \
double_buffer.o \
event_loop.o \
frame_listener_impl.o \
libfreenect2.o \
resource.o \
rgb_packet_processor.o \
rgb_packet_stream_parser.o \
tinythread.o \
transfer_pool.o \
turbo_jpeg_rgb_packet_processor.o \
usb_control.o \
opengl.o \
opengl_depth_packet_processor.o
#generate_resources.o \
#test_opengl_depth_packet_processor.o
include ../../Makefile.inc
INCLUDES+=-DDEBUG\
-I/usr/local/include/libusb-1.0/ \
-I. \
-DLIBFREENECT2_THREADING_TINYTHREAD \
-DRESOURCES_INC \
-DGLEW_MX -DGLEW_STATIC
LDFLAGS+=\
-lglfw3 -lGLEW -lGLEWmx \
-lturbojpeg -lusb-1.0
#-lopencv_core
#-lfreenect2
