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
opengl_depth_packet_processor.o \
opencl_depth_packet_processor.o
#generate_resources.o \
#test_opengl_depth_packet_processor.o
include ../../Makefile.inc
INCLUDES+=\
-I/Users/smcgill3/src/libfreenect2/depends/libusb/include/libusb-1.0 \
-I/usr/local/Cellar/jpeg-turbo/1.3.1/include \
-I. \
-DLIBFREENECT2_THREADING_TINYTHREAD \
-DRESOURCES_INC \
-DGLEW_MX -DGLEW_STATIC -DDEBUG
LDFLAGS+=\
-lglfw3 -lGLEW -lGLEWmx \
-lturbojpeg -lusb-1.0
#-DDEBUG
#-lopencv_core
#-lfreenect2
#-I/usr/local/include/libusb-1.0/
#-I/usr/include/libusb-1.0/
#-I/Users/smcgill3/src/libfreenect2-cl/depends/libusb/include/libusb-1.0
#EXTRA_LINK+=-L/Users/smcgill3/src/libfreenect2-cl/depends/libusb/lib
EXTRA_LINK+=-L/Users/smcgill3/src/libfreenect2/depends/libusb/lib \
						-L/usr/local/Cellar/jpeg-turbo/1.3.1/lib/
