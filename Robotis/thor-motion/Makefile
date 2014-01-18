##################################################
# PROJECT: Thor - ROBOTIS Inc.
##################################################

#---------------------------------------------------------------------
# C++ COMPILER, COMPILER FLAGS, AND TARGET PROGRAM NAME
#---------------------------------------------------------------------
DIR_THOR    = .
DIR_OBJS    = build

INSTALL_ROOT = /usr/local
 
MAJ_VERSION = 1
MIN_VERSION = 0
REV_VERSION = 0

TARGET      = libthor.so
TARGET1     = $(TARGET).$(MAJ_VERSION)
TARGET2     = $(TARGET).$(MAJ_VERSION).$(MIN_VERSION)
TARGET3     = $(TARGET).$(MAJ_VERSION).$(MIN_VERSION).$(REV_VERSION)

CHK_DIR_EXISTS = test -d
PRINT       = echo
STRIP       = strip
AR          = ar
ARFLAGS     = cr
LD          = g++
LDFLAGS     = -shared -fPIC #-Wl,-soname,thor
LD_CONFIG   = ldconfig
CP          = cp
CP_ALL      = cp -r
RM          = rm
RM_ALL      = rm -rf
SYMLINK     = ln -s
MKDIR       = mkdir
CC          = gcc
CX          = g++
CCFLAGS     = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c $(INCLUDES) -fPIC -g
CXFLAGS     = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall -c $(INCLUDES) -fPIC -g
# CXFLAGS    += -DDEBUG
INCLUDES   += -I$(DIR_THOR)/include -I$(DIR_THOR)/include/framework -I$(DIR_THOR)/include/linux

#---------------------------------------------------------------------
# Required external libraries
#---------------------------------------------------------------------
LIBRARIES  += -lpthread
LIBRARIES  += -ljpeg
LIBRARIES  += -lpthread 
LIBRARIES  += -ldl
LIBRARIES  += -lncurses 
LIBRARIES  += -lrt

#---------------------------------------------------------------------
# Framework Files
#---------------------------------------------------------------------
SOURCES  = src/framework/dynamixel.cpp \
           src/framework/bulkread.cpp\
           src/framework/math/point.cpp\
           src/framework/math/vector.cpp\
           src/framework/math/matrix.cpp\
           src/framework/math/plane.cpp\
           src/framework/math/linear_algebra.cpp\
           src/framework/minIni/minIni.c\
           src/framework/motion/kinematics.cpp\
           src/framework/motion/dynamics.cpp\
           src/framework/motion/motionstatus.cpp\
           src/framework/motion/motionmanager.cpp\
           src/framework/motion/modules/action.cpp\
           src/framework/motion/modules/walking.cpp\
           src/framework/motion/modules/walking130610.cpp\
           src/framework/motion/modules/dynamic_walking.cpp\
           src/framework/motion/modules/dynamic_walking131012.cpp\
           src/framework/motion/modules/preview_control_walking.cpp\
           src/framework/motion/modules/recursive_walking.cpp\
           src/framework/motion/modules/head.cpp\
           src/framework/motion/modules/testmodule.cpp\
           src/framework/sensor/vision/camera.cpp\
           src/framework/sensor/vision/image.cpp\
           src/framework/sensor/vision/imgprocess.cpp\
           src/framework/sensor/vision/colorfinder.cpp\
           src/framework/sensor/vision/balltracker.cpp\
           src/framework/sensor/vision/ballfollower.cpp\
           src/framework/sensor/ins/mip_sdk_user_functions.c\
           src/framework/sensor/ins/mip_sdk_system.c\
           src/framework/sensor/ins/mip_sdk_base.c\
           src/framework/sensor/ins/mip_sdk_gps.c\
           src/framework/sensor/ins/mip.c\
           src/framework/sensor/ins/mip_sdk_3dm.c\
           src/framework/sensor/ins/ring_buffer.c\
           src/framework/sensor/ins/mip_sdk_inteface.c\
           src/framework/sensor/ins/mip_sdk_nav.c\
           src/framework/sensor/ins/mip_sdk_ahrs.c\
           src/framework/sensor/ins/byteswap_utilities.c \
           src/framework/sensor/ins/ins.cpp\
           src/framework/sensor/lidar/urg_connection.c \
           src/framework/sensor/lidar/urg_debug.c \
           src/framework/sensor/lidar/urg_ring_buffer.c \
           src/framework/sensor/lidar/urg_sensor.c \
           src/framework/sensor/lidar/urg_tcpclient.c \
           src/framework/sensor/lidar/urg_utils.c \
           src/framework/sensor/lidar/urg_serial.c \
           src/framework/sensor/lidar/urg_serial_utils.c\
           src/framework/sensor/lidar/open_urg_sensor.c \
           src/framework/sensor/lidar/lidar.cpp \
           src/framework/sensor/forcetorquesensor/forcetorquesensor.cpp \
           src/framework/sensor/fsr/fsr.cpp \



#---------------------------------------------------------------------
# Linux Files
#---------------------------------------------------------------------
SOURCES += src/linux/serialport.cpp \


OBJECTS=$(addsuffix .o,$(addprefix $(DIR_OBJS)/,$(basename $(notdir $(SOURCES)))))


#---------------------------------------------------------------------
# COMPILING RULES
#---------------------------------------------------------------------
$(TARGET): makedirs $(OBJECTS)
	#$(AR) $(ARFLAGS) $(DIR_THOR)/lib/$(TARGET) $(OBJECTS)
	$(LD) $(LDFLAGS) -o $(DIR_THOR)/lib/$(TARGET) $(OBJECTS) $(LIBS)

makedirs:
	mkdir -p build
	mkdir -p lib

clean:
	rm -f $(OBJECTS) $(DIR_THOR)/lib/$(TARGET)

install: $(TARGET)
    # copy the binaries into the lib directory
	@$(CHK_DIR_EXISTS) $(INSTALL_ROOT)/lib || $(MKDIR) $(INSTALL_ROOT)/lib
	-$(CP) "$(DIR_THOR)/lib/$(TARGET)" "$(INSTALL_ROOT)/lib/$(TARGET)"
	-$(SYMLINK) "$(INSTALL_ROOT)/lib/$(TARGET)" "$(INSTALL_ROOT)/lib/$(TARGET1)"
	-$(SYMLINK) "$(INSTALL_ROOT)/lib/$(TARGET)" "$(INSTALL_ROOT)/lib/$(TARGET2)"
	-$(SYMLINK) "$(INSTALL_ROOT)/lib/$(TARGET)" "$(INSTALL_ROOT)/lib/$(TARGET3)"

    # copy the headers into the include directory
	@$(CHK_DIR_EXISTS) $(INSTALL_ROOT)/include || $(MKDIR) $(INSTALL_ROOT)/include
	@$(CHK_DIR_EXISTS) $(INSTALL_ROOT)/include/thor || $(MKDIR) $(INSTALL_ROOT)/include/thor
	$(CP_ALL) $(DIR_THOR)/include/* $(INSTALL_ROOT)/include/

	$(LD_CONFIG)

uninstall:
	$(RM) $(INSTALL_ROOT)/lib/$(TARGET)
	$(RM) $(INSTALL_ROOT)/lib/$(TARGET1)
	$(RM) $(INSTALL_ROOT)/lib/$(TARGET2)
	$(RM) $(INSTALL_ROOT)/lib/$(TARGET3)

	$(RM_ALL) $(INSTALL_ROOT)/include/thor

reinstall: uninstall install


#---------------------------------------------------------------------
# Make rules for all .c and .cpp files in each directory
#---------------------------------------------------------------------

$(DIR_OBJS)/%.o: src/framework/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@
	
$(DIR_OBJS)/%.o: src/framework/math/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/minIni/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/motion/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/motion/modules/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/sensor/vision/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/sensor/ins/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/sensor/ins/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/sensor/lidar/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/sensor/lidar/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/sensor/forcetorquesensor/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/framework/sensor/fsr/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/linux/%.c
	$(CC) $(CCFLAGS) -c $? -o $@

$(DIR_OBJS)/%.o: src/linux/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@


#---------------------------------------------------------------------
# END OF MAKEFILE
#---------------------------------------------------------------------
