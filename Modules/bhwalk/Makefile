## Makefile Method for making torch into a module for any lua
## Daniel D. Lee, Feb 2013.
## <ddlee@seas.upenn.edu>
## Stephen McGill, Apr 2014
## <smcgill3@seas.upenn.edu>
# http://clang.llvm.org/docs/ClangTools.html (unordered map)

ifndef OSTYPE 
	OSTYPE = $(shell uname -s|awk '{print tolower($$0)}')
endif

CXX=g++-4.8

BH_SOURCES=\
	MotionSelector.cpp \
	WalkingEngine.cpp \
	WalkingEngineKick.cpp \
	Modules/Sensing/FallDownStateDetector.cpp \
	Modules/Sensing/GroundContactDetector.cpp \
	Modules/Sensing/InertiaSensorCalibrator.cpp \
	Modules/Sensing/InertiaSensorFilter.cpp \
	Modules/Sensing/InertiaSensorInspector.cpp \
	Modules/Sensing/JointFilter.cpp \
	Modules/Sensing/RobotModelProvider.cpp \
	Modules/Sensing/SensorFilter.cpp \
	Modules/Sensing/TorsoMatrixProvider.cpp \
	Tools/Streams/InOut.cpp \
	Tools/Streams/InStreams.cpp \
	Tools/Streams/OutStreams.cpp \
	Tools/Streams/StreamHandler.cpp \
	Tools/Streams/Streamable.cpp \
	Tools/Global.cpp \
	Representations/Sensing/RobotModel.cpp \
	Representations/MotionControl/BikeRequest.cpp \
	Representations/MotionControl/MotionRequest.cpp \
	Representations/MotionControl/MotionSelection.cpp \
	Platform/SystemCall.cpp \
	Platform/BHAssert.cpp \
	Platform/File.cpp

BH_OBJECTS=$(BH_SOURCES:.cpp=.o)

CXXFLAGS= \
	-c \
	-I/usr/local/include -I. \
	-I/usr/include/lua5.1 \
	-O3 -fpic -pedantic \
	-fno-stack-protector \
	-fomit-frame-pointer \
	-DRELEASE \
	-Wall -Wno-unused-function -Wno-unknown-pragmas
#	-march=native -mtune=native
#-std=c++11

ifeq ($(OSTYPE),darwin)
LD=ld -macosx_version_min 10.8
LDFLAGS= \
	-undefined dynamic_lookup \
	-lm \
	-L/usr/local/lib
CXX=clang++
else
LD=g++
LDFLAGS=-shared -fpic -lm
endif

all: bhwalk

.cpp.o:
	$(CXX) $(CXXFLAGS) $< -o $@
	
lua_bhwalk.cpp.o:
	$(CXX) $(CXXFLAGS) lua_bhwalk.cpp -o lua_bhwalk.cpp.o
	
clean:
	rm -f $(BH_OBJECTS)
	rm -f *.so

ifeq ($(OSTYPE),darwin)
# OSX linking and installation
# Mach-O means BUNDLE for lua loading, DYLIB for linking (2 diff files...)
# GCC is -dynamiclib, clang is -dylib for the DYLIB
# lua loads .so files, dylib files are linked

libbh: $(BH_OBJECTS)
	$(LD) -bundle $^ $(LDFLAGS) -o $@.so
#	$(LD) -dylib $^ $(LDFLAGS) -o $@.dylib

bhwalk: $(BH_OBJECTS) lua_bhwalk.cpp.o
	$(LD) -bundle $^ $(LDFLAGS) -o $@.so
#	$(LD) -dylib $^ $(LDFLAGS) -o $@.dylib

else
# Linux linking and installation

libbh: $(TORCH_OBJECTS)
	$(LD) $^ $(LDFLAGS) -o $@.so

endif