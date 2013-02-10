LOCAL_PATH := $(call my-dir)

# carray module to manipulate C arrays in lua
include $(CLEAR_VARS)
LOCAL_MODULE := torch
LOCAL_SRC_FILES := \
	src/TH/THGeneral.c \
	src/TH/THStorage.c \
	src/TH/THTensor.c \
	src/TH/THBlas.c \
	src/TH/THLapack.c \
	src/TH/THLogAdd.c \
	src/TH/THRandom.c \
	src/TH/THFile.c \
	src/TH/THDiskFile.c \
	src/TH/THMemoryFile.c \
	src/luaT/luaT.c \
	src/torch/DiskFile.c \
	src/torch/File.c \
	src/torch/MemoryFile.c \
	src/torch/PipeFile.c \
	src/torch/Storage.c \
	src/torch/Tensor.c \
	src/torch/Timer.c \
	src/torch/utils.c \
	src/torch/init.c \
	src/torch/TensorOperator.c \
	src/torch/TensorMath.c \
	src/torch/random.c
LOCAL_C_INCLUDES := $(LOCAL_PATH)/include \
	$(LOCAL_PATH)/include/TH \
	$(LOCAL_PATH)/src/torch
LOCAL_LDLIBS := -llog
LOCAL_SHARED_LIBRARIES := lua-activity
include $(BUILD_SHARED_LIBRARY)
