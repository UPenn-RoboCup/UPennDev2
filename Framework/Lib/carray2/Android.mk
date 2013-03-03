LOCAL_PATH := $(call my-dir)

# carray module to manipulate C arrays in lua
include $(CLEAR_VARS)
LOCAL_MODULE := carray
LOCAL_SRC_FILES := luacarray.cpp
LOCAL_SHARED_LIBRARIES := lua-activity
include $(BUILD_SHARED_LIBRARY)
