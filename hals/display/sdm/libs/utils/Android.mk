LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
include $(LOCAL_PATH)/../../../common.mk

LOCAL_MODULE                  := libsdmutils
LOCAL_SANITIZE                := integer_overflow
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(common_includes)
LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_CFLAGS                  := -DLOG_TAG=\"SDM\" $(common_flags)
LOCAL_SRC_FILES               := debug.cpp \
                                 rect.cpp \
                                 sys.cpp \
                                 fence.cpp \
                                 formats.cpp \
                                 utils.cpp

LOCAL_SHARED_LIBRARIES        := libdisplaydebug
include $(BUILD_SHARED_LIBRARY)
