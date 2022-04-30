LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE                  := libdisplaydebug
LOCAL_SANITIZE                := integer_overflow
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_TAGS             := optional
LOCAL_SHARED_LIBRARIES        := libdl
LOCAL_CFLAGS                  := -DLOG_TAG=\"SDM\" -Wall -Werror -fno-operator-names
LOCAL_CLANG                   := true
LOCAL_SRC_FILES               := debug_handler.cpp

include $(BUILD_SHARED_LIBRARY)
