ifneq ($(TARGET_IS_HEADLESS), true)
LOCAL_PATH := $(call my-dir)
include $(LOCAL_PATH)/../common.mk
include $(CLEAR_VARS)

LOCAL_MODULE                  := libdrmutils
LOCAL_SANITIZE                := integer_overflow
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := external/libdrm \
                                 $(kernel_includes)
LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_SHARED_LIBRARIES        := libdrm libdl libdisplaydebug
LOCAL_CFLAGS                  := -DLOG_TAG=\"DRMUTILS\" -Wall -Werror -fno-operator-names
LOCAL_CFLAGS                  += $(common_flags)
LOCAL_CLANG                   := true
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
LOCAL_SRC_FILES               := drm_master.cpp drm_res_mgr.cpp drm_lib_loader.cpp

include $(BUILD_SHARED_LIBRARY)
endif
