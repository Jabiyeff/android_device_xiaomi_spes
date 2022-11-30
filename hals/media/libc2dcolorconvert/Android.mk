LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

ifeq ($(TARGET_ENABLE_VIDC_INTSAN), true)
LOCAL_SANITIZE := integer_overflow
ifeq ($(TARGET_ENABLE_VIDC_INTSAN_DIAG), true)
$(warning INTSAN_DIAG_ENABLED)
LOCAL_SANITIZE_DIAG := integer_overflow
endif
endif

LOCAL_SRC_FILES := \
        C2DColorConverter.cpp

LOCAL_C_INCLUDES := \
    $(TARGET_OUT_HEADERS)/adreno

# KONA_TODO_UPDATE: Check if we need to change the path for dispaly header files
LOCAL_C_INCLUDES += $(TARGET_OUT_HEADERS)/adreno
LOCAL_C_INCLUDES += $(TOP)/vendor/qcom/opensource/commonsys-intf/display/gralloc

LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include

LOCAL_HEADER_LIBRARIES := \
        libutils_headers \
        libhardware_headers \
        display_headers

LOCAL_SHARED_LIBRARIES := liblog libdl

LOCAL_MODULE_TAGS := optional

LOCAL_MODULE := libc2dcolorconvert

LOCAL_VENDOR_MODULE := true

LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

include $(BUILD_SHARED_LIBRARY)
