ROOT_DIR := $(call my-dir)
include $(CLEAR_VARS)
include $(LIBION_HEADER_PATH_WRAPPER)
LOCAL_PATH:= $(ROOT_DIR)

# ---------------------------------------------------------------------------------
# 				Common definitons
# ---------------------------------------------------------------------------------

libmm-vidc-def := -g -O3 -Dlrintf=_ffix_r
libmm-vidc-def += -D__align=__alignx
libmm-vidc-def += -D__alignx\(x\)=__attribute__\(\(__aligned__\(x\)\)\)
libmm-vidc-def += -DT_ARM
libmm-vidc-def += -Dinline=__inline
libmm-vidc-def += -D_ANDROID_
libmm-vidc-def += -Werror
libmm-vidc-def += -D_ANDROID_ICS_

ifeq ($(TARGET_USES_ION),true)
libmm-vidc-def += -DUSE_ION
endif

# ---------------------------------------------------------------------------------
# 			Make the Shared library (libOmxVidcCommon)
# ---------------------------------------------------------------------------------

libmm-vidc-inc      := $(LOCAL_PATH)/inc
libmm-vidc-inc      += $(TOP)/device/xiaomi/spes/hals/media/mm-core/inc
libmm-vidc-inc      += $(TOP)/device/xiaomi/spes/hals/media/mm-video-v4l2/vidc/vdec/inc
libmm-vidc-inc      += $(TOP)/device/xiaomi/spes/hals/media/libc2dcolorconvert
libmm-vidc-inc      += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
libmm-vidc-inc      += $(TOP)/hardware/libhardware/include
libmm-vidc-inc      += $(TOP)/frameworks/native/headers/media_plugin
libmm-vidc-inc      += $(TARGET_OUT_HEADERS)/qcom/display
libmm-vidc-inc      += $(LIBION_HEADER_PATHS)



LOCAL_MODULE                    := libOmxVidcCommon
LOCAL_MODULE_TAGS               := optional
LOCAL_VENDOR_MODULE             := true
LOCAL_CFLAGS                    := $(libmm-vidc-def)
LOCAL_C_INCLUDES                := $(libmm-vidc-inc)

# The type of overflow check must be specified for static libraries
ifeq ($(TARGET_ENABLE_VIDC_INTSAN), true)
LOCAL_SANITIZE := signed-integer-overflow unsigned-integer-overflow
ifeq ($(TARGET_ENABLE_VIDC_INTSAN_DIAG), true)
$(warning INTSAN_DIAG_ENABLED)
LOCAL_SANITIZE_DIAG := signed-integer-overflow unsigned-integer-overflow
endif
endif

LOCAL_PRELINK_MODULE      := false
LOCAL_SHARED_LIBRARIES    := liblog libcutils libdl

LOCAL_HEADER_LIBRARIES := \
        libutils_headers display_intf_headers

LOCAL_SRC_FILES   += src/vidc_common.cpp
LOCAL_SRC_FILES   += src/vidc_debug.cpp
LOCAL_SRC_FILES   += src/vidc_vendor_extensions.cpp

LOCAL_ADDITIONAL_DEPENDENCIES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

include $(BUILD_STATIC_LIBRARY)

# ---------------------------------------------------------------------------------
# 					END
# ---------------------------------------------------------------------------------
