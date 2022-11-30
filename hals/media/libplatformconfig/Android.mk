LOCAL_PATH := $(call my-dir)
LOCAL_DIR_PATH:= $(call my-dir)
include $(CLEAR_VARS)

libplatformconfig-def := \
      -g0 -O3 -fpic \
      -Wno-deprecated-declarations -Werror \
        -Wno-error=unused-variable \
        -w -Wall -Wextra\
    -fexceptions \
    -Wno-missing-field-initializers \
    -D_ANDROID_

COMMON_CFLAGS := -O3

LOCAL_CFLAGS := $(COMMON_CFLAGS) $(libplatformconfig-def)

ifeq ($(TARGET_ENABLE_VIDC_INTSAN), true)
LOCAL_SANITIZE := integer_overflow
ifeq ($(TARGET_ENABLE_VIDC_INTSAN_DIAG), true)
$(warning INTSAN_DIAG_ENABLED)
LOCAL_SANITIZE_DIAG := integer_overflow
endif
endif

LOCAL_SHARED_LIBRARIES += \
            libexpat \
            liblog \
            libcutils \
            libutils

LOCAL_STATIC_LIBRARIES := libOmxVidcCommon

LOCAL_C_INCLUDES += \
            external/expat/lib \
            $(LOCAL_PATH)/../mm-core/inc \
            $(LOCAL_PATH)/../mm-video-v4l2/vidc/common/inc/ \

LOCAL_SRC_FILES := PlatformConfig.cpp
LOCAL_SRC_FILES += ConfigParser.cpp

####################
ENABLE_CONFIGSTORE = true
ifeq ($(ENABLE_CONFIGSTORE),true)
LOCAL_SRC_FILES += ConfigStore.cpp
LOCAL_CFLAGS += -DENABLE_CONFIGSTORE
LOCAL_SHARED_LIBRARIES += libhidlbase
LOCAL_SHARED_LIBRARIES += vendor.qti.hardware.capabilityconfigstore@1.0
endif
####################

LOCAL_MODULE := libplatformconfig
LOCAL_VENDOR_MODULE := true

include $(BUILD_SHARED_LIBRARY)
