# Gralloc module
LOCAL_PATH := $(call my-dir)

qmaa_flags := ""
ifeq ($(TARGET_USES_QMAA),true)
ifeq ($(TARGET_USES_QMAA_OVERRIDE_DISPLAY),false)
qmaa_flags := -DQMAA -Wno-unused-parameter -Wno-unused-variable
qmaa_flags += -DTARGET_ION_ABI_VERSION=2
endif
endif

include $(LOCAL_PATH)/../common.mk
include $(LIBION_HEADER_PATH_WRAPPER)
include $(CLEAR_VARS)

LOCAL_MODULE                  := gralloc.$(TARGET_BOARD_PLATFORM)
LOCAL_SANITIZE                := integer_overflow
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_RELATIVE_PATH    := hw
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(common_includes) $(kernel_includes)

LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_SHARED_LIBRARIES        := $(common_libs) libqdMetaData libsync libgrallocutils \
                                 libgralloccore \
                                 android.hardware.graphics.mapper@2.0 \
                                 android.hardware.graphics.mapper@2.1 \
                                 android.hardware.graphics.mapper@3.0 \
                                 android.hardware.graphics.mapper@4.0
LOCAL_CFLAGS                  := $(common_flags) $(qmaa_flags) -DLOG_TAG=\"qdgralloc\" -Wall -Werror \
                                 -D__QTI_DISPLAY_GRALLOC__
LOCAL_CLANG                   := true
LOCAL_ADDITIONAL_DEPENDENCIES := $(common_deps)
LOCAL_SRC_FILES               := gr_device_impl.cpp
include $(BUILD_SHARED_LIBRARY)

#libgrallocutils
include $(CLEAR_VARS)
LOCAL_MODULE                  := libgrallocutils
LOCAL_VENDOR_MODULE           := true
LOCAL_SANITIZE                := integer_overflow
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(common_includes) $(kernel_includes)
LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_SHARED_LIBRARIES        := $(common_libs) libqdMetaData libdl  \
                                  android.hardware.graphics.common@1.2 \
                                  android.hardware.graphics.mapper@2.0 \
                                  android.hardware.graphics.mapper@2.1 \
                                  android.hardware.graphics.mapper@3.0 \
                                  android.hardware.graphics.mapper@4.0
LOCAL_CFLAGS                  := $(common_flags) $(qmaa_flags) -DLOG_TAG=\"qdgralloc\" -Wno-sign-conversion \
                                 -D__QTI_DISPLAY_GRALLOC__

ifeq ($(TARGET_USES_YCRCB_CAMERA_PREVIEW),true)
    LOCAL_CFLAGS              += -DUSE_YCRCB_CAMERA_PREVIEW
else ifeq ($(TARGET_USES_YCRCB_VENUS_CAMERA_PREVIEW),true)
    LOCAL_CFLAGS              += -DUSE_YCRCB_CAMERA_PREVIEW_VENUS
endif

ifeq ($(TARGET_NO_RAW10_CUSTOM_FORMAT),true)
    LOCAL_CFLAGS              += -DNO_RAW10_CUSTOM_FORMAT
endif

LOCAL_ADDITIONAL_DEPENDENCIES := $(common_deps)
LOCAL_SRC_FILES               := gr_utils.cpp gr_adreno_info.cpp gr_camera_info.cpp
include $(BUILD_SHARED_LIBRARY)

#libgralloccore
include $(CLEAR_VARS)
LOCAL_MODULE                  := libgralloccore
LOCAL_SANITIZE                := integer_overflow
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(common_includes) \
                                 $(LIBION_HEADER_PATHS) \
                                 $(kernel_includes)

LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_SHARED_LIBRARIES        := $(common_libs) libqdMetaData libdl libgrallocutils libion libgralloctypes \
                                  libgralloc.qti libhidlbase \
                                  android.hardware.graphics.mapper@2.1 \
                                  android.hardware.graphics.mapper@3.0 \
                                  android.hardware.graphics.mapper@4.0
LOCAL_CFLAGS                  := $(common_flags) $(qmaa_flags) -DLOG_TAG=\"qdgralloc\" -Wno-sign-conversion \
                                 -D__QTI_DISPLAY_GRALLOC__
ifneq ($(TARGET_USES_GRALLOC4),false)
LOCAL_CFLAGS                  += -DTARGET_USES_GRALLOC4
endif
LOCAL_ADDITIONAL_DEPENDENCIES := $(common_deps)
LOCAL_SRC_FILES               := gr_allocator.cpp gr_buf_mgr.cpp gr_ion_alloc.cpp
include $(BUILD_SHARED_LIBRARY)

#mapper
include $(CLEAR_VARS)
LOCAL_MODULE                  := android.hardware.graphics.mapper@3.0-impl-qti-display
LOCAL_SANITIZE                := integer_overflow
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_RELATIVE_PATH    := hw
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(common_includes) $(kernel_includes)
LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_SHARED_LIBRARIES        := $(common_libs) \
                                  libhidlbase \
                                  libqdMetaData \
                                  libgrallocutils \
                                  libgralloccore \
                                  libsync \
                                  vendor.qti.hardware.display.mapper@3.0 \
                                  vendor.qti.hardware.display.mapperextensions@1.0 \
                                  android.hardware.graphics.mapper@2.0 \
                                  android.hardware.graphics.mapper@2.1 \
                                  vendor.qti.hardware.display.mapperextensions@1.1 \
                                  android.hardware.graphics.mapper@3.0
LOCAL_CFLAGS                  := $(common_flags) $(qmaa_flags) -DLOG_TAG=\"qdgralloc\" -Wno-sign-conversion \
                                 -D__QTI_DISPLAY_GRALLOC__
LOCAL_ADDITIONAL_DEPENDENCIES := $(common_deps)
LOCAL_SRC_FILES               := QtiMapper.cpp QtiMapperExtensions.cpp
LOCAL_VINTF_FRAGMENTS         := android.hardware.graphics.mapper-impl-qti-display.xml
include $(BUILD_SHARED_LIBRARY)

ifneq ($(TARGET_USES_GRALLOC4),false)
include $(CLEAR_VARS)
LOCAL_MODULE                  := android.hardware.graphics.mapper@4.0-impl-qti-display
LOCAL_SANITIZE                := integer_overflow
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_RELATIVE_PATH    := hw
LOCAL_MODULE_TAGS             := optional
LOCAL_C_INCLUDES              := $(common_includes) $(kernel_includes)
LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_SHARED_LIBRARIES        := $(common_libs) \
                                  libhidlbase \
                                  libqdMetaData \
                                  libgrallocutils \
                                  libgralloccore \
                                  libsync \
                                  libgralloctypes \
                                  vendor.qti.hardware.display.mapper@4.0 \
                                  vendor.qti.hardware.display.mapperextensions@1.0 \
                                  android.hardware.graphics.mapper@2.0 \
                                  android.hardware.graphics.mapper@2.1 \
                                  vendor.qti.hardware.display.mapperextensions@1.1 \
                                  android.hardware.graphics.mapper@3.0 \
                                  android.hardware.graphics.mapper@4.0

ifeq ($(shell expr $(PLATFORM_SDK_VERSION) \<= 28), 1)
LOCAL_SHARED_LIBRARIES += libhidltransport
endif

LOCAL_CFLAGS                  := $(common_flags) $(qmaa_flags) -DLOG_TAG=\"qdgralloc\" -Wno-sign-conversion \
                                 -D__QTI_DISPLAY_GRALLOC__
LOCAL_ADDITIONAL_DEPENDENCIES := $(common_deps)
LOCAL_SRC_FILES               := QtiMapper4.cpp QtiMapperExtensions.cpp
LOCAL_VINTF_FRAGMENTS         := android.hardware.graphics.mapper-impl-qti-display.xml
include $(BUILD_SHARED_LIBRARY)
endif

#allocator
include $(CLEAR_VARS)
LOCAL_MODULE                  := vendor.qti.hardware.display.allocator-service
LOCAL_SANITIZE                := integer_overflow
LOCAL_VENDOR_MODULE           := true
LOCAL_MODULE_RELATIVE_PATH    := hw
LOCAL_MODULE_TAGS             := optional
LOCAL_HEADER_LIBRARIES        := display_headers
LOCAL_SHARED_LIBRARIES        := $(common_libs) \
                                 libhidlbase \
                                 libqdMetaData \
                                 libgrallocutils \
                                 libgralloccore \
                                 libgralloctypes \
                                 vendor.qti.hardware.display.allocator@4.0 \
                                 vendor.qti.hardware.display.allocator@3.0 \
                                 vendor.qti.hardware.display.mapper@4.0 \
                                 vendor.qti.hardware.display.mapper@3.0 \
                                 android.hardware.graphics.mapper@4.0 \
                                 android.hardware.graphics.mapper@3.0 \
                                 android.hardware.graphics.mapper@2.1 \
                                 android.hardware.graphics.allocator@4.0 \
                                 android.hardware.graphics.allocator@3.0 \
                                 vendor.qti.hardware.display.mapperextensions@1.0 \
                                 vendor.qti.hardware.display.mapperextensions@1.1
LOCAL_CFLAGS                  := -DLOG_TAG=\"qdgralloc\" $(common_flags) $(qmaa_flags)
ifneq ($(TARGET_USES_GRALLOC4),false)
LOCAL_CFLAGS                  += -DTARGET_USES_GRALLOC4
endif
LOCAL_ADDITIONAL_DEPENDENCIES := $(common_deps)
LOCAL_C_INCLUDES              := $(common_includes) $(kernel_includes)
LOCAL_SRC_FILES               := QtiAllocator.cpp service.cpp
LOCAL_INIT_RC                 := vendor.qti.hardware.display.allocator-service.rc
LOCAL_VINTF_FRAGMENTS         := vendor.qti.hardware.display.allocator-service.xml
include $(BUILD_EXECUTABLE)
