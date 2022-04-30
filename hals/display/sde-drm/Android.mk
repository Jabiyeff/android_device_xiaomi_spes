ifneq ($(TARGET_IS_HEADLESS), true)
LOCAL_PATH := $(call my-dir)
include $(LOCAL_PATH)/../common.mk
include $(CLEAR_VARS)

common_header_export_path := qcom/display

LOCAL_MODULE              := libsdedrm
LOCAL_SANITIZE            := integer_overflow
LOCAL_MODULE_TAGS         := optional
LOCAL_SHARED_LIBRARIES    := libdrm libdrmutils libdisplaydebug
LOCAL_HEADER_LIBRARIES    := display_headers
LOCAL_C_INCLUDES          := $(kernel_includes) \
                             -isystem external/libdrm
LOCAL_ADDITIONAL_DEPENDENCIES := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
LOCAL_CFLAGS              := -Wno-missing-field-initializers -Wall -Werror -fno-operator-names \
                             -Wno-unused-parameter -DLOG_TAG=\"SDE_DRM\"
LOCAL_CFLAGS              += $(common_flags)
LOCAL_CLANG               := true
LOCAL_SRC_FILES           := drm_manager.cpp \
                             drm_connector.cpp \
                             drm_encoder.cpp \
                             drm_crtc.cpp \
                             drm_plane.cpp \
                             drm_atomic_req.cpp \
                             drm_utils.cpp \
                             drm_pp_manager.cpp \
                             drm_property.cpp \
                             drm_panel_feature_mgr.cpp \
                             drm_dpps_mgr_imp.cpp

ifeq ($(TARGET_USES_DRM_PP),true)
LOCAL_CFLAGS              += -DPP_DRM_ENABLE
endif

ifeq ($(LLVM_SA), true)
LOCAL_CFLAGS += --compile-and-analyze --analyzer-perf
endif

LOCAL_VENDOR_MODULE       := true
include $(BUILD_SHARED_LIBRARY)
endif
