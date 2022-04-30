QCOM_MEDIA_ROOT := $(call my-dir)

VIDC_STUB_HAL := false
ifeq ($(TARGET_USES_QMAA),true)
  ifneq ($(TARGET_USES_QMAA_OVERRIDE_VIDEO),true)
    VIDC_STUB_HAL := true
  endif #TARGET_USES_QMAA_OVERRIDE_VIDEO
endif #TARGET_USES_QMAA

#Compile these for all targets under QCOM_BOARD_PLATFORMS list.
ifeq ($(call is-board-platform-in-list, $(QCOM_BOARD_PLATFORMS)),true)
include $(QCOM_MEDIA_ROOT)/libstagefrighthw/Android.mk
include $(QCOM_MEDIA_ROOT)/mm-core/Android.mk
endif

ifeq ($(call is-board-platform-in-list, $(MSM_VIDC_TARGET_LIST)),true)
ifeq ($(VIDC_STUB_HAL),false)
include $(QCOM_MEDIA_ROOT)/libplatformconfig/Android.mk
include $(QCOM_MEDIA_ROOT)/mm-video-v4l2/Android.mk
include $(QCOM_MEDIA_ROOT)/libc2dcolorconvert/Android.mk
include $(QCOM_MEDIA_ROOT)/media-prop/Android.mk
endif #VIDC_STUB_HAL
endif
