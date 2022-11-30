MSM_VIDC_TARGET_LIST := kona lito bengal

ifeq ($(call is-board-platform-in-list, $(QCOM_BOARD_PLATFORMS)),true)

#MM_CORE
MM_CORE := libmm-omxcore
MM_CORE += libOmxCore

PRODUCT_PACKAGES += $(MM_CORE)

endif

ifeq ($(call is-board-platform-in-list, $(MSM_VIDC_TARGET_LIST)), true)

MM_VIDEO := ExoplayerDemo
MM_VIDEO += libc2dcolorconvert
MM_VIDEO += libOmxSwVdec
MM_VIDEO += libOmxSwVencMpeg4
MM_VIDEO += libOmxVdec
MM_VIDEO += libOmxVenc
MM_VIDEO += libstagefrighthw
MM_VIDEO += init.qti.media.sh

PRODUCT_PACKAGES += $(MM_VIDEO)

include $(TARGET_HALS_PATH)/media/conf_files/$(TARGET_BOARD_PLATFORM)/$(TARGET_BOARD_PLATFORM).mk

endif

#Vendor property to enable Codec2 for audio and OMX for Video
PRODUCT_VENDOR_PROPERTIES += debug.stagefright.ccodec=1
