sdm-libs := sdm/libs
display-hals := include $(sdm-libs)/utils $(sdm-libs)/core libdebug gpu_tonemapper

ifneq ($(TARGET_IS_HEADLESS), true)
    display-hals += libcopybit libmemtrack hdmi_cec \
                    libdrmutils libhistogram drm.vendor
endif

display-hals += gralloc
display-hals += sde-drm
display-hals += composer
display-hals += init

ifneq ($(TARGET_PROVIDES_LIBLIGHT),true)
    display-hals += liblight
endif

ifeq ($(call is-vendor-board-platform,QCOM),true)
    include $(call all-named-subdir-makefiles,$(display-hals))
else
ifneq ($(filter msm% apq%,$(TARGET_BOARD_PLATFORM)),)
    include $(call all-named-subdir-makefiles,$(display-hals))
endif
endif
