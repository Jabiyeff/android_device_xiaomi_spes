#
# Copyright (C) 2022 The LineageOS Project
#
# SPDX-License-Identifier: Apache-2.0
#

# Enable updating of APEXes
$(call inherit-product, $(SRC_TARGET_DIR)/product/updatable_apex.mk)

# Include GSI
$(call inherit-product, $(SRC_TARGET_DIR)/product/gsi_keys.mk)

# Enable Virtual A/B
$(call inherit-product, $(SRC_TARGET_DIR)/product/virtual_ab_ota.mk)

# Inherit from vendor if exists
$(call inherit-product-if-exists, vendor/xiaomi/spes/spes-vendor.mk)

# Enable Dynamic partition
BOARD_DYNAMIC_PARTITION_ENABLE := true
PRODUCT_USE_DYNAMIC_PARTITIONS := true

# API level
BOARD_SHIPPING_API_LEVEL := 30

# A/B
AB_OTA_POSTINSTALL_CONFIG += \
RUN_POSTINSTALL_vendor=true \
POSTINSTALL_PATH_vendor=bin/checkpoint_gc \
FILESYSTEM_TYPE_vendor=ext4 \
POSTINSTALL_OPTIONAL_vendor=true

PRODUCT_PACKAGES += \
    checkpoint_gc

# Audio
PRODUCT_PACKAGES += \
    android.hardware.audio@6.0-impl \
    android.hardware.audio.effect@6.0-impl \
    android.hardware.audio.service \
    android.hardware.bluetooth.audio@2.1-impl \
    android.hardware.soundtrigger@2.1-impl \
    android.hardware.soundtrigger@2.2-impl \
    android.hardware.soundtrigger@2.3-impl

PRODUCT_PACKAGES += \
    audio.a2dp.default \
    audio.bluetooth.default \
    audio.r_submix.default \
    audio.primary.bengal \
    audio.usb.default

PRODUCT_PACKAGES += \
    libaudio-resampler \
    libqcompostprocbundle \
    libqcomvisualizer \
    libqcomvoiceprocessing \
    libtinycompress \
    libvolumelistener

PRODUCT_PACKAGES += \
    vendor.qti.hardware.audiohalext@1.0 \
    vendor.qti.hardware.audiohalext@1.0.vendor\
    vendor.qti.hardware.audiohalext-utils

BOARD_SUPPORTS_OPENSOURCE_STHAL := true
BOARD_SUPPORTS_SOUND_TRIGGER := true
TARGET_USES_AOSP_FOR_AUDIO := true

# Audio configs
PRODUCT_COPY_FILES += \
    $(call find-copy-subdir-files,*,$(LOCAL_PATH)/configs/audio/,$(TARGET_COPY_OUT_VENDOR)/etc)

PRODUCT_COPY_FILES += \
    frameworks/av/services/audiopolicy/config/a2dp_audio_policy_configuration.xml:$(TARGET_COPY_OUT_VENDOR)/etc/a2dp_audio_policy_configuration.xml \
    frameworks/av/services/audiopolicy/config/audio_policy_volumes.xml:$(TARGET_COPY_OUT_VENDOR)/etc/audio_policy_volumes.xml \
    frameworks/av/services/audiopolicy/config/bluetooth_audio_policy_configuration.xml:$(TARGET_COPY_OUT_VENDOR)/etc/bluetooth_audio_policy_configuration.xml \
    frameworks/av/services/audiopolicy/config/default_volume_tables.xml:$(TARGET_COPY_OUT_VENDOR)/etc/default_volume_tables.xml \
    frameworks/av/services/audiopolicy/config/r_submix_audio_policy_configuration.xml:$(TARGET_COPY_OUT_VENDOR)/etc/r_submix_audio_policy_configuration.xml \
    frameworks/av/services/audiopolicy/config/usb_audio_policy_configuration.xml:$(TARGET_COPY_OUT_VENDOR)/etc/usb_audio_policy_configuration.xml

# Bluetooth
PRODUCT_PACKAGES += \
    vendor.qti.hardware.bluetooth_audio@2.0.vendor \
    vendor.qti.hardware.btconfigstore@1.0.vendor \
    vendor.qti.hardware.btconfigstore@2.0.vendor

# Camera
PRODUCT_PACKAGES += \
    android.hardware.camera.provider@2.4-impl \
    android.hardware.camera.provider@2.4-service_64

PRODUCT_PACKAGES += \
    libcamera2ndk_vendor \
    libgui_vendor \
    libstdc++.vendor

PRODUCT_PACKAGES += \
    Snap

PRODUCT_PACKAGES += \
    vendor.qti.hardware.camera.device@1.0.vendor \
    vendor.qti.hardware.camera.device@3.5.vendor \
    vendor.qti.hardware.camera.postproc@1.0.vendor

# Fastboot
PRODUCT_PACKAGES += \
    android.hardware.fastboot@1.0-impl-mock \
    fastbootd

# Enable incremental FS feature
PRODUCT_PROPERTY_OVERRIDES += \
    ro.incremental.enable=1

# Manufacturer
PRODUCT_PROPERTY_OVERRIDES += \
    ro.soc.manufacturer=QTI

# RRO configuration
TARGET_USES_RRO := true

# Kernel configurations
TARGET_KERNEL_VERSION := 4.19
KERNEL_LLVM_SUPPORT := true
KERNEL_SD_LLVM_SUPPORT := true

# Boot control HAL
PRODUCT_PACKAGES += \
    android.hardware.boot@1.1-impl-qti \
    android.hardware.boot@1.1-impl-qti.recovery \
    android.hardware.boot@1.1-service

PRODUCT_HOST_PACKAGES += \
    brillo_update_payload

PRODUCT_PACKAGES_DEBUG += \
    bootctl

PRODUCT_PACKAGES += \
    update_engine \
    update_engine_client \
    update_engine_sideload \
    update_verifier

# QSPM
PRODUCT_PROPERTY_OVERRIDES += \
    ro.vendor.qspm.enable=true

# Telephony
PRODUCT_PACKAGES += \
    telephony-ext

PRODUCT_BOOT_JARS += \
    telephony-ext

# Vendor property to enable advanced network scanning
PRODUCT_PROPERTY_OVERRIDES += \
    persist.vendor.radio.enableadvancedscan=true

# Property to disable ZSL mode
PRODUCT_PROPERTY_OVERRIDES += \
    camera.disable_zsl_mode=1

PRODUCT_PROPERTY_OVERRIDES += \
    ro.crypto.volume.filenames_mode = "aes-256-cts" \
    ro.crypto.allow_encrypt_override = true
