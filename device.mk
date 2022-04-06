#
# Copyright (C) 2022 The LineageOS Project
#
# SPDX-License-Identifier: Apache-2.0
#

# Inherit from vendor if exists
$(call inherit-product-if-exists, vendor/xiaomi/spes/spes-vendor.mk)

# Include GSI
$(call inherit-product, build/make/target/product/gsi_keys.mk)

# Enable Virtual A/B
$(call inherit-product, $(SRC_TARGET_DIR)/product/virtual_ab_ota.mk)

# Audio
PRODUCT_PACKAGES += libvolumelistener

SYSTEMEXT_SEPARATE_PARTITION_ENABLE = true

# Enable Dynamic partition
BOARD_DYNAMIC_PARTITION_ENABLE := true
PRODUCT_USE_DYNAMIC_PARTITIONS := true

# API level
BOARD_SHIPPING_API_LEVEL := 30
BOARD_API_LEVEL := 30

# Fastboot
PRODUCT_PACKAGES += fastbootd
PRODUCT_PACKAGES += android.hardware.fastboot@1.0-impl-mock

# F2FS
PRODUCT_PACKAGES += \
    f2fs_io \
    check_f2fs \
    sg_write_buffer

# Userdata checkpoint
AB_OTA_POSTINSTALL_CONFIG += \
RUN_POSTINSTALL_vendor=true \
POSTINSTALL_PATH_vendor=bin/checkpoint_gc \
FILESYSTEM_TYPE_vendor=ext4 \
POSTINSTALL_OPTIONAL_vendor=true

PRODUCT_PACKAGES += \
    checkpoint_gc

# Enable incremental FS feature
PRODUCT_PROPERTY_OVERRIDES += ro.incremental.enable=1

# Manufacturer
PRODUCT_PROPERTY_OVERRIDES += \
    ro.soc.manufacturer=QTI

# Hal Hardware
TARGET_USES_AOSP := false
TARGET_USES_AOSP_FOR_AUDIO := false
TARGET_USES_QCOM_BSP := false

# RRO configuration
TARGET_USES_RRO := true

# Kernel configurations
TARGET_KERNEL_VERSION := 4.19
KERNEL_LLVM_SUPPORT := true
KERNEL_SD_LLVM_SUPPORT := true

# A/B related packages
PRODUCT_PACKAGES += \
    android.hardware.boot@1.1-impl-qti \
    android.hardware.boot@1.1-impl-qti.recovery \
    android.hardware.boot@1.1-service
    update_engine \
    update_engine_client \
    update_verifier

PRODUCT_HOST_PACKAGES += \
    brillo_update_payload

# Boot control HAL
PRODUCT_PACKAGES_DEBUG += \
    bootctl

PRODUCT_PACKAGES += \
    update_engine_sideload

# QSPM
PRODUCT_PROPERTY_OVERRIDES += \
    ro.vendor.qspm.enable=true

# Telephony
PRODUCT_PACKAGES += telephony-ext
PRODUCT_BOOT_JARS += telephony-ext
PRODUCT_BOOT_JARS += tcmiface

# Vendor property to enable advanced network scanning
PRODUCT_PROPERTY_OVERRIDES += \
    persist.vendor.radio.enableadvancedscan=true

# Property to disable ZSL mode
PRODUCT_PROPERTY_OVERRIDES += \
    camera.disable_zsl_mode=1

PRODUCT_PROPERTY_OVERRIDES += \
ro.crypto.volume.filenames_mode = "aes-256-cts" \
ro.crypto.allow_encrypt_override = true
