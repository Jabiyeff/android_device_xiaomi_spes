# Board specific SELinux policy variable definitions
SEPOLICY_PATH:= device/xiaomi/spes/sepolicy/vndr
QSSI_SEPOLICY_PATH:= device/qcom/sepolicy
SYS_ATTR_PROJECT_PATH := $(TOP)/device/qcom/sepolicy/generic/public/attribute
BOARD_PLAT_PUBLIC_SEPOLICY_DIR := \
    $(BOARD_PLAT_PUBLIC_SEPOLICY_DIR) \
    $(QSSI_SEPOLICY_PATH)/generic/public \
    $(QSSI_SEPOLICY_PATH)/generic/public/attribute

BOARD_PLAT_PRIVATE_SEPOLICY_DIR := \
    $(BOARD_PLAT_PRIVATE_SEPOLICY_DIR) \
    $(QSSI_SEPOLICY_PATH)/generic/private

BOARD_PLAT_PUBLIC_SEPOLICY_DIR := \
    $(BOARD_PLAT_PUBLIC_SEPOLICY_DIR) \
    $(QSSI_SEPOLICY_PATH)/qva/public \
    $(QSSI_SEPOLICY_PATH)/qva/public/attribute

BOARD_PLAT_PRIVATE_SEPOLICY_DIR := \
    $(BOARD_PLAT_PRIVATE_SEPOLICY_DIR) \
    $(QSSI_SEPOLICY_PATH)/qva/private

#once all the services are moved to Product /ODM above lines will be removed.
# sepolicy rules for product images
PRODUCT_PUBLIC_SEPOLICY_DIRS := \
    $(PRODUCT_PUBLIC_SEPOLICY_DIRS) \
    $(QSSI_SEPOLICY_PATH)/generic/product/public \
    $(QSSI_SEPOLICY_PATH)/qva/product/public

PRODUCT_PRIVATE_SEPOLICY_DIRS := \
    $(PRODUCT_PRIVATE_SEPOLICY_DIRS) \
    $(QSSI_SEPOLICY_PATH)/generic/product/private \
    $(QSSI_SEPOLICY_PATH)/qva/product/private

BOARD_VENDOR_SEPOLICY_DIRS := \
   $(BOARD_VENDOR_SEPOLICY_DIRS) \
   $(SEPOLICY_PATH) \
   $(SEPOLICY_PATH)/generic/vendor/common \
   $(SEPOLICY_PATH)/generic/vendor/common/attribute \
   $(SEPOLICY_PATH)/qva/vendor/ssg \
   $(SEPOLICY_PATH)/qva/vendor/common

ifeq ($(TARGET_SEPOLICY_DIR),)
    BOARD_VENDOR_SEPOLICY_DIRS += $(SEPOLICY_PATH)/generic/vendor/$(TARGET_BOARD_PLATFORM)
    BOARD_VENDOR_SEPOLICY_DIRS += $(SEPOLICY_PATH)/qva/vendor/$(TARGET_BOARD_PLATFORM)
else
    BOARD_VENDOR_SEPOLICY_DIRS += $(SEPOLICY_PATH)/generic/vendor/$(TARGET_SEPOLICY_DIR)
    BOARD_VENDOR_SEPOLICY_DIRS += $(SEPOLICY_PATH)/qva/vendor/$(TARGET_SEPOLICY_DIR)
endif

ifneq (,$(filter userdebug eng, $(TARGET_BUILD_VARIANT)))
    BOARD_VENDOR_SEPOLICY_DIRS += $(SEPOLICY_PATH)/generic/vendor/test
    BOARD_VENDOR_SEPOLICY_DIRS += $(SEPOLICY_PATH)/qva/vendor/test
    BOARD_VENDOR_SEPOLICY_DIRS += $(SEPOLICY_PATH)/qva/vendor/test/sysmonapp
    BOARD_VENDOR_SEPOLICY_DIRS += $(SEPOLICY_PATH)/qva/vendor/test/mst_test_app
endif

-include device/lineage/sepolicy/qcom/sepolicy.mk
