#Common headers
display_top := $(call my-dir)

#Common C flags
common_flags := -Wno-missing-field-initializers
common_flags += -Wall -Werror
common_flags += -DUSE_GRALLOC1
ifeq ($(TARGET_IS_HEADLESS), true)
    common_flags += -DTARGET_HEADLESS
endif

ifeq ($(TARGET_USES_COLOR_METADATA), true)
    common_flags += -DUSE_COLOR_METADATA
endif

ifeq ($(TARGET_USES_5.4_KERNEL),true)
    common_flags += -DKERNEL_5_4
endif

ifeq ($(TARGET_USES_QCOM_BSP),true)
    common_flags += -DQTI_BSP
endif

ifeq ($(ARCH_ARM_HAVE_NEON),true)
    common_flags += -D__ARM_HAVE_NEON
endif

ifeq ($(call is-board-platform-in-list, $(MASTER_SIDE_CP_TARGET_LIST)), true)
    common_flags += -DMASTER_SIDE_CP
endif

use_hwc2 := false
ifeq ($(TARGET_USES_HWC2), true)
    use_hwc2 := true
    common_flags += -DVIDEO_MODE_DEFER_RETIRE_FENCE
endif

ifneq (,$(filter userdebug eng, $(TARGET_BUILD_VARIANT)))
    common_flags += -DUSER_DEBUG
endif

ifeq ($(LLVM_SA), true)
    common_flags += --compile-and-analyze --analyzer-perf --analyzer-Werror
endif

#Common libraries external to display HAL
common_libs := liblog libutils libcutils libhardware
common_deps  :=
kernel_includes :=

ifeq ($(TARGET_COMPILE_WITH_MSM_KERNEL),true)
# This check is to pick the kernel headers from the right location.
# If the macro above is defined, we make the assumption that we have the kernel
# available in the build tree.
# If the macro is not present, the headers are picked from hardware/qcom/msmXXXX
# failing which, they are picked from bionic.
    common_deps += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
    kernel_includes += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
ifeq ($(TARGET_USES_5.4_KERNEL), true)
    kernel_includes += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include/display
    kernel_includes += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include/vidc
endif
endif
