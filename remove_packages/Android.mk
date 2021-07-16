LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := RemovePkgs
LOCAL_MODULE_CLASS := APPS
LOCAL_MODULE_TAGS := optional
LOCAL_OVERRIDES_PACKAGES := AmbientSensePrebuilt AndroidAutoStubPrebuilt AppDirectedSMSService Drive ConnMO DCMO
LOCAL_OVERRIDES_PACKAGES += DMService GCS GoogleCamera MaestroPrebuilt Maps MicropaperPrebuilt MyVerizonServices
LOCAL_OVERRIDES_PACKAGES += YouTube YouTubeMusicPrebuilt SafetyHubPrebuilt SCONE ScribePrebuilt Showcase Snap
LOCAL_OVERRIDES_PACKAGES += SoundAmplifierPrebuilt SprintDM SprintHM TurboPrebuilt Tycho USCCDM VZWAPNLib
LOCAL_OVERRIDES_PACKAGES += VzwOmaTrigger OBDM_Permissions obdm_stub arcore DevicePolicyPrebuilt RecorderPrebuilt
LOCAL_UNINSTALLABLE_MODULE := true
LOCAL_CERTIFICATE := platform
LOCAL_SRC_FILES := /dev/null
include $(BUILD_PREBUILT)
