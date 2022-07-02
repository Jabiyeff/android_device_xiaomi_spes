/*
* Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
* Changes from Qualcomm Innovation Center are provided under the following license:
*
* Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the
* disclaimer below) provided that the following conditions are met:
*
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*
*    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
* GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
* HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
* IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __HW_DEVICE_DRM_H__
#define __HW_DEVICE_DRM_H__

#include <drm_interface.h>
#include <errno.h>
#include <pthread.h>
#include <xf86drmMode.h>
#include <atomic>
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>

#include "hw_interface.h"
#include "hw_scale_drm.h"
#include "hw_color_manager_drm.h"

#define IOCTL_LOGE(ioctl, type) \
  DLOGE("ioctl %s, device = %d errno = %d, desc = %s", #ioctl, type, errno, strerror(errno))

#define UI_FBID_LIMIT 4
#define VIDEO_FBID_LIMIT 16
#define OFFLINE_ROTATOR_FBID_LIMIT 2

using sde_drm::DRMPowerMode;
namespace sdm {
class HWInfoInterface;

struct SDECsc {
  struct sde_drm_csc_v1 csc_v1 = {};
  // More here, maybe in a union
};

class HWDeviceDRM : public HWInterface {
 public:
  HWDeviceDRM(BufferAllocator *buffer_allocator, HWInfoInterface *hw_info_intf);
  virtual ~HWDeviceDRM() {}
  virtual DisplayError Init();
  virtual DisplayError Deinit();
  void GetDRMDisplayToken(sde_drm::DRMDisplayToken *token) const;
  bool IsPrimaryDisplay() const { return hw_panel_info_.is_primary_panel; }
  virtual PanelFeaturePropertyIntf *GetPanelFeaturePropertyIntf() { return nullptr; }

 protected:
  // From HWInterface
  virtual DisplayError GetDisplayId(int32_t *display_id);
  virtual DisplayError GetActiveConfig(uint32_t *active_config);
  virtual DisplayError GetDefaultConfig(uint32_t *default_config) { return kErrorNotSupported; }
  virtual DisplayError GetNumDisplayAttributes(uint32_t *count);
  virtual DisplayError GetDisplayAttributes(uint32_t index,
                                            HWDisplayAttributes *display_attributes);
  virtual DisplayError GetHWPanelInfo(HWPanelInfo *panel_info);
  virtual DisplayError SetDisplayAttributes(uint32_t index);
  virtual DisplayError SetDisplayAttributes(const HWDisplayAttributes &display_attributes);
  virtual DisplayError GetConfigIndex(char *mode, uint32_t *index);
  virtual DisplayError PowerOn(const HWQosData &qos_data, shared_ptr<Fence> *release_fence);
  virtual DisplayError PowerOff(bool teardown);
  virtual DisplayError Doze(const HWQosData &qos_data, shared_ptr<Fence> *release_fence);
  virtual DisplayError DozeSuspend(const HWQosData &qos_data, shared_ptr<Fence> *release_fence);
  virtual DisplayError Standby();
  virtual DisplayError Validate(HWLayers *hw_layers);
  virtual DisplayError Commit(HWLayers *hw_layers);
  virtual DisplayError Flush(HWLayers *hw_layers);
  virtual DisplayError GetPPFeaturesVersion(PPFeatureVersion *vers);
  virtual DisplayError SetPPFeatures(PPFeaturesConfig *feature_list);
  virtual DisplayError DelayFirstCommit() { return kErrorNotSupported; };
  // This API is no longer supported, expectation is to call the correct API on HWEvents
  virtual DisplayError SetVSyncState(bool enable);
  virtual void SetIdleTimeoutMs(uint32_t timeout_ms);
  virtual DisplayError SetDisplayMode(const HWDisplayMode hw_display_mode);
  virtual DisplayError SetRefreshRate(uint32_t refresh_rate);
  virtual DisplayError SetPanelBrightness(int level) { return kErrorNotSupported; }
  virtual DisplayError GetHWScanInfo(HWScanInfo *scan_info);
  virtual DisplayError GetVideoFormat(uint32_t config_index, uint32_t *video_format);
  virtual DisplayError GetMaxCEAFormat(uint32_t *max_cea_format);
  virtual DisplayError SetCursorPosition(HWLayers *hw_layers, int x, int y);
  virtual DisplayError OnMinHdcpEncryptionLevelChange(uint32_t min_enc_level);
  virtual DisplayError GetPanelBrightness(int *level) { return kErrorNotSupported; }
  virtual void GetHWPanelMaxBrightness() { return; }
  virtual DisplayError SetAutoRefresh(bool enable) { autorefresh_ = enable; return kErrorNone; }
  virtual DisplayError SetScaleLutConfig(HWScaleLutInfo *lut_info);
  virtual DisplayError UnsetScaleLutConfig();
  virtual DisplayError SetMixerAttributes(const HWMixerAttributes &mixer_attributes);
  virtual DisplayError GetMixerAttributes(HWMixerAttributes *mixer_attributes);
  virtual void InitializeConfigs();
  virtual DisplayError DumpDebugData();
  virtual void PopulateHWPanelInfo();
  virtual DisplayError SetDppsFeature(void *payload, size_t size) { return kErrorNotSupported; }
  virtual DisplayError GetDppsFeatureInfo(void *payload, size_t size) { return kErrorNotSupported; }
  virtual DisplayError TeardownConcurrentWriteback(void) { return kErrorNotSupported; }
  virtual DisplayError HandleSecureEvent(SecureEvent secure_event, HWLayers *hw_layers) {
    return kErrorNotSupported;
  }
  virtual DisplayError ControlIdlePowerCollapse(bool enable, bool synchronous) {
    return kErrorNotSupported;
  }
  virtual DisplayError SetDisplayDppsAdROI(void *payload) { return kErrorNotSupported; }
  virtual DisplayError SetDynamicDSIClock(uint64_t bit_clk_rate);
  virtual DisplayError GetDynamicDSIClock(uint64_t *bit_clk_rate);
  virtual DisplayError GetDisplayIdentificationData(uint8_t *out_port, uint32_t *out_data_size,
                                                    uint8_t *out_data);
  virtual DisplayError SetFrameTrigger(FrameTriggerMode mode) { return kErrorNotSupported; }
  virtual DisplayError SetBLScale(uint32_t level) { return kErrorNotSupported; }
  virtual DisplayError GetPanelBrightnessBasePath(std::string *base_path) {
    return kErrorNotSupported;
  }
  virtual DisplayError SetBlendSpace(const PrimariesTransfer &blend_space);

  enum {
    kHWEventVSync,
    kHWEventBlank,
  };

  static const int kMaxStringLength = 1024;
  static const int kNumPhysicalDisplays = 2;
  static const int kMaxSysfsCommandLength = 12;

  // Max tolerable power-state-change wait-times in milliseconds.
  static const int kTimeoutMsPowerOn = 5000;
  static const int kTimeoutMsPowerOff = 3000;
  static const int kTimeoutMsDoze = kTimeoutMsPowerOff;
  static const int kTimeoutMsDozeSuspend = kTimeoutMsPowerOff;

  DisplayError SetFormat(const LayerBufferFormat &source, uint32_t *target);
  DisplayError SetStride(HWDeviceType device_type, LayerBufferFormat format, uint32_t width,
                         uint32_t *target);
  DisplayError PopulateDisplayAttributes(uint32_t index);
  void GetHWDisplayPortAndMode();
  bool EnableHotPlugDetection(int enable);
  void UpdateMixerAttributes();
  void SetSolidfillStages();
  void AddSolidfillStage(const HWSolidfillStage &sf, uint32_t plane_alpha);
  void ClearSolidfillStages();
  void SetBlending(const LayerBlending &source, sde_drm::DRMBlendType *target);
  void SetSrcConfig(const LayerBuffer &input_buffer, const HWRotatorMode &mode, uint32_t *config);
  void SelectCscType(const LayerBuffer &input_buffer, sde_drm::DRMCscType *type);
  void SetRect(const LayerRect &source, sde_drm::DRMRect *target);
  void SetRotation(LayerTransform transform, const HWLayerConfig &layer_config,
                   uint32_t* rot_bit_mask);
  DisplayError DefaultCommit(HWLayers *hw_layers);
  DisplayError AtomicCommit(HWLayers *hw_layers);
  void SetupAtomic(Fence::ScopedRef &scoped_ref, HWLayers *hw_layers, bool validate,
                   int64_t *release_fence_fd, int64_t *retire_fence_fd);
  void SetSecureConfig(const LayerBuffer &input_buffer, sde_drm::DRMSecureMode *fb_secure_mode,
                       sde_drm::DRMSecurityLevel *security_level);
  bool IsResolutionSwitchEnabled() const { return resolution_switch_enabled_; }
  void SetTopology(sde_drm::DRMTopology drm_topology, HWTopology *hw_topology);
  void SetMultiRectMode(const uint32_t flags, sde_drm::DRMMultiRectMode *target);
  void SetSsppTonemapFeatures(HWPipeInfo *pipe_info);
  void SetDGMCsc(const HWPipeCscInfo &dgm_csc_info, SDECsc *csc);
  void SetDGMCscV1(const HWCsc &dgm_csc, sde_drm_csc_v1 *csc_v1);
  void SetSsppLutFeatures(HWPipeInfo *pipe_info);
  void AddDimLayerIfNeeded();
  DisplayError NullCommit(bool synchronous, bool retain_planes);
  void DumpConnectorModeInfo();
  void ResetROI();
  void SetQOSData(const HWQosData &qos_data);
  void DumpHWLayers(HWLayers *hw_layers);
  bool IsFullFrameUpdate(const HWLayersInfo &hw_layer_info);

  class Registry {
   public:
    explicit Registry(BufferAllocator *buffer_allocator);
    // Called on each Validate and Commit to map the handle_id to fb_id of each layer buffer.
    void Register(HWLayers *hw_layers);
    // Called on display disconnect to clear output buffer map and remove fb_ids.
    void Clear();
    // Create the fd_id for the given buffer.
    int CreateFbId(const LayerBuffer &buffer, uint32_t *fb_id);
    // Find handle_id in the layer map. Else create fb_id and add <handle_id,fb_id> in map.
    void MapBufferToFbId(Layer* layer, const LayerBuffer &buffer);
    // Find handle_id in output buffer map. Else create fb_id and add <handle_id,fb_id> in map.
    void MapOutputBufferToFbId(LayerBuffer* buffer);
    // Find fb_id for given handle_id in the layer map.
    uint32_t GetFbId(Layer *layer, uint64_t handle_id);
    // Find fb_id for given handle_id in output buffer map.
    uint32_t GetOutputFbId(uint64_t handle_id);

   private:
    bool disable_fbid_cache_ = false;
    std::unordered_map<uint64_t, std::shared_ptr<LayerBufferObject>> output_buffer_map_ {};
    BufferAllocator *buffer_allocator_ = {};
    uint8_t fbid_cache_limit_ = UI_FBID_LIMIT;
  };

 protected:
  bool IsSeamlessTransition() {
    return (hw_panel_info_.dynamic_fps && (vrefresh_ || seamless_mode_switch_)) ||
     panel_mode_changed_ || bit_clk_rate_;
  }

  const char *device_name_ = {};
  bool default_mode_ = false;
  int32_t display_id_ = -1;
  sde_drm::DRMDisplayType disp_type_ = {};
  HWInfoInterface *hw_info_intf_ = {};
  int dev_fd_ = -1;
  Registry registry_;
  sde_drm::DRMDisplayToken token_ = {};
  HWResourceInfo hw_resource_ = {};
  HWPanelInfo hw_panel_info_ = {};
  HWScaleDRM *hw_scale_ = {};
  sde_drm::DRMManagerInterface *drm_mgr_intf_ = {};
  sde_drm::DRMAtomicReqInterface *drm_atomic_intf_ = {};
  std::vector<HWDisplayAttributes> display_attributes_ = {};
  uint32_t current_mode_index_ = 0;
  sde_drm::DRMConnectorInfo connector_info_ = {};
  bool first_cycle_ = true;
  HWMixerAttributes mixer_attributes_ = {};
  std::vector<sde_drm::DRMSolidfillStage> solid_fills_ {};
  bool secure_display_active_ = false;
  uint64_t debug_dump_count_ = 0;
  bool synchronous_commit_ = false;
  uint32_t topology_control_ = 0;
  uint32_t vrefresh_ = 0;
  uint32_t panel_mode_changed_ = 0;
  bool reset_output_fence_offset_ = false;
  uint64_t bit_clk_rate_ = 0;
  bool update_mode_ = false;
  bool pending_doze_ = false;
  uint32_t video_mode_index_ = 0;
  uint32_t cmd_mode_index_ = 0;
  bool switch_mode_valid_ = false;
  bool doze_poms_switch_done_ = false;
  bool pending_poms_switch_ = false;
  bool active_ = false;
  PrimariesTransfer blend_space_ = {};
  DRMPowerMode last_power_mode_ = DRMPowerMode::OFF;
  uint32_t dest_scaler_blocks_used_ = 0;  // Dest scaler blocks in use by this HWDeviceDRM instance.
  // Destination scaler blocks in use by all HWDeviceDRM instances.
  static std::atomic<uint32_t> hw_dest_scaler_blocks_used_;
  bool null_display_commit_ = false;
  bool delay_first_commit_ = false;

 private:
  void SetDisplaySwitchMode(uint32_t index);

  std::string interface_str_ = "DSI";
  bool resolution_switch_enabled_ = false;
  bool autorefresh_ = false;
  std::unique_ptr<HWColorManagerDrm> hw_color_mgr_ = {};
  bool seamless_mode_switch_ = false;
};

}  // namespace sdm

#endif  // __HW_DEVICE_DRM_H__
