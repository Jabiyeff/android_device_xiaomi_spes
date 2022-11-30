/*
* Copyright (c) 2014-2021, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification, are permitted
* provided that the following conditions are met:
*    * Redistributions of source code must retain the above copyright notice, this list of
*      conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright notice, this list of
*      conditions and the following disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of The Linux Foundation nor the names of its contributors may be used to
*      endorse or promote products derived from this software without specific prior written
*      permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#ifndef __DISPLAY_BASE_H__
#define __DISPLAY_BASE_H__

#include <core/display_interface.h>
#include <private/strategy_interface.h>
#include <private/color_interface.h>
#include <private/rc_intf.h>
#include <private/panel_feature_property_intf.h>
#include <private/panel_feature_factory_intf.h>

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "hw_interface.h"
#include "comp_manager.h"
#include "color_manager.h"
#include "hw_events_interface.h"

namespace sdm {

using std::recursive_mutex;
using std::lock_guard;

class DisplayBase : public DisplayInterface {
 public:
  DisplayBase(DisplayType display_type, DisplayEventHandler *event_handler,
              HWDeviceType hw_device_type, BufferAllocator *buffer_allocator,
              CompManager *comp_manager, HWInfoInterface *hw_info_intf);
  DisplayBase(int32_t display_id, DisplayType display_type, DisplayEventHandler *event_handler,
              HWDeviceType hw_device_type, BufferAllocator *buffer_allocator,
              CompManager *comp_manager, HWInfoInterface *hw_info_intf);
  virtual ~DisplayBase() { }
  virtual DisplayError Init();
  virtual DisplayError Deinit();
  DisplayError Prepare(LayerStack *layer_stack);
  DisplayError Commit(LayerStack *layer_stack);
  virtual DisplayError Flush(LayerStack *layer_stack);
  virtual DisplayError GetDisplayState(DisplayState *state);
  virtual DisplayError GetNumVariableInfoConfigs(uint32_t *count);
  virtual DisplayError GetConfig(uint32_t index, DisplayConfigVariableInfo *variable_info);
  virtual DisplayError GetConfig(DisplayConfigFixedInfo *fixed_info);
  virtual DisplayError GetActiveConfig(uint32_t *index);
  virtual DisplayError GetVSyncState(bool *enabled);
  virtual DisplayError SetDisplayState(DisplayState state, bool teardown,
                                       shared_ptr<Fence> *release_fence);
  virtual DisplayError SetActiveConfig(uint32_t index);
  virtual DisplayError SetActiveConfig(DisplayConfigVariableInfo *variable_info) {
    return kErrorNotSupported;
  }
  virtual DisplayError SetMaxMixerStages(uint32_t max_mixer_stages);
  virtual DisplayError ControlPartialUpdate(bool enable, uint32_t *pending) {
    return kErrorNotSupported;
  }
  virtual DisplayError DisablePartialUpdateOneFrame() {
    return kErrorNotSupported;
  }
  virtual DisplayError SetDisplayMode(uint32_t mode) {
    return kErrorNotSupported;
  }
  virtual bool IsUnderscanSupported() {
    return false;
  }
  virtual DisplayError SetPanelBrightness(float brightness) {
    return kErrorNotSupported;
  }
  virtual DisplayError OnMinHdcpEncryptionLevelChange(uint32_t min_enc_level);
  virtual DisplayError ColorSVCRequestRoute(const PPDisplayAPIPayload &in_payload,
                                            PPDisplayAPIPayload *out_payload,
                                            PPPendingParams *pending_action);
  virtual DisplayError SetDynamicDSIClock(uint64_t bit_clk_rate) {
    return kErrorNotSupported;
  }
  virtual DisplayError GetDynamicDSIClock(uint64_t *bit_clk_rate) {
    return kErrorNotSupported;
  }
  virtual DisplayError GetSupportedDSIClock(std::vector<uint64_t> *bitclk_rates) {
    return kErrorNotSupported;
  }
  virtual DisplayError SetPanelLuminanceAttributes(float min_lum, float max_lum) {
    return kErrorNotSupported;
  }
  virtual DisplayError GetColorModeCount(uint32_t *mode_count);
  virtual DisplayError GetColorModes(uint32_t *mode_count, std::vector<std::string> *color_modes);
  virtual DisplayError GetColorModeAttr(const std::string &color_mode, AttrVal *attr);
  virtual DisplayError SetColorMode(const std::string &color_mode);
  virtual DisplayError SetColorModeById(int32_t color_mode_id);
  virtual DisplayError GetColorModeName(int32_t mode_id, std::string *mode_name);
  virtual DisplayError SetColorTransform(const uint32_t length, const double *color_transform);
  virtual DisplayError GetDefaultColorMode(std::string *color_mode);
  virtual DisplayError ApplyDefaultDisplayMode(void);
  virtual DisplayError SetCursorPosition(int x, int y);
  virtual DisplayError GetRefreshRateRange(uint32_t *min_refresh_rate, uint32_t *max_refresh_rate);
  virtual DisplayError GetPanelBrightness(float *brightness) {
    return kErrorNotSupported;
  }
  virtual DisplayError GetPanelMaxBrightness(uint32_t *max_brightness_level) {
    return kErrorNotSupported;
  }
  virtual DisplayError SetVSyncState(bool enable);
  virtual void SetIdleTimeoutMs(uint32_t active_ms, uint32_t inactive_ms) {}
  virtual DisplayError SetMixerResolution(uint32_t width, uint32_t height);
  virtual DisplayError GetMixerResolution(uint32_t *width, uint32_t *height);
  virtual DisplayError SetFrameBufferConfig(const DisplayConfigVariableInfo &variable_info);
  virtual DisplayError GetFrameBufferConfig(DisplayConfigVariableInfo *variable_info);
  virtual DisplayError SetDetailEnhancerData(const DisplayDetailEnhancerData &de_data);
  virtual DisplayError GetDisplayPort(DisplayPort *port);
  virtual DisplayError GetDisplayId(int32_t *display_id);
  virtual DisplayError GetDisplayType(DisplayType *display_type);
  virtual bool IsPrimaryDisplay();
  virtual DisplayError SetCompositionState(LayerComposition composition_type, bool enable);
  virtual DisplayError GetClientTargetSupport(uint32_t width, uint32_t height,
                                              LayerBufferFormat format,
                                              const ColorMetaData &color_metadata);
  virtual DisplayError HandleSecureEvent(SecureEvent secure_event, LayerStack *layer_stack) {
    return kErrorNotSupported;
  }
  virtual DisplayError SetDisplayDppsAdROI(void *payload) {
    return kErrorNotSupported;
  }
  virtual DisplayError SetQSyncMode(QSyncMode qsync_mode) { return kErrorNotSupported; }
  virtual std::string Dump();
  virtual DisplayError InitializeColorModes();
  virtual DisplayError ControlIdlePowerCollapse(bool enable, bool synchronous) {
    return kErrorNotSupported;
  }
  virtual bool IsSupportSsppTonemap();
  virtual DisplayError GetDisplayIdentificationData(uint8_t *out_port, uint32_t *out_data_size,
                                                    uint8_t *out_data);
  virtual DisplayError SetFrameTriggerMode(FrameTriggerMode mode) {
    return kErrorNotSupported;
  }
  virtual DisplayError GetRefreshRate(uint32_t *refresh_rate) { return kErrorNotSupported; }
  virtual bool CanSkipValidate();
  virtual DisplayError SetBLScale(uint32_t level) { return kErrorNotSupported; }
  virtual bool CheckResourceState();
  virtual bool GameEnhanceSupported();
  virtual DisplayError GetQSyncMode(QSyncMode *qsync_mode) { return kErrorNotSupported; }
  virtual DisplayError colorSamplingOn();
  virtual DisplayError colorSamplingOff();
  virtual DisplayError ReconfigureDisplay();
  virtual DisplayError ClearLUTs() {
    return kErrorNotSupported;
  }
  virtual DisplayError DelayFirstCommit();
  QSyncMode active_qsync_mode_ = kQSyncModeNone;

 protected:
  const char *kBt2020Pq = "bt2020_pq";
  const char *kBt2020Hlg = "bt2020_hlg";
  const char *kDisplayBt2020 = "display_bt2020";
  DisplayError BuildLayerStackStats(LayerStack *layer_stack);
  virtual DisplayError ValidateGPUTargetParams();
  void CommitLayerParams(LayerStack *layer_stack);
  void PostCommitLayerParams(LayerStack *layer_stack);
  DisplayError ValidateScaling(uint32_t width, uint32_t height);
  DisplayError ValidateDataspace(const ColorMetaData &color_metadata);
  void HwRecovery(const HWRecoveryEvent sdm_event_code);

  const char *GetName(const LayerComposition &composition);
  bool NeedsMixerReconfiguration(LayerStack *layer_stack, uint32_t *new_mixer_width,
                                 uint32_t *new_mixer_height);
  DisplayError ReconfigureMixer(uint32_t width, uint32_t height);
  bool NeedsDownScale(const LayerRect &src_rect, const LayerRect &dst_rect, bool needs_rotation);
  void DeInitializeColorModes();
  DisplayError SetColorModeInternal(const std::string &color_mode,
                                    const std::string &str_render_intent,
                                    const PrimariesTransfer &pt);
  DisplayError GetValueOfModeAttribute(const AttrVal &attr, const std::string &type,
                                       std::string *value);
  bool IsSupportColorModeAttribute(const std::string &color_mode);
  void SetPUonDestScaler();
  void ClearColorInfo();
  void GetColorPrimaryTransferFromAttributes(const AttrVal &attr,
      std::vector<PrimariesTransfer> *supported_pt);
  bool DisplayPowerResetPending();
  bool SetHdrModeAtStart(LayerStack *layer_stack);
  PrimariesTransfer GetBlendSpaceFromColorMode();
  bool IsHdrMode(const AttrVal &attr);
  void InsertBT2020PqHlgModes(const std::string &str_render_intent);
  DisplayError SetupRC();
  DisplayError HandlePendingVSyncEnable(const shared_ptr<Fence> &retire_fence);
  DisplayError HandlePendingPowerState(const shared_ptr<Fence> &retire_fence);

  recursive_mutex recursive_mutex_;
  int32_t display_id_ = -1;
  DisplayType display_type_;
  DisplayEventHandler *event_handler_ = NULL;
  HWDeviceType hw_device_type_;
  HWInterface *hw_intf_ = NULL;
  HWPanelInfo hw_panel_info_;
  HWResourceInfo hw_resource_info_ = {};
  BufferAllocator *buffer_allocator_ {};
  CompManager *comp_manager_ = NULL;
  DisplayState state_ = kStateOff;
  bool active_ = false;
  Handle hw_device_ = 0;
  Handle display_comp_ctx_ = 0;
  HWLayers hw_layers_;
  bool needs_validate_ = true;
  bool vsync_enable_ = false;
  uint32_t max_mixer_stages_ = 0;
  HWInfoInterface *hw_info_intf_ = NULL;
  ColorManagerProxy *color_mgr_ = NULL;  // each display object owns its ColorManagerProxy
  bool partial_update_control_ = true;
  HWEventsInterface *hw_events_intf_ = NULL;
  bool disable_pu_one_frame_ = false;
  // TODO(user): Temporary changes, to be removed when DRM driver supports
  // Partial update with Destination scaler enabled.
  bool disable_pu_on_dest_scaler_ = false;
  uint32_t num_color_modes_ = 0;
  std::vector<SDEDisplayMode> color_modes_;
  typedef std::map<std::string, SDEDisplayMode *> ColorModeMap;
  ColorModeMap color_mode_map_ = {};
  typedef std::map<std::string, AttrVal> ColorModeAttrMap;
  ColorModeAttrMap color_mode_attr_map_ = {};
  std::vector<PrimariesTransfer> color_modes_cs_ = {};  // Gamut+Gamma(color space) of color mode
  HWDisplayAttributes display_attributes_ = {};
  HWMixerAttributes mixer_attributes_ = {};
  DisplayConfigVariableInfo fb_config_ = {};
  uint32_t req_mixer_width_ = 0;
  uint32_t req_mixer_height_ = 0;
  std::string current_color_mode_ = "hal_native";
  bool hw_recovery_logs_captured_ = false;
  int disable_hw_recovery_dump_ = 0;
  HWQosData cached_qos_data_;
  uint32_t default_clock_hz_ = 0;
  bool drop_hw_vsync_ = false;
  uint32_t current_refresh_rate_ = 0;
  bool drop_skewed_vsync_ = false;
  bool custom_mixer_resolution_ = false;
  bool vsync_enable_pending_ = false;
  bool pending_doze_ = false;
  bool pending_power_on_ = false;
  QSyncMode qsync_mode_ = kQSyncModeNone;
  bool needs_avr_update_ = false;

  static Locker display_power_reset_lock_;
  static bool display_power_reset_pending_;
  bool rc_panel_feature_init_ = false;
  bool rc_enable_prop_ = false;
  PanelFeatureFactoryIntf *pf_factory_ = nullptr;
  PanelFeaturePropertyIntf *prop_intf_ = nullptr;
  bool first_cycle_ = true;

 private:
  bool StartDisplayPowerReset();
  void EndDisplayPowerReset();
  void SetRCData(LayerStack *layer_stack);
  unsigned int rc_cached_res_width_ = 0;
  unsigned int rc_cached_res_height_ = 0;
  std::unique_ptr<RCIntf> rc_core_ = nullptr;
};

}  // namespace sdm

#endif  // __DISPLAY_BASE_H__
