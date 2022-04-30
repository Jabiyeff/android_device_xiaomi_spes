/*
* Copyright (c) 2014 - 2020, The Linux Foundation. All rights reserved.
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

#ifndef __COMP_MANAGER_H__
#define __COMP_MANAGER_H__

#include <core/display_interface.h>
#include <private/extension_interface.h>
#include <utils/locker.h>
#include <bitset>
#include <set>
#include <vector>
#include <string>

#include "strategy.h"
#include "resource_default.h"
#include "hw_interface.h"

namespace sdm {

class CompManager {
 public:
  DisplayError Init(const HWResourceInfo &hw_res_info_, ExtensionInterface *extension_intf,
                    BufferAllocator *buffer_allocator, SocketHandler *socket_handler);
  DisplayError Deinit();
  DisplayError RegisterDisplay(int32_t display_id, DisplayType type,
                               const HWDisplayAttributes &display_attributes,
                               const HWPanelInfo &hw_panel_info,
                               const HWMixerAttributes &mixer_attributes,
                               const DisplayConfigVariableInfo &fb_config, Handle *display_ctx,
                               uint32_t *default_clk_hz);
  DisplayError UnregisterDisplay(Handle display_ctx);
  DisplayError ReconfigureDisplay(Handle display_ctx, const HWDisplayAttributes &display_attributes,
                                  const HWPanelInfo &hw_panel_info,
                                  const HWMixerAttributes &mixer_attributes,
                                  const DisplayConfigVariableInfo &fb_config,
                                  uint32_t *default_clk_hz);
  void PrePrepare(Handle display_ctx, HWLayers *hw_layers);
  DisplayError Prepare(Handle display_ctx, HWLayers *hw_layers);
  DisplayError Commit(Handle display_ctx, HWLayers *hw_layers);
  DisplayError PostPrepare(Handle display_ctx, HWLayers *hw_layers);
  DisplayError ReConfigure(Handle display_ctx, HWLayers *hw_layers);
  DisplayError PostCommit(Handle display_ctx, HWLayers *hw_layers);
  void Purge(Handle display_ctx);
  DisplayError SetIdleTimeoutMs(Handle display_ctx, uint32_t active_ms, uint32_t inactive_ms);
  void ProcessIdleTimeout(Handle display_ctx);
  void ProcessThermalEvent(Handle display_ctx, int64_t thermal_level);
  void ProcessIdlePowerCollapse(Handle display_ctx);
  DisplayError SetMaxMixerStages(Handle display_ctx, uint32_t max_mixer_stages);
  void ControlPartialUpdate(Handle display_ctx, bool enable);
  DisplayError ValidateScaling(const LayerRect &crop, const LayerRect &dst, bool rotate90);
  DisplayError ValidateAndSetCursorPosition(Handle display_ctx, HWLayers *hw_layers, int x, int y);
  bool SetDisplayState(Handle display_ctx, DisplayState state,
                       const shared_ptr<Fence> &sync_handle);
  DisplayError SetMaxBandwidthMode(HWBwModes mode);
  DisplayError GetScaleLutConfig(HWScaleLutInfo *lut_info);
  DisplayError SetDetailEnhancerData(Handle display_ctx, const DisplayDetailEnhancerData &de_data);
  DisplayError SetCompositionState(Handle display_ctx, LayerComposition composition_type,
                                   bool enable);
  DisplayError ControlDpps(bool enable);
  DisplayError SetColorModesInfo(Handle display_ctx,
                                 const std::vector<PrimariesTransfer> &colormodes_cs);
  DisplayError SetBlendSpace(Handle display_ctx, const PrimariesTransfer &blend_space);
  void HandleSecureEvent(Handle display_ctx, SecureEvent secure_event);
  void SetSafeMode(bool enable) { safe_mode_ = enable; }
  bool CanSkipValidate(Handle display_ctx, bool *needs_buffer_swap);
  bool IsSafeMode() { return safe_mode_; }
  void GenerateROI(Handle display_ctx, HWLayers *hw_layers);
  DisplayError CheckEnforceSplit(Handle comp_handle, uint32_t new_refresh_rate);
  DppsControlInterface* GetDppsControlIntf() { return dpps_ctrl_intf_; }
  bool CheckResourceState(Handle display_ctx);
  bool IsRotatorSupportedFormat(LayerBufferFormat format);
  DisplayError SwapBuffers(Handle display_ctx);

 private:
  static const int kMaxThermalLevel = 3;
  static const int kSafeModeThreshold = 4;

  void PrepareStrategyConstraints(Handle display_ctx, HWLayers *hw_layers);
  void UpdateStrategyConstraints(bool is_primary, bool disabled);
  std::string StringDisplayList(const std::set<int32_t> &displays);

  struct DisplayCompositionContext {
    Strategy *strategy = NULL;
    StrategyConstraints constraints;
    Handle display_resource_ctx = NULL;
    int32_t display_id = -1;
    DisplayType display_type = kBuiltIn;
    uint32_t max_strategies = 0;
    uint32_t remaining_strategies = 0;
    bool idle_fallback = false;
    bool thermal_fallback_ = false;
    // Using primary panel flag of hw panel to configure Constraints. We do not need other hw
    // panel parameters for now.
    bool is_primary_panel = false;
    PUConstraints pu_constraints = {};
    DisplayConfigVariableInfo fb_config = {};
    bool first_cycle_ = true;
    uint32_t dest_scaler_blocks_used = 0;
  };

  Locker locker_;
  ResourceInterface *resource_intf_ = NULL;
  std::set<int32_t> registered_displays_;  // List of registered displays
  std::set<int32_t> configured_displays_;  // List of sucessfully configured displays
  std::set<int32_t> powered_on_displays_;  // List of powered on displays.
  bool safe_mode_ = false;              // Flag to notify all displays to be in resource crunch
                                        // mode, where strategy manager chooses the best strategy
                                        // that uses optimal number of pipes for each display
  HWResourceInfo hw_res_info_;
  BufferAllocator *buffer_allocator_ = NULL;
  ExtensionInterface *extension_intf_ = NULL;
  uint32_t max_sde_ext_layers_ = 0;
  uint32_t max_sde_builtin_layers_ = 2;
  DppsControlInterface *dpps_ctrl_intf_ = NULL;
};

}  // namespace sdm

#endif  // __COMP_MANAGER_H__

