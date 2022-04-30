/*
* Copyright (c) 2014-2020, The Linux Foundation. All rights reserved.
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

#include <core/buffer_allocator.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <set>
#include <string>
#include <vector>

#include "comp_manager.h"
#include "strategy.h"

#define __CLASS__ "CompManager"

namespace sdm {

DisplayError CompManager::Init(const HWResourceInfo &hw_res_info,
                               ExtensionInterface *extension_intf,
                               BufferAllocator *buffer_allocator,
                               SocketHandler *socket_handler) {
  SCOPE_LOCK(locker_);

  DisplayError error = kErrorNone;

  if (extension_intf) {
    error = extension_intf->CreateResourceExtn(hw_res_info, buffer_allocator, &resource_intf_);
    extension_intf->CreateDppsControlExtn(&dpps_ctrl_intf_, socket_handler);
  } else {
    error = ResourceDefault::CreateResourceDefault(hw_res_info, &resource_intf_);
  }

  if (error != kErrorNone) {
    if (extension_intf) {
      extension_intf->DestroyDppsControlExtn(dpps_ctrl_intf_);
    }
    return error;
  }

  hw_res_info_ = hw_res_info;
  buffer_allocator_ = buffer_allocator;
  extension_intf_ = extension_intf;

  return error;
}

DisplayError CompManager::Deinit() {
  SCOPE_LOCK(locker_);

  if (extension_intf_) {
    extension_intf_->DestroyResourceExtn(resource_intf_);
    extension_intf_->DestroyDppsControlExtn(dpps_ctrl_intf_);
  } else {
    ResourceDefault::DestroyResourceDefault(resource_intf_);
  }

  return kErrorNone;
}

DisplayError CompManager::RegisterDisplay(int32_t display_id, DisplayType type,
                                          const HWDisplayAttributes &display_attributes,
                                          const HWPanelInfo &hw_panel_info,
                                          const HWMixerAttributes &mixer_attributes,
                                          const DisplayConfigVariableInfo &fb_config,
                                          Handle *display_ctx, uint32_t *default_clk_hz) {
  SCOPE_LOCK(locker_);

  DisplayError error = kErrorNone;

  DisplayCompositionContext *display_comp_ctx = new DisplayCompositionContext();
  if (!display_comp_ctx) {
    return kErrorMemory;
  }

  Strategy *&strategy = display_comp_ctx->strategy;
  strategy = new Strategy(extension_intf_, buffer_allocator_, display_id, type,
                          hw_res_info_, hw_panel_info, mixer_attributes, display_attributes,
                          fb_config);
  if (!strategy) {
    DLOGE("Unable to create strategy");
    delete display_comp_ctx;
    return kErrorMemory;
  }

  error = strategy->Init();
  if (error != kErrorNone) {
    delete strategy;
    delete display_comp_ctx;
    return error;
  }

  error =
      resource_intf_->RegisterDisplay(display_id, type, display_attributes, hw_panel_info,
                                      mixer_attributes, &display_comp_ctx->display_resource_ctx);
  if (error != kErrorNone) {
    strategy->Deinit();
    delete strategy;
    delete display_comp_ctx;
    display_comp_ctx = NULL;
    return error;
  }

  error = resource_intf_->Perform(ResourceInterface::kCmdGetDefaultClk,
                                  display_comp_ctx->display_resource_ctx, default_clk_hz);
  if (error != kErrorNone) {
    strategy->Deinit();
    delete strategy;
    resource_intf_->UnregisterDisplay(display_comp_ctx->display_resource_ctx);
    delete display_comp_ctx;
    display_comp_ctx = NULL;
    return error;
  }

  error = resource_intf_->Perform(ResourceInterface::kCmdDedicatePipes,
                                  display_comp_ctx->display_resource_ctx);
  if (error != kErrorNone) {
    strategy->Deinit();
    delete strategy;
    resource_intf_->UnregisterDisplay(display_comp_ctx->display_resource_ctx);
    delete display_comp_ctx;
    display_comp_ctx = NULL;
    return error;
  }

  registered_displays_.insert(display_id);
  display_comp_ctx->is_primary_panel = hw_panel_info.is_primary_panel;
  display_comp_ctx->display_id = display_id;
  display_comp_ctx->display_type = type;
  display_comp_ctx->fb_config = fb_config;
  display_comp_ctx->dest_scaler_blocks_used = mixer_attributes.dest_scaler_blocks_used;
  *display_ctx = display_comp_ctx;
  // New non-primary display device has been added, so move the composition mode to safe mode until
  // resources for the added display is configured properly.
  if (!display_comp_ctx->is_primary_panel) {
    max_sde_ext_layers_ = UINT32(Debug::GetExtMaxlayers());
  }

  DLOGV_IF(kTagCompManager, "Registered displays [%s], display %d-%d",
           StringDisplayList(registered_displays_).c_str(), display_comp_ctx->display_id,
           display_comp_ctx->display_type);

  return kErrorNone;
}

DisplayError CompManager::UnregisterDisplay(Handle display_ctx) {
  SCOPE_LOCK(locker_);

  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  if (!display_comp_ctx) {
    return kErrorParameters;
  }

  resource_intf_->UnregisterDisplay(display_comp_ctx->display_resource_ctx);

  Strategy *&strategy = display_comp_ctx->strategy;
  strategy->Deinit();
  delete strategy;

  registered_displays_.erase(display_comp_ctx->display_id);
  powered_on_displays_.erase(display_comp_ctx->display_id);

  DLOGV_IF(kTagCompManager, "Registered displays [%s], display %d-%d",
           StringDisplayList(registered_displays_).c_str(), display_comp_ctx->display_id,
           display_comp_ctx->display_type);

  delete display_comp_ctx;
  display_comp_ctx = NULL;
  return kErrorNone;
}

DisplayError CompManager::CheckEnforceSplit(Handle comp_handle,
                                            uint32_t new_refresh_rate) {
  SCOPE_LOCK(locker_);
  DisplayError error = kErrorNone;
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(comp_handle);

  error = resource_intf_->Perform(ResourceInterface::kCmdCheckEnforceSplit,
                                  display_comp_ctx->display_resource_ctx, new_refresh_rate);
  return error;
}

DisplayError CompManager::ReconfigureDisplay(Handle comp_handle,
                                             const HWDisplayAttributes &display_attributes,
                                             const HWPanelInfo &hw_panel_info,
                                             const HWMixerAttributes &mixer_attributes,
                                             const DisplayConfigVariableInfo &fb_config,
                                             uint32_t *default_clk_hz) {
  SCOPE_LOCK(locker_);
  DTRACE_SCOPED();

  DisplayError error = kErrorNone;
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(comp_handle);

  error = resource_intf_->ReconfigureDisplay(display_comp_ctx->display_resource_ctx,
                                             display_attributes, hw_panel_info, mixer_attributes);
  if (error != kErrorNone) {
    return error;
  }

  error = resource_intf_->Perform(ResourceInterface::kCmdGetDefaultClk,
                                  display_comp_ctx->display_resource_ctx, default_clk_hz);
  if (error != kErrorNone) {
    return error;
  }

  error = resource_intf_->Perform(ResourceInterface::kCmdCheckEnforceSplit,
                                  display_comp_ctx->display_resource_ctx, display_attributes.fps);
  if (error != kErrorNone) {
    return error;
  }

  if (display_comp_ctx->strategy) {
    error = display_comp_ctx->strategy->Reconfigure(hw_panel_info, display_attributes,
                                                    mixer_attributes, fb_config);
    if (error != kErrorNone) {
      DLOGE("Unable to Reconfigure strategy.");
      display_comp_ctx->strategy->Deinit();
      delete display_comp_ctx->strategy;
      display_comp_ctx->strategy = NULL;
      return error;
    }
  }

  // Update new resolution.
  display_comp_ctx->fb_config = fb_config;
  return error;
}

void CompManager::PrepareStrategyConstraints(Handle comp_handle, HWLayers *hw_layers) {
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(comp_handle);
  StrategyConstraints *constraints = &display_comp_ctx->constraints;

  constraints->safe_mode = safe_mode_;
  constraints->max_layers = hw_res_info_.num_blending_stages;

  // Limit 2 layer SDE Comp if its not a Primary Display.
  // Safe mode is the policy for External display on a low end device.
  if (!display_comp_ctx->is_primary_panel) {
    bool low_end_hw = ((hw_res_info_.num_vig_pipe + hw_res_info_.num_rgb_pipe +
                        hw_res_info_.num_dma_pipe) <= kSafeModeThreshold);
    constraints->max_layers = display_comp_ctx->display_type == kBuiltIn ?
                              max_sde_builtin_layers_ : max_sde_ext_layers_;
    constraints->safe_mode = (low_end_hw && !hw_res_info_.separate_rotator) ? true : safe_mode_;
  }

  // If a strategy fails after successfully allocating resources, then set safe mode
  if (display_comp_ctx->remaining_strategies != display_comp_ctx->max_strategies) {
    constraints->safe_mode = true;
  }

  uint32_t app_layer_count = UINT32(hw_layers->info.stack->layers.size()) - 1;
  if (display_comp_ctx->idle_fallback || display_comp_ctx->thermal_fallback_) {
    // Handle the idle timeout by falling back
    constraints->safe_mode = true;
  }

  // Avoid safe mode, if there is only one app layer.
  if (app_layer_count == 1) {
     constraints->safe_mode = false;
  }
}

void CompManager::GenerateROI(Handle display_ctx, HWLayers *hw_layers) {
  SCOPE_LOCK(locker_);
  DisplayCompositionContext *disp_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  return disp_comp_ctx->strategy->GenerateROI(&hw_layers->info, disp_comp_ctx->pu_constraints);
}

void CompManager::PrePrepare(Handle display_ctx, HWLayers *hw_layers) {
  SCOPE_LOCK(locker_);
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  display_comp_ctx->strategy->Start(&hw_layers->info, &display_comp_ctx->max_strategies);
  display_comp_ctx->remaining_strategies = display_comp_ctx->max_strategies;
}

DisplayError CompManager::Prepare(Handle display_ctx, HWLayers *hw_layers) {
  SCOPE_LOCK(locker_);

  DTRACE_SCOPED();
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  Handle &display_resource_ctx = display_comp_ctx->display_resource_ctx;

  DisplayError error = kErrorUndefined;

  PrepareStrategyConstraints(display_ctx, hw_layers);

  // Select a composition strategy, and try to allocate resources for it.
  resource_intf_->Start(display_resource_ctx);

  bool exit = false;
  uint32_t &count = display_comp_ctx->remaining_strategies;
  for (; !exit && count > 0; count--) {
    error = display_comp_ctx->strategy->GetNextStrategy(&display_comp_ctx->constraints);
    if (error != kErrorNone) {
      // Composition strategies exhausted. Resource Manager could not allocate resources even for
      // GPU composition. This will never happen.
      exit = true;
    }

    if (!exit) {
      error = resource_intf_->Prepare(display_resource_ctx, hw_layers);
      // Exit if successfully prepared resource, else try next strategy.
      exit = (error == kErrorNone);
    }
  }

  if (error != kErrorNone) {
    resource_intf_->Stop(display_resource_ctx, hw_layers);
    if (safe_mode_ && display_comp_ctx->first_cycle_) {
      DLOGW("Composition strategies exhausted for display = %d on first cycle",
            display_comp_ctx->display_type);
    } else {
      DLOGE("Composition strategies exhausted for display = %d", display_comp_ctx->display_type);
    }
    return error;
  }

  return error;
}

DisplayError CompManager::PostPrepare(Handle display_ctx, HWLayers *hw_layers) {
  SCOPE_LOCK(locker_);
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  Handle &display_resource_ctx = display_comp_ctx->display_resource_ctx;

  DisplayError error = kErrorNone;

  display_comp_ctx->strategy->Stop();

  error = resource_intf_->PostPrepare(display_resource_ctx, hw_layers);
  if (error != kErrorNone) {
    return error;
  }

  return kErrorNone;
}

DisplayError CompManager::Commit(Handle display_ctx, HWLayers *hw_layers) {
  SCOPE_LOCK(locker_);

  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  return resource_intf_->Commit(display_comp_ctx->display_resource_ctx, hw_layers);
}

DisplayError CompManager::ReConfigure(Handle display_ctx, HWLayers *hw_layers) {
  SCOPE_LOCK(locker_);

  DTRACE_SCOPED();
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  Handle &display_resource_ctx = display_comp_ctx->display_resource_ctx;

  DisplayError error = kErrorUndefined;
  resource_intf_->Start(display_resource_ctx);
  error = resource_intf_->Prepare(display_resource_ctx, hw_layers);

  if (error != kErrorNone) {
    DLOGE("Reconfigure failed for display = %d", display_comp_ctx->display_type);
  }

  resource_intf_->Stop(display_resource_ctx, hw_layers);
  if (error != kErrorNone) {
      error = resource_intf_->PostPrepare(display_resource_ctx, hw_layers);
  }

  return error;
}

DisplayError CompManager::PostCommit(Handle display_ctx, HWLayers *hw_layers) {
  SCOPE_LOCK(locker_);

  DisplayError error = kErrorNone;
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  Handle &display_resource_ctx = display_comp_ctx->display_resource_ctx;
  error = resource_intf_->Stop(display_resource_ctx, hw_layers);
  if (error != kErrorNone) {
    DLOGE("Resource stop failed for display = %d", display_comp_ctx->display_type);
  }

  error = resource_intf_->PostCommit(display_comp_ctx->display_resource_ctx, hw_layers);
  if (error != kErrorNone) {
    return error;
  }

  display_comp_ctx->idle_fallback = false;
  display_comp_ctx->first_cycle_ = false;

  DLOGV_IF(kTagCompManager, "Registered displays [%s], display %d-%d",
           StringDisplayList(registered_displays_).c_str(), display_comp_ctx->display_id,
           display_comp_ctx->display_type);

  return kErrorNone;
}

void CompManager::Purge(Handle display_ctx) {
  SCOPE_LOCK(locker_);

  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  resource_intf_->Purge(display_comp_ctx->display_resource_ctx);

  display_comp_ctx->strategy->Purge();
}

DisplayError CompManager::SetIdleTimeoutMs(Handle display_ctx, uint32_t active_ms,
                                           uint32_t inactive_ms) {
  SCOPE_LOCK(locker_);

  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  return display_comp_ctx->strategy->SetIdleTimeoutMs(active_ms, inactive_ms);
}

void CompManager::ProcessIdleTimeout(Handle display_ctx) {
  DTRACE_SCOPED();
  SCOPE_LOCK(locker_);

  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  if (!display_comp_ctx) {
    return;
  }
  display_comp_ctx->idle_fallback = true;
}

void CompManager::ProcessThermalEvent(Handle display_ctx, int64_t thermal_level) {
  SCOPE_LOCK(locker_);

  DisplayCompositionContext *display_comp_ctx =
          reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  if (thermal_level >= kMaxThermalLevel) {
    display_comp_ctx->thermal_fallback_ = true;
  } else {
    display_comp_ctx->thermal_fallback_ = false;
  }
}

void CompManager::ProcessIdlePowerCollapse(Handle display_ctx) {
  SCOPE_LOCK(locker_);

  DisplayCompositionContext *display_comp_ctx =
          reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  if (display_comp_ctx) {
    resource_intf_->Perform(ResourceInterface::kCmdResetLUT,
                            display_comp_ctx->display_resource_ctx);
  }
}

DisplayError CompManager::SetMaxMixerStages(Handle display_ctx, uint32_t max_mixer_stages) {
  SCOPE_LOCK(locker_);

  DisplayError error = kErrorNone;
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  if (display_comp_ctx) {
    error = resource_intf_->SetMaxMixerStages(display_comp_ctx->display_resource_ctx,
                                              max_mixer_stages);
  }

  return error;
}

void CompManager::ControlPartialUpdate(Handle display_ctx, bool enable) {
  SCOPE_LOCK(locker_);

  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  display_comp_ctx->pu_constraints.enable = enable;
}

DisplayError CompManager::ValidateScaling(const LayerRect &crop, const LayerRect &dst,
                                          bool rotate90) {
  BufferLayout layout = Debug::IsUbwcTiledFrameBuffer() ? kUBWC : kLinear;
  return resource_intf_->ValidateScaling(crop, dst, rotate90, layout, true);
}

DisplayError CompManager::ValidateAndSetCursorPosition(Handle display_ctx, HWLayers *hw_layers,
                                                 int x, int y) {
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  Handle &display_resource_ctx = display_comp_ctx->display_resource_ctx;
  return resource_intf_->ValidateAndSetCursorPosition(display_resource_ctx, hw_layers, x, y,
                                                      &display_comp_ctx->fb_config);
}

DisplayError CompManager::SetMaxBandwidthMode(HWBwModes mode) {
  SCOPE_LOCK(locker_);
  if (mode >= kBwModeMax) {
    return kErrorNotSupported;
  }

  return resource_intf_->SetMaxBandwidthMode(mode);
}

DisplayError CompManager::GetScaleLutConfig(HWScaleLutInfo *lut_info) {
  return resource_intf_->GetScaleLutConfig(lut_info);
}

DisplayError CompManager::SetDetailEnhancerData(Handle display_ctx,
                                                const DisplayDetailEnhancerData &de_data) {
  SCOPE_LOCK(locker_);

  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  if (!display_comp_ctx->dest_scaler_blocks_used) {
    return kErrorResources;
  }

  return resource_intf_->SetDetailEnhancerData(display_comp_ctx->display_resource_ctx, de_data);
}

DisplayError CompManager::SetCompositionState(Handle display_ctx,
                                              LayerComposition composition_type, bool enable) {
  SCOPE_LOCK(locker_);

  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  return display_comp_ctx->strategy->SetCompositionState(composition_type, enable);
}

DisplayError CompManager::ControlDpps(bool enable) {
  // DPPS feature and HDR using SSPP tone mapping can co-exist
  // DPPS feature and HDR using DSPP tone mapping are mutually exclusive
  if (dpps_ctrl_intf_ && hw_res_info_.src_tone_map.none()) {
    int err = 0;
    if (enable) {
      err = dpps_ctrl_intf_->On();
    } else {
      err = dpps_ctrl_intf_->Off();
    }
    if (err) {
      return kErrorUndefined;
    }
  }

  return kErrorNone;
}

bool CompManager::SetDisplayState(Handle display_ctx, DisplayState state,
                                  const shared_ptr<Fence> &sync_handle) {
  DisplayCompositionContext *display_comp_ctx =
      reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  resource_intf_->Perform(ResourceInterface::kCmdSetDisplayState,
                          display_comp_ctx->display_resource_ctx, state);

  switch (state) {
  case kStateOff:
    Purge(display_ctx);
    powered_on_displays_.erase(display_comp_ctx->display_id);
    break;

  case kStateOn:
  case kStateDoze:
    resource_intf_->Perform(ResourceInterface::kCmdDedicatePipes,
                            display_comp_ctx->display_resource_ctx);
    powered_on_displays_.insert(display_comp_ctx->display_id);
    break;

  case kStateDozeSuspend:
    powered_on_displays_.erase(display_comp_ctx->display_id);
    break;

  default:
    break;
  }

  bool inactive = (state == kStateOff) || (state == kStateDozeSuspend);
  UpdateStrategyConstraints(display_comp_ctx->is_primary_panel, inactive);

  resource_intf_->UpdateSyncHandle(display_comp_ctx->display_resource_ctx, sync_handle);

  return true;
}

DisplayError CompManager::SetColorModesInfo(Handle display_ctx,
                                            const std::vector<PrimariesTransfer> &colormodes_cs) {
  DisplayCompositionContext *display_comp_ctx =
      reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  display_comp_ctx->strategy->SetColorModesInfo(colormodes_cs);

  return kErrorNone;
}

std::string CompManager::StringDisplayList(const std::set<int32_t> &displays) {
  std::string displays_str;
  for (auto disps : displays) {
    if (displays_str.empty()) {
      displays_str = std::to_string(disps);
    } else {
      displays_str += ", " + std::to_string(disps);
    }
  }
  return displays_str;
}

DisplayError CompManager::SetBlendSpace(Handle display_ctx, const PrimariesTransfer &blend_space) {
  DisplayCompositionContext *display_comp_ctx =
      reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  display_comp_ctx->strategy->SetBlendSpace(blend_space);

  return kErrorNone;
}

void CompManager::HandleSecureEvent(Handle display_ctx, SecureEvent secure_event) {
  DisplayCompositionContext *display_comp_ctx =
                             reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  // Disable rotator for non secure layers at the end of secure display session, because scm call
  // has been made to end secure display session during the display commit. Since then access to
  // non secure memory is unavailable. So this results in smmu page fault when rotator tries to
  // access the non secure memory.
  if (secure_event == kSecureDisplayEnd) {
    resource_intf_->Perform(ResourceInterface::kCmdDisableRotatorOneFrame,
                            display_comp_ctx->display_resource_ctx);
  }
}

void CompManager::UpdateStrategyConstraints(bool is_primary, bool disabled) {
  if (!is_primary) {
    return;
  }

  // Allow builtin display to use all pipes when primary is suspended.
  // Restore it back to 2 after primary poweron.
  max_sde_builtin_layers_ = (disabled && (powered_on_displays_.size() <= 1)) ? kMaxSDELayers : 2;
}

bool CompManager::CanSkipValidate(Handle display_ctx, bool *needs_buffer_swap) {
  DisplayCompositionContext *display_comp_ctx =
      reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  return display_comp_ctx->strategy->CanSkipValidate(needs_buffer_swap);
}

bool CompManager::CheckResourceState(Handle display_ctx) {
  SCOPE_LOCK(locker_);
  DisplayCompositionContext *display_comp_ctx =
      reinterpret_cast<DisplayCompositionContext *>(display_ctx);
  bool res_wait_needed = false;

  resource_intf_->Perform(ResourceInterface::kCmdGetResourceStatus,
                          display_comp_ctx->display_resource_ctx, &res_wait_needed);
  return res_wait_needed;
}

bool CompManager::IsRotatorSupportedFormat(LayerBufferFormat format) {
  if (resource_intf_) {
    return resource_intf_->IsRotatorSupportedFormat(format);
  }

  return false;
}

DisplayError CompManager::SwapBuffers(Handle display_ctx) {
  SCOPE_LOCK(locker_);
  DisplayCompositionContext *display_comp_ctx =
      reinterpret_cast<DisplayCompositionContext *>(display_ctx);

  return display_comp_ctx->strategy->SwapBuffers();
}

}  // namespace sdm
