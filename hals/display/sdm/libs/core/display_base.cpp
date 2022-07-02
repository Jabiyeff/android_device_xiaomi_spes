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

#include <stdio.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/formats.h>
#include <utils/rect.h>
#include <utils/utils.h>

#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

#include "display_base.h"
#include "hw_info_interface.h"

#define __CLASS__ "DisplayBase"

namespace sdm {

bool DisplayBase::display_power_reset_pending_ = false;
Locker DisplayBase::display_power_reset_lock_;

static ColorPrimaries GetColorPrimariesFromAttribute(const std::string &gamut) {
  if (gamut.find(kDisplayP3) != std::string::npos || gamut.find(kDcip3) != std::string::npos) {
    return ColorPrimaries_DCIP3;
  } else if (gamut.find(kHdr) != std::string::npos || gamut.find("bt2020") != std::string::npos ||
             gamut.find("BT2020") != std::string::npos) {
    // BT2020 is hdr, but the dynamicrange of kHdr means its BT2020
    return ColorPrimaries_BT2020;
  } else if (gamut.find(kSrgb) != std::string::npos) {
    return ColorPrimaries_BT709_5;
  } else if (gamut.find(kNative) != std::string::npos) {
    DLOGW("Native Gamut found, returning default: sRGB");
    return ColorPrimaries_BT709_5;
  }

  return ColorPrimaries_BT709_5;
}

// TODO(user): Have a single structure handle carries all the interface pointers and variables.
DisplayBase::DisplayBase(DisplayType display_type, DisplayEventHandler *event_handler,
                         HWDeviceType hw_device_type, BufferAllocator *buffer_allocator,
                         CompManager *comp_manager, HWInfoInterface *hw_info_intf)
  : display_type_(display_type), event_handler_(event_handler), hw_device_type_(hw_device_type),
    buffer_allocator_(buffer_allocator), comp_manager_(comp_manager), hw_info_intf_(hw_info_intf) {
}

DisplayBase::DisplayBase(int32_t display_id, DisplayType display_type,
                         DisplayEventHandler *event_handler, HWDeviceType hw_device_type,
                         BufferAllocator *buffer_allocator, CompManager *comp_manager,
                         HWInfoInterface *hw_info_intf)
  : display_id_(display_id),
    display_type_(display_type),
    event_handler_(event_handler),
    hw_device_type_(hw_device_type),
    buffer_allocator_(buffer_allocator),
    comp_manager_(comp_manager),
    hw_info_intf_(hw_info_intf) {}

DisplayError DisplayBase::Init() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  hw_panel_info_ = HWPanelInfo();
  hw_intf_->GetHWPanelInfo(&hw_panel_info_);
  if (hw_info_intf_) {
    hw_info_intf_->GetHWResourceInfo(&hw_resource_info_);
  }
  auto max_mixer_stages = hw_resource_info_.num_blending_stages;
  int property_value = Debug::GetMaxPipesPerMixer(display_type_);

  uint32_t active_index = 0;
  int drop_vsync = 0;
  hw_intf_->GetActiveConfig(&active_index);
  hw_intf_->GetDisplayAttributes(active_index, &display_attributes_);
  fb_config_ = display_attributes_;

  error = Debug::GetMixerResolution(&mixer_attributes_.width, &mixer_attributes_.height);
  if (error == kErrorNone) {
    if (hw_intf_->SetMixerAttributes(mixer_attributes_) == kErrorNone) {
      custom_mixer_resolution_ = true;
    }
  }

  error = hw_intf_->GetMixerAttributes(&mixer_attributes_);
  if (error != kErrorNone) {
    return error;
  }

  // Override x_pixels and y_pixels of frame buffer with mixer width and height
  fb_config_.x_pixels = mixer_attributes_.width;
  fb_config_.y_pixels = mixer_attributes_.height;

  if (IsPrimaryDisplay()) {
    HWScaleLutInfo lut_info = {};
    error = comp_manager_->GetScaleLutConfig(&lut_info);
    if (error == kErrorNone) {
      error = hw_intf_->SetScaleLutConfig(&lut_info);
      if (error != kErrorNone) {
        goto CleanupOnError;
      }
    }
  }

  // ColorManager supported for built-in display.
  if (kBuiltIn == display_type_) {
    DppsControlInterface *dpps_intf = comp_manager_->GetDppsControlIntf();
    color_mgr_ = ColorManagerProxy::CreateColorManagerProxy(display_type_, hw_intf_,
                                                            display_attributes_, hw_panel_info_,
                                                            dpps_intf);

    if (color_mgr_) {
      if (InitializeColorModes() != kErrorNone) {
        DLOGW("InitColorModes failed for display %d-%d", display_id_, display_type_);
      }
      color_mgr_->ColorMgrCombineColorModes();
    } else {
      DLOGW("Unable to create ColorManagerProxy for display %d-%d", display_id_, display_type_);
    }
  }

  error = comp_manager_->RegisterDisplay(display_id_, display_type_, display_attributes_,
                                         hw_panel_info_, mixer_attributes_, fb_config_,
                                         &display_comp_ctx_, &(default_clock_hz_));
  if (error != kErrorNone) {
    DLOGW("Display %d comp manager registration failed!", display_id_);
    goto CleanupOnError;
  }
  cached_qos_data_.clock_hz = default_clock_hz_;

  if (color_modes_cs_.size() > 0) {
    error = comp_manager_->SetColorModesInfo(display_comp_ctx_, color_modes_cs_);
    if (error) {
      DLOGW("SetColorModesInfo failed on display = %d", display_type_);
    }
  }

  if (property_value >= 0) {
    max_mixer_stages = std::min(UINT32(property_value), hw_resource_info_.num_blending_stages);
  }
  DisplayBase::SetMaxMixerStages(max_mixer_stages);

  // TODO(user): Temporary changes, to be removed when DRM driver supports
  // Partial update with Destination scaler enabled.
  SetPUonDestScaler();

  Debug::GetProperty(DISABLE_HW_RECOVERY_DUMP_PROP, &disable_hw_recovery_dump_);
  DLOGI("disable_hw_recovery_dump_ set to %d", disable_hw_recovery_dump_);

  Debug::Get()->GetProperty(DROP_SKEWED_VSYNC, &drop_vsync);
  drop_skewed_vsync_ = (drop_vsync == 1);

  return kErrorNone;

CleanupOnError:
  ClearColorInfo();
  if (display_comp_ctx_) {
    comp_manager_->UnregisterDisplay(display_comp_ctx_);
  }

  return error;
}

DisplayError DisplayBase::Deinit() {
  {  // Scope for lock
    lock_guard<recursive_mutex> obj(recursive_mutex_);
    ClearColorInfo();
    comp_manager_->UnregisterDisplay(display_comp_ctx_);
    if (IsPrimaryDisplay()) {
      hw_intf_->UnsetScaleLutConfig();
    }
  }
  HWEventsInterface::Destroy(hw_events_intf_);
  HWInterface::Destroy(hw_intf_);
  if (rc_panel_feature_init_) {
    rc_core_->Deinit();
    rc_panel_feature_init_ =  false;
  }
  return kErrorNone;
}

// Query the dspp capabilities and enable the RC feature.
DisplayError DisplayBase::SetupRC() {
  RCInputConfig input_cfg = {};
  input_cfg.display_id = display_id_;
  input_cfg.display_type = display_type_;
  input_cfg.display_xres = display_attributes_.x_pixels;
  input_cfg.display_yres = display_attributes_.y_pixels;
  input_cfg.max_mem_size = hw_resource_info_.rc_total_mem_size;
  rc_core_ = pf_factory_->CreateRCIntf(input_cfg, prop_intf_);
  GenericPayload dummy;
  int err = 0;
  if (!rc_core_) {
    DLOGE("Failed to create RC Intf");
    return kErrorUndefined;
  }
  err = rc_core_->GetParameter(kRCFeatureQueryDspp, &dummy);
  if (!err) {
    // Since the query succeeded, this display has a DSPP.
    if (rc_core_->Init() != 0) {
      DLOGW("Failed to initialize RC");
      return kErrorNotSupported;
    }
  } else {
    DLOGW("RC HW block is not present for display %d-%d.", display_id_, display_type_);
    return kErrorResources;
  }

  return kErrorNone;
}

DisplayError DisplayBase::BuildLayerStackStats(LayerStack *layer_stack) {
  std::vector<Layer *> &layers = layer_stack->layers;
  HWLayersInfo &hw_layers_info = hw_layers_.info;
  hw_layers_info.app_layer_count = 0;

  hw_layers_info.stack = layer_stack;

  for (auto &layer : layers) {
    if (layer->buffer_map == nullptr) {
      layer->buffer_map = std::make_shared<LayerBufferMap>();
    }
    if (layer->composition == kCompositionGPUTarget) {
      hw_layers_info.gpu_target_index = hw_layers_info.app_layer_count;
    } else if (layer->composition == kCompositionStitchTarget) {
      hw_layers_info.stitch_target_index = hw_layers_info.gpu_target_index + 1;
      break;
    } else {
      hw_layers_info.app_layer_count++;
    }
    if (IsWideColor(layer->input_buffer.color_metadata.colorPrimaries)) {
      hw_layers_info.wide_color_primaries.push_back(
          layer->input_buffer.color_metadata.colorPrimaries);
    }
    if (layer->flags.is_game) {
      hw_layers_info.game_present = true;
    }
  }

  hw_layers_info.stitch_target_index = hw_layers_info.gpu_target_index + 1;
  DLOGD_IF(kTagDisplay, "LayerStack layer_count: %zu, app_layer_count: %d, "
                        "gpu_target_index: %d, stitch_index: %d game_present: %d, display: %d-%d",
                        layers.size(), hw_layers_info.app_layer_count,
                        hw_layers_info.gpu_target_index, hw_layers_info.stitch_target_index,
                        hw_layers_info.game_present, display_id_, display_type_);

  if (!hw_layers_info.app_layer_count) {
    DLOGW("Layer count is zero");
    return kErrorNoAppLayers;
  }

  if (hw_layers_info.gpu_target_index) {
    return ValidateGPUTargetParams();
  }

  return kErrorNone;
}

DisplayError DisplayBase::ValidateGPUTargetParams() {
  HWLayersInfo &hw_layers_info = hw_layers_.info;
  Layer *gpu_target_layer = hw_layers_info.stack->layers.at(hw_layers_info.gpu_target_index);

  if (!IsValid(gpu_target_layer->src_rect)) {
    DLOGE("Invalid src rect for GPU target layer");
    return kErrorParameters;
  }

  if (!IsValid(gpu_target_layer->dst_rect)) {
    DLOGE("Invalid dst rect for GPU target layer");
    return kErrorParameters;
  }

  float layer_mixer_width = FLOAT(mixer_attributes_.width);
  float layer_mixer_height = FLOAT(mixer_attributes_.height);
  float fb_width = FLOAT(fb_config_.x_pixels);
  float fb_height = FLOAT(fb_config_.y_pixels);
  LayerRect src_domain = (LayerRect){0.0f, 0.0f, fb_width, fb_height};
  LayerRect dst_domain = (LayerRect){0.0f, 0.0f, layer_mixer_width, layer_mixer_height};
  LayerRect out_rect = gpu_target_layer->dst_rect;

  MapRect(src_domain, dst_domain, gpu_target_layer->dst_rect, &out_rect);
  Normalize(1, 1, &out_rect);

  auto gpu_target_layer_dst_xpixels = out_rect.right - out_rect.left;
  auto gpu_target_layer_dst_ypixels = out_rect.bottom - out_rect.top;

  if (gpu_target_layer_dst_xpixels > mixer_attributes_.width ||
    gpu_target_layer_dst_ypixels > mixer_attributes_.height) {
    DLOGE("GPU target layer dst rect is not with in limits gpu wxh %fx%f, mixer wxh %dx%d",
                  gpu_target_layer_dst_xpixels, gpu_target_layer_dst_ypixels,
                  mixer_attributes_.width, mixer_attributes_.height);
    return kErrorParameters;
  }

  return kErrorNone;
}

DisplayError DisplayBase::Prepare(LayerStack *layer_stack) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  needs_validate_ = true;

  DTRACE_SCOPED();
  // Allow prepare as pending doze/pending_power_on is handled as a part of draw cycle
  if (!active_ && !pending_doze_ && !pending_power_on_) {
    return kErrorPermission;
  }

  if (!layer_stack) {
    return kErrorParameters;
  }

  DLOGI_IF(kTagDisplay, "Entering Prepare for display: %d-%d", display_id_, display_type_);
  error = BuildLayerStackStats(layer_stack);
  if (error != kErrorNone) {
    return error;
  }

  if (!rc_core_ && !first_cycle_ && rc_enable_prop_ && pf_factory_ && prop_intf_) {
    error = SetupRC();
    if (error == kErrorNone) {
      rc_panel_feature_init_ = true;
    } else {
      DLOGW("RC feature not supported");
    }
  }
  if (rc_panel_feature_init_) {
    SetRCData(layer_stack);
  }


  if (color_mgr_ && color_mgr_->NeedsPartialUpdateDisable()) {
    DisablePartialUpdateOneFrame();
  }
  // TODO(user): Temporary changes, to be removed when DRM driver supports
  // Partial update with Destination scaler enabled.
  if (!partial_update_control_ || disable_pu_one_frame_ ||
      disable_pu_on_dest_scaler_) {
    comp_manager_->ControlPartialUpdate(display_comp_ctx_, false /* enable */);
    disable_pu_one_frame_ = false;
  }

  hw_layers_.updates_mask.set(kUpdateResources);
  comp_manager_->GenerateROI(display_comp_ctx_, &hw_layers_);

  comp_manager_->PrePrepare(display_comp_ctx_, &hw_layers_);

  while (true) {
    error = comp_manager_->Prepare(display_comp_ctx_, &hw_layers_);
    if (error != kErrorNone) {
      break;
    }

    if (layer_stack->flags.fast_path && hw_layers_.info.fast_path_composition) {
      // In Fast Path, driver validation happens in COMMIT Phase.
      DLOGI_IF(kTagDisplay, "Draw cycle qualifies for Fast Path!");
      needs_validate_ = false;
      break;
    }

    error = hw_intf_->Validate(&hw_layers_);
    if (error == kErrorNone) {
      // Strategy is successful now, wait for Commit().
      needs_validate_ = false;
      break;
    }
    if (error == kErrorShutDown) {
      comp_manager_->PostPrepare(display_comp_ctx_, &hw_layers_);
      return error;
    }
  }

  if (color_mgr_)
    color_mgr_->Validate(&hw_layers_);

  comp_manager_->PostPrepare(display_comp_ctx_, &hw_layers_);

  DLOGI_IF(kTagDisplay, "Exiting Prepare for display type : %d error: %d", display_type_, error);
  return error;
}

// Send layer stack to RC core to generate and configure the mask on HW.
void DisplayBase::SetRCData(LayerStack *layer_stack) {
  int ret = -1;
  HWLayersInfo &hw_layers_info = hw_layers_.info;
  DLOGI_IF(kTagDisplay, "Display resolution: %dx%d", display_attributes_.x_pixels,
           display_attributes_.y_pixels);
  if (rc_cached_res_width_ != display_attributes_.x_pixels) {
    GenericPayload in;
    uint32_t *display_xres = nullptr;
    ret = in.CreatePayload<uint32_t>(display_xres);
    if (ret) {
      DLOGE("failed to create the payload. Error:%d", ret);
      return;
    }
    *display_xres = rc_cached_res_width_ = display_attributes_.x_pixels;
    ret = rc_core_->SetParameter(kRCFeatureDisplayXRes, in);
    if (ret) {
      DLOGE("failed to set display X resolution. Error:%d", ret);
      return;
    }
  }

  if (rc_cached_res_height_ != display_attributes_.y_pixels) {
    GenericPayload in;
    uint32_t *display_yres = nullptr;
    ret = in.CreatePayload<uint32_t>(display_yres);
    if (ret) {
      DLOGE("failed to create the payload. Error:%d", ret);
      return;
    }
    *display_yres = rc_cached_res_height_ = display_attributes_.y_pixels;
    ret = rc_core_->SetParameter(kRCFeatureDisplayYRes, in);
    if (ret) {
      DLOGE("failed to set display Y resolution. Error:%d", ret);
      return;
    }
  }

  GenericPayload in;
  LayerStack **layer_stack_ptr = nullptr;
  ret = in.CreatePayload<LayerStack *>(layer_stack_ptr);
  if (ret) {
    DLOGE("failed to create the payload. Error:%d", ret);
    return;
  }
  *layer_stack_ptr = layer_stack;
  GenericPayload out;
  RCOutputConfig *rc_out_config = nullptr;
  ret = out.CreatePayload<RCOutputConfig>(rc_out_config);
  if (ret) {
    DLOGE("failed to create the payload. Error:%d", ret);
    return;
  }
  ret = rc_core_->ProcessOps(kRCFeaturePrepare, in, &out);
  if (!ret) {
    DLOGD_IF(kTagDisplay, "RC top_height = %d, RC bot_height = %d", rc_out_config->top_height,
             rc_out_config->bottom_height);
    if (rc_out_config->rc_needs_full_roi) {
      DisablePartialUpdateOneFrame();
    }
    hw_layers_info.rc_config = true;
    hw_layers_info.rc_layers_info.top_width = rc_out_config->top_width;
    hw_layers_info.rc_layers_info.top_height = rc_out_config->top_height;
    hw_layers_info.rc_layers_info.bottom_width = rc_out_config->bottom_width;
    hw_layers_info.rc_layers_info.bottom_height = rc_out_config->bottom_height;
  }
}

DisplayError DisplayBase::Commit(LayerStack *layer_stack) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;

  if (rc_panel_feature_init_) {
    GenericPayload in, out;
    RCMaskCfgState *mask_status = nullptr;
    int ret = -1;
    ret = out.CreatePayload<RCMaskCfgState>(mask_status);
    if (ret) {
      DLOGE("failed to create the payload. Error:%d", ret);
      return kErrorUndefined;
    }
    ret = rc_core_->ProcessOps(kRCFeatureCommit, in, &out);
    if (ret) {
     // If RC commit failed, fall back to default (GPU/SDE pipes) drawing of "handled" mask layers.
     DLOGW("Failed to set the data on driver for display: %d-%d, Error: %d, status: %d",
           display_id_, display_type_, ret, (*mask_status).rc_mask_state);
      if ((*mask_status).rc_mask_state == kStatusRcMaskStackHandled) {
        needs_validate_ = true;
        DLOGW("Need to call Corresponding prepare to handle the mask layers %d %d.",
              display_id_, display_type_);
        for (auto &layer : layer_stack->layers) {
          if (layer->input_buffer.flags.mask_layer) {
            layer->request.flags.rc = false;
          }
        }
        return kErrorNotValidated;
      }
    } else {
      DLOGI_IF(kTagDisplay, "Status of RC mask data: %d.", (*mask_status).rc_mask_state);
      if ((*mask_status).rc_mask_state == kStatusRcMaskStackDirty) {
        DisablePartialUpdateOneFrame();
        needs_validate_ = true;
        DLOGI_IF(kTagDisplay, "Mask is ready for display %d-%d, call Corresponding Prepare()",
                 display_id_, display_type_);
        return kErrorNotValidated;
      }
    }
  }

  // Allow commit as pending doze/pending_power_on is handled as a part of draw cycle
  if (!active_ && !pending_doze_ && !pending_power_on_) {
    needs_validate_ = true;
    return kErrorPermission;
  }

  if (!layer_stack) {
    return kErrorParameters;
  }

  if (needs_validate_) {
    DLOGE("Commit: Corresponding Prepare() is not called for display %d-%d", display_id_,
          display_type_);
    return kErrorNotValidated;
  }

  DLOGI_IF(kTagDisplay, "Entering commit for display: %d-%d", display_id_, display_type_);
  CommitLayerParams(layer_stack);

  error = comp_manager_->Commit(display_comp_ctx_, &hw_layers_);
  if (error != kErrorNone) {
    return error;
  }

  // check if feature list cache is dirty and pending.
  // If dirty, need program to hardware blocks.
  if (color_mgr_)
    error = color_mgr_->Commit();
  if (error != kErrorNone) {  // won't affect this execution path.
    DLOGW("ColorManager::Commit(...) isn't working");
  }

  error = hw_intf_->Commit(&hw_layers_);
  if (error != kErrorNone) {
    if (layer_stack->flags.fast_path && hw_layers_.info.fast_path_composition) {
      // If COMMIT fails on the Fast Path, set Safe Mode.
      DLOGE("COMMIT failed in Fast Path, set Safe Mode!");
      comp_manager_->SetSafeMode(true);
      error = kErrorNotValidated;
    }
    return error;
  }

  PostCommitLayerParams(layer_stack);

  if (partial_update_control_) {
    comp_manager_->ControlPartialUpdate(display_comp_ctx_, true /* enable */);
  }

  error = comp_manager_->PostCommit(display_comp_ctx_, &hw_layers_);
  if (error != kErrorNone) {
    return error;
  }

  // Stop dropping vsync when first commit is received after idle fallback.
  drop_hw_vsync_ = false;

  // Reset pending power state if any after the commit
  error = HandlePendingPowerState(layer_stack->retire_fence);
  if (error != kErrorNone) {
    return error;
  }

  // Handle pending vsync enable if any after the commit
  error = HandlePendingVSyncEnable(layer_stack->retire_fence);
  if (error != kErrorNone) {
    return error;
  }

  comp_manager_->SetSafeMode(false);

  DLOGI_IF(kTagDisplay, "Exiting commit for display: %d-%d", display_id_, display_type_);

  return error;
}

DisplayError DisplayBase::Flush(LayerStack *layer_stack) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;

  if (!active_) {
    return kErrorPermission;
  }
  hw_layers_.info.hw_layers.clear();
  hw_layers_.info.stack = layer_stack;
  error = hw_intf_->Flush(&hw_layers_);
  if (error == kErrorNone) {
    comp_manager_->Purge(display_comp_ctx_);
    needs_validate_ = true;
  } else {
    DLOGW("Unable to flush display %d-%d", display_id_, display_type_);
  }

  return error;
}

DisplayError DisplayBase::GetDisplayState(DisplayState *state) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!state) {
    return kErrorParameters;
  }

  *state = state_;
  return kErrorNone;
}

DisplayError DisplayBase::GetNumVariableInfoConfigs(uint32_t *count) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  return hw_intf_->GetNumDisplayAttributes(count);
}

DisplayError DisplayBase::GetConfig(uint32_t index, DisplayConfigVariableInfo *variable_info) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  HWDisplayAttributes attrib;
  if (hw_intf_->GetDisplayAttributes(index, &attrib) == kErrorNone) {
    *variable_info = attrib;
    if (custom_mixer_resolution_) {
      variable_info->x_pixels = fb_config_.x_pixels;
      variable_info->y_pixels = fb_config_.y_pixels;
    }
    return kErrorNone;
  }

  return kErrorNotSupported;
}

DisplayError DisplayBase::GetConfig(DisplayConfigFixedInfo *fixed_info) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  fixed_info->is_cmdmode = (hw_panel_info_.mode == kModeCommand);

  HWResourceInfo hw_resource_info = HWResourceInfo();
  hw_info_intf_->GetHWResourceInfo(&hw_resource_info);
  bool hdr_supported = hw_resource_info.has_hdr;
  bool hdr_plus_supported = false;
  HWDisplayInterfaceInfo hw_disp_info = {};
  hw_info_intf_->GetFirstDisplayInterfaceType(&hw_disp_info);
  if (hw_disp_info.type == kHDMI) {
    hdr_supported = (hdr_supported && hw_panel_info_.hdr_enabled);
  }

  // For non-builtin displays, check panel capability for HDR10+
  if (hdr_supported && hw_panel_info_.hdr_plus_enabled) {
    hdr_plus_supported = true;
  }

  fixed_info->hdr_supported = hdr_supported;
  fixed_info->hdr_plus_supported = hdr_plus_supported;
  // Populate luminance values only if hdr will be supported on that display
  fixed_info->max_luminance = fixed_info->hdr_supported ? hw_panel_info_.peak_luminance: 0;
  fixed_info->average_luminance = fixed_info->hdr_supported ? hw_panel_info_.average_luminance : 0;
  fixed_info->min_luminance = fixed_info->hdr_supported ?  hw_panel_info_.blackness_level: 0;
  fixed_info->hdr_eotf = hw_panel_info_.hdr_eotf;
  fixed_info->hdr_metadata_type_one = hw_panel_info_.hdr_metadata_type_one;
  fixed_info->partial_update = hw_panel_info_.partial_update;
  fixed_info->readback_supported = hw_resource_info.has_concurrent_writeback;

  return kErrorNone;
}

DisplayError DisplayBase::GetActiveConfig(uint32_t *index) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  return hw_intf_->GetActiveConfig(index);
}

DisplayError DisplayBase::GetVSyncState(bool *enabled) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!enabled) {
    return kErrorParameters;
  }

  *enabled = vsync_enable_;

  return kErrorNone;
}

DisplayError DisplayBase::SetDisplayState(DisplayState state, bool teardown,
                                          shared_ptr<Fence> *release_fence) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  bool active = false;

  DLOGI("Set state = %d, display %d-%d, teardown = %d", state, display_id_,
        display_type_, teardown);

  if (state == state_) {
    DLOGI("Same state transition is requested.");
    return kErrorNone;
  }

  // If vsync is enabled, disable vsync before power off/Doze suspend
  if (vsync_enable_ && (state == kStateOff || state == kStateDozeSuspend)) {
    error = SetVSyncState(false /* enable */);
    if (error == kErrorNone) {
      vsync_enable_pending_ = true;
    }
  }

  switch (state) {
  case kStateOff:
    hw_layers_.info.hw_layers.clear();
    error = hw_intf_->Flush(&hw_layers_);
    if (error == kErrorNone) {
      error = hw_intf_->PowerOff(teardown);
    }
    cached_qos_data_ = {};
    cached_qos_data_.clock_hz = default_clock_hz_;
    break;

  case kStateOn:
    error = hw_intf_->PowerOn(cached_qos_data_, release_fence);
    if (error != kErrorNone) {
      if (error == kErrorDeferred) {
        pending_power_on_ = true;
        error = kErrorNone;
      } else {
        return error;
      }
    }

    error = comp_manager_->ReconfigureDisplay(display_comp_ctx_, display_attributes_,
                                              hw_panel_info_, mixer_attributes_, fb_config_,
                                              &(default_clock_hz_));
    if (error != kErrorNone) {
      return error;
    }
    cached_qos_data_.clock_hz = default_clock_hz_;

    active = true;
    break;

  case kStateDoze:
    error = hw_intf_->Doze(cached_qos_data_, release_fence);
    if (error != kErrorNone) {
      if (error == kErrorDeferred) {
        pending_doze_ = true;
        error = kErrorNone;
      } else {
        return error;
      }
    }
    active = true;
    break;

  case kStateDozeSuspend:
    error = hw_intf_->DozeSuspend(cached_qos_data_, release_fence);
    if (display_type_ != kBuiltIn) {
      active = true;
    }
    break;

  case kStateStandby:
    error = hw_intf_->Standby();
    break;

  default:
    DLOGE("Spurious state = %d transition requested.", state);
    return kErrorParameters;
  }

  error = ReconfigureDisplay();
  if (error != kErrorNone) {
    return error;
  }

  DisablePartialUpdateOneFrame();

  if (error == kErrorNone) {
    // If previously requested doze state is still pending reset it on any new display state request
    // and handle the new request.
    if (state != kStateDoze) {
      pending_doze_ = false;
    }

    if (!pending_doze_ && !pending_power_on_) {
      active_ = active;
      state_ = state;
    }
    comp_manager_->SetDisplayState(display_comp_ctx_, state,
                                   release_fence ? *release_fence : nullptr);

    // If previously requested power on state is still pending reset it on any new display state
    // request and handle the new request.
    if (state != kStateOn) {
      pending_power_on_ = false;
    }
  }

  // Handle vsync pending on resume, Since the power on commit is synchronous we pass -1 as retire
  // fence otherwise pass valid retire fence
  if (state_ == kStateOn) {
    return HandlePendingVSyncEnable(nullptr /* retire fence */);
  }

  return error;
}

DisplayError DisplayBase::SetActiveConfig(uint32_t index) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  uint32_t active_index = 0;

  hw_intf_->GetActiveConfig(&active_index);

  if (active_index == index) {
    return kErrorNone;
  }

  // Reject active config changes if qsync is in use.
  if (needs_avr_update_ || qsync_mode_ != kQSyncModeNone) {
    DLOGE("Failed: needs_avr_update_: %d, qsync_mode_: %d", needs_avr_update_, qsync_mode_);
    return kErrorNotSupported;
  }

  error = hw_intf_->SetDisplayAttributes(index);
  if (error != kErrorNone) {
    return error;
  }

  return ReconfigureDisplay();
}

DisplayError DisplayBase::SetMaxMixerStages(uint32_t max_mixer_stages) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;

  error = comp_manager_->SetMaxMixerStages(display_comp_ctx_, max_mixer_stages);

  if (error == kErrorNone) {
    max_mixer_stages_ = max_mixer_stages;
  }

  return error;
}

std::string DisplayBase::Dump() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  HWDisplayAttributes attrib;
  uint32_t active_index = 0;
  uint32_t num_modes = 0;
  std::ostringstream os;

  hw_intf_->GetNumDisplayAttributes(&num_modes);
  hw_intf_->GetActiveConfig(&active_index);
  hw_intf_->GetDisplayAttributes(active_index, &attrib);

  os << "device type:" << display_type_;
  os << "\nstate: " << state_ << " vsync on: " << vsync_enable_
     << " max. mixer stages: " << max_mixer_stages_;
  os << "\nnum configs: " << num_modes << " active config index: " << active_index;
  os << "\nDisplay Attributes:";
  os << "\n Mode:" << (hw_panel_info_.mode == kModeVideo ? "Video" : "Command");
  os << std::boolalpha;
  os << " Primary:" << hw_panel_info_.is_primary_panel;
  os << " DynFPS:" << hw_panel_info_.dynamic_fps;
  os << "\n HDR Panel:" << hw_panel_info_.hdr_enabled;
  os << " QSync:" << hw_panel_info_.qsync_support;
  os << " DynBitclk:" << hw_panel_info_.dyn_bitclk_support;
  os << "\n Left Split:" << hw_panel_info_.split_info.left_split
     << " Right Split:" << hw_panel_info_.split_info.right_split;
  os << "\n PartialUpdate:" << hw_panel_info_.partial_update;
  if (hw_panel_info_.partial_update) {
    os << "\n ROI Min w:" << hw_panel_info_.min_roi_width;
    os << " Min h:" << hw_panel_info_.min_roi_height;
    os << " NeedsMerge: " << hw_panel_info_.needs_roi_merge;
    os << " Alignment: l:" << hw_panel_info_.left_align << " w:" << hw_panel_info_.width_align;
    os << " t:" << hw_panel_info_.top_align << " b:" << hw_panel_info_.height_align;
  }
  os << "\n FPS min:" << hw_panel_info_.min_fps << " max:" << hw_panel_info_.max_fps
     << " cur:" << display_attributes_.fps;
  os << " TransferTime: " << hw_panel_info_.transfer_time_us << "us";
  os << " MaxBrightness:" << hw_panel_info_.panel_max_brightness;
  os << "\n Display WxH: " << display_attributes_.x_pixels << "x" << display_attributes_.y_pixels;
  os << " MixerWxH: " << mixer_attributes_.width << "x" << mixer_attributes_.height;
  os << " DPI: " << display_attributes_.x_dpi << "x" << display_attributes_.y_dpi;
  os << " LM_Split: " << display_attributes_.is_device_split;
  os << "\n vsync_period " << display_attributes_.vsync_period_ns;
  os << " v_back_porch: " << display_attributes_.v_back_porch;
  os << " v_front_porch: " << display_attributes_.v_front_porch;
  os << " v_pulse_width: " << display_attributes_.v_pulse_width;
  os << "\n v_total: " << display_attributes_.v_total;
  os << " h_total: " << display_attributes_.h_total;
  os << " clk: " << display_attributes_.clock_khz;
  os << " Topology: " << display_attributes_.topology;
  os << " Qsync mode: " << active_qsync_mode_;
  os << std::noboolalpha;

  os << "\nCurrent Color Mode: " << current_color_mode_.c_str();
  os << "\nAvailable Color Modes:\n";
  for (auto it : color_mode_map_) {
    os << "  " << it.first << " " << std::setw(35 - INT(it.first.length())) <<
       it.second->id;
    os << " ";
    for (auto attr_it : color_mode_attr_map_[it.first]) {
      os << std::right << " " << attr_it.first << ": " << attr_it.second;
    }
    os << "\n";
  }

  uint32_t num_hw_layers = 0;
  if (hw_layers_.info.stack) {
    num_hw_layers = UINT32(hw_layers_.info.hw_layers.size());
  }

  if (num_hw_layers == 0) {
    os << "\nNo hardware layers programmed";
    return os.str();
  }

  LayerBuffer *out_buffer = hw_layers_.info.stack->output_buffer;
  if (out_buffer) {
    os << "\n Output buffer res: " << out_buffer->width << "x" << out_buffer->height
       << " format: " << GetFormatString(out_buffer->format);
  }
  HWLayersInfo &layer_info = hw_layers_.info;
  for (uint32_t i = 0; i < layer_info.left_frame_roi.size(); i++) {
    LayerRect &l_roi = layer_info.left_frame_roi.at(i);
    LayerRect &r_roi = layer_info.right_frame_roi.at(i);

    os << "\nROI(LTRB)#" << i << " LEFT(" << INT(l_roi.left) << " " << INT(l_roi.top) << " " <<
      INT(l_roi.right) << " " << INT(l_roi.bottom) << ")";
    if (IsValid(r_roi)) {
    os << " RIGHT(" << INT(r_roi.left) << " " << INT(r_roi.top) << " " << INT(r_roi.right) << " "
      << INT(r_roi.bottom) << ")";
    }
  }

  LayerRect &fb_roi = layer_info.partial_fb_roi;
  if (IsValid(fb_roi)) {
    os << "\nPartial FB ROI(LTRB):(" << INT(fb_roi.left) << " " << INT(fb_roi.top) << " " <<
      INT(fb_roi.right) << " " << INT(fb_roi.bottom) << ")";
  }

  const char *header  = "\n| Idx |   Comp Type   |   Split   | Pipe |    W x H    |          Format          |  Src Rect (L T R B) |  Dst Rect (L T R B) |  Z | Pipe Flags | Deci(HxV) | CS | Rng | Tr |";  //NOLINT
  const char *newline = "\n|-----|---------------|-----------|------|-------------|--------------------------|---------------------|---------------------|----|------------|-----------|----|-----|----|";  //NOLINT
  const char *format  = "\n| %3s | %13s | %9s | %4d | %4d x %4d | %24s | %4d %4d %4d %4d | %4d %4d %4d %4d | %2s | %10s | %9s | %2s | %3s | %2s |";  //NOLINT

  os << "\n";
  os << newline;
  os << header;
  os << newline;

  for (uint32_t i = 0; i < num_hw_layers; i++) {
    uint32_t layer_index = hw_layers_.info.index.at(i);
    // sdm-layer from client layer stack
    Layer *sdm_layer = hw_layers_.info.stack->layers.at(layer_index);
    // hw-layer from hw layers info
    Layer &hw_layer = hw_layers_.info.hw_layers.at(i);
    LayerBuffer *input_buffer = &hw_layer.input_buffer;
    HWLayerConfig &layer_config = hw_layers_.config[i];
    HWRotatorSession &hw_rotator_session = layer_config.hw_rotator_session;

    const char *comp_type = GetName(sdm_layer->composition);
    const char *buffer_format = GetFormatString(input_buffer->format);
    const char *pipe_split[2] = { "Pipe-1", "Pipe-2" };
    const char *rot_pipe[2] = { "Rot-inl-1", "Rot-inl-2" };
    char idx[8];

    snprintf(idx, sizeof(idx), "%d", layer_index);

    for (uint32_t count = 0; count < hw_rotator_session.hw_block_count; count++) {
      char row[1024];
      HWRotateInfo &rotate = hw_rotator_session.hw_rotate_info[count];
      LayerRect &src_roi = rotate.src_roi;
      LayerRect &dst_roi = rotate.dst_roi;
      char rot[12] = { 0 };

      snprintf(rot, sizeof(rot), "Rot-%s-%d", layer_config.use_inline_rot ?
               "inl" : "off", count + 1);

      snprintf(row, sizeof(row), format, idx, comp_type, rot,
               0, input_buffer->width, input_buffer->height, buffer_format,
               INT(src_roi.left), INT(src_roi.top), INT(src_roi.right), INT(src_roi.bottom),
               INT(dst_roi.left), INT(dst_roi.top), INT(dst_roi.right), INT(dst_roi.bottom),
               "-", "-    ", "-    ", "-", "-", "-");
      os << row;
      // print the below only once per layer block, fill with spaces for rest.
      idx[0] = 0;
      comp_type = "";
    }

    if (hw_rotator_session.hw_block_count > 0) {
      input_buffer = &hw_rotator_session.output_buffer;
      buffer_format = GetFormatString(input_buffer->format);
    }

    if (layer_config.use_solidfill_stage) {
      LayerRect src_roi = layer_config.hw_solidfill_stage.roi;
      const char *decimation = "";
      char flags[16] = { 0 };
      char z_order[8] = { 0 };
      const char *color_primary = "";
      const char *range = "";
      const char *transfer = "";
      char row[1024] = { 0 };

      snprintf(z_order, sizeof(z_order), "%d", layer_config.hw_solidfill_stage.z_order);
      snprintf(flags, sizeof(flags), "0x%08x", hw_layer.flags.flags);
      snprintf(row, sizeof(row), format, idx, comp_type, pipe_split[0],
               0, INT(src_roi.right), INT(src_roi.bottom),
               buffer_format, INT(src_roi.left), INT(src_roi.top),
               INT(src_roi.right), INT(src_roi.bottom), INT(src_roi.left),
               INT(src_roi.top), INT(src_roi.right), INT(src_roi.bottom),
               z_order, flags, decimation, color_primary, range, transfer);
      os << row;
      continue;
    }

    for (uint32_t count = 0; count < 2; count++) {
      char decimation[16] = { 0 };
      char flags[16] = { 0 };
      char z_order[8] = { 0 };
      char color_primary[8] = { 0 };
      char range[8] = { 0 };
      char transfer[8] = { 0 };
      bool rot = layer_config.use_inline_rot;

      HWPipeInfo &pipe = (count == 0) ? layer_config.left_pipe : layer_config.right_pipe;

      if (!pipe.valid) {
        continue;
      }

      LayerRect src_roi = pipe.src_roi;
      LayerRect &dst_roi = pipe.dst_roi;

      snprintf(z_order, sizeof(z_order), "%d", pipe.z_order);
      snprintf(flags, sizeof(flags), "0x%08x", pipe.flags);
      snprintf(decimation, sizeof(decimation), "%3d x %3d", pipe.horizontal_decimation,
               pipe.vertical_decimation);
      ColorMetaData &color_metadata = hw_layer.input_buffer.color_metadata;
      snprintf(color_primary, sizeof(color_primary), "%d", color_metadata.colorPrimaries);
      snprintf(range, sizeof(range), "%d", color_metadata.range);
      snprintf(transfer, sizeof(transfer), "%d", color_metadata.transfer);

      char row[1024];
      snprintf(row, sizeof(row), format, idx, comp_type, rot ? rot_pipe[count] : pipe_split[count],
               pipe.pipe_id, input_buffer->width, input_buffer->height,
               buffer_format, INT(src_roi.left), INT(src_roi.top),
               INT(src_roi.right), INT(src_roi.bottom), INT(dst_roi.left),
               INT(dst_roi.top), INT(dst_roi.right), INT(dst_roi.bottom),
               z_order, flags, decimation, color_primary, range, transfer);

      os << row;
      // print the below only once per layer block, fill with spaces for rest.
      idx[0] = 0;
      comp_type = "";
    }
  }

  os << newline << "\n";

  return os.str();
}

const char * DisplayBase::GetName(const LayerComposition &composition) {
  switch (composition) {
  case kCompositionGPU:           return "GPU";
  case kCompositionSDE:           return "SDE";
  case kCompositionCursor:        return "CURSOR";
  case kCompositionStitch:        return "STITCH";
  case kCompositionGPUTarget:     return "GPU_TARGET";
  case kCompositionStitchTarget:  return "STITCH_TARGET";
  default:                        return "UNKNOWN";
  }
}

DisplayError DisplayBase::ColorSVCRequestRoute(const PPDisplayAPIPayload &in_payload,
                                               PPDisplayAPIPayload *out_payload,
                                               PPPendingParams *pending_action) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (color_mgr_)
    return color_mgr_->ColorSVCRequestRoute(in_payload, out_payload, pending_action);
  else
    return kErrorParameters;
}

DisplayError DisplayBase::GetColorModeCount(uint32_t *mode_count) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!mode_count) {
    return kErrorParameters;
  }

  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  DLOGV_IF(kTagQDCM, "Display = %d Number of modes from color manager = %d", display_type_,
           num_color_modes_);

  *mode_count = num_color_modes_;

  return kErrorNone;
}

DisplayError DisplayBase::GetColorModes(uint32_t *mode_count,
                                        std::vector<std::string> *color_modes) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!mode_count || !color_modes) {
    return kErrorParameters;
  }

  if (!color_mgr_) {
    return kErrorNotSupported;
  }
  uint32_t i = 0;
  for (ColorModeAttrMap::iterator it = color_mode_attr_map_.begin();
       ((i < num_color_modes_) && (it != color_mode_attr_map_.end())); i++, it++) {
    DLOGI("ColorMode name = %s", it->first.c_str());
    color_modes->at(i) = it->first.c_str();
  }

  return kErrorNone;
}

DisplayError DisplayBase::GetColorModeAttr(const std::string &color_mode, AttrVal *attr) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!attr) {
    return kErrorParameters;
  }

  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  auto it = color_mode_attr_map_.find(color_mode);
  if (it == color_mode_attr_map_.end()) {
    DLOGI("Mode %s has no attribute", color_mode.c_str());
    return kErrorNotSupported;
  }
  *attr = it->second;

  return kErrorNone;
}

DisplayError DisplayBase::SetColorMode(const std::string &color_mode) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  if (color_mode.empty()) {
    return kErrorParameters;
  }

  DisplayError error = kErrorNone;
  std::string dynamic_range = kSdr, str_render_intent;
  if (IsSupportColorModeAttribute(color_mode)) {
    auto it_mode = color_mode_attr_map_.find(color_mode);
    GetValueOfModeAttribute(it_mode->second, kDynamicRangeAttribute, &dynamic_range);
    GetValueOfModeAttribute(it_mode->second, kRenderIntentAttribute, &str_render_intent);
  }

  current_color_mode_ = color_mode;
  PrimariesTransfer blend_space = {};
  blend_space = GetBlendSpaceFromColorMode();
  error = comp_manager_->SetBlendSpace(display_comp_ctx_, blend_space);
  if (error != kErrorNone) {
    DLOGE("SetBlendSpace failed, error = %d display_type_ = %d", error, display_type_);
  }

  error = hw_intf_->SetBlendSpace(blend_space);
  if (error != kErrorNone) {
    DLOGE("Failed to pass blend space, error = %d display_type_ = %d", error, display_type_);
  }

  error = SetColorModeInternal(color_mode, str_render_intent,  blend_space);
  if (error != kErrorNone) {
    return error;
  }

  comp_manager_->ControlDpps(dynamic_range != kHdr);

  return error;
}

DisplayError DisplayBase::SetColorModeById(int32_t color_mode_id) {
  for (auto it : color_mode_map_) {
    if (it.second->id == color_mode_id) {
      return SetColorMode(it.first);
    }
  }

  return kErrorNotSupported;
}

DisplayError DisplayBase::SetColorModeInternal(const std::string &color_mode,
                                               const std::string &str_render_intent,
                                               const PrimariesTransfer &pt) {
  DLOGV_IF(kTagQDCM, "Color Mode = %s", color_mode.c_str());

  ColorModeMap::iterator it = color_mode_map_.find(color_mode);
  if (it == color_mode_map_.end()) {
    DLOGE("Failed: Unknown Mode : %s", color_mode.c_str());
    return kErrorNotSupported;
  }

  SDEDisplayMode *sde_display_mode = it->second;

  DLOGV_IF(kTagQDCM, "Color Mode Name = %s corresponding mode_id = %d", sde_display_mode->name,
           sde_display_mode->id);
  DisplayError error = kErrorNone;
  int32_t render_intent = 0;
  if (!str_render_intent.empty()) {
    render_intent = std::stoi(str_render_intent);
  }

  if (render_intent < 0 || render_intent > MAX_EXTENDED_RENDER_INTENT) {
    DLOGW("Invalid render intent %d for mode id = %d", render_intent, sde_display_mode->id);
    return kErrorNotSupported;
  }

  error = color_mgr_->ColorMgrSetMode(sde_display_mode->id);
  if (error != kErrorNone) {
    DLOGE("Failed for mode id = %d", sde_display_mode->id);
    return error;
  }

  error = color_mgr_->ColorMgrSetModeWithRenderIntent(sde_display_mode->id, pt, render_intent);
  if (error != kErrorNone) {
    DLOGE("Failed for mode id = %d", sde_display_mode->id);
    return error;
  }

  return error;
}

DisplayError DisplayBase::GetColorModeName(int32_t mode_id, std::string *mode_name) {
  if (!mode_name) {
    DLOGE("Invalid parameters");
    return kErrorParameters;
  }
  for (uint32_t i = 0; i < num_color_modes_; i++) {
    if (color_modes_[i].id == mode_id) {
      *mode_name = color_modes_[i].name;
      return kErrorNone;
    }
  }

  DLOGE("Failed to get color mode name for mode id = %d", mode_id);
  return kErrorUndefined;
}

DisplayError DisplayBase::GetValueOfModeAttribute(const AttrVal &attr, const std::string &type,
                                                  std::string *value) {
  if (!value) {
    return kErrorParameters;
  }
  for (auto &it : attr) {
    if (it.first.find(type) != std::string::npos) {
      *value = it.second;
    }
  }

  return kErrorNone;
}

bool DisplayBase::IsSupportColorModeAttribute(const std::string &color_mode) {
  auto it = color_mode_attr_map_.find(color_mode);
  if (it == color_mode_attr_map_.end()) {
    return false;
  }
  return true;
}

DisplayError DisplayBase::SetColorTransform(const uint32_t length, const double *color_transform) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  if (!color_transform) {
    return kErrorParameters;
  }

  return color_mgr_->ColorMgrSetColorTransform(length, color_transform);
}

DisplayError DisplayBase::GetDefaultColorMode(std::string *color_mode) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!color_mode) {
    return kErrorParameters;
  }

  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  int32_t default_id = kInvalidModeId;
  DisplayError error = color_mgr_->ColorMgrGetDefaultModeID(&default_id);
  if (error != kErrorNone) {
    DLOGE("Failed for get default color mode id");
    return error;
  }

  for (uint32_t i = 0; i < num_color_modes_; i++) {
    if (color_modes_[i].id == default_id) {
      *color_mode = color_modes_[i].name;
      return kErrorNone;
    }
  }

  return kErrorNotSupported;
}

DisplayError DisplayBase::ApplyDefaultDisplayMode() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  if (color_mgr_) {
    error = color_mgr_->ApplyDefaultDisplayMode();
    // Apply default mode failed
    if (error != kErrorNone) {
      DLOGI("default mode not found");
      return error;
    }
    DeInitializeColorModes();
    // Default mode apply is called during first frame, if file system
    // where mode files is present, ColorManager will not find any modes.
    // Once boot animation is complete we re-try to apply the modes, since
    // file system should be mounted. InitColorModes needs to called again
    error = InitializeColorModes();
    if (error != kErrorNone) {
      DLOGE("failed to initial modes\n");
      return error;
    }
    if (color_modes_cs_.size() > 0) {
      error = comp_manager_->SetColorModesInfo(display_comp_ctx_, color_modes_cs_);
      if (error) {
        DLOGW("SetColorModesInfo failed on display = %d", display_type_);
      }
    }
  } else {
    return kErrorParameters;
  }
  return kErrorNone;
}

DisplayError DisplayBase::SetCursorPosition(int x, int y) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (state_ != kStateOn) {
    return kErrorNotSupported;
  }

  DisplayError error = comp_manager_->ValidateAndSetCursorPosition(display_comp_ctx_, &hw_layers_,
                                                                   x, y);
  if (error == kErrorNone) {
    return hw_intf_->SetCursorPosition(&hw_layers_, x, y);
  }

  return kErrorNone;
}

DisplayError DisplayBase::GetRefreshRateRange(uint32_t *min_refresh_rate,
                                              uint32_t *max_refresh_rate) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  // The min and max refresh rates will be same when the HWPanelInfo does not contain valid rates.
  // Usually for secondary displays, command mode panels
  HWDisplayAttributes display_attributes;
  uint32_t active_index = 0;
  hw_intf_->GetActiveConfig(&active_index);
  DisplayError error = hw_intf_->GetDisplayAttributes(active_index, &display_attributes);
  if (error) {
    return error;
  }

  *min_refresh_rate = display_attributes.fps;
  *max_refresh_rate = display_attributes.fps;

  return error;
}

DisplayError DisplayBase::HandlePendingVSyncEnable(const shared_ptr<Fence> &retire_fence) {
  if (vsync_enable_pending_) {
    // Retire fence signalling confirms that CRTC enabled, hence wait for retire fence before
    // we enable vsync
    Fence::Wait(retire_fence);

    DisplayError error = SetVSyncState(true /* enable */);
    if (error != kErrorNone) {
      return error;
    }
    vsync_enable_pending_ = false;
  }
  return kErrorNone;
}

DisplayError DisplayBase::SetVSyncState(bool enable) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  if (state_ == kStateOff && enable) {
    DLOGW("Can't enable vsync when display %d-%d is powered off!! Defer it when display is active",
          display_id_, display_type_);
    vsync_enable_pending_ = true;
    return kErrorNone;
  }
  DisplayError error = kErrorNone;
  if (vsync_enable_ != enable) {
    error = hw_intf_->SetVSyncState(enable);
    if (error == kErrorNotSupported) {
      if (drop_skewed_vsync_ && (hw_panel_info_.mode == kModeVideo) &&
        enable && (current_refresh_rate_ < hw_panel_info_.max_fps)) {
        drop_hw_vsync_ = true;
      }
      error = hw_events_intf_->SetEventState(HWEvent::VSYNC, enable);
    }
    if (error == kErrorNone) {
      vsync_enable_ = enable;
    } else {
      vsync_enable_pending_ = true;
    }
  }
  vsync_enable_pending_ = !enable ? false : vsync_enable_pending_;

  return error;
}

DisplayError DisplayBase::ReconfigureDisplay() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  HWDisplayAttributes display_attributes;
  HWMixerAttributes mixer_attributes;
  HWPanelInfo hw_panel_info;
  uint32_t active_index = 0;

  DTRACE_SCOPED();

  error = hw_intf_->GetActiveConfig(&active_index);
  if (error != kErrorNone) {
    return error;
  }

  error = hw_intf_->GetDisplayAttributes(active_index, &display_attributes);
  if (error != kErrorNone) {
    return error;
  }

  error = hw_intf_->GetMixerAttributes(&mixer_attributes);
  if (error != kErrorNone) {
    return error;
  }

  error = hw_intf_->GetHWPanelInfo(&hw_panel_info);
  if (error != kErrorNone) {
    return error;
  }

  bool display_unchanged = (display_attributes == display_attributes_);
  bool mixer_unchanged = (mixer_attributes == mixer_attributes_);
  bool panel_unchanged = (hw_panel_info == hw_panel_info_);
  if (display_unchanged && mixer_unchanged && panel_unchanged) {
    return kErrorNone;
  }

  error = comp_manager_->ReconfigureDisplay(display_comp_ctx_, display_attributes, hw_panel_info,
                                            mixer_attributes, fb_config_,
                                            &(default_clock_hz_));
  if (error != kErrorNone) {
    return error;
  }
  cached_qos_data_.clock_hz = default_clock_hz_;

  bool disble_pu = true;
  if (mixer_unchanged && panel_unchanged) {
    // Do not disable Partial Update for one frame, if only FPS has changed.
    // Because if first frame after transition, has a partial Frame-ROI and
    // is followed by Skip Validate frames, then it can benefit those frames.
    disble_pu = !display_attributes_.OnlyFpsChanged(display_attributes);
  }

  if (disble_pu) {
    DisablePartialUpdateOneFrame();
  }

  display_attributes_ = display_attributes;
  mixer_attributes_ = mixer_attributes;
  hw_panel_info_ = hw_panel_info;
  // TODO(user): Temporary changes, to be removed when DRM driver supports
  // Partial update with Destination scaler enabled.
  SetPUonDestScaler();

  return kErrorNone;
}

DisplayError DisplayBase::SetMixerResolution(uint32_t width, uint32_t height) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  DisplayError error = ReconfigureMixer(width, height);
  if (error != kErrorNone) {
    return error;
  }

  req_mixer_width_ = width;
  req_mixer_height_ = height;

  return kErrorNone;
}

DisplayError DisplayBase::GetMixerResolution(uint32_t *width, uint32_t *height) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!width || !height) {
    return kErrorParameters;
  }

  *width = mixer_attributes_.width;
  *height = mixer_attributes_.height;

  return kErrorNone;
}

DisplayError DisplayBase::ReconfigureMixer(uint32_t width, uint32_t height) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;

  DTRACE_SCOPED();
  if (!width || !height) {
    return kErrorParameters;
  }

  DLOGD_IF(kTagQDCM, "Reconfiguring mixer with width : %d, height : %d", width, height);

  LayerRect fb_rect = { 0.0f, 0.0f, FLOAT(fb_config_.x_pixels), FLOAT(fb_config_.y_pixels) };
  LayerRect mixer_rect = { 0.0f, 0.0f, FLOAT(width), FLOAT(height) };

  error = comp_manager_->ValidateScaling(fb_rect, mixer_rect, false /* rotate90 */);
  if (error != kErrorNone) {
    return error;
  }

  HWMixerAttributes mixer_attributes;
  mixer_attributes.width = width;
  mixer_attributes.height = height;

  error = hw_intf_->SetMixerAttributes(mixer_attributes);
  if (error != kErrorNone) {
    return error;
  }

  return ReconfigureDisplay();
}

bool DisplayBase::NeedsDownScale(const LayerRect &src_rect, const LayerRect &dst_rect,
                                 bool needs_rotation) {
  float src_width = FLOAT(src_rect.right - src_rect.left);
  float src_height = FLOAT(src_rect.bottom - src_rect.top);
  float dst_width = FLOAT(dst_rect.right - dst_rect.left);
  float dst_height = FLOAT(dst_rect.bottom - dst_rect.top);

  if (needs_rotation) {
    std::swap(src_width, src_height);
  }

  if ((src_width > dst_width) || (src_height > dst_height)) {
    return true;
  }

  return false;
}

bool DisplayBase::NeedsMixerReconfiguration(LayerStack *layer_stack, uint32_t *new_mixer_width,
                                            uint32_t *new_mixer_height) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  uint32_t mixer_width = mixer_attributes_.width;
  uint32_t mixer_height = mixer_attributes_.height;
  uint32_t fb_width = fb_config_.x_pixels;
  uint32_t fb_height = fb_config_.y_pixels;

  if (req_mixer_width_ && req_mixer_height_) {
    DLOGD_IF(kTagDisplay, "Required mixer width : %d, height : %d",
             req_mixer_width_, req_mixer_height_);
    *new_mixer_width = req_mixer_width_;
    *new_mixer_height = req_mixer_height_;
    return (req_mixer_width_ != mixer_width || req_mixer_height_ != mixer_height);
  }

  if (!custom_mixer_resolution_ && mixer_width == fb_width && mixer_height == fb_height) {
    return false;
  }

  uint32_t layer_count = UINT32(layer_stack->layers.size());
  uint32_t fb_area = fb_width * fb_height;
  LayerRect fb_rect = (LayerRect) {0.0f, 0.0f, FLOAT(fb_width), FLOAT(fb_height)};
  uint32_t display_width = display_attributes_.x_pixels;
  uint32_t display_height = display_attributes_.y_pixels;

  RectOrientation fb_orientation = GetOrientation(fb_rect);
  uint32_t max_layer_area = 0;
  uint32_t max_area_layer_index = 0;
  std::vector<Layer *> layers = layer_stack->layers;
  uint32_t align_x = display_attributes_.is_device_split ? 4 : 2;
  uint32_t align_y = 2;

  for (uint32_t i = 0; i < layer_count; i++) {
    Layer *layer = layers.at(i);

    uint32_t layer_width = UINT32(layer->src_rect.right - layer->src_rect.left);
    uint32_t layer_height = UINT32(layer->src_rect.bottom - layer->src_rect.top);
    uint32_t layer_area = layer_width * layer_height;

    if (layer_area > max_layer_area) {
      max_layer_area = layer_area;
      max_area_layer_index = i;
    }
  }
  DLOGV_IF(kTagDisplay, "Max area layer at index : %d", max_area_layer_index);

  // TODO(user): Mark layer which needs downscaling on GPU fallback as priority layer and use MDP
  // for composition to avoid quality mismatch between GPU and MDP switch(idle timeout usecase).
  if (max_layer_area >= fb_area) {
    Layer *layer = layers.at(max_area_layer_index);
    bool needs_rotation = (layer->transform.rotation == 90.0f);

    uint32_t layer_width = UINT32(layer->src_rect.right - layer->src_rect.left);
    uint32_t layer_height = UINT32(layer->src_rect.bottom - layer->src_rect.top);
    LayerRect layer_dst_rect = {};

    RectOrientation layer_orientation = GetOrientation(layer->src_rect);
    if (layer_orientation != kOrientationUnknown &&
        fb_orientation != kOrientationUnknown) {
      if (layer_orientation != fb_orientation) {
        std::swap(layer_width, layer_height);
      }
    }

    // Align the width and height according to fb's aspect ratio
    *new_mixer_width = FloorToMultipleOf(UINT32((FLOAT(fb_width) / FLOAT(fb_height)) *
                                         layer_height), align_x);
    *new_mixer_height = FloorToMultipleOf(layer_height, align_y);

    LayerRect dst_domain = {0.0f, 0.0f, FLOAT(*new_mixer_width), FLOAT(*new_mixer_height)};

    MapRect(fb_rect, dst_domain, layer->dst_rect, &layer_dst_rect);
    if (NeedsDownScale(layer->src_rect, layer_dst_rect, needs_rotation)) {
      *new_mixer_width = display_width;
      *new_mixer_height = display_height;
    }
    if (*new_mixer_width > display_width || *new_mixer_height > display_height) {
      *new_mixer_width = display_width;
      *new_mixer_height = display_height;
    }
    return ((*new_mixer_width != mixer_width) || (*new_mixer_height != mixer_height));
  }

  return false;
}

DisplayError DisplayBase::SetFrameBufferConfig(const DisplayConfigVariableInfo &variable_info) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  uint32_t width = variable_info.x_pixels;
  uint32_t height = variable_info.y_pixels;

  if (width == 0 || height == 0) {
    DLOGE("Unsupported resolution: (%dx%d)", width, height);
    return kErrorParameters;
  }

  // Create rects to represent the new source and destination crops
  LayerRect crop = LayerRect(0, 0, FLOAT(width), FLOAT(height));
  LayerRect dst = LayerRect(0, 0, FLOAT(mixer_attributes_.width), FLOAT(mixer_attributes_.height));
  // Set rotate90 to false since this is taken care of during regular composition.
  bool rotate90 = false;

  DisplayError error = comp_manager_->ValidateScaling(crop, dst, rotate90);
  if (error != kErrorNone) {
    DLOGE("Unsupported resolution: (%dx%d)", width, height);
    return kErrorParameters;
  }

  error =  comp_manager_->ReconfigureDisplay(display_comp_ctx_, display_attributes_, hw_panel_info_,
                                             mixer_attributes_, variable_info,
                                             &(default_clock_hz_));
  if (error != kErrorNone) {
    return error;
  }
  cached_qos_data_.clock_hz = default_clock_hz_;

  fb_config_.x_pixels = width;
  fb_config_.y_pixels = height;

  DLOGI("New framebuffer resolution (%dx%d)", fb_config_.x_pixels, fb_config_.y_pixels);

  return kErrorNone;
}

DisplayError DisplayBase::GetFrameBufferConfig(DisplayConfigVariableInfo *variable_info) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!variable_info) {
    return kErrorParameters;
  }

  *variable_info = fb_config_;

  return kErrorNone;
}

DisplayError DisplayBase::SetDetailEnhancerData(const DisplayDetailEnhancerData &de_data) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = comp_manager_->SetDetailEnhancerData(display_comp_ctx_, de_data);
  if (error != kErrorNone) {
    return error;
  }
  // TODO(user): Temporary changes, to be removed when DRM driver supports
  // Partial update with Destination scaler enabled.
  if (de_data.enable) {
    disable_pu_on_dest_scaler_ = true;
  } else {
    SetPUonDestScaler();
  }

  return kErrorNone;
}

DisplayError DisplayBase::GetDisplayPort(DisplayPort *port) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  if (!port) {
    return kErrorParameters;
  }

  *port = hw_panel_info_.port;

  return kErrorNone;
}

DisplayError DisplayBase::GetDisplayId(int32_t *display_id) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  if (!display_id) {
    return kErrorParameters;
  }

  *display_id = display_id_;

  return kErrorNone;
}

DisplayError DisplayBase::GetDisplayType(DisplayType *display_type) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  if (!display_type) {
    return kErrorParameters;
  }

  *display_type = display_type_;

  return kErrorNone;
}

bool DisplayBase::IsPrimaryDisplay() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  return hw_panel_info_.is_primary_panel;
}

DisplayError DisplayBase::SetCompositionState(LayerComposition composition_type, bool enable) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  return comp_manager_->SetCompositionState(display_comp_ctx_, composition_type, enable);
}

void DisplayBase::CommitLayerParams(LayerStack *layer_stack) {
  // Copy the acquire fence from clients layers  to HWLayers
  uint32_t hw_layers_count = UINT32(hw_layers_.info.hw_layers.size());

  for (uint32_t i = 0; i < hw_layers_count; i++) {
    uint32_t sdm_layer_index = hw_layers_.info.index.at(i);
    Layer *sdm_layer = layer_stack->layers.at(sdm_layer_index);
    Layer &hw_layer = hw_layers_.info.hw_layers.at(i);

    hw_layer.input_buffer.planes[0].fd = sdm_layer->input_buffer.planes[0].fd;
    hw_layer.input_buffer.planes[0].offset = sdm_layer->input_buffer.planes[0].offset;
    hw_layer.input_buffer.planes[0].stride = sdm_layer->input_buffer.planes[0].stride;
    hw_layer.input_buffer.size = sdm_layer->input_buffer.size;
    hw_layer.input_buffer.acquire_fence = sdm_layer->input_buffer.acquire_fence;
    hw_layer.input_buffer.handle_id = sdm_layer->input_buffer.handle_id;
    // TODO(user): Other FBT layer attributes like surface damage, dataspace, secure camera and
    // secure display flags are also updated during SetClientTarget() called between validate and
    // commit. Need to revist this and update it accordingly for FBT layer.
    if (hw_layers_.info.gpu_target_index == sdm_layer_index) {
      hw_layer.input_buffer.flags.secure = sdm_layer->input_buffer.flags.secure;
      hw_layer.input_buffer.format = sdm_layer->input_buffer.format;
      hw_layer.input_buffer.width = sdm_layer->input_buffer.width;
      hw_layer.input_buffer.height = sdm_layer->input_buffer.height;
      hw_layer.input_buffer.unaligned_width = sdm_layer->input_buffer.unaligned_width;
      hw_layer.input_buffer.unaligned_height = sdm_layer->input_buffer.unaligned_height;
    }
  }

  if (layer_stack->elapse_timestamp) {
    hw_layers_.elapse_timestamp = layer_stack->elapse_timestamp;
  }

  return;
}

void DisplayBase::PostCommitLayerParams(LayerStack *layer_stack) {
  // Copy the release fence from HWLayers to clients layers
    uint32_t hw_layers_count = UINT32(hw_layers_.info.hw_layers.size());

  std::vector<uint32_t> fence_dup_flag;

  for (uint32_t i = 0; i < hw_layers_count; i++) {
    uint32_t sdm_layer_index = hw_layers_.info.index.at(i);
    Layer *sdm_layer = layer_stack->layers.at(sdm_layer_index);
    Layer &hw_layer = hw_layers_.info.hw_layers.at(i);

    // Copy the release fence only once for a SDM Layer.
    // In S3D use case, two hw layers can share the same input buffer, So make sure to merge the
    // output fence fd and assign it to layer's input buffer release fence fd.
    if (std::find(fence_dup_flag.begin(), fence_dup_flag.end(), sdm_layer_index) ==
        fence_dup_flag.end()) {
      sdm_layer->input_buffer.release_fence = hw_layer.input_buffer.release_fence;
      fence_dup_flag.push_back(sdm_layer_index);
    } else {
      sdm_layer->input_buffer.release_fence = Fence::Merge(
              hw_layer.input_buffer.release_fence, sdm_layer->input_buffer.release_fence);
    }
  }
  cached_qos_data_ = hw_layers_.qos_data;

  return;
}

DisplayError DisplayBase::InitializeColorModes() {
  if (!color_mgr_) {
    return kErrorNotSupported;
  }

  DisplayError error = color_mgr_->ColorMgrGetNumOfModes(&num_color_modes_);
  if (error != kErrorNone || !num_color_modes_) {
    DLOGV_IF(kTagQDCM, "GetNumModes failed = %d count = %d", error, num_color_modes_);
    return kErrorNotSupported;
  }
  DLOGI("Number of Color Modes = %d", num_color_modes_);

  if (!color_modes_.size()) {
    color_modes_.resize(num_color_modes_);

    DisplayError error = color_mgr_->ColorMgrGetModes(&num_color_modes_, color_modes_.data());
    if (error != kErrorNone) {
      color_modes_.clear();
      DLOGE("Failed");
      return error;
    }

    AttrVal var;
    uint32_t num_insert_color_modes = 0;
    for (uint32_t i = 0; i < num_color_modes_; i++) {
      DLOGV_IF(kTagQDCM, "Color Mode[%d]: Name = %s mode_id = %d", i, color_modes_[i].name,
               color_modes_[i].id);
      auto it = color_mode_map_.find(color_modes_[i].name);
      if (it != color_mode_map_.end()) {
        if (it->second->id < color_modes_[i].id) {
          color_mode_map_.erase(it);
          color_mode_map_.insert(std::make_pair(color_modes_[i].name, &color_modes_[i]));
        }
      } else {
        color_mode_map_.insert(std::make_pair(color_modes_[i].name, &color_modes_[i]));
      }

      var.clear();
      error = color_mgr_->ColorMgrGetModeInfo(color_modes_[i].id, &var);
      if (error != kErrorNone) {
        DLOGE("Failed for get attributes of mode_id = %d", color_modes_[i].id);
        continue;
      }
      if (!var.empty()) {
        auto it = color_mode_attr_map_.find(color_modes_[i].name);
        if (it == color_mode_attr_map_.end()) {
          color_mode_attr_map_.insert(std::make_pair(color_modes_[i].name, var));
          // If target doesn't support SSPP tone maping and color mode is HDR,
          // add bt2020pq and bt2020hlg color modes.
          if (hw_resource_info_.src_tone_map.none() && IsHdrMode(var)) {
            std::string str_render_intent;
            GetValueOfModeAttribute(var, kRenderIntentAttribute, &str_render_intent);
            color_mode_map_.insert(std::make_pair(kBt2020Pq, &color_modes_[i]));
            color_mode_map_.insert(std::make_pair(kBt2020Hlg, &color_modes_[i]));
            num_insert_color_modes = 2;
            InsertBT2020PqHlgModes(str_render_intent);
          }
        }
        std::vector<PrimariesTransfer> pt_list = {};
        GetColorPrimaryTransferFromAttributes(var, &pt_list);
        for (const PrimariesTransfer &pt : pt_list) {
          if (std::find(color_modes_cs_.begin(), color_modes_cs_.end(), pt) ==
              color_modes_cs_.end()) {
            color_modes_cs_.push_back(pt);
          }
        }
      }
    }
    PrimariesTransfer pt = {};
    if (std::find(color_modes_cs_.begin(), color_modes_cs_.end(), pt) ==
        color_modes_cs_.end()) {
      color_modes_cs_.push_back(pt);
    }

    num_color_modes_ += num_insert_color_modes;
  }

  return kErrorNone;
}

DisplayError DisplayBase::GetDisplayIdentificationData(uint8_t *out_port, uint32_t *out_data_size,
                                                       uint8_t *out_data) {
  if (!out_port || !out_data_size) {
    return kErrorParameters;
  }

  return hw_intf_->GetDisplayIdentificationData(out_port, out_data_size, out_data);
}

DisplayError DisplayBase::GetClientTargetSupport(uint32_t width, uint32_t height,
                                                 LayerBufferFormat format,
                                                 const ColorMetaData &color_metadata) {
  if (format != kFormatRGBA8888 && format != kFormatRGBA1010102) {
    DLOGW("Unsupported format = %d", format);
    return kErrorNotSupported;
  } else if (ValidateScaling(width, height) != kErrorNone) {
    DLOGW("Unsupported width = %d height = %d", width, height);
    return kErrorNotSupported;
  } else if (color_metadata.transfer && color_metadata.colorPrimaries) {
    DisplayError error = ValidateDataspace(color_metadata);
    if (error != kErrorNone) {
      DLOGW("Unsupported Transfer Request = %d Color Primary = %d",
             color_metadata.transfer, color_metadata.colorPrimaries);
      return error;
    }

    // Check for BT2020 support
    if (color_metadata.colorPrimaries == ColorPrimaries_BT2020) {
      DLOGW("Unsupported Color Primary = %d", color_metadata.colorPrimaries);
      return kErrorNotSupported;
    }
  }

  return kErrorNone;
}

bool DisplayBase::IsSupportSsppTonemap() {
  if (hw_resource_info_.src_tone_map.none()) {
    return false;
  } else {
    return true;
  }
}

DisplayError DisplayBase::ValidateScaling(uint32_t width, uint32_t height) {
  uint32_t display_width = display_attributes_.x_pixels;
  uint32_t display_height = display_attributes_.y_pixels;

  float max_scale_down = FLOAT(hw_resource_info_.max_scale_down);
  float max_scale_up = FLOAT(hw_resource_info_.max_scale_up);

  float scale_x = FLOAT(width / display_width);
  float scale_y = FLOAT(height / display_height);

  if (scale_x > max_scale_down || scale_y > max_scale_down) {
    return kErrorNotSupported;
  }

  if (UINT32(scale_x) < 1 && scale_x > 0.0f) {
    if ((1.0f / scale_x) > max_scale_up) {
      return kErrorNotSupported;
    }
  }

  if (UINT32(scale_y) < 1 && scale_y > 0.0f) {
    if ((1.0f / scale_y) > max_scale_up) {
      return kErrorNotSupported;
    }
  }

  return kErrorNone;
}

DisplayError DisplayBase::ValidateDataspace(const ColorMetaData &color_metadata) {
  // Handle transfer
  switch (color_metadata.transfer) {
    case Transfer_sRGB:
    case Transfer_SMPTE_170M:
    case Transfer_SMPTE_ST2084:
    case Transfer_HLG:
    case Transfer_Linear:
    case Transfer_Gamma2_2:
      break;
    default:
      DLOGW("Unsupported Transfer Request = %d", color_metadata.transfer);
      return kErrorNotSupported;
  }

  // Handle colorPrimaries
  switch (color_metadata.colorPrimaries) {
    case ColorPrimaries_BT709_5:
    case ColorPrimaries_BT601_6_525:
    case ColorPrimaries_BT601_6_625:
    case ColorPrimaries_DCIP3:
    case ColorPrimaries_BT2020:
      break;
    default:
      DLOGW("Unsupported Color Primary = %d", color_metadata.colorPrimaries);
      return kErrorNotSupported;
  }

  return kErrorNone;
}

// TODO(user): Temporary changes, to be removed when DRM driver supports
// Partial update with Destination scaler enabled.
void DisplayBase::SetPUonDestScaler() {
  uint32_t mixer_width = mixer_attributes_.width;
  uint32_t mixer_height = mixer_attributes_.height;
  uint32_t display_width = display_attributes_.x_pixels;
  uint32_t display_height = display_attributes_.y_pixels;

  disable_pu_on_dest_scaler_ = (mixer_width != display_width ||
                                mixer_height != display_height);
}

void DisplayBase::ClearColorInfo() {
  color_modes_.clear();
  color_mode_map_.clear();
  color_mode_attr_map_.clear();
  color_modes_cs_.clear();

  if (color_mgr_) {
    delete color_mgr_;
    color_mgr_ = NULL;
  }
}

void DisplayBase::DeInitializeColorModes() {
    color_mode_map_.clear();
    color_modes_.clear();
    color_mode_attr_map_.clear();
    num_color_modes_ = 0;
}

void DisplayBase::GetColorPrimaryTransferFromAttributes(const AttrVal &attr,
    std::vector<PrimariesTransfer> *supported_pt) {
  std::string attribute_field = {};
  if (attr.empty()) {
    return;
  }

  for (auto &it : attr) {
    if ((it.first.find(kColorGamutAttribute) != std::string::npos) ||
        (it.first.find(kDynamicRangeAttribute) != std::string::npos)) {
      attribute_field = it.second;
      PrimariesTransfer pt = {};
      pt.primaries = GetColorPrimariesFromAttribute(attribute_field);
      if (pt.primaries == ColorPrimaries_BT709_5) {
        pt.transfer = Transfer_sRGB;
        supported_pt->push_back(pt);
      } else if (pt.primaries == ColorPrimaries_DCIP3) {
        pt.transfer = Transfer_sRGB;
        supported_pt->push_back(pt);
      } else if (pt.primaries == ColorPrimaries_BT2020) {
        pt.transfer = Transfer_SMPTE_ST2084;
        supported_pt->push_back(pt);
        pt.transfer = Transfer_HLG;
        supported_pt->push_back(pt);
      }
    }
  }
}

void DisplayBase::HwRecovery(const HWRecoveryEvent sdm_event_code) {
  DLOGI("Handling event = %" PRIu32, sdm_event_code);
  if (DisplayPowerResetPending()) {
    DLOGI("Skipping handling for display = %d, display power reset in progress", display_type_);
    return;
  }
  switch (sdm_event_code) {
    case HWRecoveryEvent::kSuccess:
      hw_recovery_logs_captured_ = false;
      break;
    case HWRecoveryEvent::kCapture:
      if (!disable_hw_recovery_dump_ && !hw_recovery_logs_captured_) {
        hw_intf_->DumpDebugData();
        hw_recovery_logs_captured_ = true;
        DLOGI("Captured debugfs data for display = %d", display_type_);
      } else if (!disable_hw_recovery_dump_) {
        DLOGI("Multiple capture events without intermediate success event, skipping debugfs"
              "capture for display = %d", display_type_);
      } else {
        DLOGI("Debugfs data dumping is disabled for display = %d", display_type_);
      }
      break;
    case HWRecoveryEvent::kDisplayPowerReset:
      DLOGI("display = %d attempting to start display power reset", display_type_);
      if (StartDisplayPowerReset()) {
        DLOGI("display = %d allowed to start display power reset", display_type_);
        event_handler_->HandleEvent(kDisplayPowerResetEvent);
        EndDisplayPowerReset();
        DLOGI("display = %d has finished display power reset", display_type_);
      }
      break;
    default:
      return;
  }
}

bool DisplayBase::DisplayPowerResetPending() {
  SCOPE_LOCK(display_power_reset_lock_);
  return display_power_reset_pending_;
}

bool DisplayBase::StartDisplayPowerReset() {
  SCOPE_LOCK(display_power_reset_lock_);
  if (!display_power_reset_pending_) {
    display_power_reset_pending_ = true;
    return true;
  }
  return false;
}

void DisplayBase::EndDisplayPowerReset() {
  SCOPE_LOCK(display_power_reset_lock_);
  display_power_reset_pending_ = false;
}

bool DisplayBase::SetHdrModeAtStart(LayerStack *layer_stack) {
  return (hw_resource_info_.src_tone_map.none() && layer_stack->flags.hdr_present);
}

PrimariesTransfer DisplayBase::GetBlendSpaceFromColorMode() {
  PrimariesTransfer pt = {};
  auto current_color_attr_ = color_mode_attr_map_.find(current_color_mode_);
  if (current_color_attr_ == color_mode_attr_map_.end()) {
    DLOGE("The attritbutes is not present in color mode: %s", current_color_mode_.c_str());
    return pt;
  }

  AttrVal attr = current_color_attr_->second;
  std::string color_gamut = kNative, dynamic_range = kSdr, pic_quality = kStandard;
  std::string transfer = {};

  if (attr.begin() != attr.end()) {
    for (auto &it : attr) {
      if (it.first.find(kColorGamutAttribute) != std::string::npos) {
        color_gamut = it.second;
      } else if (it.first.find(kDynamicRangeAttribute) != std::string::npos) {
        dynamic_range = it.second;
      } else if (it.first.find(kPictureQualityAttribute) != std::string::npos) {
        pic_quality = it.second;
      } else if (it.first.find(kGammaTransferAttribute) != std::string::npos) {
        transfer = it.second;
      }
    }
  }
  // TODO(user): Check is if someone calls with hal_display_p3
  if (hw_resource_info_.src_tone_map.none() &&
      (pic_quality == kStandard && color_gamut == kBt2020)) {
    pt.primaries = GetColorPrimariesFromAttribute(color_gamut);
    if (transfer == kHlg) {
      pt.transfer = Transfer_HLG;
    } else {
      pt.transfer = Transfer_SMPTE_ST2084;
    }
  } else if (color_gamut == kDcip3) {
    pt.primaries = GetColorPrimariesFromAttribute(color_gamut);
    pt.transfer = Transfer_sRGB;
  }

  return pt;
}

void DisplayBase::InsertBT2020PqHlgModes(const std::string &str_render_intent) {
  AttrVal hdr_var = {};
  hdr_var.push_back(std::make_pair(kColorGamutAttribute, kBt2020));
  hdr_var.push_back(std::make_pair(kPictureQualityAttribute, kStandard));
  if (!str_render_intent.empty()) {
    hdr_var.push_back(std::make_pair(kRenderIntentAttribute, str_render_intent));
  }
  hdr_var.push_back(std::make_pair(kGammaTransferAttribute, kSt2084));
  color_mode_attr_map_.insert(std::make_pair(kBt2020Pq, hdr_var));
  hdr_var.pop_back();
  hdr_var.push_back(std::make_pair(kGammaTransferAttribute, kHlg));
  color_mode_attr_map_.insert(std::make_pair(kBt2020Hlg, hdr_var));

  return;
}

bool DisplayBase::IsHdrMode(const AttrVal &attr) {
  std::string color_gamut, dynamic_range;
  GetValueOfModeAttribute(attr, kColorGamutAttribute, &color_gamut);
  GetValueOfModeAttribute(attr, kDynamicRangeAttribute, &dynamic_range);
  if (color_gamut == kDcip3 && dynamic_range == kHdr) {
    return true;
  }

  return false;
}

bool DisplayBase::CanSkipValidate() {
  bool needs_buffer_swap = false;
  bool skip_validate = comp_manager_->CanSkipValidate(display_comp_ctx_, &needs_buffer_swap);

  if (needs_buffer_swap) {
    hw_layers_.updates_mask.set(kSwapBuffers);
    DisplayError error = comp_manager_->SwapBuffers(display_comp_ctx_);
    if (error != kErrorNone) {
      // Buffers couldn't be swapped.
      skip_validate = false;
    }
  }

  return skip_validate;
}

DisplayError DisplayBase::HandlePendingPowerState(const shared_ptr<Fence> &retire_fence) {
  if (pending_doze_ || pending_power_on_) {
    // Retire fence signalling confirms that CRTC enabled, hence wait for retire fence before
    // we enable vsync
    Fence::Wait(retire_fence);

    if (pending_doze_) {
      state_ = kStateDoze;
      DisplayError error = ReconfigureDisplay();
      if (error != kErrorNone) {
        return error;
      }
      event_handler_->Refresh();
    }
    if (pending_power_on_) {
      state_ = kStateOn;
    }
    active_ = true;

    pending_doze_ = false;
    pending_power_on_ = false;
  }
  return kErrorNone;
}

bool DisplayBase::CheckResourceState() {
  return comp_manager_->CheckResourceState(display_comp_ctx_);
}
DisplayError DisplayBase::colorSamplingOn() {
  return kErrorNone;
}

DisplayError DisplayBase::colorSamplingOff() {
  return kErrorNone;
}

bool DisplayBase::GameEnhanceSupported() {
  if (color_mgr_) {
    return color_mgr_->GameEnhanceSupported();
  }
  return false;
}

DisplayError DisplayBase::OnMinHdcpEncryptionLevelChange(uint32_t min_enc_level) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  return hw_intf_->OnMinHdcpEncryptionLevelChange(min_enc_level);
}

DisplayError DisplayBase::DelayFirstCommit() {
  return hw_intf_->DelayFirstCommit();
}

}  // namespace sdm
