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

#include <utils/constants.h>
#include <utils/debug.h>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "display_pluggable.h"
#include "hw_info_interface.h"
#include "hw_interface.h"

#define __CLASS__ "DisplayPluggable"

namespace sdm {

DisplayPluggable::DisplayPluggable(DisplayEventHandler *event_handler,
                                   HWInfoInterface *hw_info_intf,
                                   BufferAllocator *buffer_allocator, CompManager *comp_manager)
  : DisplayBase(kPluggable, event_handler, kDevicePluggable, buffer_allocator,
                comp_manager, hw_info_intf) {}

DisplayPluggable::DisplayPluggable(int32_t display_id, DisplayEventHandler *event_handler,
                                   HWInfoInterface *hw_info_intf,
                                   BufferAllocator *buffer_allocator, CompManager *comp_manager)
  : DisplayBase(display_id, kPluggable, event_handler, kDevicePluggable,
                buffer_allocator, comp_manager, hw_info_intf) {}

DisplayError DisplayPluggable::Init() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  DisplayError error = HWInterface::Create(display_id_, kPluggable, hw_info_intf_,
                                           buffer_allocator_, &hw_intf_);
  if (error != kErrorNone) {
    if (kErrorDeviceRemoved == error) {
      DLOGW("Aborted creating hardware interface. Device removed.");
    } else {
      DLOGE("Failed to create hardware interface. Error = %d", error);
    }
    return error;
  }

  if (-1 == display_id_) {
    hw_intf_->GetDisplayId(&display_id_);
  }

  uint32_t active_mode_index = 0;
  error = hw_intf_->GetActiveConfig(&active_mode_index);
  if (error != kErrorNone) {
    HWInterface::Destroy(hw_intf_);
    return error;
  }

  uint32_t override_mode_index = active_mode_index;
  error = GetOverrideConfig(&override_mode_index);
  if (error == kErrorNone && override_mode_index != active_mode_index) {
    DLOGI("Overriding display mode %d with mode %d.", active_mode_index, override_mode_index);
    error = hw_intf_->SetDisplayAttributes(override_mode_index);
    if (error != kErrorNone) {
      DLOGI("Failed overriding display mode %d with mode %d. Continuing with display mode %d.",
            active_mode_index, override_mode_index, active_mode_index);
    }
  }

  error = DisplayBase::Init();
  if (error == kErrorResources) {
    DLOGI("Reattempting display creation for Pluggable %d", display_id_);
    uint32_t default_mode_index = 0;
    error = hw_intf_->GetDefaultConfig(&default_mode_index);
    if (error == kErrorNone) {
      hw_intf_->SetDisplayAttributes(default_mode_index);
      error = DisplayBase::Init();
    } else {
      DLOGE("640x480 default mode not found, failing creation!");
    }
  }
  if (error != kErrorNone) {
    HWInterface::Destroy(hw_intf_);
    return error;
  }

  GetScanSupport();
  underscan_supported_ = (scan_support_ == kScanAlwaysUnderscanned) || (scan_support_ == kScanBoth);

  error = HWEventsInterface::Create(display_id_, kPluggable, this, event_list_, hw_intf_,
                                    &hw_events_intf_);
  if (error != kErrorNone) {
    DisplayBase::Deinit();
    HWInterface::Destroy(hw_intf_);
    DLOGE("Failed to create hardware events interface. Error = %d", error);
  }

  InitializeColorModes();

  current_refresh_rate_ = hw_panel_info_.max_fps;

  return error;
}

DisplayError DisplayPluggable::Prepare(LayerStack *layer_stack) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;
  uint32_t new_mixer_width = 0;
  uint32_t new_mixer_height = 0;
  uint32_t display_width = display_attributes_.x_pixels;
  uint32_t display_height = display_attributes_.y_pixels;

  if (NeedsMixerReconfiguration(layer_stack, &new_mixer_width, &new_mixer_height)) {
    error = ReconfigureMixer(new_mixer_width, new_mixer_height);
    if (error != kErrorNone) {
      ReconfigureMixer(display_width, display_height);
    }
  }

  // Clean hw layers for reuse.
  hw_layers_ = HWLayers();

  return DisplayBase::Prepare(layer_stack);
}

DisplayError DisplayPluggable::GetRefreshRateRange(uint32_t *min_refresh_rate,
                                                   uint32_t *max_refresh_rate) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  DisplayError error = kErrorNone;

  if (hw_panel_info_.min_fps && hw_panel_info_.max_fps) {
    *min_refresh_rate = hw_panel_info_.min_fps;
    *max_refresh_rate = hw_panel_info_.max_fps;
  } else {
    error = DisplayBase::GetRefreshRateRange(min_refresh_rate, max_refresh_rate);
  }

  return error;
}

DisplayError DisplayPluggable::SetRefreshRate(uint32_t refresh_rate, bool final_rate,
                                              bool idle_screen) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);

  if (!active_) {
    return kErrorPermission;
  }

  if (current_refresh_rate_ != refresh_rate) {
    DisplayError error = hw_intf_->SetRefreshRate(refresh_rate);
    if (error != kErrorNone) {
      return error;
    }
  }

  current_refresh_rate_ = refresh_rate;
  return DisplayBase::ReconfigureDisplay();
}

bool DisplayPluggable::IsUnderscanSupported() {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  return underscan_supported_;
}

DisplayError DisplayPluggable::GetOverrideConfig(uint32_t *mode_index) {
  DisplayError error = kErrorNone;

  if (!mode_index) {
    DLOGE("Invalid mode index parameter.");
    return kErrorParameters;
  }

  char val[kPropertyMax] = {};
  // Used for changing HDMI Resolution - Override the preferred mode with user set config.
  bool user_config = Debug::GetExternalResolution(val);
  if (user_config) {
    uint32_t config_index = 0;
    // For the config, get the corresponding index
    error = hw_intf_->GetConfigIndex(val, &config_index);
    if (error == kErrorNone) {
      *mode_index = config_index;
    }
  }

  return error;
}

void DisplayPluggable::GetScanSupport() {
  DisplayError error = kErrorNone;
  uint32_t video_format = 0;
  uint32_t max_cea_format = 0;
  HWScanInfo scan_info = HWScanInfo();
  hw_intf_->GetHWScanInfo(&scan_info);

  uint32_t active_mode_index = 0;
  hw_intf_->GetActiveConfig(&active_mode_index);

  error = hw_intf_->GetVideoFormat(active_mode_index, &video_format);
  if (error != kErrorNone) {
    return;
  }

  error = hw_intf_->GetMaxCEAFormat(&max_cea_format);
  if (error != kErrorNone) {
    return;
  }

  // The scan support for a given HDMI TV must be read from scan info corresponding to
  // Preferred Timing if the preferred timing of the display is currently active, and if it is
  // valid. In all other cases, we must read the scan support from CEA scan info if
  // the resolution is a CEA resolution, or from IT scan info for all other resolutions.
  if (active_mode_index == 0 && scan_info.pt_scan_support != kScanNotSupported) {
    scan_support_ = scan_info.pt_scan_support;
  } else if (video_format < max_cea_format) {
    scan_support_ = scan_info.cea_scan_support;
  } else {
    scan_support_ = scan_info.it_scan_support;
  }
}

void DisplayPluggable::CECMessage(char *message) {
  event_handler_->CECMessage(message);
}

// HWEventHandler overload, not DisplayBase
void DisplayPluggable::HwRecovery(const HWRecoveryEvent sdm_event_code) {
  DisplayBase::HwRecovery(sdm_event_code);
}

void DisplayPluggable::Histogram(int /* histogram_fd */, uint32_t /* blob_id */) {}

DisplayError DisplayPluggable::VSync(int64_t timestamp) {
  if (vsync_enable_) {
    DisplayEventVSync vsync;
    vsync.timestamp = timestamp;
    event_handler_->VSync(vsync);
  }

  return kErrorNone;
}

DisplayError DisplayPluggable::InitializeColorModes() {
  PrimariesTransfer pt = {};
  AttrVal var = {};
  if (!hw_panel_info_.hdr_enabled && !hw_panel_info_.supported_colorspaces) {
    return kErrorNone;
  } else {
    if (hw_panel_info_.supported_colorspaces) {
      InitializeColorModesFromColorspace();
    }
    color_modes_cs_.push_back(pt);
    var.push_back(std::make_pair(kColorGamutAttribute, kSrgb));
    var.push_back(std::make_pair(kDynamicRangeAttribute, kSdr));
    var.push_back(std::make_pair(kPictureQualityAttribute, kStandard));
    var.push_back(std::make_pair(kRenderIntentAttribute, "0"));
    color_mode_attr_map_.insert(std::make_pair(kSrgb, var));

    // native mode
    color_modes_cs_.push_back(pt);
    var.clear();
    color_mode_attr_map_.insert(std::make_pair("hal_native", var));
  }

  var.clear();
  var.push_back(std::make_pair(kColorGamutAttribute, kBt2020));
  var.push_back(std::make_pair(kPictureQualityAttribute, kStandard));
  var.push_back(std::make_pair(kRenderIntentAttribute, "0"));
  if (hw_panel_info_.hdr_eotf & kHdrEOTFHDR10) {
    pt.transfer = Transfer_SMPTE_ST2084;
    var.push_back(std::make_pair(kGammaTransferAttribute, kSt2084));
    color_modes_cs_.push_back(pt);
    color_mode_attr_map_.insert(std::make_pair(kBt2020Pq, var));
  }
  if (hw_panel_info_.hdr_eotf & kHdrEOTFHLG) {
    pt.transfer = Transfer_HLG;
    var.pop_back();
    var.push_back(std::make_pair(kGammaTransferAttribute, kHlg));
    color_modes_cs_.push_back(pt);
    color_mode_attr_map_.insert(std::make_pair(kBt2020Hlg, var));
  }
  current_color_mode_ = kSrgb;
  UpdateColorModes();

  return kErrorNone;
}

void DisplayPluggable::InitializeColorModesFromColorspace() {
  PrimariesTransfer pt = {};
  AttrVal var = {};
  if (hw_panel_info_.supported_colorspaces & kColorspaceDcip3) {
    pt.primaries = ColorPrimaries_DCIP3;
    pt.transfer = Transfer_sRGB;
    var.clear();
    var.push_back(std::make_pair(kColorGamutAttribute, kDcip3));
    var.push_back(std::make_pair(kGammaTransferAttribute, kSrgb));
    var.push_back(std::make_pair(kPictureQualityAttribute, kStandard));
    var.push_back(std::make_pair(kRenderIntentAttribute, "0"));
    color_modes_cs_.push_back(pt);
    color_mode_attr_map_.insert(std::make_pair(kDisplayP3, var));
  }
  if (hw_panel_info_.supported_colorspaces & kColorspaceBt2020rgb) {
    pt.primaries = ColorPrimaries_BT2020;
    pt.transfer = Transfer_sRGB;
    var.clear();
    var.push_back(std::make_pair(kColorGamutAttribute, kBt2020));
    var.push_back(std::make_pair(kGammaTransferAttribute, kSrgb));
    var.push_back(std::make_pair(kPictureQualityAttribute, kStandard));
    var.push_back(std::make_pair(kRenderIntentAttribute, "0"));
    color_modes_cs_.push_back(pt);
    color_mode_attr_map_.insert(std::make_pair(kDisplayBt2020, var));
  }
}

static PrimariesTransfer GetBlendSpaceFromAttributes(const std::string &color_gamut,
                                                     const std::string &transfer) {
  PrimariesTransfer blend_space_ = {};
  if (color_gamut == kBt2020) {
    blend_space_.primaries = ColorPrimaries_BT2020;
    if (transfer == kHlg) {
      blend_space_.transfer = Transfer_HLG;
    } else if (transfer == kSt2084) {
      blend_space_.transfer = Transfer_SMPTE_ST2084;
    } else if (transfer == kGamma2_2) {
      blend_space_.transfer = Transfer_Gamma2_2;
    }
  } else if (color_gamut == kDcip3) {
    blend_space_.primaries = ColorPrimaries_DCIP3;
    blend_space_.transfer = Transfer_sRGB;
  } else if (color_gamut == kSrgb) {
    blend_space_.primaries = ColorPrimaries_BT709_5;
    blend_space_.transfer = Transfer_sRGB;
  } else {
    DLOGW("Failed to Get blend space color_gamut = %s transfer = %s", color_gamut.c_str(),
          transfer.c_str());
  }
  DLOGI("Blend Space Primaries = %d Transfer = %d", blend_space_.primaries, blend_space_.transfer);

  return blend_space_;
}

DisplayError DisplayPluggable::SetColorMode(const std::string &color_mode) {
  auto current_color_attr_ = color_mode_attr_map_.find(color_mode);
  if (current_color_attr_ == color_mode_attr_map_.end()) {
    DLOGW("Failed to get the color mode = %s", color_mode.c_str());
    return kErrorNone;
  }
  AttrVal attr = current_color_attr_->second;
  std::string color_gamut = kNative, transfer = {};

  if (attr.begin() != attr.end()) {
    for (auto &it : attr) {
      if (it.first.find(kColorGamutAttribute) != std::string::npos) {
        color_gamut = it.second;
      } else if (it.first.find(kGammaTransferAttribute) != std::string::npos) {
        transfer = it.second;
      }
    }
  }

  DisplayError error = kErrorNone;
  PrimariesTransfer blend_space = GetBlendSpaceFromAttributes(color_gamut, transfer);
  error = comp_manager_->SetBlendSpace(display_comp_ctx_, blend_space);
  if (error != kErrorNone) {
    DLOGW("Failed Set blend space, error = %d display_type_ = %d", error, display_type_);
  }

  error = hw_intf_->SetBlendSpace(blend_space);
  if (error != kErrorNone) {
    DLOGW("Failed to pass blend space, error = %d display_type_ = %d", error, display_type_);
  }

  current_color_mode_ = color_mode;

  return kErrorNone;
}

DisplayError DisplayPluggable::GetColorModeCount(uint32_t *mode_count) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!mode_count) {
    return kErrorParameters;
  }

  DLOGI("Display = %d Number of modes = %d", display_type_, num_color_modes_);
  *mode_count = num_color_modes_;

  return kErrorNone;
}

DisplayError DisplayPluggable::GetColorModes(uint32_t *mode_count,
                                             std::vector<std::string> *color_modes) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!mode_count || !color_modes) {
    return kErrorParameters;
  }

  for (uint32_t i = 0; i < num_color_modes_; i++) {
    DLOGI_IF(kTagDisplay, "ColorMode[%d] = %s", i, color_modes_[i].name);
    color_modes->at(i) = color_modes_[i].name;
  }

  return kErrorNone;
}

DisplayError DisplayPluggable::GetColorModeAttr(const std::string &color_mode, AttrVal *attr) {
  lock_guard<recursive_mutex> obj(recursive_mutex_);
  if (!attr) {
    return kErrorParameters;
  }

  auto it = color_mode_attr_map_.find(color_mode);
  if (it == color_mode_attr_map_.end()) {
    DLOGI("Mode %s has no attribute", color_mode.c_str());
    return kErrorNotSupported;
  }
  *attr = it->second;

  return kErrorNone;
}

void DisplayPluggable::UpdateColorModes() {
  uint32_t i = 0;
  num_color_modes_ = UINT32(color_mode_attr_map_.size());
  color_modes_.resize(num_color_modes_);
  for (ColorModeAttrMap::iterator it = color_mode_attr_map_.begin();
       ((i < num_color_modes_) && (it != color_mode_attr_map_.end())); i++, it++) {
    color_modes_[i].id = INT32(i);
    std::size_t length = (it->first).copy(color_modes_[i].name, sizeof(SDEDisplayMode::name) - 1);
    color_modes_[i].name[length] = '\0';
    color_mode_map_.insert(std::make_pair(color_modes_[i].name, &color_modes_[i]));
    DLOGI("Color mode = %s", color_modes_[i].name);
  }
  return;
}

DisplayError DisplayPluggable::colorSamplingOn() {
    return kErrorNone;
}

DisplayError DisplayPluggable::colorSamplingOff() {
    return kErrorNone;
}

}  // namespace sdm
