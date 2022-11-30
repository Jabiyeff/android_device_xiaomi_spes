/*
 * Copyright (c) 2014-2021, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 *
 * Copyright 2015 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cutils/properties.h>
#include <errno.h>
#include <math.h>
#include <sync/sync.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/utils.h>
#include <utils/formats.h>
#include <utils/rect.h>
#include <qd_utils.h>
#include <vendor/qti/hardware/display/composer/3.0/IQtiComposerClient.h>

#include <algorithm>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "hwc_display.h"
#include "hwc_debugger.h"
#include "hwc_tonemapper.h"
#include "hwc_session.h"

#ifdef QTI_BSP
#include <hardware/display_defs.h>
#endif

#define __CLASS__ "HWCDisplay"

namespace sdm {

uint32_t HWCDisplay::throttling_refresh_rate_ = 60;

bool NeedsToneMap(const LayerStack &layer_stack) {
  for (Layer *layer : layer_stack.layers) {
    if (layer->request.flags.tone_map) {
      return true;
    }
  }
  return false;
}

bool IsTimeAfterOrEqualVsyncTime(int64_t time, int64_t vsync_time) {
  return ((vsync_time != INT64_MAX) && ((time - vsync_time) >= 0));
}

HWCColorMode::HWCColorMode(DisplayInterface *display_intf) : display_intf_(display_intf) {}

HWC2::Error HWCColorMode::Init() {
  PopulateColorModes();
  return HWC2::Error::None;
}

HWC2::Error HWCColorMode::DeInit() {
  color_mode_map_.clear();
  return HWC2::Error::None;
}

uint32_t HWCColorMode::GetColorModeCount() {
  uint32_t count = UINT32(color_mode_map_.size());
  DLOGI("Supported color mode count = %d", count);
  return std::max(1U, count);
}

uint32_t HWCColorMode::GetRenderIntentCount(ColorMode mode) {
  uint32_t count = UINT32(color_mode_map_[mode].size());
  DLOGI("mode: %d supported rendering intent count = %d", mode, count);
  return std::max(1U, count);
}

HWC2::Error HWCColorMode::GetColorModes(uint32_t *out_num_modes, ColorMode *out_modes) {
  auto it = color_mode_map_.begin();
  *out_num_modes = std::min(*out_num_modes, UINT32(color_mode_map_.size()));
  for (uint32_t i = 0; i < *out_num_modes; it++, i++) {
    out_modes[i] = it->first;
    DLOGI("Color mode = %d is supported", out_modes[i]);
  }
  return HWC2::Error::None;
}

HWC2::Error HWCColorMode::GetRenderIntents(ColorMode mode, uint32_t *out_num_intents,
                                           RenderIntent *out_intents) {
  if (color_mode_map_.find(mode) == color_mode_map_.end()) {
    return HWC2::Error::BadParameter;
  }
  auto it = color_mode_map_[mode].begin();
  *out_num_intents = std::min(*out_num_intents, UINT32(color_mode_map_[mode].size()));
  for (uint32_t i = 0; i < *out_num_intents; it++, i++) {
    out_intents[i] = it->first;
    DLOGI("Color mode = %d is supported with render intent = %d", mode, out_intents[i]);
  }
  return HWC2::Error::None;
}

HWC2::Error HWCColorMode::ValidateColorModeWithRenderIntent(ColorMode mode, RenderIntent intent) {
  if (mode < ColorMode::NATIVE || mode > ColorMode::DISPLAY_BT2020) {
    DLOGE("Invalid mode: %d", mode);
    return HWC2::Error::BadParameter;
  }
  if (color_mode_map_.find(mode) == color_mode_map_.end()) {
    DLOGE("Could not find mode: %d", mode);
    return HWC2::Error::Unsupported;
  }
  if (color_mode_map_[mode].find(intent) == color_mode_map_[mode].end()) {
    DLOGE("Could not find render intent %d in mode %d", intent, mode);
    return HWC2::Error::Unsupported;
  }

  return HWC2::Error::None;
}

HWC2::Error HWCColorMode::SetColorModeWithRenderIntent(ColorMode mode, RenderIntent intent) {
  DTRACE_SCOPED();
  HWC2::Error hwc_error = ValidateColorModeWithRenderIntent(mode, intent);
  if (hwc_error != HWC2::Error::None) {
    return hwc_error;
  }

  if (current_color_mode_ == mode && current_render_intent_ == intent) {
    return HWC2::Error::None;
  }

  auto mode_string = color_mode_map_[mode][intent][kSdrType];
  DisplayError error = display_intf_->SetColorMode(mode_string);
  if (error != kErrorNone) {
    DLOGE("failed for mode = %d intent = %d name = %s", mode, intent, mode_string.c_str());
    return HWC2::Error::Unsupported;
  }
  // The mode does not have the PCC configured, restore the transform
  RestoreColorTransform();

  current_color_mode_ = mode;
  current_render_intent_ = intent;
  DLOGV_IF(kTagClient, "Successfully applied mode = %d intent = %d name = %s", mode, intent,
           mode_string.c_str());
  return HWC2::Error::None;
}

HWC2::Error HWCColorMode::CacheColorModeWithRenderIntent(ColorMode mode, RenderIntent intent) {
  HWC2::Error error = ValidateColorModeWithRenderIntent(mode, intent);
  if (error != HWC2::Error::None) {
    return error;
  }

  if (current_color_mode_ == mode && current_render_intent_ == intent) {
    return HWC2::Error::None;
  }

  current_color_mode_ = mode;
  current_render_intent_ = intent;
  apply_mode_ = true;

  return HWC2::Error::None;
}

HWC2::Error HWCColorMode::ApplyCurrentColorModeWithRenderIntent(bool hdr_present) {
  // If panel does not support color modes, do not set color mode.
  if (color_mode_map_.size() <= 1) {
    return HWC2::Error::None;
  }
  if (!apply_mode_) {
    if ((hdr_present && curr_dynamic_range_ == kHdrType) ||
      (!hdr_present && curr_dynamic_range_ == kSdrType))
      return HWC2::Error::None;
  }

  apply_mode_ = false;
  curr_dynamic_range_ = (hdr_present)? kHdrType : kSdrType;

  // select mode according to the blend space and dynamic range
  std::string mode_string = preferred_mode_[current_color_mode_][curr_dynamic_range_];
  if (mode_string.empty()) {
    mode_string = color_mode_map_[current_color_mode_][current_render_intent_][curr_dynamic_range_];
    if (mode_string.empty() && hdr_present) {
      // Use the colorimetric HDR mode, if an HDR mode with the current render intent is not present
      mode_string = color_mode_map_[current_color_mode_][RenderIntent::COLORIMETRIC][kHdrType];
    }
    if (mode_string.empty() &&
       (current_color_mode_ == ColorMode::DISPLAY_P3 ||
       current_color_mode_ == ColorMode::DISPLAY_BT2020) &&
       curr_dynamic_range_ == kHdrType) {
      // fall back to display_p3/display_bt2020 SDR mode if there is no HDR mode
      mode_string = color_mode_map_[current_color_mode_][current_render_intent_][kSdrType];
    }

    if (mode_string.empty() &&
       (current_color_mode_ == ColorMode::BT2100_PQ) && (curr_dynamic_range_ == kSdrType)) {
      // fallback to hdr mode.
      mode_string = color_mode_map_[current_color_mode_][current_render_intent_][kHdrType];
      DLOGI("fall back to hdr mode for ColorMode::BT2100_PQ kSdrType");
    }
  }

  auto error = SetPreferredColorModeInternal(mode_string, false, NULL, NULL);
  if (error == HWC2::Error::None) {
    // The mode does not have the PCC configured, restore the transform
    RestoreColorTransform();
    DLOGV_IF(kTagClient, "Successfully applied mode = %d intent = %d range = %d name = %s",
             current_color_mode_, current_render_intent_, curr_dynamic_range_, mode_string.c_str());
  }

  return error;
}

HWC2::Error HWCColorMode::SetColorModeById(int32_t color_mode_id) {
  DLOGI("Applying mode: %d", color_mode_id);
  DisplayError error = display_intf_->SetColorModeById(color_mode_id);
  if (error != kErrorNone) {
    DLOGI_IF(kTagClient, "Failed to apply mode: %d", color_mode_id);
    return HWC2::Error::BadParameter;
  }
  return HWC2::Error::None;
}

HWC2::Error HWCColorMode::SetPreferredColorModeInternal(const std::string &mode_string,
              bool from_client, ColorMode *color_mode, DynamicRangeType *dynamic_range) {
  DisplayError error = kErrorNone;
  ColorMode mode = ColorMode::NATIVE;
  DynamicRangeType range = kSdrType;

  if (from_client) {
    // get blend space and dynamic range of the mode
    AttrVal attr;
    std::string color_gamut_string, dynamic_range_string;
    error = display_intf_->GetColorModeAttr(mode_string, &attr);
    if (error) {
      DLOGE("Failed to get mode attributes for mode %s", mode_string.c_str());
      return HWC2::Error::BadParameter;
    }

    if (!attr.empty()) {
      for (auto &it : attr) {
        if (it.first.find(kColorGamutAttribute) != std::string::npos) {
          color_gamut_string = it.second;
        } else if (it.first.find(kDynamicRangeAttribute) != std::string::npos) {
          dynamic_range_string = it.second;
        }
      }
    }

    if (color_gamut_string.empty() || dynamic_range_string.empty()) {
      DLOGE("Invalid attributes for mode %s: color_gamut = %s, dynamic_range = %s",
            mode_string.c_str(), color_gamut_string.c_str(), dynamic_range_string.c_str());
      return HWC2::Error::BadParameter;
    }

    if (color_gamut_string == kDcip3) {
      mode = ColorMode::DISPLAY_P3;
    } else if (color_gamut_string == kSrgb) {
      mode = ColorMode::SRGB;
    }
    if (dynamic_range_string == kHdr) {
      range = kHdrType;
    }

    if (color_mode) {
      *color_mode = mode;
    }
    if (dynamic_range) {
      *dynamic_range = range;
    }
  }

  // apply the mode from client if it matches
  // the current blend space and dynamic range,
  // skip the check for the mode from SF.
  if ((!from_client) || (current_color_mode_ == mode && curr_dynamic_range_ == range)) {
    DLOGI("Applying mode: %s", mode_string.c_str());
    error = display_intf_->SetColorMode(mode_string);
    if (error != kErrorNone) {
      DLOGE("Failed to apply mode: %s", mode_string.c_str());
      return HWC2::Error::BadParameter;
    }
  }

  return HWC2::Error::None;
}

HWC2::Error HWCColorMode::SetColorModeFromClientApi(std::string mode_string) {
  ColorMode mode = ColorMode::NATIVE;
  DynamicRangeType range = kSdrType;

  auto error = SetPreferredColorModeInternal(mode_string, true, &mode, &range);
  if (error == HWC2::Error::None) {
    preferred_mode_[mode][range] = mode_string;
    DLOGV_IF(kTagClient, "Put mode %s(mode %d, range %d) into preferred_mode",
             mode_string.c_str(), mode, range);
  }

  return error;
}

HWC2::Error HWCColorMode::RestoreColorTransform() {
  DisplayError error = display_intf_->SetColorTransform(kColorTransformMatrixCount, color_matrix_);
  if (error != kErrorNone) {
    DLOGE("Failed to set Color Transform");
    return HWC2::Error::BadParameter;
  }

  return HWC2::Error::None;
}

HWC2::Error HWCColorMode::SetColorTransform(const float *matrix,
                                            android_color_transform_t /*hint*/) {
  DTRACE_SCOPED();
  auto status = HWC2::Error::None;
  double color_matrix[kColorTransformMatrixCount] = {0};
  CopyColorTransformMatrix(matrix, color_matrix);

  DisplayError error = display_intf_->SetColorTransform(kColorTransformMatrixCount, color_matrix);
  if (error != kErrorNone) {
    DLOGE("Failed to set Color Transform Matrix");
    status = HWC2::Error::Unsupported;
  }
  CopyColorTransformMatrix(matrix, color_matrix_);
  return status;
}

void HWCColorMode::PopulateColorModes() {
  uint32_t color_mode_count = 0;
  // SDM returns modes which have attributes defining mode and rendering intent
  DisplayError error = display_intf_->GetColorModeCount(&color_mode_count);
  if (error != kErrorNone || (color_mode_count == 0)) {
    DLOGW("GetColorModeCount failed, use native color mode");
    color_mode_map_[ColorMode::NATIVE][RenderIntent::COLORIMETRIC]
                   [kSdrType] = "hal_native_identity";
    return;
  }

  DLOGV_IF(kTagClient, "Color Modes supported count = %d", color_mode_count);

  std::vector<std::string> color_modes(color_mode_count);
  error = display_intf_->GetColorModes(&color_mode_count, &color_modes);
  for (uint32_t i = 0; i < color_mode_count; i++) {
    std::string &mode_string = color_modes.at(i);
    DLOGV_IF(kTagClient, "Color Mode[%d] = %s", i, mode_string.c_str());
    AttrVal attr;
    error = display_intf_->GetColorModeAttr(mode_string, &attr);
    std::string color_gamut = kNative, dynamic_range = kSdr, pic_quality = kStandard, transfer;
    int int_render_intent = -1;
    if (!attr.empty()) {
      for (auto &it : attr) {
        if (it.first.find(kColorGamutAttribute) != std::string::npos) {
          color_gamut = it.second;
        } else if (it.first.find(kDynamicRangeAttribute) != std::string::npos) {
          dynamic_range = it.second;
        } else if (it.first.find(kPictureQualityAttribute) != std::string::npos) {
          pic_quality = it.second;
        } else if (it.first.find(kGammaTransferAttribute) != std::string::npos) {
          transfer = it.second;
        } else if (it.first.find(kRenderIntentAttribute) != std::string::npos) {
          int_render_intent = std::stoi(it.second);
        }
      }

      if (int_render_intent < 0 || int_render_intent > MAX_EXTENDED_RENDER_INTENT) {
        DLOGW("Invalid render intent %d for mode %s", int_render_intent, mode_string.c_str());
        continue;
      }
      DLOGV_IF(kTagClient, "color_gamut : %s, dynamic_range : %s, pic_quality : %s, "
               "render_intent : %d", color_gamut.c_str(), dynamic_range.c_str(),
               pic_quality.c_str(), int_render_intent);

      auto render_intent = static_cast<RenderIntent>(int_render_intent);
      if (color_gamut == kNative) {
        color_mode_map_[ColorMode::NATIVE][render_intent][kSdrType] = mode_string;
      }

      if (color_gamut == kSrgb && dynamic_range == kSdr) {
        color_mode_map_[ColorMode::SRGB][render_intent][kSdrType] = mode_string;
      }

      if (color_gamut == kDcip3 && dynamic_range == kSdr) {
        color_mode_map_[ColorMode::DISPLAY_P3][render_intent][kSdrType] = mode_string;
      }
      if (color_gamut == kDcip3 && dynamic_range == kHdr) {
        if (display_intf_->IsSupportSsppTonemap()) {
          color_mode_map_[ColorMode::DISPLAY_P3][render_intent][kHdrType] = mode_string;
        } else if (pic_quality == kStandard) {
          color_mode_map_[ColorMode::BT2100_PQ][render_intent]
                         [kHdrType] = mode_string;
          color_mode_map_[ColorMode::BT2100_HLG][render_intent]
                         [kHdrType] = mode_string;
        }
      } else if (color_gamut == kBt2020) {
        if (transfer == kSt2084) {
          color_mode_map_[ColorMode::BT2100_PQ][RenderIntent::COLORIMETRIC]
                         [kHdrType] = mode_string;
        } else if (transfer == kHlg) {
          color_mode_map_[ColorMode::BT2100_HLG][RenderIntent::COLORIMETRIC]
                         [kHdrType] = mode_string;
        } else if (transfer == kSrgb) {
          color_mode_map_[ColorMode::DISPLAY_BT2020][RenderIntent::COLORIMETRIC]
                         [kSdrType] = mode_string;
        }
      }
    } else {
      // Look at the mode names, if no attributes are found
      if (mode_string.find("hal_native") != std::string::npos) {
        color_mode_map_[ColorMode::NATIVE][RenderIntent::COLORIMETRIC]
                       [kSdrType] = mode_string;
      }
    }
  }
}

void HWCColorMode::Dump(std::ostringstream* os) {
  *os << "color modes supported: \n";
  for (auto it : color_mode_map_) {
    *os << "mode: " << static_cast<int32_t>(it.first) << " RIs { ";
    for (auto render_intent_it : color_mode_map_[it.first]) {
      *os << static_cast<int32_t>(render_intent_it.first) << " dynamic_range [ ";
      for (auto range_it : color_mode_map_[it.first][render_intent_it.first]) {
        *os << static_cast<int32_t>(range_it.first) << " ";
      }
      *os << "] ";
    }
    *os << "} \n";
  }
  *os << "current mode: " << static_cast<uint32_t>(current_color_mode_) << std::endl;
  *os << "current render_intent: " << static_cast<uint32_t>(current_render_intent_) << std::endl;
  if (curr_dynamic_range_ == kHdrType) {
    *os << "current dynamic_range: HDR" << std::endl;
  } else {
    *os << "current dynamic_range: SDR" << std::endl;
  }
  *os << "current transform: ";
  for (uint32_t i = 0; i < kColorTransformMatrixCount; i++) {
    if (i % 4 == 0) {
     *os << std::endl;
    }
    *os << std::fixed << std::setprecision(2) << std::setw(6) << std::setfill(' ')
        << color_matrix_[i] << " ";
  }
  *os << std::endl;
}

HWCDisplay::HWCDisplay(CoreInterface *core_intf, BufferAllocator *buffer_allocator,
                       HWCCallbacks *callbacks, HWCDisplayEventHandler* event_handler,
                       qService::QService *qservice, DisplayType type, hwc2_display_t id,
                       int32_t sdm_id, DisplayClass display_class)
    : core_intf_(core_intf),
      callbacks_(callbacks),
      event_handler_(event_handler),
      type_(type),
      id_(id),
      sdm_id_(sdm_id),
      qservice_(qservice),
      display_class_(display_class) {
  buffer_allocator_ = static_cast<HWCBufferAllocator *>(buffer_allocator);
}

int HWCDisplay::Init() {
  DisplayError error = kErrorNone;

  HWCDebugHandler::Get()->GetProperty(ENABLE_NULL_DISPLAY_PROP, &null_display_mode_);
  HWCDebugHandler::Get()->GetProperty(ENABLE_ASYNC_POWERMODE, &async_power_mode_);

  if (null_display_mode_) {
    DisplayNull *disp_null = new DisplayNull();
    disp_null->Init();
    use_metadata_refresh_rate_ = false;
    display_intf_ = disp_null;
    DLOGI("Enabling null display mode for display type %d", type_);
  } else {
    error = core_intf_->CreateDisplay(sdm_id_, this, &display_intf_);
    if (error != kErrorNone) {
      if (kErrorDeviceRemoved == error) {
        DLOGW("Display creation cancelled. Display %d-%d removed.", sdm_id_, type_);
        return -ENODEV;
      } else {
        DLOGE("Display create failed. Error = %d display_id = %d event_handler = %p disp_intf = %p",
              error, sdm_id_, this, &display_intf_);
        return -EINVAL;
      }
    }
  }

  validated_ = false;
  HWCDebugHandler::Get()->GetProperty(DISABLE_HDR, &disable_hdr_handling_);
  if (disable_hdr_handling_) {
    DLOGI("HDR Handling disabled");
  }

  int property_swap_interval = 1;
  HWCDebugHandler::Get()->GetProperty(ZERO_SWAP_INTERVAL, &property_swap_interval);
  if (property_swap_interval == 0) {
    swap_interval_zero_ = true;
  }

  client_target_ = new HWCLayer(id_, buffer_allocator_);

  error = display_intf_->GetNumVariableInfoConfigs(&num_configs_);
  if (error != kErrorNone) {
    DLOGE("Getting config count failed. Error = %d", error);
    return -EINVAL;
  }

  bool is_primary_ = display_intf_->IsPrimaryDisplay();
  if (is_primary_) {
    int value = 0;
    HWCDebugHandler::Get()->GetProperty(ENABLE_POMS_DURING_DOZE, &value);
    enable_poms_during_doze_ = (value == 1);
    if (enable_poms_during_doze_) {
      DLOGI("Enable POMS during Doze mode %" PRIu64 , id_);
    }
  }

  UpdateConfigs();

  tone_mapper_ = new HWCToneMapper(buffer_allocator_);

  display_intf_->GetRefreshRateRange(&min_refresh_rate_, &max_refresh_rate_);
  current_refresh_rate_ = max_refresh_rate_;

  GetUnderScanConfig();

  DisplayConfigFixedInfo fixed_info = {};
  display_intf_->GetConfig(&fixed_info);
  is_cmd_mode_ = fixed_info.is_cmdmode;
  partial_update_enabled_ = fixed_info.partial_update || (!fixed_info.is_cmdmode);
  client_target_->SetPartialUpdate(partial_update_enabled_);

  int disable_fast_path = 0;
  HWCDebugHandler::Get()->GetProperty(DISABLE_FAST_PATH, &disable_fast_path);
  fast_path_enabled_ = !(disable_fast_path == 1);

  game_supported_ = display_intf_->GameEnhanceSupported();

  DLOGI("Display created with id: %d, game_supported_: %d", UINT32(id_), game_supported_);

  return 0;
}

void HWCDisplay::UpdateConfigs() {
  // SF doesnt care about dynamic bit clk support.
  // Exposing all configs will result in getting/setting of redundant configs.

  // For each config store the corresponding index which client understands.
  hwc_config_map_.resize(num_configs_);

  for (uint32_t i = 0; i < num_configs_; i++) {
    DisplayConfigVariableInfo info = {};
    GetDisplayAttributesForConfig(INT(i), &info);
    bool config_exists = false;

    if (!smart_panel_config_ && info.smart_panel) {
      smart_panel_config_ = true;
    }

    for (auto &config : variable_config_map_) {
      if (config.second == info) {
        if (enable_poms_during_doze_ || (config.second.smart_panel == info.smart_panel)) {
          config_exists = true;
          hwc_config_map_.at(i) = config.first;
          break;
        }
      }
    }

    if (!config_exists) {
      variable_config_map_[i] = info;
      hwc_config_map_.at(i) = i;
    }
  }

  if (num_configs_ != 0) {
    hwc2_config_t active_config = hwc_config_map_.at(0);
    GetActiveConfig(&active_config);
    SetActiveConfigIndex(active_config);
  }

  // Update num config count.
  num_configs_ = UINT32(variable_config_map_.size());
  DLOGI("num_configs = %d smart_panel_config_ = %d", num_configs_, smart_panel_config_);
}

int HWCDisplay::Deinit() {
  if (null_display_mode_) {
    delete static_cast<DisplayNull *>(display_intf_);
    display_intf_ = nullptr;
  } else {
    DisplayError error = core_intf_->DestroyDisplay(display_intf_);
    if (error != kErrorNone) {
      DLOGE("Display destroy failed. Error = %d", error);
      return -EINVAL;
    }
  }

  delete client_target_;
  for (auto hwc_layer : layer_set_) {
    delete hwc_layer;
  }

  if (color_mode_) {
    color_mode_->DeInit();
    delete color_mode_;
  }

  if (tone_mapper_) {
    delete tone_mapper_;
    tone_mapper_ = nullptr;
  }

  return 0;
}

// LayerStack operations
HWC2::Error HWCDisplay::CreateLayer(hwc2_layer_t *out_layer_id) {
  HWCLayer *layer = *layer_set_.emplace(new HWCLayer(id_, buffer_allocator_));
  layer_map_.emplace(std::make_pair(layer->GetId(), layer));
  *out_layer_id = layer->GetId();
  geometry_changes_ |= GeometryChanges::kAdded;
  validated_ = false;
  layer_stack_invalid_ = true;
  layer->SetPartialUpdate(partial_update_enabled_);

  return HWC2::Error::None;
}

HWCLayer *HWCDisplay::GetHWCLayer(hwc2_layer_t layer_id) {
  const auto map_layer = layer_map_.find(layer_id);
  if (map_layer == layer_map_.end()) {
    DLOGW("[%" PRIu64 "] GetLayer(%" PRIu64 ") failed: no such layer", id_, layer_id);
    return nullptr;
  } else {
    return map_layer->second;
  }
}

HWC2::Error HWCDisplay::DestroyLayer(hwc2_layer_t layer_id) {
  const auto map_layer = layer_map_.find(layer_id);
  if (map_layer == layer_map_.end()) {
    DLOGW("[%" PRIu64 "] destroyLayer(%" PRIu64 ") failed: no such layer", id_, layer_id);
    return HWC2::Error::BadLayer;
  }
  const auto layer = map_layer->second;
  layer_map_.erase(map_layer);
  const auto z_range = layer_set_.equal_range(layer);
  for (auto current = z_range.first; current != z_range.second; ++current) {
    if (*current == layer) {
      current = layer_set_.erase(current);
      delete layer;
      break;
    }
  }

  geometry_changes_ |= GeometryChanges::kRemoved;
  validated_ = false;
  layer_stack_invalid_ = true;

  return HWC2::Error::None;
}


void HWCDisplay::BuildLayerStack() {
  layer_stack_ = LayerStack();
  display_rect_ = LayerRect();
  metadata_refresh_rate_ = 0;
  layer_stack_.flags.animating = animating_;
  layer_stack_.flags.fast_path = fast_path_enabled_ && fast_path_composition_;

  DTRACE_SCOPED();
  // Add one layer for fb target
  for (auto hwc_layer : layer_set_) {
    // Reset layer data which SDM may change
    hwc_layer->ResetPerFrameData();

    Layer *layer = hwc_layer->GetSDMLayer();
    layer->flags = {};   // Reset earlier flags
    // Mark all layers to skip, when client target handle is NULL
    if (hwc_layer->GetClientRequestedCompositionType() == HWC2::Composition::Client ||
        !client_target_->GetSDMLayer()->input_buffer.buffer_id) {
      layer->flags.skip = true;
    } else if (hwc_layer->GetClientRequestedCompositionType() == HWC2::Composition::SolidColor) {
      layer->flags.solid_fill = true;
    }

#ifdef FOD_ZPOS
    if (hwc_layer->IsFodPressed()) {
      layer->flags.fod_pressed = true;
      layer_stack_.flags.fod_pressed_present = true;
    } else {
      layer->flags.fod_pressed = false;
      layer_stack_.flags.fod_pressed_present = false;
    }
#endif

    if (!hwc_layer->IsDataSpaceSupported()) {
      layer->flags.skip = true;
    }

    // set default composition as GPU for SDM
    layer->composition = kCompositionGPU;

    if (swap_interval_zero_) {
      layer->input_buffer.acquire_fence = nullptr;
    }

    bool is_secure = false;
    bool is_video = false;
    const private_handle_t *handle =
        reinterpret_cast<const private_handle_t *>(layer->input_buffer.buffer_id);
    if (handle) {
      if (handle->buffer_type == BUFFER_TYPE_VIDEO) {
        layer_stack_.flags.video_present = true;
        is_video = true;
      }
      // TZ Protected Buffer - L1
      // Gralloc Usage Protected Buffer - L3 - which needs to be treated as Secure & avoid fallback
      if (handle->flags & private_handle_t::PRIV_FLAGS_PROTECTED_BUFFER ||
          handle->flags & private_handle_t::PRIV_FLAGS_SECURE_BUFFER) {
        layer_stack_.flags.secure_present = true;
        is_secure = true;
      }
      // UBWC PI format
      if (handle->flags & private_handle_t::PRIV_FLAGS_UBWC_ALIGNED_PI) {
        layer->input_buffer.flags.ubwc_pi = true;
      }
    }

    if (layer->input_buffer.flags.secure_display) {
      layer_stack_.flags.secure_present = true;
      is_secure = true;
    }

    if (IS_RGB_FORMAT(layer->input_buffer.format) && hwc_layer->IsScalingPresent()) {
      layer_stack_.flags.scaling_rgb_layer_present = true;
    }

    if (hwc_layer->IsSingleBuffered() &&
       !(hwc_layer->IsRotationPresent() || hwc_layer->IsScalingPresent())) {
      layer->flags.single_buffer = true;
      layer_stack_.flags.single_buffered_layer_present = true;
    }

    bool hdr_layer = layer->input_buffer.color_metadata.colorPrimaries == ColorPrimaries_BT2020 &&
                     (layer->input_buffer.color_metadata.transfer == Transfer_SMPTE_ST2084 ||
                     layer->input_buffer.color_metadata.transfer == Transfer_HLG);
    if (hdr_layer && !disable_hdr_handling_) {
      // Dont honor HDR when its handling is disabled
      layer->input_buffer.flags.hdr = true;
      layer_stack_.flags.hdr_present = true;
    }

    if (game_supported_ && (hwc_layer->GetType() == kLayerGame)) {
      layer->flags.is_game = true;
      layer->input_buffer.flags.game = true;
    }

    if (hwc_layer->IsNonIntegralSourceCrop() && !is_secure && !hdr_layer &&
        !layer->flags.single_buffer && !layer->flags.solid_fill && !is_video &&
        !layer->flags.is_game) {
      layer->flags.skip = true;
    }

    if (!layer->flags.skip &&
        (hwc_layer->GetClientRequestedCompositionType() == HWC2::Composition::Cursor)) {
      // Currently we support only one HWCursor & only at top most z-order
      if ((*layer_set_.rbegin())->GetId() == hwc_layer->GetId()) {
        layer->flags.cursor = true;
        layer_stack_.flags.cursor_present = true;
      }
    }

    if (layer->flags.skip) {
      layer_stack_.flags.skip_present = true;
    }

    // TODO(user): Move to a getter if this is needed at other places
    hwc_rect_t scaled_display_frame = {INT(layer->dst_rect.left), INT(layer->dst_rect.top),
                                       INT(layer->dst_rect.right), INT(layer->dst_rect.bottom)};
    if (hwc_layer->GetGeometryChanges() & kDisplayFrame) {
      ApplyScanAdjustment(&scaled_display_frame);
    }
    hwc_layer->SetLayerDisplayFrame(scaled_display_frame);
    hwc_layer->ResetPerFrameData();
    // SDM requires these details even for solid fill
    if (layer->flags.solid_fill) {
      LayerBuffer *layer_buffer = &layer->input_buffer;
      layer_buffer->width = UINT32(layer->dst_rect.right - layer->dst_rect.left);
      layer_buffer->height = UINT32(layer->dst_rect.bottom - layer->dst_rect.top);
      layer_buffer->unaligned_width = layer_buffer->width;
      layer_buffer->unaligned_height = layer_buffer->height;
      layer->src_rect.left = 0;
      layer->src_rect.top = 0;
      layer->src_rect.right = layer_buffer->width;
      layer->src_rect.bottom = layer_buffer->height;
    }

    if (hwc_layer->HasMetaDataRefreshRate() && layer->frame_rate > metadata_refresh_rate_) {
      metadata_refresh_rate_ = SanitizeRefreshRate(layer->frame_rate);
    }

    display_rect_ = Union(display_rect_, layer->dst_rect);
    geometry_changes_ |= hwc_layer->GetGeometryChanges();

    layer->flags.updating = true;
    if (layer_set_.size() <= kMaxLayerCount) {
      layer->flags.updating = IsLayerUpdating(hwc_layer);
    }

    if (hwc_layer->IsColorTransformSet()) {
      layer->flags.color_transform = true;
    }

    layer_stack_.flags.mask_present |= layer->input_buffer.flags.mask_layer;

    if ((hwc_layer->GetDeviceSelectedCompositionType() != HWC2::Composition::Device) ||
        (hwc_layer->GetClientRequestedCompositionType() != HWC2::Composition::Device) ||
        layer->flags.skip) {
      layer->update_mask.set(kClientCompRequest);
    }

    layer_stack_.layers.push_back(layer);
  }

  // If layer stack needs Client composition, HWC display gets into InternalValidate state. If
  // validation gets reset by any other thread in this state, enforce Geometry change to ensure
  // that Client target gets composed by SF.
  bool enforce_geometry_change = (validate_state_ == kInternalValidate) && !validated_;

  // TODO(user): Set correctly when SDM supports geometry_changes as bitmask

  layer_stack_.flags.geometry_changed = UINT32((geometry_changes_ || enforce_geometry_change ||
                                                geometry_changes_on_doze_suspend_) > 0);
  layer_stack_.flags.config_changed = !validated_;
  // Append client target to the layer stack
  Layer *sdm_client_target = client_target_->GetSDMLayer();
  sdm_client_target->flags.updating = IsLayerUpdating(client_target_);
  // Derive client target dataspace based on the color mode - bug/115482728
  int32_t client_target_dataspace = GetDataspaceFromColorMode(GetCurrentColorMode());
  SetClientTargetDataSpace(client_target_dataspace);
  layer_stack_.layers.push_back(sdm_client_target);
}

void HWCDisplay::BuildSolidFillStack() {
  layer_stack_ = LayerStack();
  display_rect_ = LayerRect();

  layer_stack_.layers.push_back(solid_fill_layer_);
  layer_stack_.flags.geometry_changed = 1U;
  // Append client target to the layer stack
  layer_stack_.layers.push_back(client_target_->GetSDMLayer());
}

HWC2::Error HWCDisplay::SetLayerType(hwc2_layer_t layer_id, IQtiComposerClient::LayerType type) {
  const auto map_layer = layer_map_.find(layer_id);
  if (map_layer == layer_map_.end()) {
    DLOGE("[%" PRIu64 "] SetLayerType failed to find layer", id_);
    return HWC2::Error::BadLayer;
  }

  const auto layer = map_layer->second;
  layer->SetLayerType(type);
  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::SetLayerZOrder(hwc2_layer_t layer_id, uint32_t z) {
  const auto map_layer = layer_map_.find(layer_id);
  if (map_layer == layer_map_.end()) {
    DLOGW("[%" PRIu64 "] updateLayerZ failed to find layer", id_);
    return HWC2::Error::BadLayer;
  }

  const auto layer = map_layer->second;
  const auto z_range = layer_set_.equal_range(layer);
  bool layer_on_display = false;
  for (auto current = z_range.first; current != z_range.second; ++current) {
    if (*current == layer) {
      if ((*current)->GetZ() == z) {
        // Don't change anything if the Z hasn't changed
        return HWC2::Error::None;
      }
      current = layer_set_.erase(current);
      layer_on_display = true;
      break;
    }
  }

  if (!layer_on_display) {
    DLOGE("[%" PRIu64 "] updateLayerZ failed to find layer on display", id_);
    return HWC2::Error::BadLayer;
  }

  layer->SetLayerZOrder(z);
  layer_set_.emplace(layer);
  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::SetVsyncEnabled(HWC2::Vsync enabled) {
  DLOGV("Display ID: %" PRId64 " enabled: %s", id_, to_string(enabled).c_str());
  ATRACE_INT("SetVsyncState ", enabled == HWC2::Vsync::Enable ? 1 : 0);
  DisplayError error = kErrorNone;

  if (shutdown_pending_ ||
      (!callbacks_->VsyncCallbackRegistered() && !callbacks_->Vsync_2_4CallbackRegistered())) {
    return HWC2::Error::None;
  }

  bool state;
  if (enabled == HWC2::Vsync::Enable)
    state = true;
  else if (enabled == HWC2::Vsync::Disable)
    state = false;
  else
    return HWC2::Error::BadParameter;

  error = display_intf_->SetVSyncState(state);

  if (error != kErrorNone) {
    if (error == kErrorShutDown) {
      shutdown_pending_ = true;
      return HWC2::Error::None;
    }
    DLOGE("Failed. enabled = %s, error = %d", to_string(enabled).c_str(), error);
    return HWC2::Error::BadDisplay;
  }

  return HWC2::Error::None;
}

void HWCDisplay::PostPowerMode() {
  if (release_fence_ == nullptr) {
    return;
  }

  for (auto hwc_layer : layer_set_) {
    shared_ptr<Fence> fence = nullptr;
    shared_ptr<Fence> merged_fence = nullptr;

    hwc_layer->PopBackReleaseFence(&fence);
    if (fence) {
      merged_fence = Fence::Merge(release_fence_, fence);
    } else {
      merged_fence = release_fence_;
    }
    hwc_layer->PushBackReleaseFence(merged_fence);
  }

  fbt_release_fence_ = release_fence_;
}

HWC2::Error HWCDisplay::SetPowerMode(HWC2::PowerMode mode, bool teardown) {
  DLOGV("display = %" PRId64 ", mode = %s", id_, to_string(mode).c_str());
  DisplayState state = kStateOff;
  bool flush_on_error = flush_on_error_;

  if (shutdown_pending_) {
    return HWC2::Error::None;
  }

  switch (mode) {
    case HWC2::PowerMode::Off:
      // During power off, all of the buffers are released.
      // Do not flush until a buffer is successfully submitted again.
      flush_on_error = false;
      state = kStateOff;
      if (tone_mapper_) {
        tone_mapper_->Terminate();
      }
      break;
    case HWC2::PowerMode::On:
      RestoreColorTransform();
      state = kStateOn;
      break;
    case HWC2::PowerMode::Doze:
      RestoreColorTransform();
      state = kStateDoze;
      break;
    case HWC2::PowerMode::DozeSuspend:
      state = kStateDozeSuspend;
      break;
    default:
      return HWC2::Error::BadParameter;
  }
  shared_ptr<Fence> release_fence = nullptr;

  ATRACE_INT("SetPowerMode ", state);
  DisplayError error = display_intf_->SetDisplayState(state, teardown, &release_fence);
  validated_ = false;

  if (error == kErrorNone) {
    flush_on_error_ = flush_on_error;
  } else {
    if (error == kErrorShutDown) {
      shutdown_pending_ = true;
      return HWC2::Error::None;
    }
    DLOGE("Set state failed. Error = %d", error);
    return HWC2::Error::BadParameter;
  }

  // Update release fence.
  release_fence_ = release_fence;
  current_power_mode_ = mode;

  // Close the release fences in synchronous power updates
  if (!async_power_mode_) {
    PostPowerMode();
  }
  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::GetClientTargetSupport(uint32_t width, uint32_t height, int32_t format,
                                               int32_t dataspace) {
  ColorMetaData color_metadata = {};
  if (dataspace != HAL_DATASPACE_UNKNOWN) {
    dataspace = TranslateFromLegacyDataspace(dataspace);
    GetColorPrimary(dataspace, &(color_metadata.colorPrimaries));
    GetTransfer(dataspace, &(color_metadata.transfer));
    GetRange(dataspace, &(color_metadata.range));
  }

  LayerBufferFormat sdm_format = HWCLayer::GetSDMFormat(format, 0);
  if (display_intf_->GetClientTargetSupport(width, height, sdm_format,
                                            color_metadata) != kErrorNone) {
    return HWC2::Error::Unsupported;
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::GetColorModes(uint32_t *out_num_modes, ColorMode *out_modes) {
  if (out_modes == nullptr) {
    *out_num_modes = 1;
  } else if (out_modes && *out_num_modes > 0) {
    *out_num_modes = 1;
    out_modes[0] = ColorMode::NATIVE;
  }
  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::GetRenderIntents(ColorMode mode, uint32_t *out_num_intents,
                                         RenderIntent *out_intents) {
  if (mode != ColorMode::NATIVE) {
    return HWC2::Error::Unsupported;
  }
  if (out_intents == nullptr) {
    *out_num_intents = 1;
  } else if (out_intents && *out_num_intents > 0) {
    *out_num_intents = 1;
    out_intents[0] = RenderIntent::COLORIMETRIC;
  }
  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::GetDisplayConfigs(uint32_t *out_num_configs, hwc2_config_t *out_configs) {
  if (out_num_configs == nullptr) {
    return HWC2::Error::BadParameter;
  }

  if (out_configs == nullptr) {
    *out_num_configs = num_configs_;
    return HWC2::Error::None;
  }

  *out_num_configs = std::min(*out_num_configs, num_configs_);

  // Expose all unique config ids to cleint.
  uint32_t i = 0;
  for (auto &info : variable_config_map_) {
    if (i == *out_num_configs) {
      break;
    }
    out_configs[i++] = info.first;
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::GetDisplayAttribute(hwc2_config_t config, HwcAttribute attribute,
                                            int32_t *out_value) {
  if (variable_config_map_.find(config) == variable_config_map_.end()) {
    DLOGE("Get variable config failed");
    return HWC2::Error::BadConfig;
  }

  DisplayConfigVariableInfo variable_config = variable_config_map_.at(config);

  uint32_t x_pixels = variable_config.x_pixels - UINT32(window_rect_.right + window_rect_.left);
  uint32_t y_pixels = variable_config.y_pixels - UINT32(window_rect_.bottom + window_rect_.top);
  if (x_pixels <= 0 || y_pixels <= 0) {
    DLOGE("window rects are not within the supported range");
    return HWC2::Error::BadDisplay;
  }

  switch (attribute) {
    case HwcAttribute::VSYNC_PERIOD:
      *out_value = INT32(variable_config.vsync_period_ns);
      break;
    case HwcAttribute::WIDTH:
      *out_value = INT32(x_pixels);
      break;
    case HwcAttribute::HEIGHT:
      *out_value = INT32(y_pixels);
      break;
    case HwcAttribute::DPI_X:
      *out_value = INT32(variable_config.x_dpi * 1000.0f);
      break;
    case HwcAttribute::DPI_Y:
      *out_value = INT32(variable_config.y_dpi * 1000.0f);
      break;
    case HwcAttribute::CONFIG_GROUP:
      *out_value = GetDisplayConfigGroup(variable_config);
      break;
    default:
      DLOGW("Spurious attribute type = %s", composer_V2_4::toString(attribute).c_str());
      *out_value = -1;
      return HWC2::Error::BadParameter;
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::GetDisplayName(uint32_t *out_size, char *out_name) {
  // TODO(user): Get panel name and EDID name and populate it here
  if (out_size == nullptr) {
    return HWC2::Error::BadParameter;
  }

  std::string name;
  switch (type_) {
    case kBuiltIn:
      name = "Built-in Display";
      break;
    case kPluggable:
      name = "Pluggable Display";
      break;
    case kVirtual:
      name = "Virtual Display";
      break;
    default:
      name = "Unknown";
      break;
  }

  if (out_name == nullptr) {
    *out_size = UINT32(name.size()) + 1;
  } else {
    *out_size = std::min((UINT32(name.size()) + 1), *out_size);
    if (*out_size > 0) {
      strlcpy(out_name, name.c_str(), *out_size);
      out_name[*out_size - 1] = '\0';
    } else {
      DLOGW("Invalid size requested");
    }
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::GetDisplayType(int32_t *out_type) {
  if (out_type == nullptr) {
    return HWC2::Error::BadParameter;
  }

  *out_type = HWC2_DISPLAY_TYPE_PHYSICAL;

  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::GetPerFrameMetadataKeys(uint32_t *out_num_keys,
                                                PerFrameMetadataKey *out_keys) {
  if (out_num_keys == nullptr) {
    return HWC2::Error::BadParameter;
  }

  DisplayConfigFixedInfo fixed_info = {};
  display_intf_->GetConfig(&fixed_info);
  uint32_t num_keys = 0;
  if (fixed_info.hdr_plus_supported) {
    num_keys = UINT32(PerFrameMetadataKey::HDR10_PLUS_SEI) + 1;
  } else {
    num_keys = UINT32(PerFrameMetadataKey::MAX_FRAME_AVERAGE_LIGHT_LEVEL) + 1;
  }
  if (out_keys == nullptr) {
    *out_num_keys = num_keys;
  } else {
    uint32_t max_out_key_elements = std::min(*out_num_keys, num_keys);
    for (int32_t i = 0; i < max_out_key_elements; i++) {
      out_keys[i] = static_cast<PerFrameMetadataKey>(i);
    }
  }
  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::GetActiveConfig(hwc2_config_t *out_config) {
  if (out_config == nullptr) {
    return HWC2::Error::BadDisplay;
  }

  if (pending_config_) {
    *out_config = pending_config_index_;
  } else {
    GetActiveDisplayConfig(out_config);
  }

  if (*out_config < hwc_config_map_.size()) {
    *out_config = hwc_config_map_.at(*out_config);
  }
  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::SetClientTarget(buffer_handle_t target, shared_ptr<Fence> acquire_fence,
                                        int32_t dataspace, hwc_region_t damage) {
  // TODO(user): SurfaceFlinger gives us a null pointer here when doing full SDE composition
  // The error is problematic for layer caching as it would overwrite our cached client target.
  // Reported bug 28569722 to resolve this.
  // For now, continue to use the last valid buffer reported to us for layer caching.
  if (target == nullptr) {
    return HWC2::Error::None;
  }

  if (acquire_fence == nullptr) {
    DLOGV_IF(kTagClient, "Re-using cached buffer");
  }

  Layer *sdm_layer = client_target_->GetSDMLayer();
  sdm_layer->frame_rate = std::min(current_refresh_rate_, HWCDisplay::GetThrottlingRefreshRate());
  client_target_->SetLayerSurfaceDamage(damage);
  client_target_->SetLayerBuffer(target, acquire_fence);
  client_target_->SetLayerDataspace(dataspace);
  client_target_handle_ = target;
  client_acquire_fence_ = acquire_fence;
  client_dataspace_     = dataspace;
  client_damage_region_ = damage;

  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::GetClientTarget(buffer_handle_t target, shared_ptr<Fence> acquire_fence,
                                        int32_t dataspace, hwc_region_t damage) {
  target        = client_target_handle_;
  acquire_fence = client_acquire_fence_;
  dataspace     = client_dataspace_;
  damage        = client_damage_region_;

  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::SetActiveConfig(hwc2_config_t config) {
  DTRACE_SCOPED();
  hwc2_config_t current_config = 0;
  GetActiveConfig(&current_config);
  if (current_config == config) {
    return HWC2::Error::None;
  }

  // DRM driver expects DRM_PREFERRED_MODE to be set as part of first commit.
  if (!IsFirstCommitDone()) {
    // Store client's config.
    // Set this as part of post commit.
    pending_first_commit_config_ = true;
    pending_first_commit_config_index_ = config;
    DLOGI("Defer config change to %d until first commit", UINT32(config));
    return HWC2::Error::None;
  } else if (pending_first_commit_config_) {
    // Config override request from client.
    // Honour latest request.
    pending_first_commit_config_ = false;
  }

  DLOGI("Active configuration changed to: %d", config);

  // Cache refresh rate set by client.
  DisplayConfigVariableInfo info = {};
  GetDisplayAttributesForConfig(INT(config), &info);
  active_refresh_rate_ = info.fps;

  // Store config index to be applied upon refresh.
  pending_config_ = true;
  pending_config_index_ = config;

  validated_ = false;

  // Trigger refresh. This config gets applied on next commit.
  callbacks_->Refresh(id_);

  return HWC2::Error::None;
}

DisplayError HWCDisplay::SetMixerResolution(uint32_t width, uint32_t height) {
  return kErrorNotSupported;
}

HWC2::Error HWCDisplay::SetFrameDumpConfig(uint32_t count, uint32_t bit_mask_layer_type,
                                           int32_t format, bool post_processed) {
  dump_frame_count_ = count;
  dump_frame_index_ = 0;
  dump_input_layers_ = ((bit_mask_layer_type & (1 << INPUT_LAYER_DUMP)) != 0);

  if (tone_mapper_) {
    tone_mapper_->SetFrameDumpConfig(count);
  }

  DLOGI("num_frame_dump %d, input_layer_dump_enable %d", dump_frame_count_, dump_input_layers_);
  validated_ = false;
  return HWC2::Error::None;
}

HWC2::PowerMode HWCDisplay::GetCurrentPowerMode() {
  return current_power_mode_;
}

DisplayError HWCDisplay::VSync(const DisplayEventVSync &vsync) {
  if (callbacks_->Vsync_2_4CallbackRegistered()) {
    VsyncPeriodNanos vsync_period;
    if (GetDisplayVsyncPeriod(&vsync_period) != HWC2::Error::None) {
      vsync_period = 0;
    }
    ATRACE_INT("VsyncPeriod", INT32(vsync_period));
    callbacks_->Vsync_2_4(id_, vsync.timestamp, vsync_period);
  } else {
    callbacks_->Vsync(id_, vsync.timestamp);
  }

  return kErrorNone;
}

DisplayError HWCDisplay::Refresh() {
  callbacks_->Refresh(id_);
  return kErrorNone;
}

DisplayError HWCDisplay::CECMessage(char *message) {
  if (qservice_) {
    qservice_->onCECMessageReceived(message, 0);
  } else {
    DLOGW("Qservice instance not available.");
  }

  return kErrorNone;
}

DisplayError HWCDisplay::HandleEvent(DisplayEvent event) {
  switch (event) {
    case kIdleTimeout: {
      SCOPE_LOCK(HWCSession::locker_[id_]);
      if (pending_commit_) {
        // If idle timeout event comes in between prepare
        // and commit, drop it since device is not really
        // idle.
        return kErrorNotSupported;
      }
      validated_ = false;
      break;
    }
    case kSyncInvalidateDisplay:
    case kIdlePowerCollapse:
    case kThermalEvent: {
      SEQUENCE_WAIT_SCOPE_LOCK(HWCSession::locker_[id_]);
      validated_ = false;
    } break;
    case kPanelDeadEvent:
    case kDisplayPowerResetEvent: {
      // Mutex scope
      {
        SEQUENCE_WAIT_SCOPE_LOCK(HWCSession::locker_[id_]);
        validated_ = false;
      }
      // TODO(user): Following scenario need to be addressed
      // If panel or HW is in bad state for either ESD or HWR, there is no acquired lock between
      // this scope and call to DisplayPowerReset.
      // Prepare or commit could operate on the display since locker_[id_] is free and most likely
      // result in a failure since ESD/HWR has been requested during this time period.
      if (event_handler_) {
        event_handler_->DisplayPowerReset();
      } else {
        DLOGW("Cannot execute DisplayPowerReset (client_id = %" PRId64 "), event_handler_ is null",
              id_);
      }
    } break;
    case kInvalidateDisplay:
      validated_ = false;
      break;
    case kPostIdleTimeout:
      display_idle_ = true;
      break;
    default:
      DLOGW("Unknown event: %d", event);
      break;
  }

  return kErrorNone;
}

DisplayError HWCDisplay::HistogramEvent(int /* fd */, uint32_t /* blob_fd */) {
  return kErrorNone;
}

HWC2::Error HWCDisplay::PrepareLayerStack(uint32_t *out_num_types, uint32_t *out_num_requests) {
  layer_changes_.clear();
  layer_requests_.clear();
  has_client_composition_ = false;
  display_idle_ = false;

  DTRACE_SCOPED();
  if (shutdown_pending_) {
    validated_ = false;
    return HWC2::Error::BadDisplay;
  }

  if (CanSkipSdmPrepare(out_num_types, out_num_requests)) {
    return ((*out_num_types > 0) ? HWC2::Error::HasChanges : HWC2::Error::None);
  }

  UpdateRefreshRate();
  UpdateActiveConfig();
  DisplayError error = display_intf_->Prepare(&layer_stack_);
  if (error != kErrorNone) {
    if (error == kErrorShutDown) {
      shutdown_pending_ = true;
    } else if (error == kErrorPermission) {
      WaitOnPreviousFence();
      MarkLayersForGPUBypass();
      geometry_changes_on_doze_suspend_ |= geometry_changes_;
    } else {
      DLOGW("Prepare failed. Error = %d", error);
      // To prevent surfaceflinger infinite wait, flush the previous frame during Commit()
      // so that previous buffer and fences are released, and override the error.
      flush_ = true;
      validated_ = false;
      // Prepare cycle can fail on a newly connected display if insufficient pipes
      // are available at this moment. Trigger refresh so that the other displays
      // can free up pipes and a valid content can be attached to virtual display.
      callbacks_->Refresh(id_);
      return HWC2::Error::BadDisplay;
    }
  } else {
    // clear geometry_changes_on_doze_suspend_ on successful prepare.
    geometry_changes_on_doze_suspend_ = GeometryChanges::kNone;
  }

  for (auto hwc_layer : layer_set_) {
    Layer *layer = hwc_layer->GetSDMLayer();
    LayerComposition &composition = layer->composition;

    if (composition == kCompositionSDE || composition == kCompositionStitch) {
      layer_requests_[hwc_layer->GetId()] = HWC2::LayerRequest::ClearClientTarget;
    }

    HWC2::Composition requested_composition = hwc_layer->GetClientRequestedCompositionType();
    // Set SDM composition to HWC2 type in HWCLayer
    hwc_layer->SetComposition(composition);
    HWC2::Composition device_composition  = hwc_layer->GetDeviceSelectedCompositionType();
    if (device_composition == HWC2::Composition::Client) {
      has_client_composition_ = true;
    }
    // Update the changes list only if the requested composition is different from SDM comp type
    if (requested_composition != device_composition) {
      layer_changes_[hwc_layer->GetId()] = device_composition;
    }
    hwc_layer->ResetValidation();
  }

  client_target_->ResetValidation();
  *out_num_types = UINT32(layer_changes_.size());
  *out_num_requests = UINT32(layer_requests_.size());
  validate_state_ = kNormalValidate;
  validated_ = true;
  layer_stack_invalid_ = false;

  return ((*out_num_types > 0) ? HWC2::Error::HasChanges : HWC2::Error::None);
}

HWC2::Error HWCDisplay::AcceptDisplayChanges() {
  if (layer_set_.empty()) {
    return HWC2::Error::None;
  }

  if (!validated_) {
    return HWC2::Error::NotValidated;
  }

  for (const auto& change : layer_changes_) {
    auto hwc_layer = layer_map_[change.first];
    auto composition = change.second;
    if (hwc_layer != nullptr) {
      hwc_layer->UpdateClientCompositionType(composition);
    } else {
      DLOGW("Invalid layer: %" PRIu64, change.first);
    }
  }
  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::GetChangedCompositionTypes(uint32_t *out_num_elements,
                                                   hwc2_layer_t *out_layers, int32_t *out_types) {
  if (layer_set_.empty()) {
    return HWC2::Error::None;
  }

  if (!validated_) {
    DLOGW("Display is not validated");
    return HWC2::Error::NotValidated;
  }

  *out_num_elements = UINT32(layer_changes_.size());
  if (out_layers != nullptr && out_types != nullptr) {
    int i = 0;
    for (auto change : layer_changes_) {
      out_layers[i] = change.first;
      out_types[i] = INT32(change.second);
      i++;
    }
  }
  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::GetReleaseFences(uint32_t *out_num_elements, hwc2_layer_t *out_layers,
                                         std::vector<shared_ptr<Fence>> *out_fences) {
  if (out_num_elements == nullptr) {
    return HWC2::Error::BadParameter;
  }

  if (out_layers != nullptr && out_fences != nullptr) {
    *out_num_elements = std::min(*out_num_elements, UINT32(layer_set_.size()));
    auto it = layer_set_.begin();
    for (uint32_t i = 0; i < *out_num_elements; i++, it++) {
      auto hwc_layer = *it;
      out_layers[i] = hwc_layer->GetId();

      shared_ptr<Fence> &fence = (*out_fences)[i];
      hwc_layer->PopFrontReleaseFence(&fence);
    }
  } else {
    *out_num_elements = UINT32(layer_set_.size());
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::GetDisplayRequests(int32_t *out_display_requests,
                                           uint32_t *out_num_elements, hwc2_layer_t *out_layers,
                                           int32_t *out_layer_requests) {
  if (layer_set_.empty()) {
    return HWC2::Error::None;
  }

  if (out_display_requests == nullptr || out_num_elements == nullptr) {
    return HWC2::Error::BadParameter;
  }

  // No display requests for now
  // Use for sharing blit buffers and
  // writing wfd buffer directly to output if there is full GPU composition
  // and no color conversion needed
  if (!validated_) {
    DLOGW("Display is not validated");
    return HWC2::Error::NotValidated;
  }

  *out_display_requests = 0;
  if (out_layers != nullptr && out_layer_requests != nullptr) {
    *out_num_elements = std::min(*out_num_elements, UINT32(layer_requests_.size()));
    auto it = layer_requests_.begin();
    for (uint32_t i = 0; i < *out_num_elements; i++, it++) {
      out_layers[i] = it->first;
      out_layer_requests[i] = INT32(it->second);
    }
  } else {
    *out_num_elements = UINT32(layer_requests_.size());
  }

  auto client_target_layer = client_target_->GetSDMLayer();
  if (client_target_layer->request.flags.flip_buffer) {
    *out_display_requests = INT32(HWC2::DisplayRequest::FlipClientTarget);
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::GetHdrCapabilities(uint32_t *out_num_types, int32_t *out_types,
                                           float *out_max_luminance,
                                           float *out_max_average_luminance,
                                           float *out_min_luminance) {
  if (out_num_types == nullptr || out_max_luminance == nullptr ||
      out_max_average_luminance == nullptr || out_min_luminance == nullptr) {
    return HWC2::Error::BadParameter;
  }

  DisplayConfigFixedInfo fixed_info = {};
  display_intf_->GetConfig(&fixed_info);

  if (!fixed_info.hdr_supported) {
    *out_num_types = 0;
    DLOGI("HDR is not supported");
    return HWC2::Error::None;
  }

  uint32_t num_types = 0;
  if (fixed_info.hdr_plus_supported) {
    num_types = UINT32(Hdr::HDR10_PLUS) - 1;
  } else {
    num_types = UINT32(Hdr::HLG) - 1;
  }

  // We support HDR10, HLG and HDR10_PLUS.
  if (out_types == nullptr) {
    *out_num_types = num_types;
  } else {
    uint32_t max_out_types = std::min(*out_num_types, num_types);
    int32_t type = static_cast<int32_t>(Hdr::DOLBY_VISION);
    for (int32_t i = 0; i < max_out_types; i++) {
      while (type == static_cast<int32_t>(Hdr::DOLBY_VISION) /* Skip list */) {
        // Skip the type
        type++;
      }
      if (type > (num_types + 1)) {
        break;
      }
      out_types[i] = type++;
    }
    *out_max_luminance = fixed_info.max_luminance;
    *out_max_average_luminance = fixed_info.average_luminance;
    *out_min_luminance = fixed_info.min_luminance;
  }

  return HWC2::Error::None;
}


HWC2::Error HWCDisplay::CommitLayerStack(void) {
  if (flush_) {
    return HWC2::Error::None;
  }

  DTRACE_SCOPED();

  if (!validated_) {
    DLOGV_IF(kTagClient, "Display %" PRIu64 "is not validated", id_);
    return HWC2::Error::NotValidated;
  }

  if (shutdown_pending_ || layer_set_.empty()) {
    return HWC2::Error::None;
  }

  if (skip_commit_) {
    DLOGV_IF(kTagClient, "Skipping Refresh on display %" PRIu64 , id_);
    return HWC2::Error::None;
  }

  DumpInputBuffers();

  DisplayError error = kErrorUndefined;
  int status = 0;
  if (tone_mapper_) {
    if (NeedsToneMap(layer_stack_)) {
      status = tone_mapper_->HandleToneMap(&layer_stack_);
      if (status != 0) {
        DLOGE("Error handling HDR in ToneMapper");
      }
    } else {
      tone_mapper_->Terminate();
    }
  }

  if (elapse_timestamp_) {
    layer_stack_.elapse_timestamp = elapse_timestamp_;
  }

  error = display_intf_->Commit(&layer_stack_);

  if (error == kErrorNone) {
    // A commit is successfully submitted, start flushing on failure now onwards.
    flush_on_error_ = true;
    first_cycle_ = false;
  } else {
    if (error == kErrorShutDown) {
      shutdown_pending_ = true;
      return HWC2::Error::Unsupported;
    } else if (error == kErrorNotValidated) {
      validated_ = false;
      return HWC2::Error::NotValidated;
    } else if (error != kErrorPermission) {
      DLOGE("Commit failed. Error = %d", error);
      // To prevent surfaceflinger infinite wait, flush the previous frame during Commit()
      // so that previous buffer and fences are released, and override the error.
      flush_ = true;
    }
  }

  validate_state_ = kSkipValidate;
  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::PostCommitLayerStack(shared_ptr<Fence> *out_retire_fence) {
  auto status = HWC2::Error::None;

  // Do no call flush on errors, if a successful buffer is never submitted.
  if (flush_ && flush_on_error_) {
    display_intf_->Flush(&layer_stack_);
    validated_ = false;
  }

  if (tone_mapper_ && tone_mapper_->IsActive()) {
     tone_mapper_->PostCommit(&layer_stack_);
  }

  // TODO(user): No way to set the client target release fence on SF
  shared_ptr<Fence> client_target_release_fence =
      client_target_->GetSDMLayer()->input_buffer.release_fence;
  if (client_target_release_fence) {
    fbt_release_fence_ = client_target_release_fence;
  }
  client_target_->ResetGeometryChanges();

  for (auto hwc_layer : layer_set_) {
    hwc_layer->ResetGeometryChanges();
    Layer *layer = hwc_layer->GetSDMLayer();
    LayerBuffer *layer_buffer = &layer->input_buffer;

    if (!flush_) {
      // If swapinterval property is set to 0 or for single buffer layers, do not update f/w
      // release fences and discard fences from driver
      if (!swap_interval_zero_ && !layer->flags.single_buffer) {
        // It may so happen that layer gets marked to GPU & app layer gets queued
        // to MDP for composition. In those scenarios, release fence of buffer should
        // have mdp and gpu sync points merged.
        hwc_layer->PushBackReleaseFence(layer_buffer->release_fence);
      }
    } else {
      // In case of flush or display paused, we don't return an error to f/w, so it will
      // get a release fence out of the hwc_layer's release fence queue
      // We should push a -1 to preserve release fence circulation semantics.
      hwc_layer->PushBackReleaseFence(nullptr);
    }

    layer->request.flags = {};
    layer_buffer->acquire_fence = nullptr;
  }

  client_target_->GetSDMLayer()->request.flags = {};
  // if swapinterval property is set to 0 then close and reset the list retire fence
  if (!swap_interval_zero_) {
    *out_retire_fence = layer_stack_.retire_fence;
  }

  if (dump_frame_count_) {
    dump_frame_count_--;
    dump_frame_index_++;
  }

  layer_stack_.flags.geometry_changed = false;
  geometry_changes_ = GeometryChanges::kNone;
  flush_ = false;
  skip_commit_ = false;

  // Handle pending config changes.
  if (pending_first_commit_config_) {
    DLOGI("Changing active config to %d", UINT32(pending_first_commit_config_));
    pending_first_commit_config_ = false;
    SetActiveConfig(pending_first_commit_config_index_);
  }

  return status;
}

void HWCDisplay::SetIdleTimeoutMs(uint32_t timeout_ms, uint32_t inactive_ms) {
  return;
}

DisplayError HWCDisplay::SetMaxMixerStages(uint32_t max_mixer_stages) {
  DisplayError error = kErrorNone;

  if (display_intf_) {
    error = display_intf_->SetMaxMixerStages(max_mixer_stages);
    validated_ = false;
  }

  return error;
}

void HWCDisplay::DumpInputBuffers() {
  char dir_path[PATH_MAX];
  int  status;

  if (!dump_frame_count_ || flush_ || !dump_input_layers_) {
    return;
  }

  DLOGI("dump_frame_count %d dump_input_layers %d", dump_frame_count_, dump_input_layers_);
  snprintf(dir_path, sizeof(dir_path), "%s/frame_dump_disp_id_%02u_%s", HWCDebugHandler::DumpDir(),
           UINT32(id_), GetDisplayString());

  status = mkdir(dir_path, 777);
  if ((status != 0) && errno != EEXIST) {
    DLOGW("Failed to create %s directory errno = %d, desc = %s", dir_path, errno, strerror(errno));
    return;
  }

  // Even if directory exists already, need to explicitly change the permission.
  if (chmod(dir_path, 0777) != 0) {
    DLOGW("Failed to change permissions on %s directory", dir_path);
    return;
  }

  for (uint32_t i = 0; i < layer_stack_.layers.size(); i++) {
    auto layer = layer_stack_.layers.at(i);
    const private_handle_t *pvt_handle =
        reinterpret_cast<const private_handle_t *>(layer->input_buffer.buffer_id);
    Fence::Wait(layer->input_buffer.acquire_fence);

    DLOGI("Dump layer[%d] of %d pvt_handle %p pvt_handle->base %" PRIx64, i,
          UINT32(layer_stack_.layers.size()), pvt_handle, pvt_handle? pvt_handle->base : 0);

    if (!pvt_handle) {
      DLOGE("Buffer handle is null");
      continue;
    }

    if (!pvt_handle->base) {
      DisplayError error = buffer_allocator_->MapBuffer(pvt_handle, nullptr);
      if (error != kErrorNone) {
        DLOGE("Failed to map buffer, error = %d", error);
        continue;
      }
    }

    char dump_file_name[PATH_MAX];
    size_t result = 0;

    snprintf(dump_file_name, sizeof(dump_file_name), "%s/input_layer%d_%dx%d_%s_frame%d.raw",
             dir_path, i, pvt_handle->width, pvt_handle->height,
             qdutils::GetHALPixelFormatString(pvt_handle->format), dump_frame_index_);

    FILE *fp = fopen(dump_file_name, "w+");
    if (fp) {
      result = fwrite(reinterpret_cast<void *>(pvt_handle->base), pvt_handle->size, 1, fp);
      fclose(fp);
    }

    int release_fence = -1;
    DisplayError error = buffer_allocator_->UnmapBuffer(pvt_handle, &release_fence);
    if (error != kErrorNone) {
      DLOGE("Failed to unmap buffer, error = %d", error);
      continue;
    }

    DLOGI("Frame Dump %s: is %s", dump_file_name, result ? "Successful" : "Failed");
  }
}

void HWCDisplay::DumpOutputBuffer(const BufferInfo &buffer_info, void *base,
                                  shared_ptr<Fence> &retire_fence) {
  char dir_path[PATH_MAX];
  int  status;

  snprintf(dir_path, sizeof(dir_path), "%s/frame_dump_disp_id_%02u_%s", HWCDebugHandler::DumpDir(),
           UINT32(id_), GetDisplayString());

  status = mkdir(dir_path, 777);
  if ((status != 0) && errno != EEXIST) {
    DLOGW("Failed to create %s directory errno = %d, desc = %s", dir_path, errno, strerror(errno));
    return;
  }

  // Even if directory exists already, need to explicitly change the permission.
  if (chmod(dir_path, 0777) != 0) {
    DLOGW("Failed to change permissions on %s directory", dir_path);
    return;
  }

  if (base) {
    char dump_file_name[PATH_MAX];
    size_t result = 0;

    if (Fence::Wait(retire_fence) != kErrorNone) {
      DLOGW("sync_wait error errno = %d, desc = %s", errno, strerror(errno));
      return;
    }

    snprintf(dump_file_name, sizeof(dump_file_name), "%s/output_layer_%dx%d_%s_frame%d.raw",
             dir_path, buffer_info.alloc_buffer_info.aligned_width,
             buffer_info.alloc_buffer_info.aligned_height,
             GetFormatString(buffer_info.buffer_config.format), dump_frame_index_);

    FILE *fp = fopen(dump_file_name, "w+");
    if (fp) {
      result = fwrite(base, buffer_info.alloc_buffer_info.size, 1, fp);
      fclose(fp);
    }

    DLOGI("Frame Dump of %s is %s", dump_file_name, result ? "Successful" : "Failed");
  }
}

const char *HWCDisplay::GetDisplayString() {
  switch (type_) {
    case kBuiltIn:
      return "builtin";
    case kPluggable:
      return "pluggable";
    case kVirtual:
      return "virtual";
    default:
      return "invalid";
  }
}

int HWCDisplay::SetFrameBufferConfig(uint32_t x_pixels, uint32_t y_pixels) {
  if (x_pixels <= 0 || y_pixels <= 0) {
    DLOGW("Unsupported config: x_pixels=%d, y_pixels=%d", x_pixels, y_pixels);
    return -EINVAL;
  }

  DisplayConfigVariableInfo fb_config;
  DisplayError error = display_intf_->GetFrameBufferConfig(&fb_config);
  if (error != kErrorNone) {
    DLOGV("Get frame buffer config failed. Error = %d", error);
    return -EINVAL;
  }

  fb_config.x_pixels = x_pixels;
  fb_config.y_pixels = y_pixels;

  error = display_intf_->SetFrameBufferConfig(fb_config);
  if (error != kErrorNone) {
    DLOGV("Set frame buffer config failed. Error = %d", error);
    return -EINVAL;
  }

  // Reduce the src_rect and dst_rect as per FBT config.
  // SF sending reduced FBT but here the src_rect is equal to mixer which is
  // higher than allocated buffer of FBT.
  if (windowed_display_) {
    x_pixels -= UINT32(window_rect_.right + window_rect_.left);
    y_pixels -= UINT32(window_rect_.bottom + window_rect_.top);
  }

  if (x_pixels <= 0 || y_pixels <= 0) {
    DLOGE("window rects are not within the supported range");
    return -EINVAL;
  }

  // Create rects to represent the new source and destination crops
  LayerRect crop = LayerRect(0, 0, FLOAT(x_pixels), FLOAT(y_pixels));
  hwc_rect_t scaled_display_frame = {0, 0, INT(x_pixels), INT(y_pixels)};
  auto client_target_layer = client_target_->GetSDMLayer();
  client_target_layer->src_rect = crop;
  ApplyScanAdjustment(&scaled_display_frame);
  client_target_->SetLayerDisplayFrame(scaled_display_frame);
  client_target_->ResetPerFrameData();

  DLOGI("New framebuffer resolution (%dx%d)", fb_config.x_pixels, fb_config.y_pixels);

  return 0;
}

int HWCDisplay::SetFrameBufferResolution(uint32_t x_pixels, uint32_t y_pixels) {
  int error = SetFrameBufferConfig(x_pixels, y_pixels);
  if (error < 0) {
    DLOGV("SetFrameBufferConfig failed. Error = %d", error);
    return error;
  }

  if (windowed_display_) {
    x_pixels -= UINT32(window_rect_.right + window_rect_.left);
    y_pixels -= UINT32(window_rect_.bottom + window_rect_.top);
  }
  auto client_target_layer = client_target_->GetSDMLayer();

  int aligned_width;
  int aligned_height;
  uint32_t usage = GRALLOC_USAGE_HW_FB;
  int format = HAL_PIXEL_FORMAT_RGBA_8888;
  int ubwc_disabled = 0;
  int flags = 0;

  // By default UBWC is enabled and below property is global enable/disable for all
  // buffers allocated through gralloc , including framebuffer targets.
  HWCDebugHandler::Get()->GetProperty(DISABLE_UBWC_PROP, &ubwc_disabled);
  if (!ubwc_disabled) {
    usage |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
    flags |= private_handle_t::PRIV_FLAGS_UBWC_ALIGNED;
  }

  buffer_allocator_->GetAlignedWidthAndHeight(INT(x_pixels), INT(y_pixels), format, usage,
                                              &aligned_width, &aligned_height);

  // TODO(user): How does the dirty region get set on the client target? File bug on Google
  client_target_layer->composition = kCompositionGPUTarget;
  client_target_layer->input_buffer.format = HWCLayer::GetSDMFormat(format, flags);
  client_target_layer->input_buffer.width = UINT32(aligned_width);
  client_target_layer->input_buffer.height = UINT32(aligned_height);
  client_target_layer->input_buffer.unaligned_width = x_pixels;
  client_target_layer->input_buffer.unaligned_height = y_pixels;
  client_target_layer->plane_alpha = 255;

  return 0;
}

void HWCDisplay::GetFrameBufferResolution(uint32_t *x_pixels, uint32_t *y_pixels) {
  DisplayConfigVariableInfo fb_config;
  display_intf_->GetFrameBufferConfig(&fb_config);

  *x_pixels = fb_config.x_pixels;
  *y_pixels = fb_config.y_pixels;
}

DisplayError HWCDisplay::GetMixerResolution(uint32_t *x_pixels, uint32_t *y_pixels) {
  return display_intf_->GetMixerResolution(x_pixels, y_pixels);
}

void HWCDisplay::GetPanelResolution(uint32_t *x_pixels, uint32_t *y_pixels) {
  DisplayConfigVariableInfo display_config;
  uint32_t active_index = 0;

  display_intf_->GetActiveConfig(&active_index);
  display_intf_->GetConfig(active_index, &display_config);

  *x_pixels = display_config.x_pixels;
  *y_pixels = display_config.y_pixels;
}

int HWCDisplay::SetDisplayStatus(DisplayStatus display_status) {
  int status = 0;

  switch (display_status) {
    case kDisplayStatusResume:
      display_paused_ = false;
      status = INT32(SetPowerMode(HWC2::PowerMode::On, false /* teardown */));
      break;
    case kDisplayStatusOnline:
      status = INT32(SetPowerMode(HWC2::PowerMode::On, false /* teardown */));
      break;
    case kDisplayStatusPause:
      display_paused_ = true;
      status = INT32(SetPowerMode(HWC2::PowerMode::Off, false /* teardown */));
      break;
    case kDisplayStatusOffline:
      status = INT32(SetPowerMode(HWC2::PowerMode::Off, false /* teardown */));
      break;
    default:
      DLOGW("Invalid display status %d", display_status);
      return -EINVAL;
  }

  return status;
}

HWC2::Error HWCDisplay::SetCursorPosition(hwc2_layer_t layer, int x, int y) {
  if (shutdown_pending_) {
    return HWC2::Error::None;
  }

  if (!layer_stack_.flags.cursor_present) {
    DLOGW("Cursor layer not present");
    return HWC2::Error::BadLayer;
  }

  HWCLayer *hwc_layer = GetHWCLayer(layer);
  if (hwc_layer == nullptr) {
    return HWC2::Error::BadLayer;
  }
  if (hwc_layer->GetDeviceSelectedCompositionType() != HWC2::Composition::Cursor) {
    return HWC2::Error::None;
  }
  if ((validate_state_ != kSkipValidate) && validated_) {
    // the device is currently in the middle of the validate/present sequence,
    // cannot set the Position(as per HWC2 spec)
    return HWC2::Error::NotValidated;
  }

  DisplayState state;
  if (display_intf_->GetDisplayState(&state) == kErrorNone) {
    if (state != kStateOn) {
      return HWC2::Error::None;
    }
  }

  // TODO(user): HWC1.5 was not letting SetCursorPosition before validateDisplay,
  // but HWC2.0 doesn't let setting cursor position after validate before present.
  // Need to revisit.

  auto error = display_intf_->SetCursorPosition(x, y);
  if (error != kErrorNone) {
    if (error == kErrorShutDown) {
      shutdown_pending_ = true;
      return HWC2::Error::None;
    }

    DLOGE("Failed for x = %d y = %d, Error = %d", x, y, error);
    return HWC2::Error::BadDisplay;
  }

  return HWC2::Error::None;
}

int HWCDisplay::OnMinHdcpEncryptionLevelChange(uint32_t min_enc_level) {
  DisplayError error = display_intf_->OnMinHdcpEncryptionLevelChange(min_enc_level);
  if (error != kErrorNone) {
    DLOGE("Failed. Error = %d", error);
    return -1;
  }

  validated_ = false;
  return 0;
}

void HWCDisplay::MarkLayersForGPUBypass() {
  for (auto hwc_layer : layer_set_) {
    auto layer = hwc_layer->GetSDMLayer();
    layer->composition = kCompositionSDE;
  }
  validated_ = true;
}

void HWCDisplay::MarkLayersForClientComposition() {
  // ClientComposition - GPU comp, to acheive this, set skip flag so that
  // SDM does not handle this layer and hwc_layer composition will be
  // set correctly at the end of Prepare.
  DLOGV_IF(kTagClient, "HWC Layers marked for GPU comp");
  for (auto hwc_layer : layer_set_) {
    Layer *layer = hwc_layer->GetSDMLayer();
    layer->flags.skip = true;
  }
  layer_stack_.flags.skip_present = true;
}

void HWCDisplay::ApplyScanAdjustment(hwc_rect_t *display_frame) {
}

int HWCDisplay::ToggleScreenUpdates(bool enable) {
  display_paused_ = enable ? false : true;
  callbacks_->Refresh(id_);
  validated_ = false;
  return 0;
}

int HWCDisplay::ColorSVCRequestRoute(const PPDisplayAPIPayload &in_payload,
                                     PPDisplayAPIPayload *out_payload,
                                     PPPendingParams *pending_action) {
  int ret = 0;

  if (display_intf_)
    ret = display_intf_->ColorSVCRequestRoute(in_payload, out_payload, pending_action);
  else
    ret = -EINVAL;

  return ret;
}

void HWCDisplay::SolidFillPrepare() {
  if (solid_fill_enable_) {
    if (solid_fill_layer_ == NULL) {
      // Create a dummy layer here
      solid_fill_layer_ = new Layer();
    }
    uint32_t primary_width = 0, primary_height = 0;
    GetMixerResolution(&primary_width, &primary_height);

    LayerBuffer *layer_buffer = &solid_fill_layer_->input_buffer;
    layer_buffer->width = primary_width;
    layer_buffer->height = primary_height;
    layer_buffer->unaligned_width = primary_width;
    layer_buffer->unaligned_height = primary_height;

    solid_fill_layer_->composition = kCompositionGPU;
    solid_fill_layer_->src_rect = solid_fill_rect_;
    solid_fill_layer_->dst_rect = solid_fill_rect_;

    solid_fill_layer_->blending = kBlendingPremultiplied;
    solid_fill_layer_->solid_fill_color = 0;
    solid_fill_layer_->solid_fill_info.bit_depth = solid_fill_color_.bit_depth;
    solid_fill_layer_->solid_fill_info.red = solid_fill_color_.red;
    solid_fill_layer_->solid_fill_info.blue = solid_fill_color_.blue;
    solid_fill_layer_->solid_fill_info.green = solid_fill_color_.green;
    solid_fill_layer_->solid_fill_info.alpha = solid_fill_color_.alpha;
    solid_fill_layer_->frame_rate = 60;
    solid_fill_layer_->visible_regions.push_back(solid_fill_layer_->dst_rect);
    solid_fill_layer_->flags.updating = 1;
    solid_fill_layer_->flags.solid_fill = true;
  } else {
    // delete the dummy layer
    delete solid_fill_layer_;
    solid_fill_layer_ = NULL;
  }

  if (solid_fill_enable_ && solid_fill_layer_) {
    BuildSolidFillStack();
    MarkLayersForGPUBypass();
  }

  return;
}

int HWCDisplay::GetVisibleDisplayRect(hwc_rect_t *visible_rect) {
  if (!IsValid(display_rect_)) {
    return -EINVAL;
  }

  visible_rect->left = INT(display_rect_.left);
  visible_rect->top = INT(display_rect_.top);
  visible_rect->right = INT(display_rect_.right);
  visible_rect->bottom = INT(display_rect_.bottom);
  DLOGI("Visible Display Rect(%d %d %d %d)", visible_rect->left, visible_rect->top,
        visible_rect->right, visible_rect->bottom);

  return 0;
}

int HWCDisplay::HandleSecureSession(const std::bitset<kSecureMax> &secure_sessions,
                                    bool *power_on_pending, bool is_active_secure_display) {
  if (!power_on_pending) {
    return -EINVAL;
  }

  if (active_secure_sessions_[kSecureDisplay] != secure_sessions[kSecureDisplay]) {
    if (secure_sessions[kSecureDisplay]) {
      pending_power_mode_ = current_power_mode_;
      HWC2::Error error = SetPowerMode(HWC2::PowerMode::Off, true /* teardown */);
      if (error != HWC2::Error::None) {
        DLOGE("SetPowerMode failed. Error = %d", error);
      }
    } else {
      *power_on_pending = (pending_power_mode_ != HWC2::PowerMode::Off) ? true : false;
    }

    DLOGI("SecureDisplay state changed from %d to %d for display %" PRId64 " %d-%d",
          active_secure_sessions_.test(kSecureDisplay), secure_sessions.test(kSecureDisplay),
          id_, sdm_id_, type_);
  }
  active_secure_sessions_ = secure_sessions;
  return 0;
}

int HWCDisplay::GetActiveSecureSession(std::bitset<kSecureMax> *secure_sessions) {
  if (!secure_sessions) {
    return -1;
  }
  secure_sessions->reset();
  for (auto hwc_layer : layer_set_) {
    Layer *layer = hwc_layer->GetSDMLayer();
    if (layer->input_buffer.flags.secure_camera) {
      secure_sessions->set(kSecureCamera);
    }
    if (layer->input_buffer.flags.secure_display) {
      secure_sessions->set(kSecureDisplay);
    }
  }
  return 0;
}

int HWCDisplay::SetActiveDisplayConfig(uint32_t config) {
  uint32_t current_config = 0;
  display_intf_->GetActiveConfig(&current_config);
  if (config == current_config) {
    return 0;
  }

  validated_ = false;
  DisplayError error = display_intf_->SetActiveConfig(config);
  if (error != kErrorNone) {
    DLOGE("Failed to set %d config! Error: %d", config, error);
    return -EINVAL;
  }

  SetActiveConfigIndex(config);
  return 0;
}

int HWCDisplay::GetActiveDisplayConfig(uint32_t *config) {
  return display_intf_->GetActiveConfig(config) == kErrorNone ? 0 : -1;
}

int HWCDisplay::GetDisplayConfigCount(uint32_t *count) {
  return display_intf_->GetNumVariableInfoConfigs(count) == kErrorNone ? 0 : -1;
}

int HWCDisplay::GetDisplayAttributesForConfig(int config,
                                            DisplayConfigVariableInfo *display_attributes) {
  return display_intf_->GetConfig(UINT32(config), display_attributes) == kErrorNone ? 0 : -1;
}

uint32_t HWCDisplay::GetUpdatingLayersCount(void) {
  uint32_t updating_count = 0;

  for (uint i = 0; i < layer_stack_.layers.size(); i++) {
    auto layer = layer_stack_.layers.at(i);
    if (layer->flags.updating) {
      updating_count++;
    }
  }

  return updating_count;
}

bool HWCDisplay::IsLayerUpdating(HWCLayer *hwc_layer) {
  auto layer = hwc_layer->GetSDMLayer();
  // Layer should be considered updating if
  //   a) layer is in single buffer mode, or
  //   b) valid dirty_regions(android specific hint for updating status), or
  //   c) layer stack geometry has changed (TODO(user): Remove when SDM accepts
  //      geometry_changed as bit fields).
  return (layer->flags.single_buffer || hwc_layer->IsSurfaceUpdated() ||
          geometry_changes_);
}

uint32_t HWCDisplay::SanitizeRefreshRate(uint32_t req_refresh_rate) {
  uint32_t refresh_rate = req_refresh_rate;

  if (refresh_rate < min_refresh_rate_) {
    // Pick the next multiple of request which is within the range
    refresh_rate =
        (((min_refresh_rate_ / refresh_rate) + ((min_refresh_rate_ % refresh_rate) ? 1 : 0)) *
         refresh_rate);
  }

  if (refresh_rate > max_refresh_rate_) {
    refresh_rate = max_refresh_rate_;
  }

  return refresh_rate;
}

DisplayClass HWCDisplay::GetDisplayClass() {
  return display_class_;
}

void HWCDisplay::Dump(std::ostringstream *os) {
  *os << "\n------------HWC----------------\n";
  *os << "HWC2 display_id: " << id_ << std::endl;
  for (auto layer : layer_set_) {
    auto sdm_layer = layer->GetSDMLayer();
    auto transform = sdm_layer->transform;
    *os << "layer: " << std::setw(4) << layer->GetId();
    *os << " z: " << layer->GetZ();
    *os << " composition: " <<
          to_string(layer->GetClientRequestedCompositionType()).c_str();
    *os << "/" <<
          to_string(layer->GetDeviceSelectedCompositionType()).c_str();
    *os << " alpha: " << std::to_string(sdm_layer->plane_alpha).c_str();
    *os << " format: " << std::setw(22) << GetFormatString(sdm_layer->input_buffer.format);
    *os << " dataspace:" << std::hex << "0x" << std::setw(8) << std::setfill('0')
        << layer->GetLayerDataspace() << std::dec << std::setfill(' ');
    *os << " transform: " << transform.rotation << "/" << transform.flip_horizontal <<
          "/"<< transform.flip_vertical;
    *os << " buffer_id: " << std::hex << "0x" << sdm_layer->input_buffer.buffer_id << std::dec;
    *os << " secure: " << layer->IsProtected()
        << std::endl;
  }

  if (has_client_composition_) {
    *os << "\n---------client target---------\n";
    auto sdm_layer = client_target_->GetSDMLayer();
    *os << "format: " << std::setw(14) << GetFormatString(sdm_layer->input_buffer.format);
    *os << " dataspace:" << std::hex << "0x" << std::setw(8) << std::setfill('0')
        << client_target_->GetLayerDataspace() << std::dec << std::setfill(' ');
    *os << "  buffer_id: " << std::hex << "0x" << sdm_layer->input_buffer.buffer_id << std::dec;
    *os << " secure: " << client_target_->IsProtected()
        << std::endl;
  }

  if (layer_stack_invalid_) {
    *os << "\n Layers added or removed but not reflected to SDM's layer stack yet\n";
    return;
  }

  if (color_mode_) {
    *os << "\n----------Color Modes---------\n";
    color_mode_->Dump(os);
  }

  if (display_intf_) {
    *os << "\n------------SDM----------------\n";
    *os << display_intf_->Dump();
  }

  *os << "\n";
}

bool HWCDisplay::CanSkipValidate() {
  if (!validated_ || solid_fill_enable_) {
    return false;
  }

  if ((tone_mapper_ && tone_mapper_->IsActive()) ||
      layer_stack_.flags.single_buffered_layer_present) {
    DLOGV_IF(kTagClient, "Tonemapping enabled or single buffer layer present = %d"
             " Returning false.", layer_stack_.flags.single_buffered_layer_present);
    return false;
  }

  if (client_target_->NeedsValidation()) {
    DLOGV_IF(kTagClient, "Framebuffer target needs validation. Returning false.");
    return false;
  }

  for (auto hwc_layer : layer_set_) {
    Layer *layer = hwc_layer->GetSDMLayer();
    if (hwc_layer->NeedsValidation()) {
      DLOGV_IF(kTagClient, "hwc_layer[%" PRIu64 "] needs validation. Returning false.",
               hwc_layer->GetId());
      return false;
    }

    // Do not allow Skip Validate, if any layer needs GPU Composition.
    if (layer->composition == kCompositionGPU || layer->composition == kCompositionNone) {
      DLOGV_IF(kTagClient, "hwc_layer[%" PRIu64 "] is %s. Returning false.", hwc_layer->GetId(),
               (layer->composition == kCompositionGPU) ? "GPU composed": "Dropped");
      return false;
    }
  }

  if (!layer_set_.empty() && !display_intf_->CanSkipValidate()) {
    return false;
  }

  return true;
}

HWC2::Error HWCDisplay::GetValidateDisplayOutput(uint32_t *out_num_types,
                                                 uint32_t *out_num_requests) {
  *out_num_types = UINT32(layer_changes_.size());
  *out_num_requests = UINT32(layer_requests_.size());

  return ((*out_num_types > 0) ? HWC2::Error::HasChanges : HWC2::Error::None);
}

HWC2::Error HWCDisplay::GetDisplayIdentificationData(uint8_t *out_port, uint32_t *out_data_size,
                                                     uint8_t *out_data) {
  DisplayError ret = display_intf_->GetDisplayIdentificationData(out_port, out_data_size, out_data);
  if (ret != kErrorNone) {
    DLOGE("Failed due to SDM/Driver (err = %d, disp id = %" PRIu64
          " %d-%d", ret, id_, sdm_id_, type_);
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::SetDisplayElapseTime(uint64_t time) {
  elapse_timestamp_ = time;
  return HWC2::Error::None;
}

bool HWCDisplay::IsDisplayCommandMode() {
  return is_cmd_mode_;
}

HWC2::Error HWCDisplay::SetDisplayedContentSamplingEnabledVndService(bool enabled) {
  return HWC2::Error::Unsupported;
}

HWC2::Error HWCDisplay::SetDisplayedContentSamplingEnabled(int32_t enabled, uint8_t component_mask,
                                                           uint64_t max_frames) {
  DLOGV("Request to start/stop histogram thread not supported on this display");
  return HWC2::Error::Unsupported;
}

HWC2::Error HWCDisplay::GetDisplayedContentSamplingAttributes(int32_t *format, int32_t *dataspace,
                                                              uint8_t *supported_components) {
  return HWC2::Error::Unsupported;
}

HWC2::Error HWCDisplay::GetDisplayedContentSample(
    uint64_t max_frames, uint64_t timestamp, uint64_t *numFrames,
    int32_t samples_size[NUM_HISTOGRAM_COLOR_COMPONENTS],
    uint64_t *samples[NUM_HISTOGRAM_COLOR_COMPONENTS]) {
  return HWC2::Error::Unsupported;
}

// Skip SDM prepare if all the layers in the current draw cycle are marked as Skip and
// previous draw cycle had GPU Composition, as the resources for GPU Target layer have
// already been validated and configured to the driver.
bool HWCDisplay::CanSkipSdmPrepare(uint32_t *num_types, uint32_t *num_requests) {
  if (!validated_ || layer_set_.empty()) {
    return false;
  }

  bool skip_prepare = true;
  for (auto hwc_layer : layer_set_) {
    if (!hwc_layer->GetSDMLayer()->flags.skip ||
        (hwc_layer->GetDeviceSelectedCompositionType() != HWC2::Composition::Client)) {
      skip_prepare = false;
      layer_changes_.clear();
      break;
    }
    if (hwc_layer->GetClientRequestedCompositionType() != HWC2::Composition::Client) {
      layer_changes_[hwc_layer->GetId()] = HWC2::Composition::Client;
    }
  }

  if (skip_prepare) {
    *num_types = UINT32(layer_changes_.size());
    *num_requests = 0;
    layer_stack_invalid_ = false;
    has_client_composition_ = true;
    client_target_->ResetValidation();
    validate_state_ = kNormalValidate;
  }

  return skip_prepare;
}

void HWCDisplay::UpdateRefreshRate() {
  for (auto hwc_layer : layer_set_) {
    if (hwc_layer->HasMetaDataRefreshRate()) {
      continue;
    }
    auto layer = hwc_layer->GetSDMLayer();
    layer->frame_rate = std::min(current_refresh_rate_, HWCDisplay::GetThrottlingRefreshRate());
  }
}

int32_t HWCDisplay::SetClientTargetDataSpace(int32_t dataspace) {
  if (client_target_->GetLayerDataspace() != dataspace) {
    client_target_->SetLayerDataspace(dataspace);
    Layer *sdm_layer = client_target_->GetSDMLayer();
    // Data space would be validated at GetClientTargetSupport, so just use here.
    sdm::GetSDMColorSpace(client_target_->GetLayerDataspace(),
                          &sdm_layer->input_buffer.color_metadata);
  }

  return 0;
}

void HWCDisplay::WaitOnPreviousFence() {
  DisplayConfigFixedInfo display_config;
  display_intf_->GetConfig(&display_config);
  if (!display_config.is_cmdmode) {
    return;
  }

  // Since prepare failed commit would follow the same.
  // Wait for previous rel fence.
  for (auto hwc_layer : layer_set_) {
    shared_ptr<Fence> fence = nullptr;

    hwc_layer->PopBackReleaseFence(&fence);
    if (Fence::Wait(fence) != kErrorNone) {
      DLOGW("sync_wait error errno = %d, desc = %s", errno, strerror(errno));
      return;
    }
    hwc_layer->PushBackReleaseFence(fence);
  }

  if (Fence::Wait(fbt_release_fence_) != kErrorNone) {
    DLOGW("sync_wait error errno = %d, desc = %s", errno, strerror(errno));
    return;
  }
}

void HWCDisplay::GetLayerStack(HWCLayerStack *stack) {
  stack->client_target = client_target_;
  stack->layer_map = layer_map_;
  stack->layer_set = layer_set_;
}

void HWCDisplay::SetLayerStack(HWCLayerStack *stack) {
  client_target_ = stack->client_target;
  layer_map_ = stack->layer_map;
  layer_set_ = stack->layer_set;
}

bool HWCDisplay::CheckResourceState() {
  if (display_intf_) {
    return display_intf_->CheckResourceState();
  }

  return false;
}

void HWCDisplay::UpdateActiveConfig() {
  if (!pending_config_) {
    return;
  }

  DisplayError error = display_intf_->SetActiveConfig(pending_config_index_);
  if (error != kErrorNone) {
    DLOGI("Failed to set %d config", INT(pending_config_index_));
  } else {
    SetActiveConfigIndex(pending_config_index_);
  }

  // Reset pending config.
  pending_config_ = false;
}

int32_t HWCDisplay::GetDisplayConfigGroup(DisplayConfigGroupInfo variable_config) {
  for (auto &config : variable_config_map_) {
    DisplayConfigGroupInfo const &group_info = config.second;
    if (group_info == variable_config) {
      return INT32(config.first);
    }
  }

  return -1;
}

HWC2::Error HWCDisplay::GetDisplayVsyncPeriod(VsyncPeriodNanos *vsync_period) {
  if (GetTransientVsyncPeriod(vsync_period)) {
    return HWC2::Error::None;
  }

  return GetVsyncPeriodByActiveConfig(vsync_period);
}

HWC2::Error HWCDisplay::SetActiveConfigWithConstraints(
    hwc2_config_t config, const VsyncPeriodChangeConstraints *vsync_period_change_constraints,
    VsyncPeriodChangeTimeline *out_timeline) {
  DTRACE_SCOPED();

  if (variable_config_map_.find(config) == variable_config_map_.end()) {
    DLOGE("Invalid config: %d", config);
    return HWC2::Error::BadConfig;
  }

  // DRM driver expects DRM_PREFERRED_MODE to be set as part of first commit
  if (!IsFirstCommitDone()) {
    // Store client's config.
    // Set this as part of post commit.
    pending_first_commit_config_ = true;
    pending_first_commit_config_index_ = config;
    DLOGI("Defer config change to %d until first commit", UINT32(config));
    return HWC2::Error::None;
  } else if (pending_first_commit_config_) {
    // Config override request from client.
    // Honour latest request.
    pending_first_commit_config_ = false;
  }

  // Cache refresh rate set by client.
  DisplayConfigVariableInfo info = {};
  GetDisplayAttributesForConfig(INT(config), &info);
  active_refresh_rate_ = info.fps;

  if (vsync_period_change_constraints->seamlessRequired && !AllowSeamless(config)) {
    DLOGE("Seamless switch to the config: %d, is not allowed!", config);
    return HWC2::Error::SeamlessNotAllowed;
  }

  VsyncPeriodNanos vsync_period;
  if (GetDisplayVsyncPeriod(&vsync_period) != HWC2::Error::None) {
    return HWC2::Error::BadConfig;
  }

  std::tie(out_timeline->refreshTimeNanos, out_timeline->newVsyncAppliedTimeNanos) =
      RequestActiveConfigChange(config, vsync_period,
                                vsync_period_change_constraints->desiredTimeNanos);

  out_timeline->refreshRequired = true;
  return HWC2::Error::None;
}

void HWCDisplay::ProcessActiveConfigChange() {
  if (!IsActiveConfigReadyToSubmit(systemTime(SYSTEM_TIME_MONOTONIC))) {
    return;
  }

  DTRACE_SCOPED();
  VsyncPeriodNanos vsync_period;
  if (GetVsyncPeriodByActiveConfig(&vsync_period) == HWC2::Error::None) {
    SubmitActiveConfigChange(vsync_period);
  }
}

HWC2::Error HWCDisplay::GetVsyncPeriodByActiveConfig(VsyncPeriodNanos *vsync_period) {
  hwc2_config_t active_config;

  auto error = GetCachedActiveConfig(&active_config);
  if (error != HWC2::Error::None) {
    DLOGE("Failed to get active config!");
    return error;
  }

  int32_t active_vsync_period;
  error = GetDisplayAttribute(active_config, HwcAttribute::VSYNC_PERIOD, &active_vsync_period);
  if (error != HWC2::Error::None) {
    DLOGE("Failed to get VsyncPeriod of config: %d", active_config);
    return error;
  }

  *vsync_period = static_cast<VsyncPeriodNanos>(active_vsync_period);
  return HWC2::Error::None;
}

bool HWCDisplay::GetTransientVsyncPeriod(VsyncPeriodNanos *vsync_period) {
  std::lock_guard<std::mutex> lock(transient_refresh_rate_lock_);
  auto now = systemTime(SYSTEM_TIME_MONOTONIC);

  while (!transient_refresh_rate_info_.empty()) {
    if (IsActiveConfigApplied(now, transient_refresh_rate_info_.front().vsync_applied_time)) {
      transient_refresh_rate_info_.pop_front();
    } else {
      *vsync_period = transient_refresh_rate_info_.front().transient_vsync_period;
      return true;
    }
  }

  return false;
}

std::tuple<int64_t, int64_t> HWCDisplay::RequestActiveConfigChange(
    hwc2_config_t config, VsyncPeriodNanos current_vsync_period, int64_t desired_time) {
  int64_t refresh_time, applied_time;
  std::tie(refresh_time, applied_time) =
      EstimateVsyncPeriodChangeTimeline(current_vsync_period, desired_time);

  pending_refresh_rate_config_ = config;
  pending_refresh_rate_refresh_time_ = refresh_time;
  pending_refresh_rate_applied_time_ = applied_time;

  return std::make_tuple(refresh_time, applied_time);
}

std::tuple<int64_t, int64_t> HWCDisplay::EstimateVsyncPeriodChangeTimeline(
    VsyncPeriodNanos current_vsync_period, int64_t desired_time) {
  const auto now = systemTime(SYSTEM_TIME_MONOTONIC);
  const auto delta = desired_time - now;
  const auto refresh_rate_activate_period = current_vsync_period * vsyncs_to_apply_rate_change_;
  nsecs_t refresh_time;

  if (delta < 0) {
    refresh_time = now + (delta % current_vsync_period);
  } else if (delta < refresh_rate_activate_period) {
    refresh_time = now + (delta % current_vsync_period) - current_vsync_period;
  } else {
    refresh_time = desired_time - refresh_rate_activate_period;
  }

  const auto applied_time = refresh_time + refresh_rate_activate_period;
  return std::make_tuple(refresh_time, applied_time);
}

void HWCDisplay::SubmitActiveConfigChange(VsyncPeriodNanos current_vsync_period) {
  HWC2::Error error = SubmitDisplayConfig(pending_refresh_rate_config_);
  if (error != HWC2::Error::None) {
    return;
  }

  std::lock_guard<std::mutex> lock(transient_refresh_rate_lock_);
  hwc_vsync_period_change_timeline_t timeline;
  std::tie(timeline.refreshTimeNanos, timeline.newVsyncAppliedTimeNanos) =
      EstimateVsyncPeriodChangeTimeline(current_vsync_period, pending_refresh_rate_refresh_time_);

  transient_refresh_rate_info_.push_back({current_vsync_period, timeline.newVsyncAppliedTimeNanos});
  if (timeline.newVsyncAppliedTimeNanos != pending_refresh_rate_applied_time_) {
    timeline.refreshRequired = false;
    callbacks_->VsyncPeriodTimingChanged(id_, &timeline);
  }

  pending_refresh_rate_config_ = UINT_MAX;
  pending_refresh_rate_refresh_time_ = INT64_MAX;
  pending_refresh_rate_applied_time_ = INT64_MAX;
}

bool HWCDisplay::IsActiveConfigReadyToSubmit(int64_t time) {
  return ((pending_refresh_rate_config_ != UINT_MAX) &&
          IsTimeAfterOrEqualVsyncTime(time, pending_refresh_rate_refresh_time_));
}

bool HWCDisplay::IsActiveConfigApplied(int64_t time, int64_t vsync_applied_time) {
  return IsTimeAfterOrEqualVsyncTime(time, vsync_applied_time);
}

bool HWCDisplay::IsSameGroup(hwc2_config_t config_id1, hwc2_config_t config_id2) {
  const auto &variable_config1 = variable_config_map_.find(config_id1);
  const auto &variable_config2 = variable_config_map_.find(config_id2);

  if ((variable_config1 == variable_config_map_.end()) ||
      (variable_config2 == variable_config_map_.end())) {
    DLOGE("Invalid config: %u, %u", config_id1, config_id2);
    return false;
  }

  const DisplayConfigGroupInfo &config_group1 = variable_config1->second;
  const DisplayConfigGroupInfo &config_group2 = variable_config2->second;

  return (config_group1 == config_group2);
}

bool HWCDisplay::AllowSeamless(hwc2_config_t config) {
  hwc2_config_t active_config;
  auto error = GetCachedActiveConfig(&active_config);
  if (error != HWC2::Error::None) {
    DLOGE("Failed to get active config!");
    return false;
  }

  return IsSameGroup(active_config, config);
}

HWC2::Error HWCDisplay::SubmitDisplayConfig(hwc2_config_t config) {
  DTRACE_SCOPED();

  hwc2_config_t current_config = 0;
  GetActiveConfig(&current_config);
  if (current_config == config) {
    return HWC2::Error::None;
  }

  DisplayError error = display_intf_->SetActiveConfig(config);
  if (error != kErrorNone) {
    DLOGE("Failed to set %d config! Error: %d", config, error);
    return HWC2::Error::BadConfig;
  }

  validated_ = false;
  SetActiveConfigIndex(config);
  DLOGI("Active configuration changed to: %d", config);

  return HWC2::Error::None;
}

HWC2::Error HWCDisplay::GetCachedActiveConfig(hwc2_config_t *active_config) {
  int config_index = GetActiveConfigIndex();
  if ((config_index < 0) || (config_index >= hwc_config_map_.size())) {
    return GetActiveConfig(active_config);
  }

  *active_config = static_cast<hwc2_config_t>(hwc_config_map_.at(config_index));
  return HWC2::Error::None;
}

void HWCDisplay::SetActiveConfigIndex(int index) {
  std::lock_guard<std::mutex> lock(active_config_lock_);
  active_config_index_ = index;
}

int HWCDisplay::GetActiveConfigIndex() {
  std::lock_guard<std::mutex> lock(active_config_lock_);
  return active_config_index_;
}

HWC2::Error HWCDisplay::GetClientTargetProperty(ClientTargetProperty *out_client_target_property) {

  Layer *client_layer = client_target_->GetSDMLayer();
  if (!client_layer->request.flags.update_format) {
    return HWC2::Error::None;
  }
  int32_t format = 0;
  uint64_t flags = 0;
  auto err = buffer_allocator_->SetBufferInfo(client_layer->request.format, &format,
                                              &flags);
  if (err) {
    DLOGE("Invalid format: %s requested", GetFormatString(client_layer->request.format));
    return HWC2::Error::BadParameter;
  }
  Dataspace dataspace;
  DisplayError error = ColorMetadataToDataspace(client_layer->request.color_metadata,
                                                   &dataspace);
  if (error != kErrorNone) {
    DLOGE("Invalid Dataspace requested: Primaries = %d Transfer = %d ds = %d",
          client_layer->request.color_metadata.colorPrimaries,
          client_layer->request.color_metadata.transfer, dataspace);
    return HWC2::Error::BadParameter;
  }
  out_client_target_property->dataspace = dataspace;
  out_client_target_property->pixelFormat =
      (android::hardware::graphics::common::V1_2::PixelFormat)format;

  return HWC2::Error::None;
}

} //namespace sdm
