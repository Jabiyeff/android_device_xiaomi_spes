/*
* Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
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

#include "hw_tv_drm.h"
#include <math.h>
#include <sys/time.h>
#include <utils/debug.h>
#include <utils/sys.h>
#include <utils/formats.h>
#include <drm_lib_loader.h>
#include <drm_master.h>
#include <drm_res_mgr.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>
#include <vector>
#include <map>
#include <utility>

#ifndef HDR_EOTF_SMTPE_ST2084
#define HDR_EOTF_SMTPE_ST2084 2
#endif
#ifndef HDR_EOTF_HLG
#define HDR_EOTF_HLG 3
#endif

#define __CLASS__ "HWTVDRM"

#define HDR_DISABLE 0
#define HDR_ENABLE 1
#define MIN_HDR_RESET_WAITTIME 2

using drm_utils::DRMMaster;
using drm_utils::DRMResMgr;
using drm_utils::DRMLibLoader;
using drm_utils::DRMBuffer;
using sde_drm::GetDRMManager;
using sde_drm::DestroyDRMManager;
using sde_drm::DRMDisplayType;
using sde_drm::DRMDisplayToken;
using sde_drm::DRMConnectorInfo;
using sde_drm::DRMPPFeatureInfo;
using sde_drm::DRMOps;
using sde_drm::DRMTopology;
using sde_drm::DRMPowerMode;
using sde_drm::DRMColorspace;

namespace sdm {

static int32_t GetEOTF(const GammaTransfer &transfer) {
  int32_t hdr_transfer = -1;

  switch (transfer) {
  case Transfer_SMPTE_ST2084:
    hdr_transfer = HDR_EOTF_SMTPE_ST2084;
    break;
  case Transfer_HLG:
    hdr_transfer = HDR_EOTF_HLG;
    break;
  default:
    DLOGW("Unknown Transfer: %d", transfer);
  }

  return hdr_transfer;
}

static float GetMaxOrAverageLuminance(float luminance) {
  return (50.0f * powf(2.0f, (luminance / 32.0f)));
}

static float GetMinLuminance(float luminance, float max_luminance) {
  return (max_luminance * ((luminance / 255.0f) * (luminance / 255.0f)) / 100.0f);
}

HWTVDRM::HWTVDRM(int32_t display_id, BufferAllocator *buffer_allocator,
                 HWInfoInterface *hw_info_intf)
  : HWDeviceDRM(buffer_allocator, hw_info_intf) {
  disp_type_ = DRMDisplayType::TV;
  device_name_ = "TV";
  display_id_ = display_id;
}

DisplayError HWTVDRM::SetDisplayAttributes(uint32_t index) {
  if (index >= connector_info_.modes.size()) {
    DLOGE("Invalid mode index %d mode size %d", index, UINT32(connector_info_.modes.size()));
    return kErrorNotSupported;
  }

  current_mode_index_ = index;
  PopulateHWPanelInfo();
  UpdateMixerAttributes();

  DLOGI("Display attributes[%d]: WxH: %dx%d, DPI: %fx%f, FPS: %d, LM_SPLIT: %d, V_BACK_PORCH: %d," \
        " V_FRONT_PORCH: %d, V_PULSE_WIDTH: %d, V_TOTAL: %d, H_TOTAL: %d, CLK: %dKHZ, TOPOLOGY: %d",
        index, display_attributes_[index].x_pixels, display_attributes_[index].y_pixels,
        display_attributes_[index].x_dpi, display_attributes_[index].y_dpi,
        display_attributes_[index].fps, display_attributes_[index].is_device_split,
        display_attributes_[index].v_back_porch, display_attributes_[index].v_front_porch,
        display_attributes_[index].v_pulse_width, display_attributes_[index].v_total,
        display_attributes_[index].h_total, display_attributes_[index].clock_khz,
        display_attributes_[index].topology);

  return kErrorNone;
}

DisplayError HWTVDRM::GetConfigIndex(char *mode, uint32_t *index) {
  uint32_t width = 0, height = 0, fps = 0, format = 0;
  std::string str(mode);

  // mode should be in width:height:fps:format
  // TODO(user): it is not fully robust, User needs to provide in above format only
  if (str.length() != 0) {
    width = UINT32(stoi(str));
    height = UINT32(stoi(str.substr(str.find(':') + 1)));
    std::string str3 = str.substr(str.find(':') + 1);
    fps = UINT32(stoi(str3.substr(str3.find(':')  + 1)));
    std::string str4 = str3.substr(str3.find(':') + 1);
    format = UINT32(stoi(str4.substr(str4.find(':') + 1)));
  }

  for (size_t idex = 0; idex < connector_info_.modes.size(); idex ++) {
    if ((height == connector_info_.modes[idex].mode.vdisplay) &&
        (width == connector_info_.modes[idex].mode.hdisplay) &&
        (fps == connector_info_.modes[idex].mode.vrefresh)) {
      if ((format >> 1) & (connector_info_.modes[idex].mode.flags >> kBitYUV)) {
        *index = UINT32(idex);
        break;
      }

      if (format & (connector_info_.modes[idex].mode.flags >> kBitRGB)) {
        *index = UINT32(idex);
        break;
      }
    }
  }

  return kErrorNone;
}

DisplayError HWTVDRM::Deinit() {
  if (hw_panel_info_.hdr_enabled) {
    memset(&hdr_metadata_, 0, sizeof(hdr_metadata_));
    hdr_metadata_.hdr_supported = 1;
    hdr_metadata_.hdr_state = HDR_DISABLE;
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_HDR_METADATA, token_.conn_id,
                              &hdr_metadata_);
  }

  return HWDeviceDRM::Deinit();
}

DisplayError HWTVDRM::GetDefaultConfig(uint32_t *default_config) {
  bool found = false;

  for (uint32_t i = 0; i < connector_info_.modes.size(); i++) {
    auto &mode = connector_info_.modes[i].mode;
    if (mode.hdisplay == 640 && mode.vdisplay == 480) {
      *default_config = i;
      found = true;
      DLOGI("Found 640x480 default mode, using as failure fallback");
      break;
    }
  }

  return found ? kErrorNone : kErrorNotSupported;
}

DisplayError HWTVDRM::PowerOff(bool teardown) {
  DTRACE_SCOPED();
  if (!drm_atomic_intf_) {
    DLOGE("DRM Atomic Interface is null!");
    return kErrorUndefined;
  }

  if (first_cycle_) {
    return kErrorNone;
  }

  if (teardown) {
    // LP connecter prop N/A for External
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 0);
  }
  int ret = drm_atomic_intf_->Commit(true /* synchronous */, false /* retain_planes*/);
  if (ret) {
    DLOGE("%s failed with error %d", __FUNCTION__, ret);
    return kErrorHardware;
  }

  return kErrorNone;
}

DisplayError HWTVDRM::Doze(const HWQosData &qos_data, shared_ptr<Fence> *release_fence) {
  return kErrorNone;
}

DisplayError HWTVDRM::DozeSuspend(const HWQosData &qos_data, shared_ptr<Fence> *release_fence) {
  return kErrorNone;
}

DisplayError HWTVDRM::Standby() {
  return kErrorNone;
}

void HWTVDRM::PopulateHWPanelInfo() {
  hw_panel_info_ = {};

  HWDeviceDRM::PopulateHWPanelInfo();
  hw_panel_info_.hdr_enabled = connector_info_.ext_hdr_prop.hdr_supported;
  hw_panel_info_.hdr_plus_enabled = connector_info_.ext_hdr_prop.hdr_plus_supported;
  hw_panel_info_.hdr_metadata_type_one = connector_info_.ext_hdr_prop.hdr_metadata_type_one;
  hw_panel_info_.hdr_eotf = connector_info_.ext_hdr_prop.hdr_eotf;
  hw_panel_info_.supported_colorspaces = connector_info_.supported_colorspaces;

  // Convert the raw luminance values from driver to Candela per meter^2 unit.
  float max_luminance = FLOAT(connector_info_.ext_hdr_prop.hdr_max_luminance);
  if (max_luminance != 0.0f) {
    max_luminance = GetMaxOrAverageLuminance(max_luminance);
  }
  bool valid_luminance = (max_luminance > kMinPeakLuminance) && (max_luminance < kMaxPeakLuminance);
  hw_panel_info_.peak_luminance = valid_luminance ? max_luminance : kDefaultMaxLuminance;

  float min_luminance = FLOAT(connector_info_.ext_hdr_prop.hdr_min_luminance);
  if (min_luminance != 0.0f) {
    min_luminance = GetMinLuminance(min_luminance, hw_panel_info_.peak_luminance);
  }
  hw_panel_info_.blackness_level = (min_luminance < 1.0f) ? min_luminance : kDefaultMinLuminance;

  float average_luminance = FLOAT(connector_info_.ext_hdr_prop.hdr_avg_luminance);
  if (average_luminance != 0.0f) {
    average_luminance = GetMaxOrAverageLuminance(average_luminance);
  } else {
    average_luminance = (hw_panel_info_.peak_luminance + hw_panel_info_.blackness_level) / 2.0f;
  }
  hw_panel_info_.average_luminance = average_luminance;

  DLOGI("TV Panel: %s%s, type_one = %d, eotf = %d, luminance[max = %f, min = %f, avg = %f]",
        hw_panel_info_.hdr_enabled ? "HDR" : "Non-HDR",
        hw_panel_info_.hdr_plus_enabled ? "10+" : "", hw_panel_info_.hdr_metadata_type_one,
        hw_panel_info_.hdr_eotf, hw_panel_info_.peak_luminance, hw_panel_info_.blackness_level,
        hw_panel_info_.average_luminance);
}

DisplayError HWTVDRM::Commit(HWLayers *hw_layers) {
  DisplayError error = UpdateHDRMetaData(hw_layers);
  if (error != kErrorNone) {
    return error;
  }
  return HWDeviceDRM::Commit(hw_layers);
}

DisplayError HWTVDRM::UpdateHDRMetaData(HWLayers *hw_layers) {
  // Set colorspace on external DP when DP supports colorspace.
  // For P3 use case set colorspace only.
  // For HDR use case set both hdr metadata and colorspace.
  if (hw_panel_info_.port == kPortDP && hw_panel_info_.supported_colorspaces) {
    sde_drm::DRMColorspace colorspace = sde_drm::DRMColorspace::DEFAULT;
    if (blend_space_.primaries == ColorPrimaries_DCIP3 &&
        blend_space_.transfer == Transfer_sRGB) {
      colorspace = sde_drm::DRMColorspace::DCI_P3_RGB_D65;
    /* In case of BT2020_YCC, BT2020_RGB is not set based on the layer format. We set it based on
       the final output of display port controller. Here even though the layer as YUV , it will be
       color converted to RGB using SSPP and the format going out of DP will be RGB. Hence we
       should set BT2020_RGB. */
    } else if (blend_space_.primaries == ColorPrimaries_BT2020) {
      colorspace = sde_drm::DRMColorspace::BT2020_RGB;
    }
    DLOGV_IF(kTagDriverConfig, "Set colorspace = %d", colorspace);
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_COLORSPACE, token_.conn_id, colorspace);
  }

  if (!hw_panel_info_.hdr_enabled) {
    return kErrorNone;
  }

  const HWHDRLayerInfo &hdr_layer_info = hw_layers->info.hdr_layer_info;
  DisplayError error = kErrorNone;
  HWHDRLayerInfo::HDROperation hdr_op = hdr_layer_info.operation;

  Layer hdr_layer = {};
  if (hdr_op == HWHDRLayerInfo::kSet && hdr_layer_info.layer_index > -1) {
    hdr_layer = *(hw_layers->info.stack->layers.at(UINT32(hdr_layer_info.layer_index)));
  }

  const LayerBuffer *layer_buffer = &hdr_layer.input_buffer;
  const MasteringDisplay &mastering_display = layer_buffer->color_metadata.masteringDisplayInfo;
  const ContentLightLevel &light_level = layer_buffer->color_metadata.contentLightLevel;
  const Primaries &primaries = mastering_display.primaries;

  if (hdr_op == HWHDRLayerInfo::kSet && hdr_layer_info.hdr_layers.size() == 1) {
    // Reset reset_hdr_flag_ to handle where there are two consecutive HDR video playbacks with not
    // enough non-HDR frames in between to reset the HDR metadata.
    reset_hdr_flag_ = false;
    in_multiset_ = false;

    int32_t eotf = GetEOTF(layer_buffer->color_metadata.transfer);
    hdr_metadata_.hdr_supported = 1;
    hdr_metadata_.hdr_state = HDR_ENABLE;
    hdr_metadata_.eotf = (eotf < 0) ? 0 : UINT32(eotf);
    hdr_metadata_.white_point_x = primaries.whitePoint[0];
    hdr_metadata_.white_point_y = primaries.whitePoint[1];
    hdr_metadata_.display_primaries_x[0] = primaries.rgbPrimaries[0][0];
    hdr_metadata_.display_primaries_y[0] = primaries.rgbPrimaries[0][1];
    hdr_metadata_.display_primaries_x[1] = primaries.rgbPrimaries[1][0];
    hdr_metadata_.display_primaries_y[1] = primaries.rgbPrimaries[1][1];
    hdr_metadata_.display_primaries_x[2] = primaries.rgbPrimaries[2][0];
    hdr_metadata_.display_primaries_y[2] = primaries.rgbPrimaries[2][1];
    hdr_metadata_.min_luminance = mastering_display.minDisplayLuminance;
    hdr_metadata_.max_luminance = mastering_display.maxDisplayLuminance;
    hdr_metadata_.max_content_light_level = light_level.maxContentLightLevel;
    hdr_metadata_.max_average_light_level = light_level.minPicAverageLightLevel;
    if (hw_panel_info_.hdr_plus_enabled && hdr_layer_info.dyn_hdr_vsif_payload.size()) {
      hdr_metadata_.hdr_plus_payload = reinterpret_cast<uint64_t>
                                        (hdr_layer_info.dyn_hdr_vsif_payload.data());
      hdr_metadata_.hdr_plus_payload_size = UINT32(hdr_layer_info.dyn_hdr_vsif_payload.size());
    } else {
      hdr_metadata_.hdr_plus_payload = reinterpret_cast<uint64_t>(nullptr);
      hdr_metadata_.hdr_plus_payload_size = 0;
    }

    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_HDR_METADATA, token_.conn_id, &hdr_metadata_);
    DumpHDRMetaData(hdr_op);
  } else if (hdr_op == HWHDRLayerInfo::kSet && !in_multiset_) {
    // Special case to handle multiple HDR layers.
    // If there are multiple HDR layers, then simply drop all metadata (which is optional) since
    // content going in and out of view (e.g., video start/stop, scrolling video preview thumbnails)
    // will cause flicker.
    InitMaxHDRMetaData();
    in_multiset_ = true;
    reset_hdr_flag_ = false;
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_HDR_METADATA, token_.conn_id, &hdr_metadata_);
    DumpHDRMetaData(hdr_op);
  } else if (hdr_op == HWHDRLayerInfo::kReset) {
    memset(&hdr_metadata_, 0, sizeof(hdr_metadata_));
    hdr_metadata_.hdr_supported = 1;
    hdr_metadata_.hdr_state = HDR_ENABLE;
    reset_hdr_flag_ = true;
    in_multiset_ = false;
    gettimeofday(&hdr_reset_start_, NULL);

    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_HDR_METADATA, token_.conn_id, &hdr_metadata_);
    DumpHDRMetaData(hdr_op);
  } else if (hdr_op == HWHDRLayerInfo::kNoOp) {
    // TODO(user): This case handles the state transition from HDR_ENABLED to HDR_DISABLED.
    // As per HDMI spec requirement, we need to send zero metadata for atleast 2 sec after end of
    // playback. This timer calculates the 2 sec window after playback stops to stop sending HDR
    // metadata. This will be replaced with an idle timer implementation in the future.
    if (reset_hdr_flag_) {
      gettimeofday(&hdr_reset_end_, NULL);
      float hdr_reset_time_start = ((hdr_reset_start_.tv_sec * 1000) +
                                    (hdr_reset_start_.tv_usec / 1000));
      float hdr_reset_time_end = ((hdr_reset_end_.tv_sec * 1000) + (hdr_reset_end_.tv_usec / 1000));

      if (((hdr_reset_time_end - hdr_reset_time_start) / 1000) >= MIN_HDR_RESET_WAITTIME) {
        memset(&hdr_metadata_, 0, sizeof(hdr_metadata_));
        hdr_metadata_.hdr_supported = 1;
        hdr_metadata_.hdr_state = HDR_DISABLE;
        reset_hdr_flag_ = false;

        drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_HDR_METADATA, token_.conn_id,
                                  &hdr_metadata_);
      }
    }
  }

  return error;
}

void HWTVDRM::DumpHDRMetaData(HWHDRLayerInfo::HDROperation operation) {
  DLOGI("Operation = %d, HDR Metadata: MaxDisplayLuminance = %d MinDisplayLuminance = %d\n"
        "MaxContentLightLevel = %d MaxAverageLightLevel = %d Red_x = %d Red_y = %d Green_x = %d\n"
        "Green_y = %d Blue_x = %d Blue_y = %d WhitePoint_x = %d WhitePoint_y = %d EOTF = %d\n"
        "HDR10+ payload size = %u\n",
        operation, hdr_metadata_.max_luminance, hdr_metadata_.min_luminance,
        hdr_metadata_.max_content_light_level, hdr_metadata_.max_average_light_level,
        hdr_metadata_.display_primaries_x[0], hdr_metadata_.display_primaries_y[0],
        hdr_metadata_.display_primaries_x[1], hdr_metadata_.display_primaries_y[1],
        hdr_metadata_.display_primaries_x[2], hdr_metadata_.display_primaries_y[2],
        hdr_metadata_.white_point_x, hdr_metadata_.white_point_y, hdr_metadata_.eotf,
        hdr_metadata_.hdr_plus_payload_size);
}

void HWTVDRM::InitMaxHDRMetaData() {
  memset(&hdr_metadata_, 0, sizeof(hdr_metadata_));
  hdr_metadata_.hdr_supported = 1;
  hdr_metadata_.hdr_state = HDR_ENABLE;
  hdr_metadata_.eotf = UINT32(GetEOTF(Transfer_SMPTE_ST2084));
  // Rec. 2020 (ITU-R Recommendation BT.2020) RGB color space parameters
  // +---------------+-----------------+-----------------------------------------------+
  // |               |   White point   |                Primary colors                 |
  // |  Color space  +--------+--------+-------+-------+-------+-------+-------+-------+
  // |               |   xW   |   yW   |  xR   |  yR   |  xG   |  yG   |  xB   |  yB   |
  // +---------------+--------+--------+-------+-------+-------+-------+-------+-------+
  // | ITU-R BT.2020 | 0.3127 | 0.3290 | 0.708 | 0.292 | 0.170 | 0.797 | 0.131 | 0.046 |
  // +---------------+--------+--------+-------+-------+-------+-------+-------+-------+
  // Rec. 2020 D65 'CIE Standard Illuminant'.
  hdr_metadata_.white_point_x = 15635;                // 0.31271 x 50000
  hdr_metadata_.white_point_y = 16451;                // 0.32902 x 50000
  // Rec. 2020 primaries.
  hdr_metadata_.display_primaries_x[0] = 35400;       // 0.708 x 50000
  hdr_metadata_.display_primaries_y[0] = 14600;       // 0.292 x 50000
  hdr_metadata_.display_primaries_x[1] = 8500;        // 0.170 x 50000
  hdr_metadata_.display_primaries_y[1] = 39850;       // 0.797 x 50000
  hdr_metadata_.display_primaries_x[2] = 6550;        // 0.131 x 50000
  hdr_metadata_.display_primaries_y[2] = 2300;        // 0.046 x 50000
  hdr_metadata_.min_luminance = 0;                    // 0 nits
  hdr_metadata_.max_luminance = 100000000;            // 10000 nits
  hdr_metadata_.max_content_light_level = 100000000;  // 10000 nits brightest pixel in content
  hdr_metadata_.max_average_light_level = 100000000;  // 10000 nits brightest frame in content
}

DisplayError HWTVDRM::PowerOn(const HWQosData &qos_data, shared_ptr<Fence> *release_fence) {
  DTRACE_SCOPED();
  if (!drm_atomic_intf_) {
    DLOGE("DRM Atomic Interface is null!");
    return kErrorUndefined;
  }

  if (first_cycle_) {
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, token_.conn_id, token_.crtc_id);
    drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, &current_mode);
  }

  return HWDeviceDRM::PowerOn(qos_data, release_fence);
}

}  // namespace sdm

