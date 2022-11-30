/*
Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <ctype.h>
#include <drm_logger.h>
#include <utils/debug.h>
#include <utils/utils.h>
#include <algorithm>
#include <vector>
#include "hw_device_drm.h"
#include "hw_virtual_drm.h"
#include "hw_info_drm.h"

#define __CLASS__ "HWVirtualDRM"

using std::vector;

using sde_drm::DRMDisplayType;
using sde_drm::DRMConnectorInfo;
using sde_drm::DRMRect;
using sde_drm::DRMOps;
using sde_drm::DRMPowerMode;
using sde_drm::DRMSecureMode;

namespace sdm {

HWVirtualDRM::HWVirtualDRM(int32_t display_id, BufferAllocator *buffer_allocator,
                           HWInfoInterface *hw_info_intf)
  : HWDeviceDRM(buffer_allocator, hw_info_intf) {
  HWDeviceDRM::device_name_ = "Virtual";
  HWDeviceDRM::disp_type_ = DRMDisplayType::VIRTUAL;
  HWDeviceDRM::display_id_ = display_id;
}

void HWVirtualDRM::ConfigureWbConnectorFbId(uint32_t fb_id) {
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_OUTPUT_FB_ID, token_.conn_id, fb_id);
  return;
}

void HWVirtualDRM::ConfigureWbConnectorDestRect() {
  DRMRect dst = {};
  dst.left = 0;
  dst.bottom = display_attributes_[current_mode_index_].y_pixels;
  dst.top = 0;
  dst.right = display_attributes_[current_mode_index_].x_pixels;
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_OUTPUT_RECT, token_.conn_id, dst);
  return;
}

void HWVirtualDRM::ConfigureWbConnectorSecureMode(bool secure) {
  DRMSecureMode secure_mode = secure ? DRMSecureMode::SECURE : DRMSecureMode::NON_SECURE;
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_FB_SECURE_MODE, token_.conn_id, secure_mode);
}

void HWVirtualDRM::InitializeConfigs() {
  display_attributes_.resize(connector_info_.modes.size());
  for (uint32_t i = 0; i < connector_info_.modes.size(); i++) {
    PopulateDisplayAttributes(i);
  }
}

DisplayError HWVirtualDRM::SetWbConfigs(const HWDisplayAttributes &display_attributes) {
  if (display_attributes.x_pixels > connector_info_.max_linewidth) {
    DLOGE("Requested width %d is more than supported %d", display_attributes.x_pixels,
           connector_info_.max_linewidth);
    return kErrorHardware;
  }

  drmModeModeInfo mode = {};
  vector<drmModeModeInfo> modes;

  mode.hdisplay = mode.hsync_start = mode.hsync_end = mode.htotal =
                                       UINT16(display_attributes.x_pixels);
  mode.vdisplay = mode.vsync_start = mode.vsync_end = mode.vtotal =
                                       UINT16(display_attributes.y_pixels);
  mode.vrefresh = UINT32(display_attributes.fps);
  mode.clock = (mode.htotal * mode.vtotal * mode.vrefresh) / 1000;
  snprintf(mode.name, DRM_DISPLAY_MODE_LEN, "%dx%d", mode.hdisplay, mode.vdisplay);
  modes.push_back(mode);
  for (auto &item : connector_info_.modes) {
    modes.push_back(item.mode);
  }

  // Inform the updated mode list to the driver
  struct sde_drm_wb_cfg wb_cfg = {};
  wb_cfg.connector_id = token_.conn_id;
  wb_cfg.flags = SDE_DRM_WB_CFG_FLAGS_CONNECTED;
  wb_cfg.count_modes = UINT32(modes.size());
  wb_cfg.modes = (uint64_t)modes.data();

  int ret = -EINVAL;
#ifdef DRM_IOCTL_SDE_WB_CONFIG
  ret = drmIoctl(dev_fd_, DRM_IOCTL_SDE_WB_CONFIG, &wb_cfg);
#endif
  if (ret) {
    DLOGE("Dump WBConfig: mode_count %d flags %x", wb_cfg.count_modes, wb_cfg.flags);
    DumpConnectorModeInfo();
    return kErrorHardware;
  }

  return kErrorNone;
}

DisplayError HWVirtualDRM::Commit(HWLayers *hw_layers) {
  if (!hw_layers->info.stack) {
    return kErrorNone;
  }

  LayerBuffer *output_buffer = hw_layers->info.stack->output_buffer;
  DisplayError err = kErrorNone;

  registry_.Register(hw_layers);
  registry_.MapOutputBufferToFbId(output_buffer);
  uint32_t fb_id = registry_.GetOutputFbId(output_buffer->handle_id);

  ConfigureWbConnectorFbId(fb_id);
  ConfigureWbConnectorDestRect();
  ConfigureWbConnectorSecureMode(output_buffer->flags.secure);

  err = HWDeviceDRM::AtomicCommit(hw_layers);
  if (err != kErrorNone) {
    DLOGE("Atomic commit failed for crtc_id %d conn_id %d", token_.crtc_id, token_.conn_id);
  }

  return(err);
}

DisplayError HWVirtualDRM::Flush(HWLayers *hw_layers) {
  DisplayError err = kErrorNone;
  err = Commit(hw_layers);

  if (err != kErrorNone) {
    return err;
  }

  return kErrorNone;
}

DisplayError HWVirtualDRM::Validate(HWLayers *hw_layers) {
  LayerBuffer *output_buffer = hw_layers->info.stack->output_buffer;

  registry_.MapOutputBufferToFbId(output_buffer);
  uint32_t fb_id = registry_.GetOutputFbId(output_buffer->handle_id);

  ConfigureWbConnectorFbId(fb_id);
  ConfigureWbConnectorDestRect();
  ConfigureWbConnectorSecureMode(output_buffer->flags.secure);

  return HWDeviceDRM::Validate(hw_layers);
}

DisplayError HWVirtualDRM::SetDisplayAttributes(const HWDisplayAttributes &display_attributes) {
  if (display_attributes.x_pixels == 0 || display_attributes.y_pixels == 0) {
    return kErrorParameters;
  }

  int mode_index = -1;
  int ret = 0;
  GetModeIndex(display_attributes, &mode_index);

  if (mode_index < 0) {
    DisplayError error = SetWbConfigs(display_attributes);
    if (error != kErrorNone) {
      return error;
    }
  }

  // Reload connector info for updated info
  ret = drm_mgr_intf_->GetConnectorInfo(token_.conn_id, &connector_info_);
  if (ret) {
    DLOGE("Failed getting info for connector id %u. Error: %d.", token_.conn_id, ret);
    return kErrorHardware;
  }
  GetModeIndex(display_attributes, &mode_index);

  if (mode_index < 0) {
    DLOGE("Mode not found for resolution %dx%d fps %d", display_attributes.x_pixels,
          display_attributes.y_pixels, UINT32(display_attributes.fps));
    DumpConnectorModeInfo();
    return kErrorNotSupported;
  }

  current_mode_index_ = UINT32(mode_index);
  InitializeConfigs();
  PopulateHWPanelInfo();
  UpdateMixerAttributes();

  DLOGI("New WB Resolution: %dx%d cur_mode_index %d", display_attributes.x_pixels,
        display_attributes.y_pixels, current_mode_index_);

  return kErrorNone;
}

DisplayError HWVirtualDRM::GetPPFeaturesVersion(PPFeatureVersion *vers) {
  return kErrorNone;
}

void HWVirtualDRM::GetModeIndex(const HWDisplayAttributes &display_attributes, int *mode_index) {
  *mode_index = -1;
  for (uint32_t i = 0; i < connector_info_.modes.size(); i++) {
    if (display_attributes.x_pixels == connector_info_.modes[i].mode.hdisplay &&
        display_attributes.y_pixels == connector_info_.modes[i].mode.vdisplay &&
        display_attributes.fps == connector_info_.modes[i].mode.vrefresh) {
      *mode_index = INT32(i);
      break;
    }
  }
}

DisplayError HWVirtualDRM::PowerOn(const HWQosData &qos_data, shared_ptr<Fence> *release_fence) {
  DTRACE_SCOPED();
  if (!drm_atomic_intf_) {
    DLOGE("DRM Atomic Interface is null!");
    return kErrorUndefined;
  }

  // Since fb id is not available until first draw cycle and driver expects fb id to be set on any
  // commit(null or atomic commit). Need to defer power on for the first cycle.
  if (first_cycle_) {
    return kErrorNone;
  }

  DisplayError err = HWDeviceDRM::PowerOn(qos_data, release_fence);
  if (err != kErrorNone) {
    return err;
  }

  return kErrorNone;
}

DisplayError HWVirtualDRM::GetDisplayIdentificationData(uint8_t *out_port, uint32_t *out_data_size,
                                                        uint8_t *out_data) {
  *out_data_size = 0;
  *out_port = token_.hw_port;

  return kErrorNone;
}

}  // namespace sdm

