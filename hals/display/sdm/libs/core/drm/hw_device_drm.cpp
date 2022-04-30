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

#define __STDC_FORMAT_MACROS

#include <ctype.h>
#include <time.h>
#include <drm/drm_fourcc.h>
#include <drm_lib_loader.h>
#include <drm_master.h>
#include <drm_res_mgr.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/fb.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/formats.h>
#include <utils/sys.h>
#include <drm/sde_drm.h>
#include <private/color_params.h>
#include <utils/rect.h>
#include <utils/utils.h>
#include <utils/fence.h>

#include <sstream>
#include <ctime>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <limits>

#include "hw_device_drm.h"
#include "hw_info_interface.h"

#define __CLASS__ "HWDeviceDRM"

#ifndef DRM_FORMAT_MOD_QCOM_COMPRESSED
#define DRM_FORMAT_MOD_QCOM_COMPRESSED fourcc_mod_code(QCOM, 1)
#endif
#ifndef DRM_FORMAT_MOD_QCOM_DX
#define DRM_FORMAT_MOD_QCOM_DX fourcc_mod_code(QCOM, 0x2)
#endif
#ifndef DRM_FORMAT_MOD_QCOM_TIGHT
#define DRM_FORMAT_MOD_QCOM_TIGHT fourcc_mod_code(QCOM, 0x4)
#endif

using std::string;
using std::to_string;
using std::fstream;
using std::unordered_map;
using std::stringstream;
using std::ifstream;
using std::ofstream;
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
using sde_drm::DRMRect;
using sde_drm::DRMRotation;
using sde_drm::DRMBlendType;
using sde_drm::DRMSrcConfig;
using sde_drm::DRMOps;
using sde_drm::DRMTopology;
using sde_drm::DRMPowerMode;
using sde_drm::DRMSecureMode;
using sde_drm::DRMSecurityLevel;
using sde_drm::DRMCscType;
using sde_drm::DRMMultiRectMode;
using sde_drm::DRMSSPPLayoutIndex;

namespace sdm {

std::atomic<uint32_t> HWDeviceDRM::hw_dest_scaler_blocks_used_(0);

static PPBlock GetPPBlock(const HWToneMapLut &lut_type) {
  PPBlock pp_block = kPPBlockMax;
  switch (lut_type) {
    case kDma1dIgc:
    case kDma1dGc:
      pp_block = kDGM;
      break;
    case kVig1dIgc:
    case kVig3dGamut:
      pp_block = kVIG;
      break;
    default:
      DLOGE("Unknown PP Block");
      break;
  }
  return pp_block;
}

static void GetDRMFormat(LayerBufferFormat format, uint32_t *drm_format,
                         uint64_t *drm_format_modifier) {
  switch (format) {
    case kFormatRGBA8888:
      *drm_format = DRM_FORMAT_ABGR8888;
      break;
    case kFormatRGBA8888Ubwc:
      *drm_format = DRM_FORMAT_ABGR8888;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatRGBA5551:
      *drm_format = DRM_FORMAT_ABGR1555;
      break;
    case kFormatRGBA4444:
      *drm_format = DRM_FORMAT_ABGR4444;
      break;
    case kFormatBGRA8888:
      *drm_format = DRM_FORMAT_ARGB8888;
      break;
    case kFormatRGBX8888:
      *drm_format = DRM_FORMAT_XBGR8888;
      break;
    case kFormatRGBX8888Ubwc:
      *drm_format = DRM_FORMAT_XBGR8888;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatBGRX8888:
      *drm_format = DRM_FORMAT_XRGB8888;
      break;
    case kFormatRGB888:
      *drm_format = DRM_FORMAT_BGR888;
      break;
    case kFormatBGR888:
      *drm_format = DRM_FORMAT_RGB888;
      break;
    case kFormatRGB565:
      *drm_format = DRM_FORMAT_BGR565;
      break;
    case kFormatBGR565:
      *drm_format = DRM_FORMAT_RGB565;
      break;
    case kFormatBGR565Ubwc:
      *drm_format = DRM_FORMAT_BGR565;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatRGBA1010102:
      *drm_format = DRM_FORMAT_ABGR2101010;
      break;
    case kFormatRGBA1010102Ubwc:
      *drm_format = DRM_FORMAT_ABGR2101010;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatARGB2101010:
      *drm_format = DRM_FORMAT_BGRA1010102;
      break;
    case kFormatRGBX1010102:
      *drm_format = DRM_FORMAT_XBGR2101010;
      break;
    case kFormatRGBX1010102Ubwc:
      *drm_format = DRM_FORMAT_XBGR2101010;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatXRGB2101010:
      *drm_format = DRM_FORMAT_BGRX1010102;
      break;
    case kFormatBGRA1010102:
      *drm_format = DRM_FORMAT_ARGB2101010;
      break;
    case kFormatABGR2101010:
      *drm_format = DRM_FORMAT_RGBA1010102;
      break;
    case kFormatBGRX1010102:
      *drm_format = DRM_FORMAT_XRGB2101010;
      break;
    case kFormatXBGR2101010:
      *drm_format = DRM_FORMAT_RGBX1010102;
      break;
    case kFormatYCbCr420SemiPlanar:
      *drm_format = DRM_FORMAT_NV12;
      break;
    case kFormatYCbCr420SemiPlanarVenus:
      *drm_format = DRM_FORMAT_NV12;
      break;
    case kFormatYCbCr420SPVenusUbwc:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED;
      break;
    case kFormatYCbCr420SPVenusTile:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_TILE;
      break;
    case kFormatYCrCb420SemiPlanar:
      *drm_format = DRM_FORMAT_NV21;
      break;
    case kFormatYCrCb420SemiPlanarVenus:
      *drm_format = DRM_FORMAT_NV21;
      break;
    case kFormatYCbCr420P010:
    case kFormatYCbCr420P010Venus:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_DX;
      break;
    case kFormatYCbCr420P010Ubwc:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED |
        DRM_FORMAT_MOD_QCOM_DX;
      break;
    case kFormatYCbCr420P010Tile:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_TILE |
        DRM_FORMAT_MOD_QCOM_DX;
      break;
    case kFormatYCbCr420TP10Ubwc:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_COMPRESSED |
        DRM_FORMAT_MOD_QCOM_DX | DRM_FORMAT_MOD_QCOM_TIGHT;
      break;
    case kFormatYCbCr420TP10Tile:
      *drm_format = DRM_FORMAT_NV12;
      *drm_format_modifier = DRM_FORMAT_MOD_QCOM_TILE |
        DRM_FORMAT_MOD_QCOM_DX | DRM_FORMAT_MOD_QCOM_TIGHT;
      break;
    case kFormatYCbCr422H2V1SemiPlanar:
      *drm_format = DRM_FORMAT_NV16;
      break;
    case kFormatYCrCb422H2V1SemiPlanar:
      *drm_format = DRM_FORMAT_NV61;
      break;
    case kFormatYCrCb420PlanarStride16:
      *drm_format = DRM_FORMAT_YVU420;
      break;
    default:
      DLOGW("Unsupported format %s", GetFormatString(format));
  }
}

class FrameBufferObject : public LayerBufferObject {
 public:
  explicit FrameBufferObject(uint32_t fb_id, LayerBufferFormat format,
                             uint32_t width, uint32_t height)
    :fb_id_(fb_id), format_(format), width_(width), height_(height) {
  }

  ~FrameBufferObject() {
    DRMMaster *master;
    DRMMaster::GetInstance(&master);
    int ret = master->RemoveFbId(fb_id_);
    if (ret < 0) {
      DLOGE("Removing fb_id %d failed with error %d", fb_id_, errno);
    }
  }
  uint32_t GetFbId() { return fb_id_; }
  bool IsEqual(LayerBufferFormat format, uint32_t width, uint32_t height) {
    return (format == format_ && width == width_ && height == height_);
  }

 private:
  uint32_t fb_id_;
  LayerBufferFormat format_;
  uint32_t width_;
  uint32_t height_;
};

HWDeviceDRM::Registry::Registry(BufferAllocator *buffer_allocator) :
  buffer_allocator_(buffer_allocator) {
  int value = 0;
  if (Debug::GetProperty(DISABLE_FBID_CACHE, &value) == kErrorNone) {
    disable_fbid_cache_ = (value == 1);
  }
}

void HWDeviceDRM::Registry::Register(HWLayers *hw_layers) {
  HWLayersInfo &hw_layer_info = hw_layers->info;
  uint32_t hw_layer_count = UINT32(hw_layer_info.hw_layers.size());

  for (uint32_t i = 0; i < hw_layer_count; i++) {
    Layer &layer = hw_layer_info.hw_layers.at(i);
    LayerBuffer input_buffer = layer.input_buffer;
    HWRotatorSession *hw_rotator_session = &hw_layers->config[i].hw_rotator_session;
    HWRotateInfo *hw_rotate_info = &hw_rotator_session->hw_rotate_info[0];
    fbid_cache_limit_ = input_buffer.flags.video ? VIDEO_FBID_LIMIT : UI_FBID_LIMIT;

    if (hw_rotator_session->mode == kRotatorOffline && hw_rotate_info->valid) {
      input_buffer = hw_rotator_session->output_buffer;
      fbid_cache_limit_ = OFFLINE_ROTATOR_FBID_LIMIT;
    }

    if (input_buffer.flags.interlace) {
      input_buffer.width *= 2;
      input_buffer.height /= 2;
    }
    MapBufferToFbId(&layer, input_buffer);
  }
}

int HWDeviceDRM::Registry::CreateFbId(const LayerBuffer &buffer, uint32_t *fb_id) {
  DRMMaster *master = nullptr;
  DRMMaster::GetInstance(&master);
  int ret = -1;

  if (!master) {
    DLOGE("Failed to acquire DRM Master instance");
    return ret;
  }

  DRMBuffer layout{};
  AllocatedBufferInfo buf_info{};
  buf_info.fd = layout.fd = buffer.planes[0].fd;
  buf_info.aligned_width = layout.width = buffer.width;
  buf_info.aligned_height = layout.height = buffer.height;
  buf_info.format = buffer.format;
  GetDRMFormat(buf_info.format, &layout.drm_format, &layout.drm_format_modifier);
  buffer_allocator_->GetBufferLayout(buf_info, layout.stride, layout.offset, &layout.num_planes);
  ret = master->CreateFbId(layout, fb_id);
  if (ret < 0) {
    DLOGE("CreateFbId failed. width %d, height %d, format: %s, stride %u, error %d",
        layout.width, layout.height, GetFormatString(buf_info.format), layout.stride[0], errno);
  }

  return ret;
}

void HWDeviceDRM::Registry::MapBufferToFbId(Layer* layer, const LayerBuffer &buffer) {
  if (buffer.planes[0].fd < 0) {
    return;
  }

  uint64_t handle_id = buffer.handle_id;
  if (!handle_id || disable_fbid_cache_) {
    // In legacy path, clear fb_id map in each frame.
    layer->buffer_map->buffer_map.clear();
  } else {
    auto it = layer->buffer_map->buffer_map.find(handle_id);
    if (it != layer->buffer_map->buffer_map.end()) {
      FrameBufferObject *fb_obj = static_cast<FrameBufferObject*>(it->second.get());
      if (fb_obj->IsEqual(buffer.format, buffer.width, buffer.height)) {
        // Found fb_id for given handle_id key
        return;
      } else {
        // Erase from fb_id map if format or size have been modified
        layer->buffer_map->buffer_map.erase(it);
      }
    }

    if (layer->buffer_map->buffer_map.size() >= fbid_cache_limit_) {
      // Clear fb_id map, if the size reaches cache limit.
      layer->buffer_map->buffer_map.clear();
    }
  }

  uint32_t fb_id = 0;
  if (CreateFbId(buffer, &fb_id) >= 0) {
    // Create and cache the fb_id in map
    layer->buffer_map->buffer_map[handle_id] = std::make_shared<FrameBufferObject>(fb_id,
        buffer.format, buffer.width, buffer.height);
  }
}

void HWDeviceDRM::Registry::MapOutputBufferToFbId(LayerBuffer *output_buffer) {
  if (output_buffer->planes[0].fd < 0) {
    return;
  }

  uint64_t handle_id = output_buffer->handle_id;
  if (!handle_id || disable_fbid_cache_) {
    // In legacy path, clear output buffer map in each frame.
    output_buffer_map_.clear();
  } else {
    auto it = output_buffer_map_.find(handle_id);
    if (it != output_buffer_map_.end()) {
      FrameBufferObject *fb_obj = static_cast<FrameBufferObject*>(it->second.get());
      if (fb_obj->IsEqual(output_buffer->format, output_buffer->width, output_buffer->height)) {
        return;
      } else {
        output_buffer_map_.erase(it);
      }
    }

    if (output_buffer_map_.size() >= UI_FBID_LIMIT) {
      // Clear output buffer map, if the size reaches cache limit.
      output_buffer_map_.clear();
    }
  }

  uint32_t fb_id = 0;
  if (CreateFbId(*output_buffer, &fb_id) >= 0) {
    output_buffer_map_[handle_id] = std::make_shared<FrameBufferObject>(fb_id,
        output_buffer->format, output_buffer->width, output_buffer->height);
  }
}

void HWDeviceDRM::Registry::Clear() {
  output_buffer_map_.clear();
}

uint32_t HWDeviceDRM::Registry::GetFbId(Layer *layer, uint64_t handle_id) {
  auto it = layer->buffer_map->buffer_map.find(handle_id);
  if (it != layer->buffer_map->buffer_map.end()) {
    FrameBufferObject *fb_obj = static_cast<FrameBufferObject*>(it->second.get());
    return fb_obj->GetFbId();
  }

  return 0;
}

uint32_t HWDeviceDRM::Registry::GetOutputFbId(uint64_t handle_id) {
  auto it = output_buffer_map_.find(handle_id);
  if (it != output_buffer_map_.end()) {
    FrameBufferObject *fb_obj = static_cast<FrameBufferObject*>(it->second.get());
    return fb_obj->GetFbId();
  }

  return 0;
}

HWDeviceDRM::HWDeviceDRM(BufferAllocator *buffer_allocator, HWInfoInterface *hw_info_intf)
    : hw_info_intf_(hw_info_intf), registry_(buffer_allocator) {
  hw_info_intf_ = hw_info_intf;
}

DisplayError HWDeviceDRM::Init() {
  int ret = 0;
  DRMMaster *drm_master = {};
  DRMMaster::GetInstance(&drm_master);
  drm_master->GetHandle(&dev_fd_);
  DRMLibLoader::GetInstance()->FuncGetDRMManager()(dev_fd_, &drm_mgr_intf_);

  if (-1 == display_id_) {
    if (drm_mgr_intf_->RegisterDisplay(disp_type_, &token_)) {
      DLOGE("RegisterDisplay (by type) failed for %s", device_name_);
      return kErrorResources;
    }
  } else if (drm_mgr_intf_->RegisterDisplay(display_id_, &token_)) {
    DLOGE("RegisterDisplay (by id) failed for %s - %d", device_name_, display_id_);
    return kErrorResources;
  }

  if (token_.conn_id > INT32_MAX) {
    DLOGE("Connector id %u beyond supported range", token_.conn_id);
    drm_mgr_intf_->UnregisterDisplay(&token_);
    return kErrorNotSupported;
  }

  display_id_ = static_cast<int32_t>(token_.conn_id);

  ret = drm_mgr_intf_->CreateAtomicReq(token_, &drm_atomic_intf_);
  if (ret) {
    DLOGE("Failed creating atomic request for connector id %u. Error: %d.", token_.conn_id, ret);
    drm_mgr_intf_->UnregisterDisplay(&token_);
    return kErrorResources;
  }

  ret = drm_mgr_intf_->GetConnectorInfo(token_.conn_id, &connector_info_);
  if (ret) {
    DLOGE("Failed getting info for connector id %u. Error: %d.", token_.conn_id, ret);
    drm_mgr_intf_->DestroyAtomicReq(drm_atomic_intf_);
    drm_atomic_intf_ = {};
    drm_mgr_intf_->UnregisterDisplay(&token_);
    return kErrorHardware;
  }

  if (!connector_info_.is_connected || connector_info_.modes.empty()) {
    DLOGW("Device removal detected on connector id %u. Connector status %s and %zu modes.",
          token_.conn_id, connector_info_.is_connected ? "connected":"disconnected",
          connector_info_.modes.size());
    drm_mgr_intf_->DestroyAtomicReq(drm_atomic_intf_);
    drm_atomic_intf_ = {};
    drm_mgr_intf_->UnregisterDisplay(&token_);
    return kErrorDeviceRemoved;
  }

  hw_info_intf_->GetHWResourceInfo(&hw_resource_);

  InitializeConfigs();
  PopulateHWPanelInfo();
  UpdateMixerAttributes();

  // TODO(user): In future, remove has_qseed3 member, add version and pass version to constructor
  if (hw_resource_.has_qseed3) {
    hw_scale_ = new HWScaleDRM(HWScaleDRM::Version::V2);
  }

  std::unique_ptr<HWColorManagerDrm> hw_color_mgr(new HWColorManagerDrm());
  hw_color_mgr_ = std::move(hw_color_mgr);

  return kErrorNone;
}

DisplayError HWDeviceDRM::Deinit() {
  DisplayError err = kErrorNone;
  // Power-on will set the CRTC_SET_MODE to valid on external display. Without first commit,
  // if external is disconnected, CRTC_SET_MODE is not set to NULL, this leads to a synchronization
  // issue and external is blank for sometime. So on successful power-on (i.e NullCommit),
  // set CRTC_SET_MODE to NULL for proper sync.
  if (!first_cycle_ || null_display_commit_) {
    // A null-commit is needed only if the first commit had gone through. e.g., If a pluggable
    // display is plugged in and plugged out immediately, HWDeviceDRM::Deinit() may be called
    // before any commit happened on the device. The driver may have removed any not-in-use
    // connector (i.e., any connector which did not have a display commit on it and a crtc path
    // setup), so token_.conn_id may have been removed if there was no commit, resulting in
    // drmModeAtomicCommit() failure with ENOENT, 'No such file or directory'.
    ClearSolidfillStages();
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, token_.conn_id, 0);
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, DRMPowerMode::OFF);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, nullptr);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 0);
    int ret = NullCommit(true /* synchronous */, false /* retain_planes */);
    if (ret) {
      DLOGE("Commit failed with error: %d", ret);
      err = kErrorHardware;
    }
  }
  delete hw_scale_;
  registry_.Clear();
  display_attributes_ = {};
  drm_mgr_intf_->DestroyAtomicReq(drm_atomic_intf_);
  drm_atomic_intf_ = {};
  drm_mgr_intf_->UnregisterDisplay(&token_);
  hw_dest_scaler_blocks_used_ -= dest_scaler_blocks_used_;
  return err;
}

DisplayError HWDeviceDRM::GetDisplayId(int32_t *display_id) {
  *display_id = display_id_;
  return kErrorNone;
}

void HWDeviceDRM::InitializeConfigs() {
  current_mode_index_ = 0;
  // Update current mode with preferred mode
  for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if (connector_info_.modes[mode_index].mode.type & DRM_MODE_TYPE_PREFERRED) {
        DLOGI("Updating current display mode %d to preferred mode %d.", current_mode_index_,
              mode_index);
        current_mode_index_ = mode_index;
        break;
      }
  }

  display_attributes_.resize(connector_info_.modes.size());

  uint32_t width = connector_info_.modes[current_mode_index_].mode.hdisplay;
  uint32_t height = connector_info_.modes[current_mode_index_].mode.vdisplay;
  for (uint32_t i = 0; i < connector_info_.modes.size(); i++) {
    auto &mode = connector_info_.modes[i].mode;
    if (mode.hdisplay != width || mode.vdisplay != height) {
      resolution_switch_enabled_ = true;
    }
    PopulateDisplayAttributes(i);
  }
  SetDisplaySwitchMode(current_mode_index_);
}

DisplayError HWDeviceDRM::PopulateDisplayAttributes(uint32_t index) {
  drmModeModeInfo mode = {};
  uint32_t mm_width = 0;
  uint32_t mm_height = 0;
  DRMTopology topology = DRMTopology::SINGLE_LM;

  if (default_mode_) {
    DRMResMgr *res_mgr = nullptr;
    int ret = DRMResMgr::GetInstance(&res_mgr);
    if (ret < 0) {
      DLOGE("Failed to acquire DRMResMgr instance");
      return kErrorResources;
    }

    res_mgr->GetMode(&mode);
    res_mgr->GetDisplayDimInMM(&mm_width, &mm_height);
  } else {
    mode = connector_info_.modes[index].mode;
    mm_width = connector_info_.mmWidth;
    mm_height = connector_info_.mmHeight;
    topology = connector_info_.modes[index].topology;
    if (mode.flags & DRM_MODE_FLAG_CMD_MODE_PANEL) {
      display_attributes_[index].smart_panel = true;
    }
  }

  display_attributes_[index].x_pixels = mode.hdisplay;
  display_attributes_[index].y_pixels = mode.vdisplay;
  display_attributes_[index].fps = mode.vrefresh;
  display_attributes_[index].vsync_period_ns =
    UINT32(1000000000L / display_attributes_[index].fps);

  /*
              Active                 Front           Sync           Back
              Region                 Porch                          Porch
     <-----------------------><----------------><-------------><-------------->
     <----- [hv]display ----->
     <------------- [hv]sync_start ------------>
     <--------------------- [hv]sync_end --------------------->
     <-------------------------------- [hv]total ----------------------------->
   */

  display_attributes_[index].v_front_porch = mode.vsync_start - mode.vdisplay;
  display_attributes_[index].v_pulse_width = mode.vsync_end - mode.vsync_start;
  display_attributes_[index].v_back_porch = mode.vtotal - mode.vsync_end;
  display_attributes_[index].v_total = mode.vtotal;
  display_attributes_[index].h_total = mode.htotal;
  display_attributes_[index].is_device_split =
      (topology == DRMTopology::DUAL_LM || topology == DRMTopology::DUAL_LM_MERGE ||
       topology == DRMTopology::DUAL_LM_MERGE_DSC || topology == DRMTopology::DUAL_LM_DSC ||
       topology == DRMTopology::DUAL_LM_DSCMERGE || topology == DRMTopology::QUAD_LM_MERGE ||
       topology == DRMTopology::QUAD_LM_DSCMERGE || topology == DRMTopology::QUAD_LM_MERGE_DSC);
  display_attributes_[index].clock_khz = mode.clock;

  // If driver doesn't return panel width/height information, default to 320 dpi
  if (INT(mm_width) <= 0 || INT(mm_height) <= 0) {
    mm_width  = UINT32(((FLOAT(mode.hdisplay) * 25.4f) / 320.0f) + 0.5f);
    mm_height = UINT32(((FLOAT(mode.vdisplay) * 25.4f) / 320.0f) + 0.5f);
    DLOGW("Driver doesn't report panel physical width and height - defaulting to 320dpi");
  }

  display_attributes_[index].x_dpi = (FLOAT(mode.hdisplay) * 25.4f) / FLOAT(mm_width);
  display_attributes_[index].y_dpi = (FLOAT(mode.vdisplay) * 25.4f) / FLOAT(mm_height);
  SetTopology(topology, &display_attributes_[index].topology);

  DLOGI("Display attributes[%d]: WxH: %dx%d, DPI: %fx%f, FPS: %d, LM_SPLIT: %d, V_BACK_PORCH: %d," \
        " V_FRONT_PORCH: %d, V_PULSE_WIDTH: %d, V_TOTAL: %d, H_TOTAL: %d, CLK: %dKHZ," \
        " TOPOLOGY: %d, HW_SPLIT: %d", index, display_attributes_[index].x_pixels,
        display_attributes_[index].y_pixels, display_attributes_[index].x_dpi,
        display_attributes_[index].y_dpi, display_attributes_[index].fps,
        display_attributes_[index].is_device_split, display_attributes_[index].v_back_porch,
        display_attributes_[index].v_front_porch, display_attributes_[index].v_pulse_width,
        display_attributes_[index].v_total, display_attributes_[index].h_total,
        display_attributes_[index].clock_khz, display_attributes_[index].topology,
        mixer_attributes_.split_type);

  return kErrorNone;
}

void HWDeviceDRM::PopulateHWPanelInfo() {
  hw_panel_info_ = {};

  snprintf(hw_panel_info_.panel_name, sizeof(hw_panel_info_.panel_name), "%s",
           connector_info_.panel_name.c_str());

  uint32_t index = current_mode_index_;
  hw_panel_info_.split_info.left_split = display_attributes_[index].x_pixels;
  if (display_attributes_[index].is_device_split) {
    hw_panel_info_.split_info.left_split = hw_panel_info_.split_info.right_split =
        display_attributes_[index].x_pixels / 2;
  }

  hw_panel_info_.partial_update = connector_info_.modes[index].num_roi;
  hw_panel_info_.left_roi_count = UINT32(connector_info_.modes[index].num_roi);
  hw_panel_info_.right_roi_count = UINT32(connector_info_.modes[index].num_roi);
  hw_panel_info_.left_align = connector_info_.modes[index].xstart;
  hw_panel_info_.top_align = connector_info_.modes[index].ystart;
  hw_panel_info_.width_align = connector_info_.modes[index].walign;
  hw_panel_info_.height_align = connector_info_.modes[index].halign;
  hw_panel_info_.min_roi_width = connector_info_.modes[index].wmin;
  hw_panel_info_.min_roi_height = connector_info_.modes[index].hmin;
  hw_panel_info_.needs_roi_merge = connector_info_.modes[index].roi_merge;
  hw_panel_info_.transfer_time_us = connector_info_.modes[index].transfer_time_us;
  hw_panel_info_.dynamic_fps = connector_info_.dynamic_fps;
  hw_panel_info_.qsync_support = connector_info_.qsync_support;
  drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
  if (hw_panel_info_.dynamic_fps) {
    uint32_t min_fps = current_mode.vrefresh;
    uint32_t max_fps = current_mode.vrefresh;
    for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if ((current_mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
          (current_mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay)) {
        if (min_fps > connector_info_.modes[mode_index].mode.vrefresh)  {
          min_fps = connector_info_.modes[mode_index].mode.vrefresh;
        }
        if (max_fps < connector_info_.modes[mode_index].mode.vrefresh)  {
          max_fps = connector_info_.modes[mode_index].mode.vrefresh;
        }
      }
    }
    hw_panel_info_.min_fps = min_fps;
    hw_panel_info_.max_fps = max_fps;
  } else {
    hw_panel_info_.min_fps = current_mode.vrefresh;
    hw_panel_info_.max_fps = current_mode.vrefresh;
  }

  hw_panel_info_.is_primary_panel = connector_info_.is_primary;
  hw_panel_info_.is_pluggable = 0;
  hw_panel_info_.hdr_enabled = connector_info_.panel_hdr_prop.hdr_enabled;
  // Convert the luminance values to cd/m^2 units.
  hw_panel_info_.peak_luminance = FLOAT(connector_info_.panel_hdr_prop.peak_brightness) / 10000.0f;
  hw_panel_info_.blackness_level = FLOAT(connector_info_.panel_hdr_prop.blackness_level) / 10000.0f;
  hw_panel_info_.average_luminance = FLOAT(connector_info_.panel_hdr_prop.peak_brightness +
                                           connector_info_.panel_hdr_prop.blackness_level) /
                                           (2 * 10000.0f);
  hw_panel_info_.primaries.white_point[0] = connector_info_.panel_hdr_prop.display_primaries[0];
  hw_panel_info_.primaries.white_point[1] = connector_info_.panel_hdr_prop.display_primaries[1];
  hw_panel_info_.primaries.red[0] = connector_info_.panel_hdr_prop.display_primaries[2];
  hw_panel_info_.primaries.red[1] = connector_info_.panel_hdr_prop.display_primaries[3];
  hw_panel_info_.primaries.green[0] = connector_info_.panel_hdr_prop.display_primaries[4];
  hw_panel_info_.primaries.green[1] = connector_info_.panel_hdr_prop.display_primaries[5];
  hw_panel_info_.primaries.blue[0] = connector_info_.panel_hdr_prop.display_primaries[6];
  hw_panel_info_.primaries.blue[1] = connector_info_.panel_hdr_prop.display_primaries[7];
  hw_panel_info_.dyn_bitclk_support = connector_info_.dyn_bitclk_support;

  // no supprt for 90 rotation only flips or 180 supported
  hw_panel_info_.panel_orientation.rotation = 0;
  hw_panel_info_.panel_orientation.flip_horizontal =
    (connector_info_.panel_orientation == DRMRotation::FLIP_H) ||
    (connector_info_.panel_orientation == DRMRotation::ROT_180);
  hw_panel_info_.panel_orientation.flip_vertical =
    (connector_info_.panel_orientation == DRMRotation::FLIP_V) ||
    (connector_info_.panel_orientation == DRMRotation::ROT_180);

  GetHWDisplayPortAndMode();
  GetHWPanelMaxBrightness();

  if (current_mode.flags & DRM_MODE_FLAG_CMD_MODE_PANEL) {
    hw_panel_info_.mode = kModeCommand;
  }
  if (current_mode.flags & DRM_MODE_FLAG_VID_MODE_PANEL) {
    hw_panel_info_.mode = kModeVideo;
  }

  DLOGI_IF(kTagDriverConfig, "%s, Panel Interface = %s, Panel Mode = %s, Is Primary = %d",
           device_name_, interface_str_.c_str(),
           hw_panel_info_.mode == kModeVideo ? "Video" : "Command",
           hw_panel_info_.is_primary_panel);
  DLOGI_IF(kTagDriverConfig, "Partial Update = %d, Dynamic FPS = %d, HDR Panel = %d QSync = %d",
           hw_panel_info_.partial_update, hw_panel_info_.dynamic_fps, hw_panel_info_.hdr_enabled,
           hw_panel_info_.qsync_support);
  DLOGI_IF(kTagDriverConfig, "Align: left = %d, width = %d, top = %d, height = %d",
           hw_panel_info_.left_align, hw_panel_info_.width_align, hw_panel_info_.top_align,
           hw_panel_info_.height_align);
  DLOGI_IF(kTagDriverConfig, "ROI: min_width = %d, min_height = %d, need_merge = %d",
           hw_panel_info_.min_roi_width, hw_panel_info_.min_roi_height,
           hw_panel_info_.needs_roi_merge);
  DLOGI_IF(kTagDriverConfig, "FPS: min = %d, max = %d", hw_panel_info_.min_fps,
           hw_panel_info_.max_fps);
  DLOGI_IF(kTagDriverConfig, "Left Split = %d, Right Split = %d",
           hw_panel_info_.split_info.left_split, hw_panel_info_.split_info.right_split);
  DLOGI_IF(kTagDriverConfig, "Panel Transfer time = %d us", hw_panel_info_.transfer_time_us);
  DLOGI_IF(kTagDriverConfig, "Dynamic Bit Clk Support = %d", hw_panel_info_.dyn_bitclk_support);
}

DisplayError HWDeviceDRM::GetDisplayIdentificationData(uint8_t *out_port, uint32_t *out_data_size,
                                                       uint8_t *out_data) {
  *out_port = token_.hw_port;
  std::vector<uint8_t> &edid = connector_info_.edid;

  if (out_data == nullptr) {
    *out_data_size = (uint32_t)(edid.size());
    if (*out_data_size == 0) {
      DLOGE("EDID blob is empty, no data to return");
      return kErrorDriverData;
    }
  } else {
    *out_data_size = std::min(*out_data_size, (uint32_t)(edid.size()));
    memcpy(out_data, edid.data(), *out_data_size);
  }

  return kErrorNone;
}

void HWDeviceDRM::GetHWDisplayPortAndMode() {
  hw_panel_info_.port = kPortDefault;
  hw_panel_info_.mode =
      (connector_info_.panel_mode == sde_drm::DRMPanelMode::VIDEO) ? kModeVideo : kModeCommand;

  if (default_mode_) {
    return;
  }

  switch (connector_info_.type) {
    case DRM_MODE_CONNECTOR_DSI:
      hw_panel_info_.port = kPortDSI;
      interface_str_ = "DSI";
      break;
    case DRM_MODE_CONNECTOR_LVDS:
      hw_panel_info_.port = kPortLVDS;
      interface_str_ = "LVDS";
      break;
    case DRM_MODE_CONNECTOR_eDP:
      hw_panel_info_.port = kPortEDP;
      interface_str_ = "EDP";
      break;
    case DRM_MODE_CONNECTOR_TV:
    case DRM_MODE_CONNECTOR_HDMIA:
    case DRM_MODE_CONNECTOR_HDMIB:
      hw_panel_info_.port = kPortDTV;
      interface_str_ = "HDMI";
      break;
    case DRM_MODE_CONNECTOR_VIRTUAL:
      hw_panel_info_.port = kPortWriteBack;
      interface_str_ = "Virtual";
      break;
    case DRM_MODE_CONNECTOR_DisplayPort:
      hw_panel_info_.port = kPortDP;
      interface_str_ = "DisplayPort";
      break;
  }

  return;
}

DisplayError HWDeviceDRM::GetActiveConfig(uint32_t *active_config) {
  *active_config = current_mode_index_;
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetNumDisplayAttributes(uint32_t *count) {
  *count = UINT32(display_attributes_.size());
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetDisplayAttributes(uint32_t index,
                                               HWDisplayAttributes *display_attributes) {
  if (index >= display_attributes_.size()) {
    return kErrorParameters;
  }
  *display_attributes = display_attributes_[index];
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetHWPanelInfo(HWPanelInfo *panel_info) {
  *panel_info = hw_panel_info_;
  return kErrorNone;
}

void HWDeviceDRM::SetDisplaySwitchMode(uint32_t index) {
  uint32_t mode_flag = 0;
  uint32_t curr_mode_flag = 0, switch_mode_flag = 0;
  drmModeModeInfo to_set = connector_info_.modes[index].mode;
  drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
  uint64_t current_bit_clk = connector_info_.modes[current_mode_index_].bit_clk_rate;
  uint32_t switch_index  = 0;

  if (to_set.flags & DRM_MODE_FLAG_CMD_MODE_PANEL) {
    mode_flag = DRM_MODE_FLAG_CMD_MODE_PANEL;
    switch_mode_flag = DRM_MODE_FLAG_VID_MODE_PANEL;
  } else if (to_set.flags & DRM_MODE_FLAG_VID_MODE_PANEL) {
    mode_flag = DRM_MODE_FLAG_VID_MODE_PANEL;
    switch_mode_flag = DRM_MODE_FLAG_CMD_MODE_PANEL;
  }

  if (current_mode.flags & DRM_MODE_FLAG_CMD_MODE_PANEL) {
    curr_mode_flag = DRM_MODE_FLAG_CMD_MODE_PANEL;
  } else if (current_mode.flags & DRM_MODE_FLAG_VID_MODE_PANEL) {
    curr_mode_flag = DRM_MODE_FLAG_VID_MODE_PANEL;
  }

  if (curr_mode_flag != mode_flag) {
    panel_mode_changed_ = mode_flag;
  }

  for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
    if ((to_set.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
        (to_set.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
        (to_set.vrefresh == connector_info_.modes[mode_index].mode.vrefresh) &&
        (current_bit_clk == connector_info_.modes[mode_index].bit_clk_rate) &&
        (mode_flag & connector_info_.modes[mode_index].mode.flags)) {
      index = mode_index;
      break;
    }
  }

  current_mode_index_ = index;

  switch_mode_valid_ = false;
  for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
    if ((to_set.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
        (to_set.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
        (to_set.vrefresh == connector_info_.modes[mode_index].mode.vrefresh) &&
        (switch_mode_flag & connector_info_.modes[mode_index].mode.flags)) {
      switch_index = mode_index;
      switch_mode_valid_ = true;
      break;
    }
  }

  if (!switch_mode_valid_) {
    // in case there is no corresponding switch mode with same fps, try for a switch
    // mode with lowest fps. This is to handle cases where there are multiple video mode fps
    // but only one command mode for doze like 30 fps.
    uint32_t refresh_rate = 0;
    for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if ((to_set.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
          (to_set.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
          (switch_mode_flag & connector_info_.modes[mode_index].mode.flags)) {
        if (!refresh_rate || (refresh_rate > connector_info_.modes[mode_index].mode.vrefresh)) {
          switch_index = mode_index;
          switch_mode_valid_ = true;
          refresh_rate = connector_info_.modes[mode_index].mode.vrefresh;
        }
      }
    }
  }

  if (switch_mode_valid_) {
    if (mode_flag & DRM_MODE_FLAG_VID_MODE_PANEL) {
      video_mode_index_ = current_mode_index_;
      cmd_mode_index_ = switch_index;
    } else {
      video_mode_index_ = switch_index;
      cmd_mode_index_ = current_mode_index_;
    }
  }
}

DisplayError HWDeviceDRM::SetDisplayAttributes(uint32_t index) {
  if (index >= display_attributes_.size()) {
    DLOGE("Invalid mode index %d mode size %d", index, UINT32(display_attributes_.size()));
    return kErrorParameters;
  }

  SetDisplaySwitchMode(index);
  PopulateHWPanelInfo();
  UpdateMixerAttributes();

  DLOGI_IF(kTagDriverConfig,
        "Display attributes[%d]: WxH: %dx%d, DPI: %fx%f, FPS: %d, LM_SPLIT: %d, V_BACK_PORCH: %d," \
        " V_FRONT_PORCH: %d, V_PULSE_WIDTH: %d, V_TOTAL: %d, H_TOTAL: %d, CLK: %dKHZ, " \
        "TOPOLOGY: %d, PanelMode %s", index, display_attributes_[index].x_pixels,
        display_attributes_[index].y_pixels, display_attributes_[index].x_dpi,
        display_attributes_[index].y_dpi, display_attributes_[index].fps,
        display_attributes_[index].is_device_split, display_attributes_[index].v_back_porch,
        display_attributes_[index].v_front_porch, display_attributes_[index].v_pulse_width,
        display_attributes_[index].v_total, display_attributes_[index].h_total,
        display_attributes_[index].clock_khz, display_attributes_[index].topology,
        (connector_info_.modes[index].mode.flags & DRM_MODE_FLAG_VID_MODE_PANEL) ?
        "Video" : "Command");

  return kErrorNone;
}

DisplayError HWDeviceDRM::SetDisplayAttributes(const HWDisplayAttributes &display_attributes) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::GetConfigIndex(char *mode, uint32_t *index) {
  return kErrorNone;
}

DisplayError HWDeviceDRM::PowerOn(const HWQosData &qos_data, shared_ptr<Fence> *release_fence) {
  SetQOSData(qos_data);

  int64_t release_fence_fd = -1;
  int64_t retire_fence_fd = -1;

  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, DRMPowerMode::ON);
  if (release_fence) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_GET_RELEASE_FENCE, token_.crtc_id, &release_fence_fd);
  }
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_GET_RETIRE_FENCE, token_.conn_id, &retire_fence_fd);

  int ret = NullCommit(false /* asynchronous */, true /* retain_planes */);
  if (ret) {
    DLOGE("Failed with error: %d", ret);
    return kErrorHardware;
  }

  shared_ptr<Fence> retire_fence = Fence::Create(INT(retire_fence_fd), "retire_power_on");

  if (release_fence) {
    *release_fence = Fence::Create(INT(release_fence_fd), "release_power_on");
    DLOGD_IF(kTagDriverConfig, "RELEASE fence: fd: %s", Fence::GetStr(*release_fence).c_str());
  }
  pending_doze_ = false;

  Fence::Wait(retire_fence, kTimeoutMsPowerOn);

  last_power_mode_ = DRMPowerMode::ON;

  return kErrorNone;
}

DisplayError HWDeviceDRM::PowerOff(bool teardown) {
  DTRACE_SCOPED();
  if (!drm_atomic_intf_) {
    DLOGE("DRM Atomic Interface is null!");
    return kErrorUndefined;
  }

  if (first_cycle_) {
    return kErrorNone;
  }

  ResetROI();
  int64_t retire_fence_fd = -1;
  drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, &current_mode);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, DRMPowerMode::OFF);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 0);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_GET_RETIRE_FENCE, token_.conn_id, &retire_fence_fd);

  int ret = NullCommit(false /* asynchronous */, false /* retain_planes */);
  if (ret) {
    DLOGE("Failed with error: %d", ret);
    return kErrorHardware;
  }

  shared_ptr<Fence> retire_fence = Fence::Create(INT(retire_fence_fd), "retire_power_off");
  pending_doze_ = false;

  Fence::Wait(retire_fence, kTimeoutMsPowerOff);

  last_power_mode_ = DRMPowerMode::OFF;

  return kErrorNone;
}

DisplayError HWDeviceDRM::Doze(const HWQosData &qos_data, shared_ptr<Fence> *release_fence) {
  DTRACE_SCOPED();

  if (first_cycle_ || ((!switch_mode_valid_) && (last_power_mode_ != DRMPowerMode::OFF))) {
    pending_doze_ = true;
    return kErrorDeferred;
  }

  SetQOSData(qos_data);

  int64_t release_fence_fd = -1;
  int64_t retire_fence_fd = -1;

  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, token_.conn_id, token_.crtc_id);
  drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, &current_mode);

  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, DRMPowerMode::DOZE);
  if (release_fence) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_GET_RELEASE_FENCE, token_.crtc_id, &release_fence_fd);
  }
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_GET_RETIRE_FENCE, token_.conn_id, &retire_fence_fd);

  int ret = NullCommit(false /* asynchronous */, true /* retain_planes */);
  if (ret) {
    DLOGE("Failed with error: %d", ret);
    return kErrorHardware;
  }

  shared_ptr<Fence> retire_fence = Fence::Create(INT(retire_fence_fd), "retire_doze");

  if (release_fence) {
    *release_fence = Fence::Create(release_fence_fd, "release_doze");
    DLOGD_IF(kTagDriverConfig, "RELEASE fence: fd: %s", Fence::GetStr(*release_fence).c_str());
  }

  Fence::Wait(retire_fence, kTimeoutMsDoze);

  last_power_mode_ = DRMPowerMode::DOZE;

  return kErrorNone;
}

DisplayError HWDeviceDRM::DozeSuspend(const HWQosData &qos_data,
                                      shared_ptr<Fence> *release_fence) {
  DTRACE_SCOPED();

  SetQOSData(qos_data);

  int64_t release_fence_fd = -1;
  int64_t retire_fence_fd = -1;

  if (first_cycle_) {
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, token_.conn_id, token_.crtc_id);
    drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, &current_mode);
  }
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id,
                            DRMPowerMode::DOZE_SUSPEND);
  if (release_fence) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_GET_RELEASE_FENCE, token_.crtc_id, &release_fence_fd);
  }
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_GET_RETIRE_FENCE, token_.conn_id, &retire_fence_fd);

  int ret = NullCommit(false /* asynchronous */, true /* retain_planes */);
  if (ret) {
    DLOGE("Failed with error: %d", ret);
    return kErrorHardware;
  }

  shared_ptr<Fence> retire_fence = Fence::Create(INT(retire_fence_fd), "retire_doze_suspend");

  if (release_fence) {
    *release_fence = Fence::Create(release_fence_fd, "release_doze_suspend");
    DLOGD_IF(kTagDriverConfig, "RELEASE fence: fd: %s", Fence::GetStr(*release_fence).c_str());
  }
  pending_doze_ = false;

  Fence::Wait(retire_fence, kTimeoutMsDozeSuspend);

  last_power_mode_ = DRMPowerMode::DOZE_SUSPEND;

  return kErrorNone;
}

void HWDeviceDRM::SetQOSData(const HWQosData &qos_data) {
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_CORE_CLK, token_.crtc_id, qos_data.clock_hz);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_CORE_AB, token_.crtc_id, qos_data.core_ab_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_CORE_IB, token_.crtc_id, qos_data.core_ib_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_LLCC_AB, token_.crtc_id, qos_data.llcc_ab_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_LLCC_IB, token_.crtc_id, qos_data.llcc_ib_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_DRAM_AB, token_.crtc_id, qos_data.dram_ab_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_DRAM_IB, token_.crtc_id, qos_data.dram_ib_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ROT_PREFILL_BW, token_.crtc_id,
                            qos_data.rot_prefill_bw_bps);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ROT_CLK, token_.crtc_id, qos_data.rot_clock_hz);
}

DisplayError HWDeviceDRM::Standby() {
  return kErrorNone;
}

void HWDeviceDRM::SetupAtomic(Fence::ScopedRef &scoped_ref, HWLayers *hw_layers, bool validate,
                              int64_t *release_fence_fd, int64_t *retire_fence_fd) {
  if (default_mode_) {
    return;
  }

  DTRACE_SCOPED();
  HWLayersInfo &hw_layer_info = hw_layers->info;
  uint32_t hw_layer_count = UINT32(hw_layer_info.hw_layers.size());
  HWQosData &qos_data = hw_layers->qos_data;
  DRMSecurityLevel crtc_security_level = DRMSecurityLevel::SECURE_NON_SECURE;
  uint32_t index = current_mode_index_;
  drmModeModeInfo current_mode = connector_info_.modes[index].mode;
  uint64_t current_bit_clk = connector_info_.modes[index].bit_clk_rate;

  solid_fills_.clear();
  bool resource_update = hw_layers->updates_mask.test(kUpdateResources);
  bool buffer_update = hw_layers->updates_mask.test(kSwapBuffers);
  bool update_config = resource_update || buffer_update ||
                       hw_layer_info.stack->flags.geometry_changed;

  if (hw_panel_info_.partial_update && update_config) {
    if (IsFullFrameUpdate(hw_layer_info)) {
      ResetROI();
    } else {
      const int kNumMaxROIs = 4;
      DRMRect crtc_rects[kNumMaxROIs] = {{0, 0, mixer_attributes_.width, mixer_attributes_.height}};
      DRMRect conn_rects[kNumMaxROIs] = {{0, 0, display_attributes_[index].x_pixels,
                                          display_attributes_[index].y_pixels}};

      for (uint32_t i = 0; i < hw_layer_info.left_frame_roi.size(); i++) {
        auto &roi = hw_layer_info.left_frame_roi.at(i);
        // TODO(user): In multi PU, stitch ROIs vertically adjacent and upate plane destination
        crtc_rects[i].left = UINT32(roi.left);
        crtc_rects[i].right = UINT32(roi.right);
        crtc_rects[i].top = UINT32(roi.top);
        crtc_rects[i].bottom = UINT32(roi.bottom);
        conn_rects[i].left = UINT32(roi.left);
        conn_rects[i].right = UINT32(roi.right);
        conn_rects[i].top = UINT32(roi.top);
        conn_rects[i].bottom = UINT32(roi.bottom);
      }

      uint32_t num_rects = std::max(1u, static_cast<uint32_t>(hw_layer_info.left_frame_roi.size()));
      drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ROI, token_.crtc_id, num_rects, crtc_rects);
      drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_ROI, token_.conn_id, num_rects, conn_rects);
    }
  }

  for (uint32_t i = 0; i < hw_layer_count; i++) {
    Layer &layer = hw_layer_info.hw_layers.at(i);
    LayerBuffer *input_buffer = &layer.input_buffer;
    HWPipeInfo *left_pipe = &hw_layers->config[i].left_pipe;
    HWPipeInfo *right_pipe = &hw_layers->config[i].right_pipe;
    HWLayerConfig &layer_config = hw_layers->config[i];
    HWRotatorSession *hw_rotator_session = &layer_config.hw_rotator_session;

    if (hw_layers->config[i].use_solidfill_stage) {
      hw_layers->config[i].hw_solidfill_stage.solid_fill_info = layer.solid_fill_info;
      AddSolidfillStage(hw_layers->config[i].hw_solidfill_stage, layer.plane_alpha);
      continue;
    }

    for (uint32_t count = 0; count < 2; count++) {
      HWPipeInfo *pipe_info = (count == 0) ? left_pipe : right_pipe;
      HWRotateInfo *hw_rotate_info = &hw_rotator_session->hw_rotate_info[count];

      if (hw_rotator_session->mode == kRotatorOffline && hw_rotate_info->valid) {
        input_buffer = &hw_rotator_session->output_buffer;
      }

      uint32_t fb_id = registry_.GetFbId(&layer, input_buffer->handle_id);

      if (pipe_info->valid && fb_id) {
        uint32_t pipe_id = pipe_info->pipe_id;

        if (update_config) {
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_ALPHA, pipe_id, layer.plane_alpha);

          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_ZORDER, pipe_id, pipe_info->z_order);

          DRMBlendType blending = {};
          SetBlending(layer.blending, &blending);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_BLEND_TYPE, pipe_id, blending);

          DRMRect src = {};
          SetRect(pipe_info->src_roi, &src);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_SRC_RECT, pipe_id, src);

          DRMRect dst = {};
          SetRect(pipe_info->dst_roi, &dst);
          LayerRect right_mixer = {FLOAT(mixer_attributes_.split_left), 0,
                                   FLOAT(mixer_attributes_.width), FLOAT(mixer_attributes_.height)};
          LayerRect dst_roi = pipe_info->dst_roi;

          // For larget displays ie; 2 * 2k * 2k * 90 fps 4 LM's get programmed.
          // Each pair of LM's drive independent displays.
          // Layout Index indicates the panel onto which pipe gets staged.
          DRMSSPPLayoutIndex layout_index = DRMSSPPLayoutIndex::NONE;
          if (mixer_attributes_.split_type == kQuadSplit) {
            layout_index = DRMSSPPLayoutIndex::LEFT;
            if (IsValid(Intersection(dst_roi, right_mixer))) {
              dst_roi = Reposition(dst_roi, -INT(mixer_attributes_.split_left), 0);
              layout_index = DRMSSPPLayoutIndex::RIGHT;
              DLOGV_IF(kTagDriverConfig, "Layer index = %d sspp layout = RIGHT", i);
              DLOGV_IF(kTagDriverConfig, "Right dst_roi l = %f t = %f r = %f b = %f",
                       dst_roi.left, dst_roi.top, dst_roi.right, dst_roi.bottom);
            } else {
              DLOGV_IF(kTagDriverConfig, "Layer index = %d sspp layout = LEFT", i);
              DLOGV_IF(kTagDriverConfig, "Left dst_roi l = %f t = %f r = %f b = %f",
                       dst_roi.left, dst_roi.top, dst_roi.right, dst_roi.bottom);
            }
          }
          SetRect(dst_roi, &dst);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_DST_RECT, pipe_id, dst);

          // Update Layout index.
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_SSPP_LAYOUT, pipe_id, layout_index);

          DRMRect excl = {};
          SetRect(pipe_info->excl_rect, &excl);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_EXCL_RECT, pipe_id, excl);

          uint32_t rot_bit_mask = 0;
          SetRotation(layer.transform, layer_config, &rot_bit_mask);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_ROTATION, pipe_id, rot_bit_mask);

          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_H_DECIMATION, pipe_id,
                                    pipe_info->horizontal_decimation);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_V_DECIMATION, pipe_id,
                                    pipe_info->vertical_decimation);

          DRMSecureMode fb_secure_mode;
          DRMSecurityLevel security_level;
          SetSecureConfig(layer.input_buffer, &fb_secure_mode, &security_level);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_FB_SECURE_MODE, pipe_id, fb_secure_mode);
          if (security_level > crtc_security_level) {
            crtc_security_level = security_level;
          }

          uint32_t config = 0;
          SetSrcConfig(layer.input_buffer, hw_rotator_session->mode, &config);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_SRC_CONFIG, pipe_id, config);;

          if (hw_scale_) {
            SDEScaler scaler_output = {};
            hw_scale_->SetScaler(pipe_info->scale_data, &scaler_output);
            // TODO(user): Remove qseed3 and add version check, then send appropriate scaler object
            if (hw_resource_.has_qseed3) {
              drm_atomic_intf_->Perform(DRMOps::PLANE_SET_SCALER_CONFIG, pipe_id,
                                        reinterpret_cast<uint64_t>(&scaler_output.scaler_v2));
            }
          }

          DRMCscType csc_type = DRMCscType::kCscTypeMax;
          SelectCscType(layer.input_buffer, &csc_type);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_CSC_CONFIG, pipe_id, &csc_type);

          DRMMultiRectMode multirect_mode;
          SetMultiRectMode(pipe_info->flags, &multirect_mode);
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_MULTIRECT_MODE, pipe_id, multirect_mode);

          SetSsppTonemapFeatures(pipe_info);
        }

        drm_atomic_intf_->Perform(DRMOps::PLANE_SET_FB_ID, pipe_id, fb_id);
        drm_atomic_intf_->Perform(DRMOps::PLANE_SET_CRTC, pipe_id, token_.crtc_id);

        if (!validate && input_buffer->acquire_fence) {
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_INPUT_FENCE, pipe_id,
                                    scoped_ref.Get(input_buffer->acquire_fence));
        }
      }
    }
  }

  if (update_config) {
    SetSolidfillStages();
    SetQOSData(qos_data);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_SECURITY_LEVEL, token_.crtc_id, crtc_security_level);
  }

  if (hw_layers->hw_avr_info.update) {
    sde_drm::DRMQsyncMode mode = sde_drm::DRMQsyncMode::NONE;
    if (hw_layers->hw_avr_info.mode == kContinuousMode) {
      mode = sde_drm::DRMQsyncMode::CONTINUOUS;
    } else if (hw_layers->hw_avr_info.mode == kOneShotMode) {
      mode = sde_drm::DRMQsyncMode::ONESHOT;
    }
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_QSYNC_MODE, token_.conn_id, mode);
  }

  drm_atomic_intf_->Perform(DRMOps::DPPS_COMMIT_FEATURE, 0 /* argument is not used */);
  drm_atomic_intf_->Perform(DRMOps::COMMIT_PANEL_FEATURES, 0 /* argument is not used */);

  if (reset_output_fence_offset_ && !validate) {
    // Change back the fence_offset
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_OUTPUT_FENCE_OFFSET, token_.crtc_id, 0);
    reset_output_fence_offset_ = false;
  }

  // Set panel mode
  if (panel_mode_changed_ & DRM_MODE_FLAG_VID_MODE_PANEL)  {
    if (!validate) {
      // Switch to video mode, corresponding change the fence_offset
      drm_atomic_intf_->Perform(DRMOps::CRTC_SET_OUTPUT_FENCE_OFFSET, token_.crtc_id, 1);
    }
    ResetROI();
  }

  if (!validate && release_fence_fd && retire_fence_fd) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_GET_RELEASE_FENCE, token_.crtc_id, release_fence_fd);
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_GET_RETIRE_FENCE, token_.conn_id, retire_fence_fd);
  }

  DLOGI_IF(kTagDriverConfig, "%s::%s System Clock=%d Hz, Core: AB=%f KBps, IB=%f Bps, " \
           "LLCC: AB=%f Bps, IB=%f Bps, DRAM AB=%f Bps, IB=%f Bps, "\
           "Rot: Bw=%f Bps, Clock=%d Hz", validate ? "Validate" : "Commit", device_name_,
           qos_data.clock_hz, qos_data.core_ab_bps / 1000.f, qos_data.core_ib_bps / 1000.f,
           qos_data.llcc_ab_bps / 1000.f, qos_data.llcc_ib_bps / 1000.f,
           qos_data.dram_ab_bps / 1000.f, qos_data.dram_ib_bps / 1000.f,
           qos_data.rot_prefill_bw_bps / 1000.f, qos_data.rot_clock_hz);

  // Set refresh rate
  if (vrefresh_) {
    for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if ((current_mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
          (current_mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
          (current_bit_clk == connector_info_.modes[mode_index].bit_clk_rate) &&
          (current_mode.flags == connector_info_.modes[mode_index].mode.flags) &&
          (vrefresh_ == connector_info_.modes[mode_index].mode.vrefresh)) {
        current_mode = connector_info_.modes[mode_index].mode;
        break;
      }
    }
  }

  if (bit_clk_rate_) {
    for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if ((current_mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
          (current_mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
          (current_mode.vrefresh == connector_info_.modes[mode_index].mode.vrefresh) &&
          (current_mode.flags == connector_info_.modes[mode_index].mode.flags) &&
          (bit_clk_rate_ == connector_info_.modes[mode_index].bit_clk_rate)) {
        current_mode = connector_info_.modes[mode_index].mode;
        break;
      }
    }
  }

  if (first_cycle_) {
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_TOPOLOGY_CONTROL, token_.conn_id,
                              topology_control_);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_CRTC, token_.conn_id, token_.crtc_id);
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, DRMPowerMode::ON);
    last_power_mode_ = DRMPowerMode::ON;
  } else if (pending_doze_ && !validate) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POWER_MODE, token_.conn_id, DRMPowerMode::DOZE);
    last_power_mode_ = DRMPowerMode::DOZE;
  }

  // Set CRTC mode, only if display config changes
  if (first_cycle_ || vrefresh_ || update_mode_ || panel_mode_changed_) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, &current_mode);
  }

  if (!validate && (hw_layer_info.set_idle_time_ms >= 0)) {
    DLOGI_IF(kTagDriverConfig, "Setting idle timeout to = %d ms",
             hw_layer_info.set_idle_time_ms);
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_IDLE_TIMEOUT, token_.crtc_id,
                              hw_layer_info.set_idle_time_ms);
  }

  if (hw_panel_info_.mode == kModeCommand) {
    drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_AUTOREFRESH, token_.conn_id, autorefresh_);
  }
}

void HWDeviceDRM::AddSolidfillStage(const HWSolidfillStage &sf, uint32_t plane_alpha) {
  sde_drm::DRMSolidfillStage solidfill;
  solidfill.bounding_rect.left = UINT32(sf.roi.left);
  solidfill.bounding_rect.top = UINT32(sf.roi.top);
  solidfill.bounding_rect.right = UINT32(sf.roi.right);
  solidfill.bounding_rect.bottom = UINT32(sf.roi.bottom);
  solidfill.is_exclusion_rect  = sf.is_exclusion_rect;
  solidfill.plane_alpha = plane_alpha;
  solidfill.z_order = sf.z_order;
  if (!sf.solid_fill_info.bit_depth) {
    solidfill.color_bit_depth = 8;
    solidfill.alpha = (0xff000000 & sf.color) >> 24;
    solidfill.red = (0xff0000 & sf.color) >> 16;
    solidfill.green = (0xff00 & sf.color) >> 8;
    solidfill.blue = 0xff & sf.color;
  } else {
    solidfill.color_bit_depth = sf.solid_fill_info.bit_depth;
    solidfill.alpha = sf.solid_fill_info.alpha;
    solidfill.red = sf.solid_fill_info.red;
    solidfill.green = sf.solid_fill_info.green;
    solidfill.blue = sf.solid_fill_info.blue;
  }
  solid_fills_.push_back(solidfill);
  DLOGI_IF(kTagDriverConfig, "Add a solidfill stage at z_order:%d argb_color:%x plane_alpha:%x",
           solidfill.z_order, solidfill.color, solidfill.plane_alpha);
}

void HWDeviceDRM::SetSolidfillStages() {
  if (hw_resource_.num_solidfill_stages) {
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_SOLIDFILL_STAGES, token_.crtc_id,
                              reinterpret_cast<uint64_t> (&solid_fills_));
  }
}

void HWDeviceDRM::ClearSolidfillStages() {
  solid_fills_.clear();
  SetSolidfillStages();
}

DisplayError HWDeviceDRM::Validate(HWLayers *hw_layers) {
  DTRACE_SCOPED();

  DisplayError err = kErrorNone;
  registry_.Register(hw_layers);

  Fence::ScopedRef scoped_ref;
  SetupAtomic(scoped_ref, hw_layers, true /* validate */, nullptr, nullptr);

  int ret = drm_atomic_intf_->Validate();
  if (ret) {
    DLOGE("failed with error %d for %s", ret, device_name_);
    DumpHWLayers(hw_layers);
    vrefresh_ = 0;
    panel_mode_changed_ = 0;
    err = kErrorHardware;
  }

  return err;
}

DisplayError HWDeviceDRM::Commit(HWLayers *hw_layers) {
  DTRACE_SCOPED();

  DisplayError err = kErrorNone;
  registry_.Register(hw_layers);

  if (default_mode_) {
    err = DefaultCommit(hw_layers);
  } else {
    err = AtomicCommit(hw_layers);
  }

  return err;
}

DisplayError HWDeviceDRM::DefaultCommit(HWLayers *hw_layers) {
  DTRACE_SCOPED();

  HWLayersInfo &hw_layer_info = hw_layers->info;

  for (Layer &layer : hw_layer_info.hw_layers) {
    layer.input_buffer.release_fence = nullptr;
  }

  DRMMaster *master = nullptr;
  int ret = DRMMaster::GetInstance(&master);
  if (ret < 0) {
    DLOGE("Failed to acquire DRMMaster instance");
    return kErrorResources;
  }

  DRMResMgr *res_mgr = nullptr;
  ret = DRMResMgr::GetInstance(&res_mgr);
  if (ret < 0) {
    DLOGE("Failed to acquire DRMResMgr instance");
    return kErrorResources;
  }

  int dev_fd = -1;
  master->GetHandle(&dev_fd);

  uint32_t connector_id = 0;
  res_mgr->GetConnectorId(&connector_id);

  uint32_t crtc_id = 0;
  res_mgr->GetCrtcId(&crtc_id);

  drmModeModeInfo mode;
  res_mgr->GetMode(&mode);

  uint64_t handle_id = hw_layer_info.hw_layers.at(0).input_buffer.handle_id;
  uint32_t fb_id = registry_.GetFbId(&hw_layer_info.hw_layers.at(0), handle_id);
  ret = drmModeSetCrtc(dev_fd, crtc_id, fb_id, 0 /* x */, 0 /* y */, &connector_id,
                       1 /* num_connectors */, &mode);
  if (ret < 0) {
    DLOGE("drmModeSetCrtc failed dev fd %d, fb_id %d, crtc id %d, connector id %d, %s", dev_fd,
          fb_id, crtc_id, connector_id, strerror(errno));
    return kErrorHardware;
  }

  return kErrorNone;
}

DisplayError HWDeviceDRM::AtomicCommit(HWLayers *hw_layers) {
  DTRACE_SCOPED();

  int64_t release_fence_fd = -1;
  int64_t retire_fence_fd = -1;

  // scoped fence fds will be automatically closed when function scope ends,
  // atomic commit will have these fds already set on kernel by then.
  Fence::ScopedRef scoped_ref;
  SetupAtomic(scoped_ref, hw_layers, false /* validate */, &release_fence_fd, &retire_fence_fd);

  if (hw_layers->elapse_timestamp > 0) {
    struct timespec t = {0, 0};
    clock_gettime(CLOCK_MONOTONIC, &t);
    uint64_t current_time = (UINT64(t.tv_sec) * 1000000000LL + t.tv_nsec);
    if (current_time < hw_layers->elapse_timestamp) {
      usleep(UINT32((hw_layers->elapse_timestamp - current_time) / 1000));
    }
  }

  int ret = drm_atomic_intf_->Commit(synchronous_commit_, false /* retain_planes*/);
  shared_ptr<Fence> release_fence = Fence::Create(INT(release_fence_fd), "release");
  shared_ptr<Fence> retire_fence = Fence::Create(INT(retire_fence_fd), "retire");
  if (ret) {
    DLOGE("%s failed with error %d crtc %d", __FUNCTION__, ret, token_.crtc_id);
    DumpHWLayers(hw_layers);
    vrefresh_ = 0;
    panel_mode_changed_ = 0;
    return kErrorHardware;
  }

  DLOGD_IF(kTagDriverConfig, "RELEASE fence: fd: %s", Fence::GetStr(release_fence).c_str());
  DLOGD_IF(kTagDriverConfig, "RETIRE fence: fd: %s", Fence::GetStr(retire_fence).c_str());

  HWLayersInfo &hw_layer_info = hw_layers->info;
  LayerStack *stack = hw_layer_info.stack;
  stack->retire_fence = retire_fence;

  for (uint32_t i = 0; i < hw_layer_info.hw_layers.size(); i++) {
    Layer &layer = hw_layer_info.hw_layers.at(i);
    HWRotatorSession *hw_rotator_session = &hw_layers->config[i].hw_rotator_session;
    if (hw_rotator_session->mode == kRotatorOffline) {
      hw_rotator_session->output_buffer.release_fence = release_fence;
    } else {
      layer.input_buffer.release_fence = release_fence;
    }
  }

  hw_layer_info.sync_handle = release_fence;

  if (vrefresh_) {
    // Update current mode index if refresh rate is changed
    drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
    uint64_t current_bit_clk = connector_info_.modes[current_mode_index_].bit_clk_rate;
    for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if ((current_mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
          (current_mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
          (current_bit_clk == connector_info_.modes[mode_index].bit_clk_rate) &&
          (vrefresh_ == connector_info_.modes[mode_index].mode.vrefresh)) {
        current_mode_index_ = mode_index;
        SetDisplaySwitchMode(mode_index);
        break;
      }
    }
    vrefresh_ = 0;
  }

  if (bit_clk_rate_) {
    // Update current mode index if bit clk rate is changed.
    drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
    for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
      if ((current_mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
          (current_mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
          (current_mode.vrefresh == connector_info_.modes[mode_index].mode.vrefresh) &&
          (bit_clk_rate_ == connector_info_.modes[mode_index].bit_clk_rate)) {
        current_mode_index_ = mode_index;
        SetDisplaySwitchMode(mode_index);
        break;
      }
    }
    bit_clk_rate_ = 0;
  }

  if (panel_mode_changed_ & DRM_MODE_FLAG_CMD_MODE_PANEL) {
    panel_mode_changed_ = 0;
    synchronous_commit_ = false;
  } else if (panel_mode_changed_ & DRM_MODE_FLAG_VID_MODE_PANEL) {
    panel_mode_changed_ = 0;
    synchronous_commit_ = false;
    reset_output_fence_offset_ = true;
  }

  first_cycle_ = false;
  update_mode_ = false;
  hw_layers->updates_mask = 0;
  pending_doze_ = false;

  return kErrorNone;
}

DisplayError HWDeviceDRM::Flush(HWLayers *hw_layers) {
  ClearSolidfillStages();
  ResetROI();
  int ret = NullCommit(secure_display_active_ /* synchronous */, false /* retain_planes*/);
  if (ret) {
    DLOGE("failed with error %d", ret);
    return kErrorHardware;
  }

  return kErrorNone;
}

void HWDeviceDRM::SetBlending(const LayerBlending &source, DRMBlendType *target) {
  switch (source) {
    case kBlendingPremultiplied:
      *target = DRMBlendType::PREMULTIPLIED;
      break;
    case kBlendingOpaque:
      *target = DRMBlendType::OPAQUE;
      break;
    case kBlendingCoverage:
      *target = DRMBlendType::COVERAGE;
      break;
    default:
      *target = DRMBlendType::UNDEFINED;
  }
}

void HWDeviceDRM::SetSrcConfig(const LayerBuffer &input_buffer, const HWRotatorMode &mode,
                               uint32_t *config) {
  // In offline rotation case, rotator will handle deinterlacing.
  if (mode == kRotatorInline) {
    if (input_buffer.flags.interlace) {
      *config |= (0x01 << UINT32(DRMSrcConfig::DEINTERLACE));
    }
  }
}

void HWDeviceDRM::SelectCscType(const LayerBuffer &input_buffer, DRMCscType *type) {
  if (type == NULL) {
    return;
  }

  *type = DRMCscType::kCscTypeMax;
  if (input_buffer.format < kFormatYCbCr420Planar) {
    return;
  }

  switch (input_buffer.color_metadata.colorPrimaries) {
    case ColorPrimaries_BT601_6_525:
    case ColorPrimaries_BT601_6_625:
      *type = ((input_buffer.color_metadata.range == Range_Full) ?
               DRMCscType::kCscYuv2Rgb601FR : DRMCscType::kCscYuv2Rgb601L);
      break;
    case ColorPrimaries_BT709_5:
      *type = DRMCscType::kCscYuv2Rgb709L;
      break;
    case ColorPrimaries_BT2020:
      *type = ((input_buffer.color_metadata.range == Range_Full) ?
                DRMCscType::kCscYuv2Rgb2020FR : DRMCscType::kCscYuv2Rgb2020L);
      break;
    default:
      break;
  }
}

void HWDeviceDRM::SetRect(const LayerRect &source, DRMRect *target) {
  target->left = UINT32(source.left);
  target->top = UINT32(source.top);
  target->right = UINT32(source.right);
  target->bottom = UINT32(source.bottom);
}

void HWDeviceDRM::SetRotation(LayerTransform transform, const HWLayerConfig &layer_config,
                              uint32_t* rot_bit_mask) {
  HWRotatorMode mode = layer_config.hw_rotator_session.mode;
  // In offline rotation case, rotator will handle flips set via offline rotator interface.
  if (mode == kRotatorOffline) {
    *rot_bit_mask = 0;
    return;
  }

  // In no rotation case or inline rotation case, plane will handle flips
  // In DRM framework rotation is applied in counter-clockwise direction.
  if (layer_config.use_inline_rot && transform.rotation == 90) {
    // a) rotate 90 clockwise = rotate 270 counter-clockwise in DRM
    // rotate 270 is translated as hflip + vflip + rotate90
    // b) rotate 270 clockwise = rotate 90 counter-clockwise in DRM
    // c) hflip + rotate 90 clockwise = vflip + rotate 90 counter-clockwise in DRM
    // d) vflip + rotate 90 clockwise = hflip + rotate 90 counter-clockwise in DRM
    *rot_bit_mask = UINT32(DRMRotation::ROT_90);
    transform.flip_horizontal = !transform.flip_horizontal;
    transform.flip_vertical = !transform.flip_vertical;
  }

  if (transform.flip_horizontal) {
    *rot_bit_mask |= UINT32(DRMRotation::FLIP_H);
  }

  if (transform.flip_vertical) {
    *rot_bit_mask |= UINT32(DRMRotation::FLIP_V);
  }
}

bool HWDeviceDRM::EnableHotPlugDetection(int enable) {
  return true;
}

DisplayError HWDeviceDRM::SetCursorPosition(HWLayers *hw_layers, int x, int y) {
  DTRACE_SCOPED();
  return kErrorNone;
}

DisplayError HWDeviceDRM::GetPPFeaturesVersion(PPFeatureVersion *vers) {
  struct DRMPPFeatureInfo info = {};

  if (!hw_color_mgr_)
    return kErrorNotSupported;

  for (uint32_t i = 0; i < kMaxNumPPFeatures; i++) {
    std::vector<DRMPPFeatureID> drm_id = {};
    memset(&info, 0, sizeof(struct DRMPPFeatureInfo));
    hw_color_mgr_->ToDrmFeatureId(kDSPP, i, &drm_id);
    if (drm_id.empty())
      continue;

    info.id = drm_id.at(0);

    drm_mgr_intf_->GetCrtcPPInfo(token_.crtc_id, &info);
    vers->version[i] = hw_color_mgr_->GetFeatureVersion(info);
  }
  return kErrorNone;
}

DisplayError HWDeviceDRM::SetPPFeatures(PPFeaturesConfig *feature_list) {
  int ret = 0;
  PPFeatureInfo *feature = NULL;

  if (!hw_color_mgr_)
    return kErrorNotSupported;

  while (true) {
    std::vector<DRMPPFeatureID> drm_id = {};
    DRMPPFeatureInfo kernel_params = {};
    bool crtc_feature = true;

    ret = feature_list->RetrieveNextFeature(&feature);
    if (ret || !feature)
      break;

    hw_color_mgr_->ToDrmFeatureId(kDSPP, feature->feature_id_, &drm_id);
    if (drm_id.empty())
      continue;

    kernel_params.id = drm_id.at(0);
    drm_mgr_intf_->GetCrtcPPInfo(token_.crtc_id, &kernel_params);
    if (kernel_params.version == std::numeric_limits<uint32_t>::max()) {
      crtc_feature = false;
    }
    DLOGV_IF(kTagDriverConfig, "feature_id = %d", feature->feature_id_);
    for (DRMPPFeatureID id : drm_id) {
      if (id >= kPPFeaturesMax) {
        DLOGE("Invalid feature id %d", id);
        continue;
      }
      kernel_params.id = id;
      ret = hw_color_mgr_->GetDrmFeature(feature, &kernel_params);
      if (!ret && crtc_feature)
        drm_atomic_intf_->Perform(DRMOps::CRTC_SET_POST_PROC,
                                  token_.crtc_id, &kernel_params);
      else if (!ret && !crtc_feature)
        drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_POST_PROC,
                                  token_.conn_id, &kernel_params);

      hw_color_mgr_->FreeDrmFeatureData(&kernel_params);
    }
  }

  // Once all features were consumed, then destroy all feature instance from feature_list,
  feature_list->Reset();

  return kErrorNone;
}

DisplayError HWDeviceDRM::SetVSyncState(bool enable) {
  return kErrorNotSupported;
}

void HWDeviceDRM::SetIdleTimeoutMs(uint32_t timeout_ms) {
  // TODO(user): This function can be removed after fb is deprecated
}

DisplayError HWDeviceDRM::SetDisplayMode(const HWDisplayMode hw_display_mode) {
  if (!switch_mode_valid_) {
    return kErrorNotSupported;
  }

  uint32_t mode_flag = 0;

  if (hw_display_mode == kModeCommand) {
    mode_flag = DRM_MODE_FLAG_CMD_MODE_PANEL;
    current_mode_index_ = cmd_mode_index_;
    DLOGI_IF(kTagDriverConfig, "switch panel mode to command");
  } else if (hw_display_mode == kModeVideo) {
    mode_flag = DRM_MODE_FLAG_VID_MODE_PANEL;
    current_mode_index_ = video_mode_index_;
    DLOGI_IF(kTagDriverConfig, "switch panel mode to video");
  }
  PopulateHWPanelInfo();
  panel_mode_changed_ = mode_flag;
  synchronous_commit_ = true;
  return kErrorNone;
}

DisplayError HWDeviceDRM::SetRefreshRate(uint32_t refresh_rate) {
  if (bit_clk_rate_) {
    // bit rate update pending.
    // Defer any refresh rate setting.
    return kErrorNotSupported;
  }

  // Check if requested refresh rate is valid
  drmModeModeInfo current_mode = connector_info_.modes[current_mode_index_].mode;
  uint64_t current_bit_clk = connector_info_.modes[current_mode_index_].bit_clk_rate;
  for (uint32_t mode_index = 0; mode_index < connector_info_.modes.size(); mode_index++) {
    if ((current_mode.vdisplay == connector_info_.modes[mode_index].mode.vdisplay) &&
        (current_mode.hdisplay == connector_info_.modes[mode_index].mode.hdisplay) &&
        (current_bit_clk == connector_info_.modes[mode_index].bit_clk_rate) &&
        (current_mode.flags == connector_info_.modes[mode_index].mode.flags) &&
        (refresh_rate == connector_info_.modes[mode_index].mode.vrefresh)) {
      vrefresh_ = refresh_rate;
      DLOGV_IF(kTagDriverConfig, "Set refresh rate to %d", refresh_rate);
      return kErrorNone;
    }
  }
  return kErrorNotSupported;
}



DisplayError HWDeviceDRM::GetHWScanInfo(HWScanInfo *scan_info) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::GetVideoFormat(uint32_t config_index, uint32_t *video_format) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::GetMaxCEAFormat(uint32_t *max_cea_format) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::OnMinHdcpEncryptionLevelChange(uint32_t min_enc_level) {
  DisplayError error = kErrorNone;
  int fd = -1;
  char data[kMaxStringLength] = {'\0'};

  snprintf(data, sizeof(data), "/sys/devices/virtual/hdcp/msm_hdcp/min_level_change");

  fd = Sys::open_(data, O_WRONLY);
  if (fd < 0) {
    DLOGE("File '%s' could not be opened. errno = %d, desc = %s", data, errno, strerror(errno));
    return kErrorHardware;
  }

  snprintf(data, sizeof(data), "%d", min_enc_level);

  ssize_t err = Sys::pwrite_(fd, data, strlen(data), 0);
  if (err <= 0) {
    DLOGE("Write failed, Error = %s", strerror(errno));
    error = kErrorHardware;
  }

  Sys::close_(fd);

  return error;
}

DisplayError HWDeviceDRM::SetScaleLutConfig(HWScaleLutInfo *lut_info) {
  sde_drm::DRMScalerLUTInfo drm_lut_info = {};
  drm_lut_info.cir_lut = lut_info->cir_lut;
  drm_lut_info.dir_lut = lut_info->dir_lut;
  drm_lut_info.sep_lut = lut_info->sep_lut;
  drm_lut_info.cir_lut_size = lut_info->cir_lut_size;
  drm_lut_info.dir_lut_size = lut_info->dir_lut_size;
  drm_lut_info.sep_lut_size = lut_info->sep_lut_size;
  drm_mgr_intf_->SetScalerLUT(drm_lut_info);

  return kErrorNone;
}

DisplayError HWDeviceDRM::UnsetScaleLutConfig() {
  drm_mgr_intf_->UnsetScalerLUT();

  return kErrorNone;
}

DisplayError HWDeviceDRM::SetMixerAttributes(const HWMixerAttributes &mixer_attributes) {
  if (IsResolutionSwitchEnabled()) {
    return kErrorNotSupported;
  }

  if (!dest_scaler_blocks_used_) {
    return kErrorNotSupported;
  }

  uint32_t index = current_mode_index_;

  if (mixer_attributes.width > display_attributes_[index].x_pixels ||
      mixer_attributes.height > display_attributes_[index].y_pixels) {
    DLOGW("Input resolution exceeds display resolution! input: res %dx%d display: res %dx%d",
          mixer_attributes.width, mixer_attributes.height, display_attributes_[index].x_pixels,
          display_attributes_[index].y_pixels);
    return kErrorNotSupported;
  }

  uint32_t max_input_width = hw_resource_.hw_dest_scalar_info.max_input_width;
  if (display_attributes_[index].is_device_split) {
    max_input_width *= 2;
  }

  if (mixer_attributes.width > max_input_width) {
    DLOGW("Input width exceeds width limit! input_width %d width_limit %d", mixer_attributes.width,
          max_input_width);
    return kErrorNotSupported;
  }

  float mixer_aspect_ratio = FLOAT(mixer_attributes.width) / FLOAT(mixer_attributes.height);
  float display_aspect_ratio =
      FLOAT(display_attributes_[index].x_pixels) / FLOAT(display_attributes_[index].y_pixels);

  if (display_aspect_ratio != mixer_aspect_ratio) {
    DLOGW("Aspect ratio mismatch! input: res %dx%d display: res %dx%d", mixer_attributes.width,
          mixer_attributes.height, display_attributes_[index].x_pixels,
          display_attributes_[index].y_pixels);
    return kErrorNotSupported;
  }

  float scale_x = FLOAT(display_attributes_[index].x_pixels) / FLOAT(mixer_attributes.width);
  float scale_y = FLOAT(display_attributes_[index].y_pixels) / FLOAT(mixer_attributes.height);
  float max_scale_up = hw_resource_.hw_dest_scalar_info.max_scale_up;
  if (scale_x > max_scale_up || scale_y > max_scale_up) {
    DLOGW(
        "Up scaling ratio exceeds for destination scalar upscale limit scale_x %f scale_y %f "
        "max_scale_up %f",
        scale_x, scale_y, max_scale_up);
    return kErrorNotSupported;
  }

  float mixer_split_ratio = FLOAT(mixer_attributes_.split_left) / FLOAT(mixer_attributes_.width);

  mixer_attributes_ = mixer_attributes;
  mixer_attributes_.split_left = mixer_attributes_.width;
  mixer_attributes_.split_type = kNoSplit;
  mixer_attributes_.dest_scaler_blocks_used = dest_scaler_blocks_used_;  // No change.
  if (display_attributes_[index].is_device_split) {
    mixer_attributes_.split_left = UINT32(FLOAT(mixer_attributes.width) * mixer_split_ratio);
    mixer_attributes_.split_type = kDualSplit;
    if (display_attributes_[index].topology == kQuadLMMerge ||
        display_attributes_[index].topology == kQuadLMDSCMerge ||
        display_attributes_[index].topology == kQuadLMMergeDSC) {
      mixer_attributes_.split_type = kQuadSplit;
    }
  }

  return kErrorNone;
}

DisplayError HWDeviceDRM::GetMixerAttributes(HWMixerAttributes *mixer_attributes) {
  if (!mixer_attributes) {
    return kErrorParameters;
  }

  *mixer_attributes = mixer_attributes_;

  return kErrorNone;
}

DisplayError HWDeviceDRM::DumpDebugData() {
  string dir_path = "/data/vendor/display/hw_recovery/";
  string device_str = device_name_;

  // Attempt to make hw_recovery dir, it may exist
  if (mkdir(dir_path.c_str(), 0777) != 0 && errno != EEXIST) {
    DLOGW("Failed to create %s directory errno = %d, desc = %s", dir_path.c_str(), errno,
          strerror(errno));
    return kErrorPermission;
  }
  // If it does exist, ensure permissions are fine
  if (errno == EEXIST && chmod(dir_path.c_str(), 0777) != 0) {
    DLOGW("Failed to change permissions on %s directory", dir_path.c_str());
    return kErrorPermission;
  }

  string filename = dir_path+device_str+"_HWR_"+to_string(debug_dump_count_);
  ofstream dst(filename);
  debug_dump_count_++;

  {
    ifstream src;
    src.open("/sys/kernel/debug/dri/0/debug/dump");
    dst << "---- Event Logs ----" << std::endl;
    dst << src.rdbuf() << std::endl;
    src.close();
  }

  {
    ifstream src;
    src.open("/sys/kernel/debug/dri/0/debug/recovery_reg");
    dst << "---- All Registers ----" << std::endl;
    dst << src.rdbuf() << std::endl;
    src.close();
  }

  {
    ifstream src;
    src.open("/sys/kernel/debug/dri/0/debug/recovery_dbgbus");
    dst << "---- Debug Bus ----" << std::endl;
    dst << src.rdbuf() << std::endl;
    src.close();
  }

  {
    ifstream src;
    src.open("/sys/kernel/debug/dri/0/debug/recovery_vbif_dbgbus");
    dst << "---- VBIF Debug Bus ----" << std::endl;
    dst << src.rdbuf() << std::endl;
    src.close();
  }

  dst.close();
  DLOGI("Wrote hw_recovery file %s", filename.c_str());

  return kErrorNone;
}

void HWDeviceDRM::GetDRMDisplayToken(sde_drm::DRMDisplayToken *token) const {
  *token = token_;
}

void HWDeviceDRM::UpdateMixerAttributes() {
  uint32_t index = current_mode_index_;

  mixer_attributes_.width = display_attributes_[index].x_pixels;
  mixer_attributes_.height = display_attributes_[index].y_pixels;
  mixer_attributes_.split_left = display_attributes_[index].is_device_split
                                     ? hw_panel_info_.split_info.left_split
                                     : mixer_attributes_.width;
  mixer_attributes_.split_type = kNoSplit;
  if (display_attributes_[index].is_device_split) {
    mixer_attributes_.split_type = kDualSplit;
    if (display_attributes_[index].topology == kQuadLMMerge ||
        display_attributes_[index].topology == kQuadLMDSCMerge ||
        display_attributes_[index].topology == kQuadLMMergeDSC) {
      mixer_attributes_.split_type = kQuadSplit;
    }
  }

  DLOGI("Mixer WxH %dx%d-%d for %s", mixer_attributes_.width, mixer_attributes_.height,
        mixer_attributes_.split_type, device_name_);
  update_mode_ = true;
}

void HWDeviceDRM::SetSecureConfig(const LayerBuffer &input_buffer, DRMSecureMode *fb_secure_mode,
                                  DRMSecurityLevel *security_level) {
  *fb_secure_mode = DRMSecureMode::NON_SECURE;
  *security_level = DRMSecurityLevel::SECURE_NON_SECURE;

  if (input_buffer.flags.secure) {
    if (input_buffer.flags.secure_camera) {
      // IOMMU configuration for this framebuffer mode is secure domain & requires
      // only stage II translation, when this buffer is accessed by Display H/W.
      // Secure and non-secure planes can be attached to this CRTC.
      *fb_secure_mode = DRMSecureMode::SECURE_DIR_TRANSLATION;
    } else if (input_buffer.flags.secure_display) {
      // IOMMU configuration for this framebuffer mode is secure domain & requires
      // only stage II translation, when this buffer is accessed by Display H/W.
      // Only secure planes can be attached to this CRTC.
      *fb_secure_mode = DRMSecureMode::SECURE_DIR_TRANSLATION;
      *security_level = DRMSecurityLevel::SECURE_ONLY;
    } else {
      // IOMMU configuration for this framebuffer mode is secure domain & requires both
      // stage I and stage II translations, when this buffer is accessed by Display H/W.
      // Secure and non-secure planes can be attached to this CRTC.
      *fb_secure_mode = DRMSecureMode::SECURE;
    }
  }
}

void HWDeviceDRM::SetTopology(sde_drm::DRMTopology drm_topology, HWTopology *hw_topology) {
  switch (drm_topology) {
    case DRMTopology::SINGLE_LM:          *hw_topology = kSingleLM;        break;
    case DRMTopology::SINGLE_LM_DSC:      *hw_topology = kSingleLMDSC;     break;
    case DRMTopology::DUAL_LM:            *hw_topology = kDualLM;          break;
    case DRMTopology::DUAL_LM_DSC:        *hw_topology = kDualLMDSC;       break;
    case DRMTopology::DUAL_LM_MERGE:      *hw_topology = kDualLMMerge;     break;
    case DRMTopology::DUAL_LM_MERGE_DSC:  *hw_topology = kDualLMMergeDSC;  break;
    case DRMTopology::DUAL_LM_DSCMERGE:   *hw_topology = kDualLMDSCMerge;  break;
    case DRMTopology::QUAD_LM_MERGE:      *hw_topology = kQuadLMMerge;     break;
    case DRMTopology::QUAD_LM_DSCMERGE:   *hw_topology = kQuadLMDSCMerge;  break;
    case DRMTopology::QUAD_LM_MERGE_DSC:  *hw_topology = kQuadLMMergeDSC;  break;
    case DRMTopology::PPSPLIT:            *hw_topology = kPPSplit;         break;
    default:                              *hw_topology = kUnknown;         break;
  }
}


void HWDeviceDRM::SetMultiRectMode(const uint32_t flags, DRMMultiRectMode *target) {
  *target = DRMMultiRectMode::NONE;
  if (flags & kMultiRect) {
    *target = DRMMultiRectMode::SERIAL;
    if (flags & kMultiRectParallelMode) {
      *target = DRMMultiRectMode::PARALLEL;
    }
  }
}

void HWDeviceDRM::SetSsppTonemapFeatures(HWPipeInfo *pipe_info) {
  if (pipe_info->dgm_csc_info.op != kNoOp) {
    SDECsc csc = {};
    SetDGMCsc(pipe_info->dgm_csc_info, &csc);
    DLOGV_IF(kTagDriverConfig, "Call Perform DGM CSC Op = %s",
            (pipe_info->dgm_csc_info.op == kSet) ? "Set" : "Reset");
    drm_atomic_intf_->Perform(DRMOps::PLANE_SET_DGM_CSC_CONFIG, pipe_info->pipe_id,
                              reinterpret_cast<uint64_t>(&csc.csc_v1));
  }
  if (pipe_info->inverse_pma_info.op != kNoOp) {
    DLOGV_IF(kTagDriverConfig, "Call Perform Inverse PMA Op = %s",
            (pipe_info->inverse_pma_info.op == kSet) ? "Set" : "Reset");
    drm_atomic_intf_->Perform(DRMOps::PLANE_SET_INVERSE_PMA, pipe_info->pipe_id,
                             (pipe_info->inverse_pma_info.inverse_pma) ? 1: 0);
  }
  SetSsppLutFeatures(pipe_info);
}

void HWDeviceDRM::SetDGMCsc(const HWPipeCscInfo &dgm_csc_info, SDECsc *csc) {
  SetDGMCscV1(dgm_csc_info.csc, &csc->csc_v1);
}

void HWDeviceDRM::SetDGMCscV1(const HWCsc &dgm_csc, sde_drm_csc_v1 *csc_v1) {
  uint32_t i = 0;
  for (i = 0; i < MAX_CSC_MATRIX_COEFF_SIZE; i++) {
    csc_v1->ctm_coeff[i] = dgm_csc.ctm_coeff[i];
    DLOGV_IF(kTagDriverConfig, " DGM csc_v1[%d] = %" PRId64, i, csc_v1->ctm_coeff[i]);
  }
  for (i = 0; i < MAX_CSC_BIAS_SIZE; i++) {
    csc_v1->pre_bias[i] = dgm_csc.pre_bias[i];
    csc_v1->post_bias[i] = dgm_csc.post_bias[i];
  }
  for (i = 0; i < MAX_CSC_CLAMP_SIZE; i++) {
    csc_v1->pre_clamp[i] = dgm_csc.pre_clamp[i];
    csc_v1->post_clamp[i] = dgm_csc.post_clamp[i];
  }
}

void HWDeviceDRM::SetSsppLutFeatures(HWPipeInfo *pipe_info) {
  for (HWPipeTonemapLutInfo &lut_info : pipe_info->lut_info) {
    if (lut_info.op != kNoOp) {
      std::shared_ptr<PPFeatureInfo> feature = lut_info.pay_load;
      if (feature == nullptr) {
        DLOGE("Null Pointer for Op = %d lut type = %d", lut_info.op, lut_info.type);
        continue;
      }
      DRMPPFeatureInfo kernel_params = {};
      std::vector<DRMPPFeatureID> drm_id = {};
      PPBlock pp_block = GetPPBlock(lut_info.type);
      hw_color_mgr_->ToDrmFeatureId(pp_block, feature->feature_id_, &drm_id);
      for (DRMPPFeatureID id : drm_id) {
        if (id >= kPPFeaturesMax) {
          DLOGE("Invalid feature id %d", id);
          continue;
        }
        kernel_params.id = id;
        bool disable = (lut_info.op == kReset);
        DLOGV_IF(kTagDriverConfig, "Lut Type = %d PPBlock = %d Op = %s Disable = %d Feature = %p",
                 lut_info.type, pp_block, (lut_info.op ==kSet) ? "Set" : "Reset", disable,
                 feature.get());
        int ret = hw_color_mgr_->GetDrmFeature(feature.get(), &kernel_params, disable);
        if (!ret) {
          drm_atomic_intf_->Perform(DRMOps::PLANE_SET_POST_PROC, pipe_info->pipe_id,
                                    &kernel_params);
          hw_color_mgr_->FreeDrmFeatureData(&kernel_params);
        } else {
          DLOGE("GetDrmFeature failed for Lut type = %d", lut_info.type);
        }
      }
      drm_id.clear();
    }
  }
}

void HWDeviceDRM::AddDimLayerIfNeeded() {
  if (secure_display_active_ && hw_resource_.secure_disp_blend_stage >= 0) {
    HWSolidfillStage sf = {};
    sf.z_order = UINT32(hw_resource_.secure_disp_blend_stage);
    sf.roi = { 0.0, 0.0, FLOAT(mixer_attributes_.width), FLOAT(mixer_attributes_.height) };
    solid_fills_.clear();
    AddSolidfillStage(sf, 0xFF);
    SetSolidfillStages();
  }

  if (!secure_display_active_) {
    DRMSecurityLevel crtc_security_level = DRMSecurityLevel::SECURE_NON_SECURE;
    drm_atomic_intf_->Perform(DRMOps::CRTC_SET_SECURITY_LEVEL, token_.crtc_id, crtc_security_level);
  }
}

DisplayError HWDeviceDRM::NullCommit(bool synchronous, bool retain_planes) {
  DTRACE_SCOPED();
  AddDimLayerIfNeeded();
  int ret = drm_atomic_intf_->Commit(synchronous , retain_planes);
  if (ret) {
    DLOGE("failed with error %d", ret);
    return kErrorHardware;
  }

  null_display_commit_ = true;
  return kErrorNone;
}

void HWDeviceDRM::DumpConnectorModeInfo() {
  for (uint32_t i = 0; i < (uint32_t)connector_info_.modes.size(); i++) {
    DLOGI("Mode[%d] Name:%s vref:%d hdisp:%d hsync_s:%d hsync_e:%d htotal:%d " \
          "vdisp:%d vsync_s:%d vsync_e:%d vtotal:%d\n", i, connector_info_.modes[i].mode.name,
          connector_info_.modes[i].mode.vrefresh, connector_info_.modes[i].mode.hdisplay,
          connector_info_.modes[i].mode.hsync_start, connector_info_.modes[i].mode.hsync_end,
          connector_info_.modes[i].mode.htotal, connector_info_.modes[i].mode.vdisplay,
          connector_info_.modes[i].mode.vsync_start, connector_info_.modes[i].mode.vsync_end,
          connector_info_.modes[i].mode.vtotal);
  }
}

void HWDeviceDRM::ResetROI() {
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ROI, token_.crtc_id, 0, nullptr);
  drm_atomic_intf_->Perform(DRMOps::CONNECTOR_SET_ROI, token_.conn_id, 0, nullptr);
}

bool HWDeviceDRM::IsFullFrameUpdate(const HWLayersInfo &hw_layer_info) {
  LayerRect full_frame = {0, 0, FLOAT(mixer_attributes_.width), FLOAT(mixer_attributes_.height)};

  const LayerRect &frame_roi = hw_layer_info.left_frame_roi.at(0);
  // If multiple ROIs are present, then it's not fullscreen update.
  if (hw_layer_info.left_frame_roi.size() > 1 ||
      (IsValid(frame_roi) && !IsCongruent(full_frame, frame_roi))) {
    return false;
  }

  return true;
}

DisplayError HWDeviceDRM::SetDynamicDSIClock(uint64_t bit_clk_rate) {
  return kErrorNotSupported;
}

DisplayError HWDeviceDRM::GetDynamicDSIClock(uint64_t *bit_clk_rate) {
  return kErrorNotSupported;
}

void HWDeviceDRM::DumpHWLayers(HWLayers *hw_layers) {
  HWLayersInfo &hw_layer_info = hw_layers->info;
  DestScaleInfoMap &dest_scale_info_map = hw_layer_info.dest_scale_info_map;
  LayerStack *stack = hw_layer_info.stack;
  uint32_t hw_layer_count = UINT32(hw_layer_info.hw_layers.size());
  std::vector<LayerRect> &left_frame_roi = hw_layer_info.left_frame_roi;
  std::vector<LayerRect> &right_frame_roi = hw_layer_info.right_frame_roi;
  DLOGI("HWLayers Stack: layer_count: %d, app_layer_count: %d, gpu_target_index: %d",
         hw_layer_count, hw_layer_info.app_layer_count, hw_layer_info.gpu_target_index);
  DLOGI("LayerStackFlags = 0x%" PRIu32 ",  blend_cs = {primaries = %d, transfer = %d}",
         UINT32(stack->flags.flags), UINT32(stack->blend_cs.primaries),
         UINT32(stack->blend_cs.transfer));
  for (uint32_t i = 0; i < left_frame_roi.size(); i++) {
    DLOGI("left_frame_roi: x = %d, y = %d, w = %d, h = %d", INT(left_frame_roi[i].left),
        INT(left_frame_roi[i].top), INT(left_frame_roi[i].right), INT(left_frame_roi[i].bottom));
  }
  for (uint32_t i = 0; i < right_frame_roi.size(); i++) {
  DLOGI("right_frame_roi: x = %d, y = %d, w = %d h = %d", INT(right_frame_roi[i].left),
        INT(right_frame_roi[i].top), INT(right_frame_roi[i].right),
        INT(right_frame_roi[i].bottom));
  }

  for (uint32_t i = 0; i < dest_scale_info_map.size(); i++) {
    HWDestScaleInfo *dest_scalar_data = dest_scale_info_map[i];
    if (dest_scalar_data->scale_data.enable.scale) {
      HWScaleData &scale_data = dest_scalar_data->scale_data;
      DLOGI("Dest scalar index %d Mixer WxH %dx%d", i,
            dest_scalar_data->mixer_width, dest_scalar_data->mixer_height);
      DLOGI("Panel ROI [%d, %d, %d, %d]", INT(dest_scalar_data->panel_roi.left),
           INT(dest_scalar_data->panel_roi.top), INT(dest_scalar_data->panel_roi.right),
           INT(dest_scalar_data->panel_roi.bottom));
      DLOGI("Dest scalar Dst WxH %dx%d", scale_data.dst_width, scale_data.dst_height);
    }
  }

  for (uint32_t i = 0; i < hw_layer_count; i++) {
    HWLayerConfig &hw_config = hw_layers->config[i];
    HWRotatorSession &hw_rotator_session = hw_config.hw_rotator_session;
    HWSessionConfig &hw_session_config = hw_rotator_session.hw_session_config;
    DLOGI("========================= HW_layer: %d =========================", i);
    DLOGI("src_width = %d, src_height = %d, src_format = %d, src_LayerBufferFlags = 0x%" PRIx32 ,
             hw_layer_info.hw_layers[i].input_buffer.width,
             hw_layer_info.hw_layers[i].input_buffer.height,
             hw_layer_info.hw_layers[i].input_buffer.format,
             hw_layer_info.hw_layers[i].input_buffer.flags.flags);
    if (hw_config.use_inline_rot) {
      DLOGI("rotator = %s, rotation = %d, flip_horizontal = %s, flip_vertical = %s",
            "inline rotator", INT(hw_session_config.transform.rotation),
            hw_session_config.transform.flip_horizontal ? "true" : "false",
            hw_session_config.transform.flip_vertical ? "true" : "false");
    } else if (hw_rotator_session.mode == kRotatorOffline) {
      DLOGI("rotator = %s, rotation = %d, flip_horizontal = %s, flip_vertical = %s",
            "offline rotator", INT(hw_session_config.transform.rotation),
            hw_session_config.transform.flip_horizontal ? "true" : "false",
            hw_session_config.transform.flip_vertical ? "true" : "false");
    }
    if (hw_config.use_solidfill_stage) {
      HWSolidfillStage &hw_solidfill_stage = hw_config.hw_solidfill_stage;
      LayerSolidFill &solid_fill_info = hw_solidfill_stage.solid_fill_info;
      DLOGI("HW Solid fill info: z_order = %d, color = %d", hw_solidfill_stage.z_order,
            hw_solidfill_stage.color);
      DLOGI("bit_depth = %d, red = %d, green = %d, blue = %d, alpha = %d",
            solid_fill_info.bit_depth, solid_fill_info.red, solid_fill_info.green,
            solid_fill_info.blue, solid_fill_info.alpha);
    }
    for (uint32_t count = 0; count < 2; count++) {
      HWPipeInfo &left_pipe = hw_config.left_pipe;
      HWPipeInfo &right_pipe = hw_config.right_pipe;
      HWPipeInfo &pipe_info = (count == 0) ? left_pipe : right_pipe;
      HWScaleData &scale_data = pipe_info.scale_data;
      if (!pipe_info.valid) {
        continue;
      }
      std::string pipe = (count == 0) ? "left_pipe" : "right_pipe";
      DLOGI("pipe = %s, pipe_id = %d, z_order = %d, flags = 0x%X",
           pipe.c_str(), pipe_info.pipe_id, pipe_info.z_order, pipe_info.flags);
      DLOGI("src_rect: x = %d, y = %d, w = %d, h = %d", INT(pipe_info.src_roi.left),
            INT(pipe_info.src_roi.top), INT(pipe_info.src_roi.right - pipe_info.src_roi.left),
            INT(pipe_info.src_roi.bottom - pipe_info.src_roi.top));
      DLOGI("dst_rect: x = %d, y = %d, w = %d, h = %d", INT(pipe_info.dst_roi.left),
            INT(pipe_info.dst_roi.top), INT(pipe_info.dst_roi.right - pipe_info.dst_roi.left),
            INT(pipe_info.dst_roi.bottom - pipe_info.dst_roi.top));
      DLOGI("excl_rect: left = %d, top = %d, right = %d, bottom = %d",
            INT(pipe_info.excl_rect.left), INT(pipe_info.excl_rect.top),
            INT(pipe_info.excl_rect.right), INT(pipe_info.excl_rect.bottom));
      if (scale_data.enable.scale) {
      DLOGI("HWScaleData enable flags: scale = %s, direction_detection = %s, detail_enhance = %s,"
            " dyn_exp_disable = %s", scale_data.enable.scale ? "true" : "false",
            scale_data.enable.direction_detection ? "true" : "false",
            scale_data.enable.detail_enhance ? "true" : "false",
            scale_data.enable.dyn_exp_disable ? "true" : "false");
      DLOGI("lut_flags: lut_swap = 0x%X, lut_dir_wr = 0x%X, lut_y_cir_wr = 0x%X, "
            "lut_uv_cir_wr = 0x%X, lut_y_sep_wr = 0x%X, lut_uv_sep_wr = 0x%X",
            scale_data.lut_flag.lut_swap, scale_data.lut_flag.lut_dir_wr,
            scale_data.lut_flag.lut_y_cir_wr, scale_data.lut_flag.lut_uv_cir_wr,
            scale_data.lut_flag.lut_y_sep_wr, scale_data.lut_flag.lut_uv_sep_wr);
      DLOGI("dir_lut_idx = %d, y_rgb_cir_lut_idx = %d, uv_cir_lut_idx = %d, "
            "y_rgb_sep_lut_idx = %d, uv_sep_lut_idx = %d", scale_data.dir_lut_idx,
            scale_data.y_rgb_cir_lut_idx, scale_data.uv_cir_lut_idx,
            scale_data.y_rgb_sep_lut_idx, scale_data.uv_sep_lut_idx);
      }
    }
  }
}

DisplayError HWDeviceDRM::SetBlendSpace(const PrimariesTransfer &blend_space) {
  blend_space_ = blend_space;
  return kErrorNone;
}

}  // namespace sdm
