/*
* Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of The Linux Foundation nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

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

#include <stdint.h>
#include <stdlib.h>
#include <drm.h>
// The 3 headers above are a workaround to prevent kernel drm.h from being used that has the
// "virtual" keyword used for a variable. In future replace libdrm version drm.h with kernel
// version drm/drm.h
#include <drm_logger.h>
#include <drm/drm_fourcc.h>
#include <drm/sde_drm.h>

#include <cstring>
#include <map>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include <algorithm>

#include "drm_utils.h"
#include "drm_plane.h"
#include "drm_property.h"

namespace sde_drm {

using std::map;
using std::string;
using std::map;
using std::pair;
using std::vector;
using std::unique_ptr;
using std::tuple;
using std::stringstream;
using std::mutex;
using std::lock_guard;

#define MAX_SCALER_LINEWIDTH 2560

static struct sde_drm_csc_v1 csc_10bit_convert[kCscTypeMax] = {
  [kCscYuv2Rgb601L] = {
    {
      0x12A000000, 0x000000000,  0x198800000,
      0x12A000000, 0x7F9B800000, 0x7F30000000,
      0x12A000000, 0x204800000,  0x000000000,
    },
    { 0xffc0, 0xfe00, 0xfe00,},
    { 0x0, 0x0, 0x0,},
    { 0x40, 0x3ac, 0x40, 0x3c0, 0x40, 0x3c0,},
    { 0x0, 0x3ff, 0x0, 0x3ff, 0x0, 0x3ff,},
  },
  [kCscYuv2Rgb601FR] = {
    {
      0x100000000, 0x0, 0x167000000,
      0x100000000, 0x7fa8000000, 0x7f49000000,
      0x100000000, 0x1c5800000, 0x0,
    },
    { 0x0000, 0xfe00, 0xfe00,},
    { 0x0, 0x0, 0x0,},
    { 0x0, 0x3ff, 0x0, 0x3ff, 0x0, 0x3ff,},
    { 0x0, 0x3ff, 0x0, 0x3ff, 0x0, 0x3ff,},
  },
  [kCscYuv2Rgb709L] = {
    {
      0x12a000000, 0x0, 0x1cb000000,
      0x12a000000, 0x7fc9800000, 0x7f77800000,
      0x12a000000, 0x21d000000, 0x0,
    },
    { 0xffc0, 0xfe00, 0xfe00,},
    { 0x0, 0x0, 0x0,},
    { 0x40, 0x3ac, 0x40, 0x3c0, 0x40, 0x3c0,},
    { 0x0, 0x3ff, 0x0, 0x3ff, 0x0, 0x3ff,},
  },
  [kCscYuv2Rgb2020L] = {
    {
      0x12b000000, 0x0, 0x1af000000,
      0x12b000000, 0x7fd0000000, 0x7f59000000,
      0x12b000000, 0x226000000, 0x0,
    },
    { 0xffc0, 0xfe00, 0xfe00,},
    { 0x0, 0x0, 0x0,},
    { 0x40, 0x3ac, 0x40, 0x3c0, 0x40, 0x3c0,},
    { 0x0, 0x3ff, 0x0, 0x3ff, 0x0, 0x3ff,},
  },
  [kCscYuv2Rgb2020FR] = {
    {
      0x100000000, 0x0, 0x179800000,
      0x100000000, 0x7fd6000000, 0x7f6d800000,
      0x100000000, 0x1e1800000, 0x0,
    },
    { 0x0000, 0xfe00, 0xfe00,},
    { 0x0, 0x0, 0x0,},
    { 0x0, 0x3ff, 0x0, 0x3ff, 0x0, 0x3ff,},
    { 0x0, 0x3ff, 0x0, 0x3ff, 0x0, 0x3ff,},
  },
};

static uint8_t REFLECT_X = 0;
static uint8_t REFLECT_Y = 0;
static uint8_t ROTATE_90 = 0;
static uint8_t ROTATE_0 = 0;

// FB Secure Modes
static uint8_t NON_SECURE = 0;
static uint8_t SECURE = 1;
static uint8_t NON_SECURE_DIR_TRANSLATION = 2;
static uint8_t SECURE_DIR_TRANSLATION = 3;

// Multi rect modes
static uint8_t MULTIRECT_NONE = 0;
static uint8_t MULTIRECT_PARALLEL = 1;
static uint8_t MULTIRECT_SERIAL = 2;

static void SetRect(DRMRect &source, drm_clip_rect *target) {
  target->x1 = uint16_t(source.left);
  target->y1 = uint16_t(source.top);
  target->x2 = uint16_t(source.right);
  target->y2 = uint16_t(source.bottom);
}

static void PopulateReflect(drmModePropertyRes *prop) {
  if (REFLECT_X) {
    return;
  }

  if (!drm_property_type_is(prop, DRM_MODE_PROP_BITMASK)) {
    return;
  }

  for (auto i = 0; i < prop->count_enums; i++) {
    string enum_name(prop->enums[i].name);
    if (enum_name == "reflect-x") {
      REFLECT_X = prop->enums[i].value;
    } else if (enum_name == "reflect-y") {
      REFLECT_Y = prop->enums[i].value;
    } else if (enum_name == "rotate-90") {
      ROTATE_90 = prop->enums[i].value;
    } else if (enum_name == "rotate-0") {
      ROTATE_0 = prop->enums[i].value;
    }
  }
}

static void PopulateSecureModes(drmModePropertyRes *prop) {
  static bool secure_modes_populated = false;
  if (!secure_modes_populated) {
    for (auto i = 0; i < prop->count_enums; i++) {
      string enum_name(prop->enums[i].name);
      if (enum_name == "non_sec") {
        NON_SECURE = prop->enums[i].value;
      } else if (enum_name == "sec") {
        SECURE = prop->enums[i].value;
      } else if (enum_name == "non_sec_direct_translation") {
        NON_SECURE_DIR_TRANSLATION = prop->enums[i].value;
      } else if (enum_name == "sec_direct_translation") {
        SECURE_DIR_TRANSLATION = prop->enums[i].value;
      }
    }
    secure_modes_populated = true;
  }
}

static InlineRotationVersion PopulateInlineRotationVersion(uint32_t ver) {
  switch (ver) {
    case 0x0000: return InlineRotationVersion::kInlineRotationNone;
    case 0x0001:
    case 0x0100: return InlineRotationVersion::kInlineRotationV1;
    case 0x0200: return InlineRotationVersion::kInlineRotationV2;
    default: return InlineRotationVersion::kInlineRotationNone;
  }
}

static QSEEDStepVersion PopulateQseedStepVersion(uint32_t hw_ver) {
  switch (hw_ver) {
    case 0x1003: return QSEEDStepVersion::V3;
    case 0x1004: return QSEEDStepVersion::V4;
    case 0x2004: return QSEEDStepVersion::V3LITE_V4;
    case 0x3000: return QSEEDStepVersion::V3LITE_V5;
    // default value. also corresponds to (hw_ver == 0x1002)
    default: return QSEEDStepVersion::V2;
  }
}

static void PopulateMultiRectModes(drmModePropertyRes *prop) {
  static bool multirect_modes_populated = false;
  if (!multirect_modes_populated) {
    for (auto i = 0; i < prop->count_enums; i++) {
      string enum_name(prop->enums[i].name);
      if (enum_name == "none") {
        MULTIRECT_NONE = prop->enums[i].value;
      } else if (enum_name == "parallel") {
        MULTIRECT_PARALLEL = prop->enums[i].value;
      } else if (enum_name == "serial") {
        MULTIRECT_SERIAL = prop->enums[i].value;
      }
    }
    multirect_modes_populated = true;
  }
}

static const char *GetColorLutString(DRMTonemapLutType lut_type) {
  switch (lut_type) {
    case DRMTonemapLutType::DMA_1D_IGC:
      return "DMA IGC";
    case DRMTonemapLutType::DMA_1D_GC:
      return "DMA GC";
    case DRMTonemapLutType::VIG_1D_IGC:
      return "VIG IGC";
    case DRMTonemapLutType::VIG_3D_GAMUT:
      return "VIG 3D";
    default:
      return "Unknown Lut";
   }
}

#define __CLASS__ "DRMPlaneManager"

static bool GetDRMonemapLutTypeFromPPFeatureID(DRMPPFeatureID id, DRMTonemapLutType *lut_type) {
  switch (id) {
    case kFeatureDgmIgc:
      *lut_type = DRMTonemapLutType::DMA_1D_IGC;
      break;
    case kFeatureDgmGc:
      *lut_type = DRMTonemapLutType::DMA_1D_GC;
      break;
    case kFeatureVigIgc:
      *lut_type = DRMTonemapLutType::VIG_1D_IGC;
      break;
    case kFeatureVigGamut:
      *lut_type = DRMTonemapLutType::VIG_3D_GAMUT;
      break;
    default:
      DRM_LOGE("Invalid DRMPPFeature id = %d", id);
      return false;
  }
  return true;
}

DRMPlaneManager::DRMPlaneManager(int fd) : fd_(fd) {}

void DRMPlaneManager::Init() {
  drmModePlaneRes *resource = drmModeGetPlaneResources(fd_);
  if (!resource) {
    return;
  }

  for (uint32_t i = 0; i < resource->count_planes; i++) {
    // The enumeration order itself is the priority from high to low
    unique_ptr<DRMPlane> plane(new DRMPlane(fd_, i));
    drmModePlane *libdrm_plane = drmModeGetPlane(fd_, resource->planes[i]);
    if (libdrm_plane) {
      plane->InitAndParse(libdrm_plane);
      plane_pool_[resource->planes[i]] = std::move(plane);
    } else {
      DRM_LOGE("Critical error: drmModeGetPlane() failed for plane %d.", resource->planes[i]);
    }
  }

  drmModeFreePlaneResources(resource);
}

void DRMPlaneManager::DumpByID(uint32_t id) {
  plane_pool_.at(id)->Dump();
}

void DRMPlaneManager::Perform(DRMOps code, uint32_t obj_id, drmModeAtomicReq *req, va_list args) {
  auto it = plane_pool_.find(obj_id);
  if (it == plane_pool_.end()) {
    DRM_LOGE("Invalid plane id %d", obj_id);
    return;
  }

  if (code == DRMOps::PLANE_SET_SCALER_CONFIG) {
    if (it->second->ConfigureScalerLUT(req, dir_lut_blob_id_, cir_lut_blob_id_,
                                       sep_lut_blob_id_)) {
      DRM_LOGD("Plane %d: Configuring scaler LUTs", obj_id);
    }
  }

  it->second->Perform(code, req, args);
}

void DRMPlaneManager::Perform(DRMOps code, drmModeAtomicReq *req, uint32_t obj_id, ...) {
  lock_guard<mutex> lock(lock_);
  va_list args;
  va_start(args, obj_id);
  Perform(code, obj_id, req, args);
  va_end(args);
}

void DRMPlaneManager::DumpAll() {
  for (uint32_t i = 0; i < plane_pool_.size(); i++) {
    plane_pool_[i]->Dump();
  }
}

void DRMPlaneManager::GetPlanesInfo(DRMPlanesInfo *info) {
  for (auto &plane : plane_pool_) {
    info->push_back(std::make_pair(plane.first, plane.second->GetPlaneTypeInfo()));
  }
}

void DRMPlaneManager::UnsetUnusedResources(uint32_t crtc_id, bool is_commit, drmModeAtomicReq *req) {
  // Unset planes that were assigned to the crtc referred to by crtc_id but are not requested
  // in this round
  lock_guard<mutex> lock(lock_);
  for (auto &plane : plane_pool_) {
    uint32_t assigned_crtc = 0;
    uint32_t requested_crtc = 0;
    plane.second->GetAssignedCrtc(&assigned_crtc);
    plane.second->GetRequestedCrtc(&requested_crtc);
    if (assigned_crtc == crtc_id && requested_crtc == 0) {
      plane.second->Unset(is_commit, req);
    } else if (requested_crtc == crtc_id) {
      // Plane is acquired, call reset color luts, which will reset if needed
      plane.second->ResetColorLUTs(is_commit, req);
    }
  }
}

void DRMPlaneManager::RetainPlanes(uint32_t crtc_id) {
  for (auto &plane : plane_pool_) {
    uint32_t assigned_crtc = 0;
    plane.second->GetAssignedCrtc(&assigned_crtc);
    if (assigned_crtc == crtc_id) {
      // Pretend this plane was requested by client
      plane.second->SetRequestedCrtc(crtc_id);
      const uint32_t plane_id = plane.first;
      DRM_LOGD("Plane %d: Retaining on CRTC %d", plane_id, crtc_id);
    }
  }
}

void DRMPlaneManager::PostValidate(uint32_t crtc_id, bool success) {
  lock_guard<mutex> lock(lock_);
  for (auto &plane : plane_pool_) {
    plane.second->PostValidate(crtc_id, success);
  }
}

void DRMPlaneManager::PostCommit(uint32_t crtc_id, bool success) {
  lock_guard<mutex> lock(lock_);
  DRM_LOGD("crtc %d", crtc_id);
  for (auto &plane : plane_pool_) {
    plane.second->PostCommit(crtc_id, success);
  }
}

void DRMPlaneManager::SetScalerLUT(const DRMScalerLUTInfo &lut_info) {
  if (lut_info.dir_lut_size) {
    drmModeCreatePropertyBlob(fd_, reinterpret_cast<void *>(lut_info.dir_lut),
                              lut_info.dir_lut_size, &dir_lut_blob_id_);
  }
  if (lut_info.cir_lut_size) {
    drmModeCreatePropertyBlob(fd_, reinterpret_cast<void *>(lut_info.cir_lut),
                              lut_info.cir_lut_size, &cir_lut_blob_id_);
  }
  if (lut_info.sep_lut_size) {
    drmModeCreatePropertyBlob(fd_, reinterpret_cast<void *>(lut_info.sep_lut),
                              lut_info.sep_lut_size, &sep_lut_blob_id_);
  }
}

void DRMPlaneManager::UnsetScalerLUT() {
  if (dir_lut_blob_id_) {
    drmModeDestroyPropertyBlob(fd_, dir_lut_blob_id_);
    dir_lut_blob_id_ = 0;
  }
  if (cir_lut_blob_id_) {
    drmModeDestroyPropertyBlob(fd_, cir_lut_blob_id_);
    cir_lut_blob_id_ = 0;
  }
  if (sep_lut_blob_id_) {
    drmModeDestroyPropertyBlob(fd_, sep_lut_blob_id_);
    sep_lut_blob_id_ = 0;
  }
}

// ==============================================================================================//

#undef __CLASS__
#define __CLASS__ "DRMPlane"

DRMPlane::DRMPlane(int fd, uint32_t priority) : fd_(fd), priority_(priority) {}

DRMPlane::~DRMPlane() {
  drmModeFreePlane(drm_plane_);
}

void DRMPlane::GetTypeInfo(const PropertyMap &prop_map) {
  uint64_t blob_id = 0;
  drmModePropertyRes *prop = nullptr;
  DRMPlaneTypeInfo *info = &plane_type_info_;
  // Ideally we should check if this property type is a blob and then proceed.
  std::tie(blob_id, prop) = prop_map.at(DRMProperty::CAPABILITIES);
  drmModePropertyBlobRes *blob = drmModeGetPropertyBlob(fd_, blob_id);
  if (!blob) {
    return;
  }

  if (!blob->data) {
    return;
  }

  char *fmt_str = new char[blob->length + 1];
  memcpy (fmt_str, blob->data, blob->length);
  fmt_str[blob->length] = '\0';

  info->max_linewidth = 2560;
  info->max_scaler_linewidth = MAX_SCALER_LINEWIDTH;
  info->max_upscale = 1;
  info->max_downscale = 1;
  info->max_horizontal_deci = 0;
  info->max_vertical_deci = 0;
  info->master_plane_id = 0;
  if (info->type == DRMPlaneType::CURSOR) {
    info->max_linewidth = 128;
  }
  // TODO(user): change default to V2 once we start getting V3 via capabilities blob
  info->qseed3_version = QSEEDStepVersion::V3;
  info->has_excl_rect = has_excl_rect_;

  // We may have multiple lines with each one dedicated for something specific
  // like formats etc
  stringstream stream(fmt_str);
  DRM_LOGI("stream str %s len %zu blob str %s len %d", stream.str().c_str(), stream.str().length(),
           static_cast<const char *>(blob->data), blob->length);

  string line = {};
  string pixel_formats = "pixel_formats=";
  string max_linewidth = "max_linewidth=";
  string max_upscale = "max_upscale=";
  string max_downscale = "max_downscale=";
  string max_horizontal_deci = "max_horizontal_deci=";
  string max_vertical_deci = "max_vertical_deci=";
  string master_plane_id = "primary_smart_plane_id=";
  string max_pipe_bw = "max_per_pipe_bw=";
  string max_pipe_bw_high = "max_per_pipe_bw_high=";
  string scaler_version = "scaler_step_ver=";
  string block_sec_ui = "block_sec_ui=";
  string true_inline_rot_rev = "true_inline_rot_rev=";
  string inline_rot_pixel_formats = "inline_rot_pixel_formats=";
  string true_inline_dwnscale_rt_numerator = "true_inline_dwnscale_rt_numerator=";
  string true_inline_dwnscale_rt_denominator = "true_inline_dwnscale_rt_denominator=";
  string true_inline_max_height = "true_inline_max_height=";

  while (std::getline(stream, line)) {
    if (line.find(inline_rot_pixel_formats) != string::npos) {
      vector<pair<uint32_t, uint64_t>> inrot_formats_supported;
      ParseFormats(line.erase(0, inline_rot_pixel_formats.length()), &inrot_formats_supported);
      info->inrot_fmts_supported = std::move(inrot_formats_supported);
    } else if (line.find(pixel_formats) != string::npos) {
      vector<pair<uint32_t, uint64_t>> formats_supported;
      ParseFormats(line.erase(0, pixel_formats.length()), &formats_supported);
      info->formats_supported = std::move(formats_supported);
    } else if (line.find(max_linewidth) != string::npos) {
      info->max_linewidth = std::stoi(line.erase(0, max_linewidth.length()));
    } else if (line.find(max_upscale) != string::npos) {
      info->max_upscale = std::stoi(line.erase(0, max_upscale.length()));
    } else if (line.find(max_downscale) != string::npos) {
      info->max_downscale = std::stoi(line.erase(0, max_downscale.length()));
    } else if (line.find(max_horizontal_deci) != string::npos) {
      info->max_horizontal_deci = std::stoi(line.erase(0, max_horizontal_deci.length()));
    } else if (line.find(max_vertical_deci) != string::npos) {
      info->max_vertical_deci = std::stoi(line.erase(0, max_vertical_deci.length()));
    } else if (line.find(master_plane_id) != string::npos) {
      info->master_plane_id = std::stoi(line.erase(0, master_plane_id.length()));
      DRM_LOGI("info->master_plane_id: detected master_plane=%d", info->master_plane_id);
    } else if (line.find(max_pipe_bw) != string::npos) {
      info->max_pipe_bandwidth = std::stoull(line.erase(0, max_pipe_bw.length()));
    } else if (line.find(max_pipe_bw_high) != string::npos) {
      info->max_pipe_bandwidth_high = std::stoull(line.erase(0, max_pipe_bw_high.length()));
    } else if (line.find(scaler_version) != string::npos) {
      info->qseed3_version =
        PopulateQseedStepVersion(std::stoi(line.erase(0, scaler_version.length())));
    } else if (line.find(block_sec_ui) != string::npos) {
      info->block_sec_ui = !!(std::stoi(line.erase(0, block_sec_ui.length())));
    } else if (line.find(true_inline_rot_rev) != string::npos) {
      info->inrot_version =
        PopulateInlineRotationVersion(std::stoi(line.erase(0, true_inline_rot_rev.length())));
    } else if (line.find(true_inline_dwnscale_rt_numerator) != string::npos) {
      info->true_inline_dwnscale_rt_num = std::stof(line.erase(0,
        true_inline_dwnscale_rt_numerator.length()));
    } else if (line.find(true_inline_dwnscale_rt_denominator) != string::npos) {
      info->true_inline_dwnscale_rt_denom = std::stof(line.erase(0,
        true_inline_dwnscale_rt_denominator.length()));
    } else if (line.find(true_inline_max_height) != string::npos) {
      info->max_rotation_linewidth = std::stoi(line.erase(0, true_inline_max_height.length()));
    }
  }

// TODO(user): Get max_scaler_linewidth and non_scaler_linewidth from driver
// max_linewidth can be smaller than 2560 for few target, so make sure to assign the minimum of both
  info->max_scaler_linewidth = (info->qseed3_version < QSEEDStepVersion::V4) ? info->max_linewidth :
                               std::min((uint32_t)MAX_SCALER_LINEWIDTH, info->max_linewidth);

  drmModeFreePropertyBlob(blob);
  delete[] fmt_str;
}

void DRMPlane::ParseProperties() {
  // Map of property name to current value and property info pointer
  PropertyMap prop_map;
  bool csc = false;
  bool scaler = false;
  bool cursor = false;
  drmModeObjectProperties *props =
      drmModeObjectGetProperties(fd_, drm_plane_->plane_id, DRM_MODE_OBJECT_PLANE);
  if (!props || !props->props || !props->prop_values) {
    drmModeFreeObjectProperties(props);
    return;
  }

  for (uint32_t j = 0; j < props->count_props; j++) {
    drmModePropertyRes *info = drmModeGetProperty(fd_, props->props[j]);
    if (!info) {
      continue;
    }

    string property_name(info->name);
    DRMProperty prop_enum = prop_mgr_.GetPropertyEnum(property_name);
    if (prop_enum == DRMProperty::INVALID) {
      DRM_LOGD("DRMProperty %s missing from global property mapping", info->name);
      drmModeFreeProperty(info);
      continue;
    }

    if (prop_enum == DRMProperty::EXCL_RECT) {
      has_excl_rect_ = true;
    }
    if (prop_enum == DRMProperty::ROTATION) {
      PopulateReflect(info);
    } else if (prop_enum == DRMProperty::FB_TRANSLATION_MODE) {
      PopulateSecureModes(info);
    } else if (prop_enum == DRMProperty::MULTIRECT_MODE) {
      PopulateMultiRectModes(info);
      plane_type_info_.multirect_prop_present = true;
    }

    prop_mgr_.SetPropertyId(prop_enum, info->prop_id);
    prop_map[prop_enum] = std::make_tuple(props->prop_values[j], info);
    csc = prop_enum == DRMProperty::CSC_V1 ? true : csc;
    scaler = (prop_enum == DRMProperty::SCALER_V1 || prop_enum == DRMProperty::SCALER_V2) \
      ? true : scaler;
    cursor = (prop_enum == DRMProperty::TYPE && props->prop_values[j] == DRM_PLANE_TYPE_CURSOR) \
      ? true : cursor;

    // Tone mapping properties.
    if (prop_enum == DRMProperty::INVERSE_PMA) {
      plane_type_info_.inverse_pma = true;
    }

    if ((uint32_t)prop_enum >= (uint32_t)DRMProperty::CSC_DMA_V1 &&
        (uint32_t)prop_enum <= (uint32_t)DRMProperty::CSC_DMA_V1) {
      plane_type_info_.dgm_csc_version =
          ((uint32_t)prop_enum - (uint32_t)DRMProperty::CSC_DMA_V1 + 1);
    }

    if ((uint32_t)prop_enum >= (uint32_t)DRMProperty::SDE_DGM_1D_LUT_IGC_V5 &&
        (uint32_t)prop_enum <= (uint32_t)DRMProperty::SDE_DGM_1D_LUT_IGC_V5) {
      plane_type_info_.tonemap_lut_version_map[DRMTonemapLutType::DMA_1D_IGC] =
          ((uint32_t)prop_enum - (uint32_t)DRMProperty::SDE_DGM_1D_LUT_IGC_V5 + 5);
    }
    if ((uint32_t)prop_enum >= (uint32_t)DRMProperty::SDE_DGM_1D_LUT_GC_V5 &&
        (uint32_t)prop_enum <= (uint32_t)DRMProperty::SDE_DGM_1D_LUT_GC_V5) {
     plane_type_info_.tonemap_lut_version_map[DRMTonemapLutType::DMA_1D_GC] =
         ((uint32_t)prop_enum - (uint32_t)DRMProperty::SDE_DGM_1D_LUT_GC_V5 + 5);
    }
    if ((uint32_t)prop_enum >= (uint32_t)DRMProperty::SDE_VIG_1D_LUT_IGC_V5 &&
        (uint32_t)prop_enum <= (uint32_t)DRMProperty::SDE_VIG_1D_LUT_IGC_V6) {
      plane_type_info_.tonemap_lut_version_map[DRMTonemapLutType::VIG_1D_IGC] =
          ((uint32_t)prop_enum - (uint32_t)DRMProperty::SDE_VIG_1D_LUT_IGC_V5 + 5);
    }
    if ((uint32_t)prop_enum >= (uint32_t)DRMProperty::SDE_VIG_3D_LUT_GAMUT_V5 &&
        (uint32_t)prop_enum <= (uint32_t)DRMProperty::SDE_VIG_3D_LUT_GAMUT_V6) {
      plane_type_info_.tonemap_lut_version_map[DRMTonemapLutType::VIG_3D_GAMUT] =
          ((uint32_t)prop_enum - (uint32_t)DRMProperty::SDE_VIG_3D_LUT_GAMUT_V5 + 5);
    }
  }

  DRMPlaneType type = DRMPlaneType::DMA;
  if (csc && scaler) {
    type = DRMPlaneType::VIG;
  } else if (cursor) {
    type = DRMPlaneType::CURSOR;
  }

  plane_type_info_.type = type;
  GetTypeInfo(prop_map);

  for (auto &prop : prop_map) {
    drmModeFreeProperty(std::get<1>(prop.second));
  }

  drmModeFreeObjectProperties(props);
}

void DRMPlane::InitAndParse(drmModePlane *plane) {
  drm_plane_ = plane;
  ParseProperties();

  unique_ptr<DRMPPManager> pp_mgr(new DRMPPManager(fd_));
  pp_mgr_ = std::move(pp_mgr);
  pp_mgr_->Init(prop_mgr_, DRM_MODE_OBJECT_PLANE);
}

bool DRMPlane::ConfigureScalerLUT(drmModeAtomicReq *req, uint32_t dir_lut_blob_id,
                                  uint32_t cir_lut_blob_id, uint32_t sep_lut_blob_id) {
  if (plane_type_info_.type != DRMPlaneType::VIG || is_lut_configured_) {
    return false;
  }

  if (dir_lut_blob_id) {
    AddProperty(req, drm_plane_->plane_id,
                prop_mgr_.GetPropertyId(DRMProperty::LUT_ED),
                dir_lut_blob_id, false /* cache */, tmp_prop_val_map_);
  }
  if (cir_lut_blob_id) {
    AddProperty(req, drm_plane_->plane_id,
                prop_mgr_.GetPropertyId(DRMProperty::LUT_CIR),
                cir_lut_blob_id, false /* cache */, tmp_prop_val_map_);
  }
  if (sep_lut_blob_id) {
    AddProperty(req, drm_plane_->plane_id,
                prop_mgr_.GetPropertyId(DRMProperty::LUT_SEP),
                sep_lut_blob_id, false /* cache */, tmp_prop_val_map_);
  }

  return true;
}

void DRMPlane::SetExclRect(drmModeAtomicReq *req, DRMRect rect) {
  auto prop_id = prop_mgr_.GetPropertyId(DRMProperty::EXCL_RECT);
  drm_clip_rect clip_rect;
  SetRect(rect, &clip_rect);
  excl_rect_copy_ = clip_rect;
  AddProperty(req, drm_plane_->plane_id, prop_id, reinterpret_cast<uint64_t>
              (&excl_rect_copy_), false /* cache */, tmp_prop_val_map_);
  DRM_LOGD("Plane %d: Setting exclusion rect [x,y,w,h][%d,%d,%d,%d]", drm_plane_->plane_id,
           clip_rect.x1, clip_rect.y1, (clip_rect.x2 - clip_rect.x1),
           (clip_rect.y2 - clip_rect.y1));
}

bool DRMPlane::SetCscConfig(drmModeAtomicReq *req, DRMCscType csc_type) {
  if (plane_type_info_.type != DRMPlaneType::VIG) {
    return false;
  }

  if (csc_type > kCscTypeMax) {
    return false;
  }

  if (!prop_mgr_.IsPropertyAvailable(DRMProperty::CSC_V1)) {
    return false;
  }

  auto prop_id = prop_mgr_.GetPropertyId(DRMProperty::CSC_V1);
  if (csc_type == kCscTypeMax) {
    AddProperty(req, drm_plane_->plane_id, prop_id, 0, false /* cache */, tmp_prop_val_map_);
  } else {
    csc_config_copy_ = csc_10bit_convert[csc_type];
    AddProperty(req, drm_plane_->plane_id, prop_id,
                reinterpret_cast<uint64_t>(&csc_config_copy_), false /* cache */,
                tmp_prop_val_map_);
  }

  return true;
}

bool DRMPlane::SetScalerConfig(drmModeAtomicReq *req, uint64_t handle) {
  if (plane_type_info_.type != DRMPlaneType::VIG) {
    return false;
  }

  if (prop_mgr_.IsPropertyAvailable(DRMProperty::SCALER_V2)) {
    auto prop_id = prop_mgr_.GetPropertyId(DRMProperty::SCALER_V2);
    sde_drm_scaler_v2 *scaler_v2_config = reinterpret_cast<sde_drm_scaler_v2 *>(handle);
    uint64_t scaler_data = 0;
    // The address needs to be valid even after async commit, since we are sending address to
    // driver directly, instead of blob. So we need to copy over contents that client sent. Client
    // may have sent an address of object on stack which will be released after this call.
    scaler_v2_config_copy_ = *scaler_v2_config;
    if (scaler_v2_config_copy_.enable) {
      scaler_data = reinterpret_cast<uint64_t>(&scaler_v2_config_copy_);
    }
    AddProperty(req, drm_plane_->plane_id, prop_id, scaler_data, false /* cache */,
                tmp_prop_val_map_);
    return true;
  }

  return false;
}

void DRMPlane::SetDecimation(drmModeAtomicReq *req, uint32_t prop_id, uint32_t prop_value) {
  if (plane_type_info_.type == DRMPlaneType::DMA || plane_type_info_.master_plane_id) {
    // if value is 0, client is just trying to clear previous decimation, so bail out silently
    if (prop_value > 0) {
      DRM_LOGE("Plane %d: Setting decimation %d is not supported.", drm_plane_->plane_id,
               prop_value);
    }
    return;
  }

  // TODO(user): Currently a ViG plane in smart DMA mode could receive a non-zero decimation value
  // but there is no good way to catch. In any case fix will be in client
  AddProperty(req, drm_plane_->plane_id, prop_id, prop_value, true /* cache */, tmp_prop_val_map_);
  DRM_LOGD("Plane %d: Setting decimation %d", drm_plane_->plane_id, prop_value);
}

void DRMPlane::PostValidate(uint32_t crtc_id, bool /*success*/) {
  if (requested_crtc_id_ == crtc_id) {
    SetRequestedCrtc(0);
    tmp_prop_val_map_ = committed_prop_val_map_;
  }
}

void DRMPlane::PostCommit(uint32_t crtc_id, bool success) {
  DRM_LOGD("crtc %d", crtc_id);
  if (!success) {
    // To reset
    PostValidate(crtc_id, success);
    return;
  }

  uint32_t assigned_crtc = 0;
  uint32_t requested_crtc = 0;

  GetAssignedCrtc(&assigned_crtc);
  GetRequestedCrtc(&requested_crtc);

  // In future, it is possible that plane is already attached in case of continuous splash. This
  // will cause the first commit to only unstage pipes. We want to mark luts as configured only
  // when they really are, which typically happens if a crtc is requested for a plane
  if (requested_crtc == crtc_id && !is_lut_configured_) {
    is_lut_configured_ = true;
  }

  if (requested_crtc && assigned_crtc && requested_crtc != assigned_crtc) {
    // We should never be here
    DRM_LOGE("Found plane %d switching from crtc %d to crtc %d", drm_plane_->plane_id,
             assigned_crtc, requested_crtc);
  }

  // If we have set a pipe OR unset a pipe during commit, update states
  if (requested_crtc == crtc_id || assigned_crtc == crtc_id) {
    committed_prop_val_map_ = tmp_prop_val_map_;
    SetAssignedCrtc(requested_crtc);
    SetRequestedCrtc(0);
  }
}

void DRMPlane::Perform(DRMOps code, drmModeAtomicReq *req, va_list args) {
  uint32_t prop_id = 0;
  uint32_t obj_id = drm_plane_->plane_id;

  switch (code) {
    // TODO(user): Check if these exist in map before attempting to access
    case DRMOps::PLANE_SET_SRC_RECT: {
      DRMRect rect = va_arg(args, DRMRect);
      // source co-ordinates accepted by DRM are 16.16 fixed point
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::SRC_X);
      AddProperty(req, obj_id, prop_id, rect.left << 16, true /* cache */, tmp_prop_val_map_);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::SRC_Y);
      AddProperty(req, obj_id, prop_id, rect.top << 16, true /* cache */, tmp_prop_val_map_);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::SRC_W);
      AddProperty(req, obj_id, prop_id, (rect.right - rect.left) << 16, true /* cache */,
                  tmp_prop_val_map_);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::SRC_H);
      AddProperty(req, obj_id, prop_id, (rect.bottom - rect.top) << 16, true /* cache */,
                  tmp_prop_val_map_);
      DRM_LOGV("Plane %d: Setting crop [x,y,w,h][%d,%d,%d,%d]", obj_id, rect.left,
               rect.top, (rect.right - rect.left), (rect.bottom - rect.top));
    } break;

    case DRMOps::PLANE_SET_DST_RECT: {
      DRMRect rect = va_arg(args, DRMRect);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::CRTC_X);
      AddProperty(req, obj_id, prop_id, rect.left, true /* cache */, tmp_prop_val_map_);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::CRTC_Y);
      AddProperty(req, obj_id, prop_id, rect.top, true /* cache */, tmp_prop_val_map_);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::CRTC_W);
      AddProperty(req, obj_id, prop_id, (rect.right - rect.left), true /* cache */,
                  tmp_prop_val_map_);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::CRTC_H);
      AddProperty(req, obj_id, prop_id, (rect.bottom - rect.top), true /* cache */,
                  tmp_prop_val_map_);
      DRM_LOGV("Plane %d: Setting dst [x,y,w,h][%d,%d,%d,%d]", obj_id, rect.left,
               rect.top, (rect.right - rect.left), (rect.bottom - rect.top));
    } break;
    case DRMOps::PLANE_SET_EXCL_RECT: {
      DRMRect excl_rect = va_arg(args, DRMRect);
      SetExclRect(req, excl_rect);
    } break;

    case DRMOps::PLANE_SET_ZORDER: {
      uint32_t zpos = va_arg(args, uint32_t);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::ZPOS);
      AddProperty(req, obj_id, prop_id, zpos, true /* cache */, tmp_prop_val_map_);
      DRM_LOGD("Plane %d: Setting z %d", obj_id, zpos);
    } break;

    case DRMOps::PLANE_SET_ROTATION: {
      uint32_t rot_bit_mask = va_arg(args, uint32_t);
      uint32_t drm_rot_bit_mask = 0;
      if (rot_bit_mask & static_cast<uint32_t>(DRMRotation::FLIP_H)) {
        drm_rot_bit_mask |= 1 << REFLECT_X;
      }
      if (rot_bit_mask & static_cast<uint32_t>(DRMRotation::FLIP_V)) {
        drm_rot_bit_mask |= 1 << REFLECT_Y;
      }
      if (rot_bit_mask & static_cast<uint32_t>(DRMRotation::ROT_90)) {
        drm_rot_bit_mask |= 1 << ROTATE_90;
      } else {
        drm_rot_bit_mask |= 1 << ROTATE_0;
      }
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::ROTATION);
      AddProperty(req, obj_id, prop_id, drm_rot_bit_mask, true /* cache */, tmp_prop_val_map_);
      DRM_LOGV("Plane %d: Setting rotation mask %x", obj_id, drm_rot_bit_mask);
    } break;

    case DRMOps::PLANE_SET_ALPHA: {
      uint32_t alpha = va_arg(args, uint32_t);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::ALPHA);
      AddProperty(req, obj_id, prop_id, alpha, true /* cache */, tmp_prop_val_map_);
      DRM_LOGV("Plane %d: Setting alpha %d", obj_id, alpha);
    } break;

    case DRMOps::PLANE_SET_BLEND_TYPE: {
      uint32_t blending = va_arg(args, uint32_t);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::BLEND_OP);
      AddProperty(req, obj_id, prop_id, blending, true /* cache */, tmp_prop_val_map_);
      DRM_LOGV("Plane %d: Setting blending %d", obj_id, blending);
    } break;

    case DRMOps::PLANE_SET_H_DECIMATION: {
      uint32_t deci = va_arg(args, uint32_t);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::H_DECIMATE);
      SetDecimation(req, prop_id, deci);
    } break;

    case DRMOps::PLANE_SET_V_DECIMATION: {
      uint32_t deci = va_arg(args, uint32_t);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::V_DECIMATE);
      SetDecimation(req, prop_id, deci);
    } break;

    case DRMOps::PLANE_SET_SRC_CONFIG: {
      bool src_config = va_arg(args, uint32_t);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::SRC_CONFIG);
      AddProperty(req, obj_id, prop_id, src_config, true /* cache */, tmp_prop_val_map_);
      DRM_LOGV("Plane %d: Setting src_config flags-%x", obj_id, src_config);
    } break;

    case DRMOps::PLANE_SET_CRTC: {
      uint32_t crtc_id = va_arg(args, uint32_t);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::CRTC_ID);
      AddProperty(req, obj_id, prop_id, crtc_id, true /* cache */, tmp_prop_val_map_);
      SetRequestedCrtc(crtc_id);
      DRM_LOGV("Plane %d: Setting crtc %d", obj_id, crtc_id);
    } break;

    case DRMOps::PLANE_SET_FB_ID: {
      uint32_t fb_id = va_arg(args, uint32_t);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::FB_ID);
      AddProperty(req, obj_id, prop_id, fb_id, true /* cache */, tmp_prop_val_map_);
      DRM_LOGV("Plane %d: Setting fb_id %d", obj_id, fb_id);
    } break;

    case DRMOps::PLANE_SET_ROT_FB_ID: {
      uint32_t fb_id = va_arg(args, uint32_t);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::ROT_FB_ID);
      drmModeAtomicAddProperty(req, obj_id, prop_id, fb_id);
      DRM_LOGV("Plane %d: Setting rot_fb_id %d", obj_id, fb_id);
    } break;

    case DRMOps::PLANE_SET_INPUT_FENCE: {
      int fence = va_arg(args, int);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::INPUT_FENCE);
      AddProperty(req, obj_id, prop_id, fence, false /* cache */, tmp_prop_val_map_);
      DRM_LOGV("Plane %d: Setting input fence %d", obj_id, fence);
    } break;

    case DRMOps::PLANE_SET_SCALER_CONFIG: {
      uint64_t handle = va_arg(args, uint64_t);
      if (SetScalerConfig(req, handle)) {
        DRM_LOGV("Plane %d: Setting scaler config", obj_id);
      }
    } break;

    case DRMOps::PLANE_SET_FB_SECURE_MODE: {
      int secure_mode = va_arg(args, int);
      uint32_t fb_secure_mode = NON_SECURE;
      switch (secure_mode) {
        case (int)DRMSecureMode::NON_SECURE:
          fb_secure_mode = NON_SECURE;
          break;
        case (int)DRMSecureMode::SECURE:
          fb_secure_mode = SECURE;
          break;
        case (int)DRMSecureMode::NON_SECURE_DIR_TRANSLATION:
          fb_secure_mode = NON_SECURE_DIR_TRANSLATION;
          break;
        case (int)DRMSecureMode::SECURE_DIR_TRANSLATION:
          fb_secure_mode = SECURE_DIR_TRANSLATION;
          break;
        default:
          DRM_LOGE("Invalid secure mode %d to set on plane %d", secure_mode, obj_id);
          break;
      }

      prop_id = prop_mgr_.GetPropertyId(DRMProperty::FB_TRANSLATION_MODE);
      AddProperty(req, obj_id, prop_id, fb_secure_mode, true /* cache */, tmp_prop_val_map_);
      DRM_LOGD("Plane %d: Setting FB secure mode %d", obj_id, fb_secure_mode);
    } break;

    case DRMOps::PLANE_SET_CSC_CONFIG: {
      uint32_t* csc_type = va_arg(args, uint32_t*);
      if (csc_type) {
        SetCscConfig(req, (DRMCscType)*csc_type);
      }
    } break;

    case DRMOps::PLANE_SET_MULTIRECT_MODE: {
      DRMMultiRectMode drm_multirect_mode = (DRMMultiRectMode)va_arg(args, uint32_t);
      SetMultiRectMode(req, drm_multirect_mode);
    } break;

    case DRMOps::PLANE_SET_INVERSE_PMA: {
       uint32_t pma = va_arg(args, uint32_t);
       prop_id = prop_mgr_.GetPropertyId(DRMProperty::INVERSE_PMA);
       AddProperty(req, obj_id, prop_id, pma, true /* cache */, tmp_prop_val_map_);
       DRM_LOGD("Plane %d: %s inverse pma", obj_id, pma ? "Setting" : "Resetting");
     } break;

    case DRMOps::PLANE_SET_DGM_CSC_CONFIG: {
      uint64_t handle = va_arg(args, uint64_t);
      if (SetDgmCscConfig(req, handle)) {
        DRM_LOGD("Plane %d: Setting Csc Lut config", obj_id);
      }
    } break;

    case DRMOps::PLANE_SET_POST_PROC: {
      DRMPPFeatureInfo *data = va_arg(args, DRMPPFeatureInfo*);
      if (data) {
        DRM_LOGD("Plane %d: Set post proc feature id - %d", obj_id, data->id);
        pp_mgr_->SetPPFeature(req, obj_id, *data);
        UpdatePPLutFeatureInuse(data);
      }
    } break;

    case DRMOps::PLANE_SET_SSPP_LAYOUT: {
      if (!prop_mgr_.IsPropertyAvailable(DRMProperty::SDE_SSPP_LAYOUT)) {
        DRM_LOGD("SSPP_LAYOUT property isn't exposed");
        break;
      }
      DRMSSPPLayoutIndex layout_index = (DRMSSPPLayoutIndex) va_arg(args, uint32_t);
      prop_id = prop_mgr_.GetPropertyId(DRMProperty::SDE_SSPP_LAYOUT);
      AddProperty(req, obj_id, prop_id, (uint32_t)layout_index , true /* cache */,
                  tmp_prop_val_map_);
      DRM_LOGD("Plane %d: Setting SSPP Layout to %d", obj_id, layout_index);
    } break;

    default:
      DRM_LOGE("Invalid opcode %d for DRM Plane %d", code, obj_id);
  }
}

void DRMPlane::UpdatePPLutFeatureInuse(DRMPPFeatureInfo *data) {
  DRMTonemapLutType lut_type = {};
  bool ret = GetDRMonemapLutTypeFromPPFeatureID(data->id, &lut_type);
  if (ret == false) {
    DRM_LOGE("Failed to get the lut type from PPFeatureID = %d", data->id);
    return;
  }

  const auto state = data->payload ? kActive : kInactive;

  switch (lut_type) {
    case DRMTonemapLutType::DMA_1D_GC:
      dgm_1d_lut_gc_state_ = state;
      break;
    case DRMTonemapLutType::DMA_1D_IGC:
      dgm_1d_lut_igc_state_ = state;
      break;
    case DRMTonemapLutType::VIG_1D_IGC:
      vig_1d_lut_igc_state_ = state;
      break;
    case DRMTonemapLutType::VIG_3D_GAMUT:
      vig_3d_lut_gamut_state_ = state;
      break;
    default:
      DRM_LOGE("Invalid lut_type = %d state = %d", lut_type, state);
  }
  return;
}

void DRMPlane::PerformWrapper(DRMOps code, drmModeAtomicReq *req, ...) {
  va_list args;
  va_start(args, req);
  Perform(code, req, args);
  va_end(args);
}

void DRMPlane::Dump() {
  DRM_LOGE(
      "id: %d\tcrtc id: %d\tfb id: %d\tCRTC_xy: %dx%d\txy: %dx%d\tgamma "
      "size: %d\tpossible crtc: 0x%x\n",
      drm_plane_->plane_id, drm_plane_->crtc_id, drm_plane_->fb_id, drm_plane_->crtc_x,
      drm_plane_->crtc_y, drm_plane_->x, drm_plane_->y, drm_plane_->gamma_size,
      drm_plane_->possible_crtcs);
  DRM_LOGE("Format Suported: \n");
  for (uint32_t i = 0; i < (uint32_t)drm_plane_->count_formats; i++)
    DRM_LOGE(" %4.4s", (char *)&drm_plane_->formats[i]);
}

void DRMPlane::SetMultiRectMode(drmModeAtomicReq *req, DRMMultiRectMode drm_multirect_mode) {
    if (!plane_type_info_.multirect_prop_present) {
      return;
    }
    uint32_t obj_id = drm_plane_->plane_id;
    uint32_t multirect_mode = MULTIRECT_NONE;
    switch (drm_multirect_mode) {
      case DRMMultiRectMode::NONE:
        multirect_mode = MULTIRECT_NONE;
        break;
      case DRMMultiRectMode::PARALLEL:
        multirect_mode = MULTIRECT_PARALLEL;
        break;
      case DRMMultiRectMode::SERIAL:
        multirect_mode = MULTIRECT_SERIAL;
        break;
      default:
        DRM_LOGE("Invalid multirect mode %d to set on plane %d", drm_multirect_mode, obj_id);
        break;
    }
    auto prop_id = prop_mgr_.GetPropertyId(DRMProperty::MULTIRECT_MODE);
    AddProperty(req, obj_id, prop_id, multirect_mode, true /* cache */, tmp_prop_val_map_);
    DRM_LOGD("Plane %d: Setting multirect_mode %d", obj_id, multirect_mode);
}

void DRMPlane::Unset(bool is_commit, drmModeAtomicReq *req) {
  DRM_LOGD("Plane %d: Unsetting from crtc %d", drm_plane_->plane_id, assigned_crtc_id_);
  PerformWrapper(DRMOps::PLANE_SET_FB_ID, req, 0);
  PerformWrapper(DRMOps::PLANE_SET_CRTC, req, 0);
  DRMRect rect = {0, 0, 0, 0};
  PerformWrapper(DRMOps::PLANE_SET_SRC_RECT, req, rect);
  PerformWrapper(DRMOps::PLANE_SET_DST_RECT, req, rect);
  PerformWrapper(DRMOps::PLANE_SET_EXCL_RECT, req, rect);
  if (plane_type_info_.inverse_pma) {
    PerformWrapper(DRMOps::PLANE_SET_INVERSE_PMA, req, 0);
  }

  // Reset the sspp tonemap properties if they were set and update the in-use only if
  // its a Commit as Unset is called in Validate as well.
  if (dgm_csc_in_use_) {
    auto prop_id = prop_mgr_.GetPropertyId(DRMProperty::CSC_DMA_V1);
    uint64_t csc_v1 = 0;
    AddProperty(req, drm_plane_->plane_id, prop_id, csc_v1, false /* cache */, tmp_prop_val_map_);
    DRM_LOGV("Plane %d Clearing DGM CSC", drm_plane_->plane_id);
    dgm_csc_in_use_ = !is_commit;
  }
  ResetColorLUTs(is_commit, req);

  tmp_prop_val_map_.clear();
  committed_prop_val_map_.clear();
}

bool DRMPlane::SetDgmCscConfig(drmModeAtomicReq *req, uint64_t handle) {
  if (plane_type_info_.type == DRMPlaneType::DMA &&
      prop_mgr_.IsPropertyAvailable(DRMProperty::CSC_DMA_V1)) {
    auto prop_id = prop_mgr_.GetPropertyId(DRMProperty::CSC_DMA_V1);
    sde_drm_csc_v1 *csc_v1 = reinterpret_cast<sde_drm_csc_v1 *>(handle);
    uint64_t csc_v1_data = 0;
    sde_drm_csc_v1 csc_v1_tmp = {};
    csc_config_copy_ = *csc_v1;
    if (std::memcmp(&csc_config_copy_, &csc_v1_tmp, sizeof(sde_drm_csc_v1)) != 0) {
      csc_v1_data = reinterpret_cast<uint64_t>(&csc_config_copy_);
    }
    AddProperty(req, drm_plane_->plane_id, prop_id,
                reinterpret_cast<uint64_t>(csc_v1_data), false /* cache */,
                tmp_prop_val_map_);
    dgm_csc_in_use_ = (csc_v1_data != 0);
    DRM_LOGV("Plane %d in_use = %d", drm_plane_->plane_id, dgm_csc_in_use_);

    return true;
  }

  return false;
}

void DRMPlane::ResetColorLUTs(bool is_commit, drmModeAtomicReq *req) {
  // Reset the color luts if they were set and update the state only if its a Commit as Unset
  // is called in Validate as well.
  for (int i = 0; i <= (int32_t)(DRMTonemapLutType::VIG_3D_GAMUT); i++) {
    auto itr = plane_type_info_.tonemap_lut_version_map.find(static_cast<DRMTonemapLutType>(i));
    if (itr != plane_type_info_.tonemap_lut_version_map.end()) {
      ResetColorLUTState(static_cast<DRMTonemapLutType>(i), is_commit, req);
    }
  }
}

void DRMPlane::ResetColorLUTState(DRMTonemapLutType lut_type, bool is_commit,
                                  drmModeAtomicReq *req) {
  DRMPlaneLutState *lut_state = nullptr;
  DRMPPFeatureID feature_id = {};
  switch (lut_type) {
    case DRMTonemapLutType::DMA_1D_GC:
      lut_state = &dgm_1d_lut_gc_state_;
      feature_id = kFeatureDgmGc;
      break;
    case DRMTonemapLutType::DMA_1D_IGC:
      lut_state = &dgm_1d_lut_igc_state_;
      feature_id = kFeatureDgmIgc;
      break;
    case DRMTonemapLutType::VIG_1D_IGC:
      lut_state = &vig_1d_lut_igc_state_;
      feature_id = kFeatureVigIgc;
      break;
    case DRMTonemapLutType::VIG_3D_GAMUT:
      lut_state = &vig_3d_lut_gamut_state_;
      feature_id = kFeatureVigGamut;
      break;
    default:
      DLOGE("Invalid lut type = %d", lut_type);
      return;
  }

  if (*lut_state == kInactive) {
    DRM_LOGV("Plane %d %s Lut not used", drm_plane_->plane_id, GetColorLutString(lut_type));
    return;
  }

  DRMPlaneLutState target_state;
  // If plane is getting unset, clearing of LUT will not be applied in hw.
  // In that case, mark LUT as dirty and make sure that these are cleared the
  // next time the plane gets used
  if (*lut_state == kActive && requested_crtc_id_ == 0) {
    target_state = kDirty;
  } else if (*lut_state == kDirty && requested_crtc_id_ != 0) {
    // If plane is getting activated while LUT is in dirty state, the new state
    // should be inactive but still need to clear exiting LUT config in hw
    target_state = kInactive;
  } else {
    return;
  }

  if (is_commit) {
    DRM_LOGD("Plane %d Clearing %s Lut, moving from (%d) -> (%d)", drm_plane_->plane_id,
              GetColorLutString(lut_type), *lut_state, target_state);

    *lut_state = target_state;
  }

  ResetColorLUT(feature_id, req);
}

void DRMPlane::ResetColorLUT(DRMPPFeatureID id, drmModeAtomicReq *req) {
  DRMPPFeatureInfo pp_feature_info = {};
  pp_feature_info.type = kPropBlob;
  pp_feature_info.payload = nullptr;
  pp_feature_info.id = id;
  pp_mgr_->SetPPFeature(req, drm_plane_->plane_id, pp_feature_info);
}

}  // namespace sde_drm
