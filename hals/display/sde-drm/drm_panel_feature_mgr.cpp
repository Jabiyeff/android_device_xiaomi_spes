/* Copyright (c) 2021, The Linux Foundataion. All rights reserved.
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
*
*/

#include <sstream>
#include <errno.h>
#include <string>
#include <drm_logger.h>
#include <cstring>
#include <regex>
#include <inttypes.h>

#include "drm_panel_feature_mgr.h"

#define __CLASS__ "DRMPanelFeatureMgr"

namespace sde_drm {

using std::map;
using std::vector;
using std::mutex;
using std::lock_guard;

static DRMPanelFeatureMgr panel_feature_mgr;

DRMPanelFeatureMgrIntf *GetPanelFeatureManagerIntf() {
  return &panel_feature_mgr;
}

void DRMPanelFeatureMgr::Init(int fd, drmModeRes* res) {
  lock_guard<mutex> lock(lock_);

  if (!res || (fd < 0)) {
    DRM_LOGE("Invalid arguments for init - fd %d and DRM resources pointer 0x%p", fd, (void *)res);
    return;
  }

  drm_res_ = res;
  dev_fd_ = fd;

  for (int i = 0; i < res->count_crtcs; i++) {
    drmModeCrtc *crtc = drmModeGetCrtc(dev_fd_, res->crtcs[i]);
    if (crtc) {
      int err = InitObjectProps(crtc->crtc_id, DRM_MODE_OBJECT_CRTC);
      if (err) {
        DRM_LOGE("Failed to get crtc props %d", crtc->crtc_id);
      }
      drmModeFreeCrtc(crtc);
    }
  }

  for (int i = 0; i < res->count_connectors; i++) {
    drmModeConnector *conn = drmModeGetConnector(dev_fd_, res->connectors[i]);
    if (conn) {
      int err = InitObjectProps(conn->connector_id, DRM_MODE_OBJECT_CONNECTOR);
      if (err) {
        DRM_LOGE("Failed to get conn %d properties", conn->connector_id);
      }
      drmModeFreeConnector(conn);
    }
  }

  drm_property_map_[kDRMPanelFeatureDsppRCInfo] = DRMProperty::DSPP_CAPABILITIES;
  drm_property_map_[kDRMPanelFeatureRCInit] = DRMProperty::DSPP_RC_MASK_V1;
  drm_prop_type_map_[kDRMPanelFeatureRCInit] = DRMPropType::kPropBlob;
  drm_prop_type_map_[kDRMPanelFeatureDsppRCInfo] = DRMPropType::kPropRange;


  feature_info_tbl_[kDRMPanelFeatureRCInit] = DRMPanelFeatureInfo {
      kDRMPanelFeatureRCInit, DRM_MODE_OBJECT_CRTC, UINT32_MAX, 1, sizeof(msm_rc_mask_cfg), 0};

  feature_info_tbl_[kDRMPanelFeatureDsppRCInfo] = DRMPanelFeatureInfo {
    kDRMPanelFeatureDsppRCInfo, DRM_MODE_OBJECT_CRTC, UINT32_MAX, 1, 64, 0};
}

void DRMPanelFeatureMgr::DeInit() {
  int ret = 0;
  for (int i = 0; i < kDRMPanelFeatureMax; i++) {
    DRMPanelFeatureID prop_id = static_cast<DRMPanelFeatureID>(i);
    if (drm_prop_blob_ids_map_[prop_id]) {
      ret = drmModeDestroyPropertyBlob(dev_fd_, drm_prop_blob_ids_map_[prop_id]);
      if (ret) {
        DRM_LOGE("failed to destroy blob for feature %d, ret = %d", prop_id, ret);
        return;
      } else {
        drm_prop_blob_ids_map_[prop_id] = 0;
      }
    }
  }
}

int DRMPanelFeatureMgr::InitObjectProps(int obj_id, int obj_type) {
  if (dev_fd_ < 0 || obj_id < 0) {
    DRM_LOGE("Invalid dev_fd_ %d or crtc_id %d", dev_fd_, obj_id);
    return -EINVAL;
  }

  drmModeObjectProperties *props =
          drmModeObjectGetProperties(dev_fd_, obj_id, obj_type);
  if (!props || !props->props || !props->prop_values) {
    DRM_LOGE("Failed to get props for obj_id:%d obj_type:%d", obj_id, obj_type);
    drmModeFreeObjectProperties(props);
    return -EINVAL;
  }

  for (uint32_t j = 0; j < props->count_props; j++) {
    drmModePropertyRes *info = drmModeGetProperty(dev_fd_, props->props[j]);
    if (!info) {
      continue;
    }

    std::string property_name(info->name);
    DRMProperty prop_enum = prop_mgr_.GetPropertyEnum(property_name);
    if (prop_enum == DRMProperty::INVALID) {
      DRM_LOGD("DRMProperty %s missing from global property mapping", info->name);
      drmModeFreeProperty(info);
      continue;
    }

    prop_mgr_.SetPropertyId(prop_enum, info->prop_id);
    drmModeFreeProperty(info);
  }

  drmModeFreeObjectProperties(props);
  return 0;
}

void DRMPanelFeatureMgr::ParseDsppCapabilities(uint32_t blob_id, std::vector<int> *values,
                                            uint32_t *size, const std::string str) {
  drmModePropertyBlobRes *blob = drmModeGetPropertyBlob(dev_fd_, blob_id);
  if (!blob) {
    DRM_LOGW("Unable to find blob for id %d", blob_id);
    return;
  }

  if (!blob->data) {
    DRM_LOGW("Invalid blob - no data for for blob-id %d", blob_id);
    return;
  }

  char *fmt_str = new char[blob->length + 1];
  std::memcpy(fmt_str, blob->data, blob->length);
  fmt_str[blob->length] = '\0';
  std::stringstream stream(fmt_str);
  std::string line = {};
  // Search for panel feature property pattern. Which is defined as rc0=1, rc1=1
  const std::regex exp(str + "(\\d+)=1");
  std::smatch sm;
  while (std::getline(stream, line)) {
    std::regex_match(line, sm, exp);
    // smatch shall include full line as a match followed by the hw block # as a match
    if (sm.size() == 2) {
      std::string tmpstr(sm[1]);
      int temp = atoi(tmpstr.c_str());  // atoi safe to use due to regex success
      values->push_back(temp);
    }
  }

  *size = sizeof(int) * values->size();
  delete[] fmt_str;
}

void DRMPanelFeatureMgr::ParseCapabilities(uint32_t blob_id, char* value, uint32_t max_len,
                                           const std::string str) {
  drmModePropertyBlobRes *blob = drmModeGetPropertyBlob(dev_fd_, blob_id);
  if (!blob) {
    DRM_LOGW("Unable to find blob for id %d", blob_id);
    return;
  }

  if (!blob->data) {
    DRM_LOGW("Invalid blob - no data for for blob-id %d", blob_id);
    return;
  }

  char *fmt_str = new char[blob->length + 1];
  std::memcpy(fmt_str, blob->data, blob->length);
  fmt_str[blob->length] = '\0';
  std::stringstream stream(fmt_str);
  std::string line = {};
  std::string val = {};
  const std::string goal = str + "=";
  while (std::getline(stream, line)) {
    if (line.find(goal) != std::string::npos) {
      val = std::string(line, goal.length());
    }
  }

  if (max_len <= val.size()) {
    DRM_LOGW("Insufficient size max_len: %d actual size: %zu", max_len, val.size());
    return;
  }
  std::copy(val.begin(), val.end(), value);
  value[val.size()] = '\0';
  delete[] fmt_str;
}

void DRMPanelFeatureMgr::GetPanelFeatureInfo(DRMPanelFeatureInfo *info) {
  lock_guard<mutex> lock(lock_);

  if (!info) {
    DRM_LOGE("Invalid input, DRMPanelFeatureInfo is NULL");
    return;
  }

  if (info->prop_id > kDRMPanelFeatureMax) {
    DRM_LOGE("Invalid feature id %d", info->prop_id);
    return;
  }

  DRMProperty prop_enum = drm_property_map_[info->prop_id];
  if (!prop_mgr_.IsPropertyAvailable(prop_enum)) {
    DRM_LOGE("Property id is not available for DRMProperty: %d feature-id: %d",
             prop_enum, info->prop_id);
    return;
  }

  // memory is not allocated by client - populate default property info
  if (!info->prop_ptr) {
    *info = feature_info_tbl_[info->prop_id];
    return;
  }

  drmModeObjectProperties *props =
          drmModeObjectGetProperties(dev_fd_, info->obj_id, info->obj_type);
  if (!props || !props->props || !props->prop_values) {
    drmModeFreeObjectProperties(props);
    DRM_LOGE("Failed to Get properties for obj: %d type:%d", info->obj_id, info->obj_type);
    return;
  }

  for (uint32_t j = 0; j < props->count_props; j++) {
    drmModePropertyRes *property = drmModeGetProperty(dev_fd_, props->props[j]);
    if (!property) {
      continue;
    }

    std::string property_name(property->name);
    if (prop_enum != prop_mgr_.GetPropertyEnum(property_name)) {
      drmModeFreeProperty(property);
      continue;
    }
    else if (info->prop_id == kDRMPanelFeatureDsppRCInfo) {
      ParseDsppCapabilities(props->prop_values[j],
              reinterpret_cast<std::vector<int> *> (info->prop_ptr), &(info->prop_size), "rc");
    } else if (drm_prop_type_map_[info->prop_id] == DRMPropType::kPropBlob) {
      drmModePropertyBlobRes *blob = drmModeGetPropertyBlob(dev_fd_, props->prop_values[j]);
      if (!blob || !blob->data || !blob->length) {
        return;
      }
      uint8_t *src_begin = reinterpret_cast<uint8_t *> (blob->data);
      uint8_t *src_end = src_begin + blob->length;
      uint8_t *dst = reinterpret_cast<uint8_t *> (info->prop_ptr);
      std::copy(src_begin, src_end, dst);
    } else {
      uint8_t *src_begin = reinterpret_cast<uint8_t *> (props->prop_values[j]);
      uint8_t *src_end = src_begin + info->prop_size;
      uint8_t *dst = reinterpret_cast<uint8_t *> (info->prop_ptr);
      std::copy(src_begin, src_end, dst);
    }

    drmModeFreeProperty(property);
  }

  drmModeFreeObjectProperties(props);
}

void DRMPanelFeatureMgr::CachePanelFeature(const DRMPanelFeatureInfo &info) {
  lock_guard<mutex> lock(lock_);

  if (info.prop_id >= kDRMPanelFeatureMax || info.obj_id == UINT32_MAX) {
    DRM_LOGE("invalid property info to set id %d value ptr %" PRIu64 , info.prop_id, info.prop_ptr);
    return;
  }

  dirty_features_[info.prop_id].first = true;
  dirty_features_[info.prop_id].second = info;
}

void DRMPanelFeatureMgr::CommitPanelFeatures(drmModeAtomicReq *req, const DRMDisplayToken &tok) {
  int ret = 0;

  lock_guard<mutex> lock(lock_);
  for (auto it = dirty_features_.begin(); it != dirty_features_.end(); it++) {
    DRMPanelFeatureInfo &info = it->second;

    if (!it->first)
        continue;

    if (info.prop_id >= kDRMPanelFeatureMax) {
      DRM_LOGE("invalid property info to set id %d value ptr %" PRIu64 , info.prop_id, info.prop_ptr);
      continue;
    }

    // Commit only features meant for the given DisplayToken
    if (tok.crtc_id != info.obj_id && tok.conn_id != info.obj_id) {
      continue;
    }

    uint32_t prop_id = prop_mgr_.GetPropertyId(drm_property_map_[info.prop_id]);
    uint64_t value = 0;

    if (DRMPropType::kPropBlob == drm_prop_type_map_[info.prop_id]) {
      uint32_t blob_id = 0;
      if (!info.prop_ptr) {
        // Reset the feature.
        ret = drmModeAtomicAddProperty(req, info.obj_id, prop_id, 0);
        if (ret < 0) {
          DRM_LOGE("failed to add property ret:%d, obj_id:%d prop_id:%u value:%" PRIu64,
                    ret, info.obj_id, prop_id, value);
          return;
        }
        continue;
      }

      ret = drmModeCreatePropertyBlob(dev_fd_, reinterpret_cast<void *> (info.prop_ptr),
              info.prop_size, &blob_id);
      if (ret || blob_id == 0) {
        DRM_LOGE("failed to create blob ret %d, id = %d prop_ptr:%" PRIu64 " prop_sz:%d",
                ret, blob_id, info.prop_ptr, info.prop_size);
        return;
      }

      if (drm_prop_blob_ids_map_[info.prop_id]) {
        ret = drmModeDestroyPropertyBlob(dev_fd_, drm_prop_blob_ids_map_[info.prop_id]);
        if (ret) {
          DRM_LOGE("failed to destroy blob for feature %d, ret = %d", info.prop_id, ret);
          return;
        }
      }
      drm_prop_blob_ids_map_[info.prop_id] = blob_id;

      value = blob_id;
    } else if (info.prop_size == sizeof(uint64_t)) {
      value = (reinterpret_cast<uint64_t *> (info.prop_ptr))[0];
    } else {
      DRM_LOGE("Unsupported property type id = %d size:%d", info.prop_id, info.prop_size);
    }

    ret = drmModeAtomicAddProperty(req, info.obj_id, prop_id, value);
    if (ret < 0) {
      DRM_LOGE("failed to add property ret:%d, obj_id:%d prop_id:%x value:%" PRIu64,
                ret, info.obj_id, prop_id, value);
    }
    *it = {};
  }
}

}  // namespace sde_drm

