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
#include <drm/sde_drm.h>
#include <drm/msm_drm.h>
#include <drm_logger.h>
#include <errno.h>
#include <string.h>

#include <algorithm>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <mutex>

#include "drm_utils.h"
#include "drm_property.h"
#include "drm_connector.h"

namespace sde_drm {

using std::string;
using std::stringstream;
using std::pair;
using std::vector;
using std::unique_ptr;
using std::map;
using std::mutex;
using std::lock_guard;
using std::set;

static uint8_t ON = 0;
static uint8_t DOZE = 1;
static uint8_t DOZE_SUSPEND = 2;
static uint8_t OFF = 5;

// Connector FB Secure Modes
static uint8_t NON_SECURE = 0;
static uint8_t SECURE = 1;

static uint8_t QSYNC_MODE_NONE = 0;
static uint8_t QSYNC_MODE_CONTINUOUS = 1;
static uint8_t QSYNC_MODE_ONESHOT = 2;

static uint8_t FRAME_TRIGGER_DEFAULT = 0;
static uint8_t FRAME_TRIGGER_SERIALIZE = 1;
static uint8_t FRAME_TRIGGER_POSTED_START = 2;

static uint8_t DRM_MODE_COLORIMETRY_DEFAULT            = 0;
static uint8_t DRM_MODE_COLORIMETRY_SMPTE_170M_YCC     = 1;
static uint8_t DRM_MODE_COLORIMETRY_BT709_YCC          = 2;
static uint8_t DRM_MODE_COLORIMETRY_XVYCC_601          = 3;
static uint8_t DRM_MODE_COLORIMETRY_XVYCC_709          = 4;
static uint8_t DRM_MODE_COLORIMETRY_SYCC_601           = 5;
static uint8_t DRM_MODE_COLORIMETRY_OPYCC_601          = 6;
static uint8_t DRM_MODE_COLORIMETRY_OPRGB              = 7;
static uint8_t DRM_MODE_COLORIMETRY_BT2020_CYCC        = 8;
static uint8_t DRM_MODE_COLORIMETRY_BT2020_RGB         = 9;
static uint8_t DRM_MODE_COLORIMETRY_BT2020_YCC         = 10;
static uint8_t DRM_MODE_COLORIMETRY_DCI_P3_RGB_D65     = 11;
static uint8_t DRM_MODE_COLORIMETRY_DCI_P3_RGB_THEATER = 12;

static void PopulatePowerModes(drmModePropertyRes *prop) {
  for (auto i = 0; i < prop->count_enums; i++) {
    string enum_name(prop->enums[i].name);
    if (enum_name == "ON") {
      ON = prop->enums[i].value;
    } else if (enum_name == "LP1") {
      DOZE = prop->enums[i].value;
    } else if (enum_name == "LP2") {
      DOZE_SUSPEND = prop->enums[i].value;
    } else if (enum_name == "OFF") {
      OFF = prop->enums[i].value;
    }
  }
}

static void PopulateSecureModes(drmModePropertyRes *prop) {
  for (auto i = 0; i < prop->count_enums; i++) {
    string enum_name(prop->enums[i].name);
    if (enum_name == "non_sec") {
      NON_SECURE = prop->enums[i].value;
    } else if (enum_name == "sec") {
      SECURE = prop->enums[i].value;
    }
  }
}

static void PopulateSupportedColorspaces(drmModePropertyRes *prop) {
  for (auto i = 0; i < prop->count_enums; i++) {
    string enum_name(prop->enums[i].name);
    if (enum_name == "Default") {
      DRM_MODE_COLORIMETRY_DEFAULT = prop->enums[i].value;
    } else if (enum_name == "SMPTE_170M_YCC") {
      DRM_MODE_COLORIMETRY_SMPTE_170M_YCC = prop->enums[i].value;
    } else if (enum_name == "BT709_YCC") {
      DRM_MODE_COLORIMETRY_BT709_YCC = prop->enums[i].value;
    } else if (enum_name == "XVYCC_601") {
      DRM_MODE_COLORIMETRY_XVYCC_601 = prop->enums[i].value;
    } else if (enum_name == "XVYCC_709") {
      DRM_MODE_COLORIMETRY_XVYCC_709 = prop->enums[i].value;
    } else if (enum_name == "SYCC_601") {
      DRM_MODE_COLORIMETRY_SYCC_601 = prop->enums[i].value;
    } else if (enum_name == "opYCC_601") {
      DRM_MODE_COLORIMETRY_OPYCC_601 = prop->enums[i].value;
    } else if (enum_name == "opRGB") {
      DRM_MODE_COLORIMETRY_OPRGB = prop->enums[i].value;
    } else if (enum_name == "BT2020_CYCC") {
      DRM_MODE_COLORIMETRY_BT2020_CYCC = prop->enums[i].value;
    } else if (enum_name == "BT2020_RGB") {
      DRM_MODE_COLORIMETRY_BT2020_RGB = prop->enums[i].value;
    } else if (enum_name == "BT2020_YCC") {
      DRM_MODE_COLORIMETRY_BT2020_YCC = prop->enums[i].value;
    } else if (enum_name == "DCI_P3_RGB_D65") {
      DRM_MODE_COLORIMETRY_DCI_P3_RGB_D65 = prop->enums[i].value;
    } else if (enum_name == "DCI_P3_RGB_Theater") {
      DRM_MODE_COLORIMETRY_DCI_P3_RGB_THEATER = prop->enums[i].value;
    }
  }
}

static DRMTopology GetTopologyEnum(const string &topology) {
  if (topology == "sde_singlepipe") return DRMTopology::SINGLE_LM;
  if (topology == "sde_singlepipe_dsc") return DRMTopology::SINGLE_LM_DSC;
  if (topology == "sde_dualpipe") return DRMTopology::DUAL_LM;
  if (topology == "sde_dualpipe_dsc") return DRMTopology::DUAL_LM_DSC;
  if (topology == "sde_dualpipemerge") return DRMTopology::DUAL_LM_MERGE;
  if (topology == "sde_dualpipemerge_dsc") return DRMTopology::DUAL_LM_MERGE_DSC;
  if (topology == "sde_dualpipe_dscmerge") return DRMTopology::DUAL_LM_DSCMERGE;
  if (topology == "sde_quadpipemerge") return DRMTopology::QUAD_LM_MERGE;
  if (topology == "sde_quadpipe_dscmerge") return DRMTopology::QUAD_LM_DSCMERGE;
  if (topology == "sde_quadpipe_3dmerge_dsc") return DRMTopology::QUAD_LM_MERGE_DSC;
  if (topology == "sde_ppsplit") return DRMTopology::PPSPLIT;
  return DRMTopology::UNKNOWN;
}

static void PopulateQsyncModes(drmModePropertyRes *prop) {
  for (auto i = 0; i < prop->count_enums; i++) {
    string enum_name(prop->enums[i].name);
    if (enum_name == "none") {
      QSYNC_MODE_NONE = prop->enums[i].value;
    } else if (enum_name == "continuous") {
      QSYNC_MODE_CONTINUOUS = prop->enums[i].value;
    } else if (enum_name == "one_shot") {
      QSYNC_MODE_ONESHOT = prop->enums[i].value;
    }
  }
}

static void PopulateFrameTriggerModes(drmModePropertyRes *prop) {
  for (auto i = 0; i < prop->count_enums; i++) {
    string enum_name(prop->enums[i].name);
    if (enum_name == "default") {
      FRAME_TRIGGER_DEFAULT = prop->enums[i].value;
    } else if (enum_name == "serilize_frame_trigger") {
      FRAME_TRIGGER_SERIALIZE = prop->enums[i].value;
    } else if (enum_name == "posted_start") {
      FRAME_TRIGGER_POSTED_START = prop->enums[i].value;
    }
  }
}

static int32_t GetColorspace(DRMColorspace drm_colorspace) {
  uint32_t colorspace = 0;
  switch (drm_colorspace) {
    case (DRMColorspace::DEFAULT):
      colorspace = DRM_MODE_COLORIMETRY_DEFAULT;
      break;
    case (DRMColorspace::SMPTE_170M_YCC):
      colorspace = DRM_MODE_COLORIMETRY_SMPTE_170M_YCC;
      break;
    case (DRMColorspace::BT709_YCC):
      colorspace = DRM_MODE_COLORIMETRY_BT709_YCC;
      break;
    case (DRMColorspace::XVYCC_601):
      colorspace = DRM_MODE_COLORIMETRY_XVYCC_601;
      break;
    case (DRMColorspace::XVYCC_709):
      colorspace = DRM_MODE_COLORIMETRY_XVYCC_709;
      break;
     case (DRMColorspace::SYCC_601):
      colorspace = DRM_MODE_COLORIMETRY_SYCC_601;
      break;
    case (DRMColorspace::OPYCC_601):
      colorspace = DRM_MODE_COLORIMETRY_OPYCC_601;
      break;
    case (DRMColorspace::OPRGB):
      colorspace = DRM_MODE_COLORIMETRY_OPRGB;
      break;
    case (DRMColorspace::BT2020_CYCC):
      colorspace = DRM_MODE_COLORIMETRY_BT2020_CYCC;
      break;
    case (DRMColorspace::BT2020_RGB):
      colorspace = DRM_MODE_COLORIMETRY_BT2020_RGB;
      break;
    case (DRMColorspace::BT2020_YCC):
      colorspace = DRM_MODE_COLORIMETRY_BT2020_YCC;
      break;
    case (DRMColorspace::DCI_P3_RGB_D65):
      colorspace = DRM_MODE_COLORIMETRY_DCI_P3_RGB_D65;
      break;
    case (DRMColorspace::DCI_P3_RGB_THEATER):
      colorspace = DRM_MODE_COLORIMETRY_DCI_P3_RGB_THEATER;
      break;
    default:
      colorspace = -1;
      break;
  }
  return colorspace;
}

#define __CLASS__ "DRMConnectorManager"

void DRMConnectorManager::Init(drmModeRes *resource) {
  lock_guard<mutex> lock(lock_);
  for (int i = 0; i < resource->count_connectors; i++) {
    unique_ptr<DRMConnector> conn(new DRMConnector(fd_));
    drmModeConnector *libdrm_conn = drmModeGetConnector(fd_, resource->connectors[i]);
    if (libdrm_conn) {
      conn->InitAndParse(libdrm_conn);
      connector_pool_[resource->connectors[i]] = std::move(conn);
    } else {
      DRM_LOGE("Critical error: drmModeGetConnector() failed for connector %u.",
               resource->connectors[i]);
    }
  }
}

void DRMConnectorManager::Update() {
  lock_guard<mutex> lock(lock_);
  drmModeRes *resource = drmModeGetResources(fd_);

  if (NULL == resource) {
    DRM_LOGE("drmModeGetResources() failed. Connector status not updated.");
    return;
  }

  // Build a map of the updated list of connector ids.
  std::map<uint32_t, uint32_t> drm_connectors;
  for (int i = 0; i < resource->count_connectors; i++) {
    drm_connectors[resource->connectors[i]] = resource->connectors[i];
  }

  // Delete connectors in connector pool.
  for (auto conn = connector_pool_.cbegin(); conn != connector_pool_.cend();) {
    auto drmconn = drm_connectors.find(conn->first);
    if (drmconn == drm_connectors.end()) {
      // A DRM Connector in our pool was deleted.
      if (conn->second->GetStatus() == DRMStatus::FREE) {
        DRM_LOGD("Removing connector id %u from pool.", conn->first);
        conn = connector_pool_.erase(conn);
      } else {
        // Physically removed DRM Connectors (displays) first go to disconnected state. When its
        // reserved resources are freed up, they are removed from the driver's connector list. Do
        // not remove DRM Connectors that are DRMStatus::BUSY.
        DRM_LOGW("In-use connector id %u removed by DRM.", conn->first);
        conn++;
      }
    } else {
      // Remove DRM Connector present in both lists to ensure that only new connector ids remain.
      drm_connectors.erase(drmconn);
      conn++;
    }
  }

  // Add new connectors in connector pool.
  for (auto &drmconn : drm_connectors) {
    DRM_LOGD("Adding connector id %u to pool.", drmconn.first);
    unique_ptr<DRMConnector> conn(new DRMConnector(fd_));
    drmModeConnector *libdrm_conn = drmModeGetConnector(fd_, drmconn.first);
    if (libdrm_conn) {
      conn->InitAndParse(libdrm_conn);
      conn->SetSkipConnectorReload(true);
      connector_pool_[drmconn.first] = std::move(conn);
    } else {
      DRM_LOGW("Critical error: drmModeGetConnector() failed for connector %u.", drmconn.first);
    }
  }

  drmModeFreeResources(resource);
}

void DRMConnectorManager::DumpByID(uint32_t id) {
  lock_guard<mutex> lock(lock_);
  connector_pool_[id]->Dump();
}

void DRMConnectorManager::DumpAll() {
  lock_guard<mutex> lock(lock_);
  for (auto &conn : connector_pool_) {
    conn.second->Dump();
  }
}

void DRMConnectorManager::Perform(DRMOps code, uint32_t obj_id, drmModeAtomicReq *req,
                                  va_list args) {
  lock_guard<mutex> lock(lock_);
  auto it = connector_pool_.find(obj_id);
  if (it == connector_pool_.end()) {
    DRM_LOGE("Invalid connector id %d", obj_id);
    return;
  }

  it->second->Perform(code, req, args);
}

int DRMConnectorManager::GetConnectorInfo(uint32_t conn_id, DRMConnectorInfo *info) {
  lock_guard<mutex> lock(lock_);
  int ret = -ENODEV;
  auto iter = connector_pool_.find(conn_id);

  if (iter !=  connector_pool_.end()) {
    ret = connector_pool_[conn_id]->GetInfo(info);
  }

  return ret;
}

void DRMConnectorManager::GetConnectorList(std::vector<uint32_t> *conn_ids) {
  lock_guard<mutex> lock(lock_);
  if (!conn_ids) {
    DRM_LOGE("No output parameter provided.");
    return;
  }
  conn_ids->clear();
  for (auto &conn : connector_pool_) {
    conn_ids->push_back(conn.first);
  }
}

static bool IsTVConnector(uint32_t type) {
  return (type == DRM_MODE_CONNECTOR_TV || type == DRM_MODE_CONNECTOR_HDMIA ||
          type == DRM_MODE_CONNECTOR_HDMIB || type == DRM_MODE_CONNECTOR_DisplayPort ||
          type == DRM_MODE_CONNECTOR_VGA);
}

int DRMConnectorManager::Reserve(DRMDisplayType disp_type, DRMDisplayToken *token) {
  lock_guard<mutex> lock(lock_);
  int ret = -ENODEV;
  token->conn_id = 0;

  for (auto &conn : connector_pool_) {
    if (conn.second->GetStatus() == DRMStatus::FREE) {
      uint32_t conn_type;
      conn.second->GetType(&conn_type);
      if ((disp_type == DRMDisplayType::PERIPHERAL && conn_type == DRM_MODE_CONNECTOR_DSI) ||
          (disp_type == DRMDisplayType::VIRTUAL && conn_type == DRM_MODE_CONNECTOR_VIRTUAL) ||
          (disp_type == DRMDisplayType::TV && IsTVConnector(conn_type))) {
        if (conn.second->IsConnected()) {
          // Free-up previously reserved connector, if any.
          if (token->conn_id) {
            connector_pool_[token->conn_id]->Unlock();
          }
          conn.second->Lock();
          token->conn_id = conn.first;
          ret = 0;
          break;
        } else {
          // Hold on to the first reserved connector.
          if (token->conn_id) {
            continue;
          }
          // Prefer a connector that is connected. Continue search.
          conn.second->Lock();
          token->conn_id = conn.first;
          ret = 0;
        }
      }
    }
  }

  return ret;
}

int DRMConnectorManager::Reserve(uint32_t conn_id, DRMDisplayToken *token) {
  lock_guard<mutex> lock(lock_);
  int ret = -ENODEV;

  auto iter = connector_pool_.find(conn_id);
  if ((iter != connector_pool_.end()) && (iter->second->GetStatus() == DRMStatus::FREE)) {
    iter->second->Lock();
    token->conn_id = iter->first;
    ret = 0;
  }

  return ret;
}

int DRMConnectorManager::GetPossibleEncoders(uint32_t connector_id,
                                             set<uint32_t> *possible_encoders) {
  lock_guard<mutex> lock(lock_);
  return connector_pool_[connector_id]->GetPossibleEncoders(possible_encoders);
}


void DRMConnectorManager::Free(DRMDisplayToken *token) {
  lock_guard<mutex> lock(lock_);
  connector_pool_.at(token->conn_id)->Unlock();
  token->conn_id = 0;
}

// ==============================================================================================//

#undef __CLASS__
#define __CLASS__ "DRMConnector"

DRMConnector::~DRMConnector() {
  if (drm_connector_) {
    drmModeFreeConnector(drm_connector_);
  }
}

void DRMConnector::ParseProperties() {
  drmModeObjectProperties *props =
      drmModeObjectGetProperties(fd_, drm_connector_->connector_id, DRM_MODE_OBJECT_CONNECTOR);
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

    if (prop_enum == DRMProperty::LP) {
      PopulatePowerModes(info);
    } else if (prop_enum == DRMProperty::FB_TRANSLATION_MODE) {
      PopulateSecureModes(info);
    } else if (prop_enum == DRMProperty::QSYNC_MODE) {
      PopulateQsyncModes(info);
    } else if (prop_enum == DRMProperty::FRAME_TRIGGER) {
      PopulateFrameTriggerModes(info);
    } else if (prop_enum == DRMProperty::COLORSPACE) {
      PopulateSupportedColorspaces(info);
    } else if (prop_enum == DRMProperty::MAX) {
      DRM_LOGD("DRMProperty %s is not defined", info->name);
      return;
    }

    prop_mgr_.SetPropertyId(prop_enum, info->prop_id);
    drmModeFreeProperty(info);
  }

  drmModeFreeObjectProperties(props);
}

void DRMConnector::ParseCapabilities(uint64_t blob_id, DRMConnectorInfo *info) {
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
  stringstream stream(fmt_str);
  DRM_LOGI("stream str %s len %zu blob str %s len %d", stream.str().c_str(), stream.str().length(),
           static_cast<const char *>(blob->data), blob->length);
  string line = {};
  const string display_type = "display type=";
  const string panel_name = "panel name=";
  const string panel_mode = "panel mode=";
  const string dfps_support = "dfps support=";
  const string pixel_formats = "pixel_formats=";
  const string max_linewidth = "maxlinewidth=";
  const string panel_orientation = "panel orientation=";
  const string qsync_support = "qsync support=";
  const string wb_ubwc = "wb_ubwc";
  const string dyn_bitclk_support = "dyn bitclk support=";

  while (std::getline(stream, line)) {
    if (line.find(pixel_formats) != string::npos) {
      vector<pair<uint32_t, uint64_t>> formats_supported;
      ParseFormats(line.erase(0, pixel_formats.length()), &formats_supported);
      info->formats_supported = move(formats_supported);
    } else if (line.find(max_linewidth) != string::npos) {
      info->max_linewidth = std::stoi(string(line, max_linewidth.length()));
    } else if (line.find(display_type) != string::npos) {
      info->is_primary = (string(line, display_type.length()) == "primary");
    } else if (line.find(panel_name) != string::npos) {
      info->panel_name = string(line, panel_name.length());
    } else if (line.find(panel_mode) != string::npos) {
      info->panel_mode = (string(line, panel_mode.length()) == "video") ? DRMPanelMode::VIDEO
                                                                        : DRMPanelMode::COMMAND;
    } else if (line.find(dfps_support) != string::npos) {
      info->dynamic_fps = (string(line, dfps_support.length()) == "true");
    } else if (line.find(panel_orientation) != string::npos) {
      if (string(line, panel_orientation.length()) == "horz flip") {
        info->panel_orientation = DRMRotation::FLIP_H;
      } else if (string(line, panel_orientation.length()) == "vert flip") {
        info->panel_orientation = DRMRotation::FLIP_V;
      } else if (string(line, panel_orientation.length()) == "horz & vert flip") {
        info->panel_orientation = DRMRotation::ROT_180;
      }
    } else if (line.find(qsync_support) != string::npos) {
      info->qsync_support = (string(line, qsync_support.length()) == "true");
    } else if (line.find(wb_ubwc) != string::npos) {
      info->is_wb_ubwc_supported = true;
    } else if (line.find(dyn_bitclk_support) != string::npos) {
      info->dyn_bitclk_support = (string(line, dyn_bitclk_support.length()) == "true");
    }
  }

  drmModeFreePropertyBlob(blob);
  delete[] fmt_str;
}

void DRMConnector::ParseCapabilities(uint64_t blob_id, drm_panel_hdr_properties *hdr_info) {
  drmModePropertyBlobRes *blob = drmModeGetPropertyBlob(fd_, blob_id);
  if (!blob) {
    return;
  }

  struct drm_panel_hdr_properties *hdr_data = (struct drm_panel_hdr_properties*)(blob->data);

  if (hdr_data) {
    hdr_info->hdr_enabled = hdr_data->hdr_enabled;
    hdr_info->peak_brightness = hdr_data->peak_brightness;
    hdr_info->blackness_level = hdr_data->blackness_level;
    for (int i = 0; i < DISPLAY_PRIMARIES_MAX; i++) {
      hdr_info->display_primaries[i] = hdr_data->display_primaries[i];
    }
  }
  drmModeFreePropertyBlob(blob);
}

void DRMConnector::ParseModeProperties(uint64_t blob_id, DRMConnectorInfo *info) {
  drmModePropertyBlobRes *blob = drmModeGetPropertyBlob(fd_, blob_id);
  if (!blob) {
    return;
  }

  if (!blob->data) {
    return;
  }

  if (!info->modes.size()) {
    return;
  }

  char *fmt_str = new char[blob->length + 1];
  memcpy (fmt_str, blob->data, blob->length);
  fmt_str[blob->length] = '\0';
  stringstream stream(fmt_str);
  DRM_LOGI("stream str %s len %zu blob str %s len %d", stream.str().c_str(), stream.str().length(),
           static_cast<const char *>(blob->data), blob->length);

  string line = {};
  const string mode_name = "mode_name=";
  const string topology = "topology=";
  const string pu_num_roi = "partial_update_num_roi=";
  const string pu_xstart = "partial_update_xstart=";
  const string pu_ystart = "partial_update_ystart=";
  const string pu_walign = "partial_update_walign=";
  const string pu_halign = "partial_update_halign=";
  const string pu_wmin = "partial_update_wmin=";
  const string pu_hmin = "partial_update_hmin=";
  const string pu_roimerge = "partial_update_roimerge=";
  const string bit_clk_rate = "bit_clk_rate=";
  const string mdp_transfer_time_us = "mdp_transfer_time_us=";

  DRMModeInfo *mode_item = &info->modes.at(0);
  unsigned int index = 0;

  while (std::getline(stream, line)) {
    if (line.find(mode_name) != string::npos) {
      string name(line, mode_name.length());
      if (index >= info->modes.size()) {
        break;
      }
      // Move to the next mode_item
      mode_item = &info->modes.at(index++);
    } else if (line.find(topology) != string::npos) {
      mode_item->topology = GetTopologyEnum(string(line, topology.length()));;
    } else if (line.find(pu_num_roi) != string::npos) {
      mode_item->num_roi = std::stoi(string(line, pu_num_roi.length()));
    } else if (line.find(pu_xstart) != string::npos) {
      mode_item->xstart = std::stoi(string(line, pu_xstart.length()));
    } else if (line.find(pu_ystart) != string::npos) {
      mode_item->ystart = std::stoi(string(line, pu_ystart.length()));
    } else if (line.find(pu_walign) != string::npos) {
      mode_item->walign = std::stoi(string(line, pu_walign.length()));
    } else if (line.find(pu_halign) != string::npos) {
      mode_item->halign = std::stoi(string(line, pu_halign.length()));
    } else if (line.find(pu_wmin) != string::npos) {
      mode_item->wmin = std::stoi(string(line, pu_wmin.length()));
    } else if (line.find(pu_hmin) != string::npos) {
      mode_item->hmin = std::stoi(string(line, pu_hmin.length()));
    } else if (line.find(pu_roimerge) != string::npos) {
      mode_item->roi_merge = std::stoi(string(line, pu_roimerge.length()));
    } else if (line.find(bit_clk_rate) != string::npos) {
      mode_item->bit_clk_rate = std::stoi(string(line, bit_clk_rate.length()));
    } else if (line.find(mdp_transfer_time_us) != string::npos) {
      mode_item->transfer_time_us = std::stoi(string(line, mdp_transfer_time_us.length()));
    }
  }

  drmModeFreePropertyBlob(blob);
  delete[] fmt_str;
}

void DRMConnector::ParseCapabilities(uint64_t blob_id, drm_msm_ext_hdr_properties *hdr_info) {
  drmModePropertyBlobRes *blob = drmModeGetPropertyBlob(fd_, blob_id);
  if (!blob) {
    return;
  }

  struct drm_msm_ext_hdr_properties *hdr_cdata = (struct drm_msm_ext_hdr_properties*)(blob->data);

  if (hdr_cdata) {
    hdr_info->hdr_supported = hdr_cdata->hdr_supported;
    hdr_info->hdr_plus_supported = hdr_cdata->hdr_plus_supported;
    hdr_info->hdr_eotf = hdr_cdata->hdr_eotf;
    hdr_info->hdr_metadata_type_one = hdr_cdata->hdr_metadata_type_one;
    hdr_info->hdr_max_luminance = hdr_cdata->hdr_max_luminance;
    hdr_info->hdr_avg_luminance = hdr_cdata->hdr_avg_luminance;
    hdr_info->hdr_min_luminance = hdr_cdata->hdr_min_luminance;
    DRM_LOGI("hdr_supported = %d, hdr_plus_supported = %d, hdr_eotf = %d, "
             "hdr_metadata_type_one = %d, hdr_max_luminance = %d, hdr_avg_luminance = %d, "
             "hdr_min_luminance = %d\n", hdr_info->hdr_supported,
             hdr_info->hdr_plus_supported,
             hdr_info->hdr_eotf, hdr_info->hdr_metadata_type_one, hdr_info->hdr_max_luminance,
             hdr_info->hdr_avg_luminance, hdr_info->hdr_min_luminance);
  }
  drmModeFreePropertyBlob(blob);
}

void DRMConnector::ParseCapabilities(uint64_t blob_id, std::vector<uint8_t> *edid) {
  drmModePropertyBlobRes *blob = drmModeGetPropertyBlob(fd_, blob_id);
  if (!blob) {
    return;
  }

  uint8_t *edid_blob = (uint8_t*)(blob->data);
  uint32_t length = blob->length;
  (*edid).assign(edid_blob, edid_blob + length);

  drmModeFreePropertyBlob(blob);
}

int DRMConnector::GetInfo(DRMConnectorInfo *info) {
  uint32_t conn_id = drm_connector_->connector_id;
  if (!skip_connector_reload_ && (IsTVConnector(drm_connector_->connector_type)
      || (DRM_MODE_CONNECTOR_VIRTUAL == drm_connector_->connector_type))) {
    // Reload since for some connectors like Virtual and DP, modes may change.
    drmModeConnectorPtr drm_connector = drmModeGetConnector(fd_, conn_id);
    if (!drm_connector) {
      // Connector resource not found. This could happen if a connector is removed before a commit
      // was done on it. Mark the connector as disconnected for graceful teardown. Update 'info'
      // with basic information from previously initialized drm_connector_ for graceful teardown.
      info->is_connected = false;
      info->modes.clear();
      info->type = drm_connector_->connector_type;
      info->type_id = drm_connector_->connector_type_id;
      DLOGW("Connector %u not found. Possibly removed.", conn_id);
      return 0;
    }
    drmModeFreeConnector(drm_connector_);
    drm_connector_ = drm_connector;
  }

  SetSkipConnectorReload(false);  // Reset skip_connector_reload_ setting.
  info->modes.clear();
  if (!drm_connector_->count_modes) {
    DRM_LOGW("Zero modes on connector %u.", conn_id);
  }

  if (!drm_connector_->modes) {
    DLOGW("Connector %u not found.", conn_id);
    return 0;
  }

  for (auto i = 0; i < drm_connector_->count_modes; i++) {
    DRMModeInfo modes_item {};
    modes_item.mode = drm_connector_->modes[i];
    info->modes.push_back(modes_item);
  }
  info->mmWidth = drm_connector_->mmWidth;
  info->mmHeight = drm_connector_->mmHeight;
  info->type = drm_connector_->connector_type;
  info->type_id = drm_connector_->connector_type_id;
  info->is_connected = IsConnected();

  drmModeObjectProperties *props =
      drmModeObjectGetProperties(fd_, drm_connector_->connector_id, DRM_MODE_OBJECT_CONNECTOR);
  if (!props || !props->props || !props->prop_values) {
    drmModeFreeObjectProperties(props);
    return -ENODEV;
  }

  uint32_t index = UINT32_MAX;

  if (prop_mgr_.IsPropertyAvailable(DRMProperty::HDR_PROPERTIES)) {
    index = std::distance(props->props,
                          std::find(props->props, props->props + props->count_props,
                                    prop_mgr_.GetPropertyId(DRMProperty::HDR_PROPERTIES)));
    if (index < props->count_props)
      ParseCapabilities(props->prop_values[index], &info->panel_hdr_prop);
  }

  if (prop_mgr_.IsPropertyAvailable(DRMProperty::CAPABILITIES)) {
    index = std::distance(props->props,
                          std::find(props->props, props->props + props->count_props,
                                    prop_mgr_.GetPropertyId(DRMProperty::CAPABILITIES)));
    if (index < props->count_props)
      ParseCapabilities(props->prop_values[index], info);
  }

  if (prop_mgr_.IsPropertyAvailable(DRMProperty::MODE_PROPERTIES)) {
    index = std::distance(props->props,
                          std::find(props->props, props->props + props->count_props,
                                    prop_mgr_.GetPropertyId(DRMProperty::MODE_PROPERTIES)));
    if (index < props->count_props)
      ParseModeProperties(props->prop_values[index], info);
  }

  if (prop_mgr_.IsPropertyAvailable(DRMProperty::EXT_HDR_PROPERTIES)) {
    index = std::distance(props->props,
                          std::find(props->props, props->props + props->count_props,
                                    prop_mgr_.GetPropertyId(DRMProperty::EXT_HDR_PROPERTIES)));
    if (index < props->count_props)
      ParseCapabilities(props->prop_values[index], &info->ext_hdr_prop);
  }

  if (prop_mgr_.IsPropertyAvailable(DRMProperty::TOPOLOGY_CONTROL)) {
    index = std::distance(props->props,
                          std::find(props->props, props->props + props->count_props,
                                    prop_mgr_.GetPropertyId(DRMProperty::TOPOLOGY_CONTROL)));
    info->topology_control = props->prop_values[index];
  }
  if (prop_mgr_.IsPropertyAvailable(DRMProperty::EDID)) {
    index = std::distance(props->props,
                          std::find(props->props, props->props + props->count_props,
                                    prop_mgr_.GetPropertyId(DRMProperty::EDID)));
    ParseCapabilities(props->prop_values[index], &info->edid);
  }

  if (prop_mgr_.IsPropertyAvailable(DRMProperty::SUPPORTED_COLORSPACES)) {
    index = std::distance(props->props,
                          std::find(props->props, props->props + props->count_props,
                                    prop_mgr_.GetPropertyId(DRMProperty::SUPPORTED_COLORSPACES)));
    info->supported_colorspaces = props->prop_values[index];
  }

  drmModeFreeObjectProperties(props);

  return 0;
}

void DRMConnector::InitAndParse(drmModeConnector *conn) {
  drm_connector_ = conn;
  ParseProperties();
  pp_mgr_ = std::unique_ptr<DRMPPManager>(new DRMPPManager(fd_));
  pp_mgr_->Init(prop_mgr_, DRM_MODE_OBJECT_CONNECTOR);
}

void DRMConnector::Perform(DRMOps code, drmModeAtomicReq *req, va_list args) {
  uint32_t obj_id = drm_connector_->connector_id;

  switch (code) {
    case DRMOps::CONNECTOR_SET_CRTC: {
      uint32_t crtc = va_arg(args, uint32_t);
      drmModeAtomicAddProperty(req, obj_id, prop_mgr_.GetPropertyId(DRMProperty::CRTC_ID), crtc);
      DRM_LOGD("Connector %d: Setting CRTC %d", obj_id, crtc);
    } break;

    case DRMOps::CONNECTOR_GET_RETIRE_FENCE: {
      int64_t *fence = va_arg(args, int64_t *);
      *fence = -1;
      uint32_t prop_id = prop_mgr_.GetPropertyId(DRMProperty::RETIRE_FENCE);
      drmModeAtomicAddProperty(req, obj_id, prop_id, reinterpret_cast<uint64_t>(fence));
    } break;

    case DRMOps::CONNECTOR_SET_OUTPUT_RECT: {
      DRMRect rect = va_arg(args, DRMRect);
      drmModeAtomicAddProperty(req, obj_id,
                               prop_mgr_.GetPropertyId(DRMProperty::DST_X), rect.left);
      drmModeAtomicAddProperty(req, obj_id,
                               prop_mgr_.GetPropertyId(DRMProperty::DST_Y), rect.top);
      drmModeAtomicAddProperty(req, obj_id, prop_mgr_.GetPropertyId(DRMProperty::DST_W),
                               rect.right - rect.left);
      drmModeAtomicAddProperty(req, obj_id, prop_mgr_.GetPropertyId(DRMProperty::DST_H),
                               rect.bottom - rect.top);
      DRM_LOGD("Connector %d: Setting dst [x,y,w,h][%d,%d,%d,%d]", obj_id, rect.left,
                  rect.top, (rect.right - rect.left), (rect.bottom - rect.top));
    } break;

    case DRMOps::CONNECTOR_SET_OUTPUT_FB_ID: {
      uint32_t fb_id = va_arg(args, uint32_t);
      drmModeAtomicAddProperty(req, obj_id, prop_mgr_.GetPropertyId(DRMProperty::FB_ID), fb_id);
      DRM_LOGD("Connector %d: Setting fb_id %d", obj_id, fb_id);
    } break;

    case DRMOps::CONNECTOR_SET_POWER_MODE: {
      int drm_power_mode = va_arg(args, int);
      uint32_t power_mode = ON;
      switch (drm_power_mode) {
        case (int)DRMPowerMode::ON:
          power_mode = ON;
          break;
        case (int)DRMPowerMode::DOZE:
          power_mode = DOZE;
          break;
        case (int)DRMPowerMode::DOZE_SUSPEND:
          power_mode = DOZE_SUSPEND;
          break;
        case (int)DRMPowerMode::OFF:
          power_mode = OFF;
          break;
        default:
          DRM_LOGE("Invalid power mode %d to set on connector %d", drm_power_mode, obj_id);
          break;
      }
      drmModeAtomicAddProperty(req, obj_id, prop_mgr_.GetPropertyId(DRMProperty::LP), power_mode);
      DRM_LOGD("Connector %d: Setting power_mode %d", obj_id, power_mode);
    } break;

    case DRMOps::CONNECTOR_SET_ROI: {
      uint32_t num_roi = va_arg(args, uint32_t);
      DRMRect *conn_rois = va_arg(args, DRMRect*);
      SetROI(req, obj_id, num_roi, conn_rois);
    } break;

    case DRMOps::CONNECTOR_SET_AUTOREFRESH: {
      uint32_t enable = va_arg(args, uint32_t);
      drmModeAtomicAddProperty(req, obj_id, prop_mgr_.GetPropertyId(DRMProperty::AUTOREFRESH),
                               enable);
      DRM_LOGD("Connector %d: Setting autorefresh %d", obj_id, enable);
    } break;

    case DRMOps::CONNECTOR_SET_FB_SECURE_MODE: {
      int secure_mode = va_arg(args, int);
      uint32_t fb_secure_mode = (secure_mode == (int)DRMSecureMode::SECURE) ? SECURE : NON_SECURE;
      drmModeAtomicAddProperty(req, obj_id,
                               prop_mgr_.GetPropertyId(DRMProperty::FB_TRANSLATION_MODE),
                               fb_secure_mode);
      DRM_LOGD("Connector %d: Setting FB secure mode %d", obj_id, fb_secure_mode);
    } break;

    case DRMOps::CONNECTOR_SET_POST_PROC: {
      DRMPPFeatureInfo *data = va_arg(args, DRMPPFeatureInfo*);
      if (data)
        pp_mgr_->SetPPFeature(req, obj_id, *data);
    } break;

    case DRMOps::CONNECTOR_SET_HDR_METADATA: {
      drm_msm_ext_hdr_metadata *hdr_metadata = va_arg(args, drm_msm_ext_hdr_metadata *);
      drmModeAtomicAddProperty(req, obj_id, prop_mgr_.GetPropertyId(DRMProperty::HDR_METADATA),
                               reinterpret_cast<uint64_t>(hdr_metadata));
    } break;

    case DRMOps::CONNECTOR_SET_QSYNC_MODE: {
      if (!prop_mgr_.IsPropertyAvailable(DRMProperty::QSYNC_MODE)) {
        return;
      }
      int drm_qsync_mode = va_arg(args, int);
      uint32_t qsync_mode = static_cast<uint32_t>(drm_qsync_mode);
      drmModeAtomicAddProperty(req, obj_id, prop_mgr_.GetPropertyId(DRMProperty::QSYNC_MODE),
                               qsync_mode);
      DRM_LOGD("Connector %d: Setting Qsync mode %d", obj_id, qsync_mode);
    } break;

    case DRMOps::CONNECTOR_SET_TOPOLOGY_CONTROL: {
      uint32_t topology_control = va_arg(args, uint32_t);
      drmModeAtomicAddProperty(req, obj_id, prop_mgr_.GetPropertyId(DRMProperty::TOPOLOGY_CONTROL),
                               topology_control);
    } break;

    case DRMOps::CONNECTOR_SET_FRAME_TRIGGER: {
      if (!prop_mgr_.IsPropertyAvailable(DRMProperty::FRAME_TRIGGER)) {
        return;
      }
      int drm_frame_trigger_mode = va_arg(args, int);
      DRMFrameTriggerMode mode = static_cast<DRMFrameTriggerMode>(drm_frame_trigger_mode);
      int32_t frame_trigger_mode = -1;
      switch (mode) {
        case (DRMFrameTriggerMode::FRAME_DONE_WAIT_DEFAULT):
          frame_trigger_mode = FRAME_TRIGGER_DEFAULT;
          break;
        case (DRMFrameTriggerMode::FRAME_DONE_WAIT_SERIALIZE):
          frame_trigger_mode = FRAME_TRIGGER_SERIALIZE;
          break;
        case (DRMFrameTriggerMode::FRAME_DONE_WAIT_POSTED_START):
          frame_trigger_mode = FRAME_TRIGGER_POSTED_START;
          break;
        default:
          DRM_LOGE("Invalid frame trigger mode %d to set on connector %d",
                   drm_frame_trigger_mode, obj_id);
          break;
      }
      if (frame_trigger_mode >= 0) {
        uint32_t prop_id = prop_mgr_.GetPropertyId(DRMProperty::FRAME_TRIGGER);
        int ret = drmModeAtomicAddProperty(req, obj_id, prop_id, frame_trigger_mode);
        if (ret < 0) {
          DRM_LOGE("AtomicAddProperty failed obj_id 0x%x, prop_id %d mode %d ret %d",
                   obj_id, prop_id, frame_trigger_mode, ret);
        } else {
          DRM_LOGD("Connector %d: Setting frame trigger mode %d", obj_id, frame_trigger_mode);
        }
      }
    } break;

    case DRMOps::CONNECTOR_SET_COLORSPACE: {
      if (!prop_mgr_.IsPropertyAvailable(DRMProperty::COLORSPACE)) {
        return;
      }
      DRMColorspace drm_colorspace = static_cast<DRMColorspace>(va_arg(args, uint32_t));
      int32_t colorspace = 0;
      colorspace = GetColorspace(drm_colorspace);
      if (colorspace >= 0) {
        uint32_t prop_id = prop_mgr_.GetPropertyId(DRMProperty::COLORSPACE);
        int ret = drmModeAtomicAddProperty(req, obj_id, prop_id, colorspace);
        if (ret < 0) {
          DRM_LOGE("AtomicAddProperty failed obj_id 0x%x, prop_id %d mode %d ret %d",
                   obj_id, prop_id, colorspace, ret);
        } else {
          DRM_LOGD("Connector %d: Setting colorspace %d", obj_id, colorspace);
        }
      } else {
        DRM_LOGE("Invalid colorspace %d", colorspace);
      }
    } break;

    default:
      DRM_LOGE("Invalid opcode %d to set on connector %d", code, obj_id);
      break;
  }
}

void DRMConnector::SetROI(drmModeAtomicReq *req, uint32_t obj_id, uint32_t num_roi,
                          DRMRect *conn_rois) {
#ifdef SDE_MAX_ROI_V1
  if (num_roi > SDE_MAX_ROI_V1 || !prop_mgr_.IsPropertyAvailable(DRMProperty::ROI_V1)) {
    return;
  }
  if (!num_roi || !conn_rois) {
    drmModeAtomicAddProperty(req, obj_id, prop_mgr_.GetPropertyId(DRMProperty::ROI_V1), 0);
    DRM_LOGD("Connector ROI is set to NULL to indicate full frame update");
    return;
  }

  static struct sde_drm_roi_v1 roi_v1 {};
  memset(&roi_v1, 0, sizeof(roi_v1));
  roi_v1.num_rects = num_roi;

  for (uint32_t i = 0; i < num_roi; i++) {
    roi_v1.roi[i].x1 = conn_rois[i].left;
    roi_v1.roi[i].x2 = conn_rois[i].right;
    roi_v1.roi[i].y1 = conn_rois[i].top;
    roi_v1.roi[i].y2 = conn_rois[i].bottom;
    DRM_LOGD("Conn %d, ROI[l,t,b,r][%d %d %d %d]", obj_id,
             roi_v1.roi[i].x1,roi_v1.roi[i].y1,roi_v1.roi[i].x2,roi_v1.roi[i].y2);
  }
  drmModeAtomicAddProperty(req, obj_id, prop_mgr_.GetPropertyId(DRMProperty::ROI_V1),
                           reinterpret_cast<uint64_t>(&roi_v1));
#endif
}

int DRMConnector::GetPossibleEncoders(set<uint32_t> *possible_encoders) {
  if (!possible_encoders) {
    return -EINVAL;
  }

  uint32_t count_enc = drm_connector_->count_encoders;
  if (count_enc == 0) {
    DRM_LOGW("No possible encoders for connector %u", drm_connector_->connector_id);
  }

  (*possible_encoders).clear();
  for (uint32_t i = 0; i < count_enc; i++) {
    (*possible_encoders).insert(drm_connector_->encoders[i]);
  }

  return 0;
}

void DRMConnector::Dump() {
  DRM_LOGE("id: %d\tenc_id: %d\tconn: %d\ttype: %d\tPhy: %dx%d\n", drm_connector_->connector_id,
           drm_connector_->encoder_id, drm_connector_->connection, drm_connector_->connector_type,
           drm_connector_->mmWidth, drm_connector_->mmHeight);
  DRM_LOGE("Modes: \n");
  for (uint32_t i = 0; i < (uint32_t)drm_connector_->count_modes; i++) {
    DRM_LOGE(
        "Name: %s\tvref: %d\thdisp: %d\t hsync_s: %d\thsync_e:%d\thtotal: %d\t"
        "vdisp: %d\tvsync_s: %d\tvsync_e: %d\tvtotal: %d\n",
        drm_connector_->modes[i].name, drm_connector_->modes[i].vrefresh,
        drm_connector_->modes[i].hdisplay, drm_connector_->modes[i].hsync_start,
        drm_connector_->modes[i].hsync_end, drm_connector_->modes[i].htotal,
        drm_connector_->modes[i].vdisplay, drm_connector_->modes[i].vsync_start,
        drm_connector_->modes[i].vsync_end, drm_connector_->modes[i].vtotal);
  }
}

}  // namespace sde_drm
