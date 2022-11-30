/* Copyright (c) 2015 - 2019, The Linux Foundataion. All rights reserved.
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

#include <dlfcn.h>
#include <private/color_interface.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <algorithm>
#include <vector>
#include <string>

#include "color_manager.h"

#define __CLASS__ "ColorManager"

namespace sdm {

DynLib ColorManagerProxy::color_lib_;
CreateColorInterface ColorManagerProxy::create_intf_ = NULL;
DestroyColorInterface ColorManagerProxy::destroy_intf_ = NULL;
HWResourceInfo ColorManagerProxy::hw_res_info_;

bool NeedsToneMap(const std::vector<Layer> &layers) {
  for (auto &layer : layers) {
    if (layer.request.flags.dest_tone_map) {
      return true;
    }
  }
  return false;
}

// Below two functions are part of concrete implementation for SDM core private
// color_params.h
void PPFeaturesConfig::Reset() {
  for (int i = 0; i < kMaxNumPPFeatures; i++) {
    if (feature_[i]) {
      delete feature_[i];
      feature_[i] = NULL;
    }
  }
  dirty_ = false;
  next_idx_ = 0;
}

DisplayError PPFeaturesConfig::RetrieveNextFeature(PPFeatureInfo **feature) {
  DisplayError ret = kErrorNone;
  uint32_t i(0);

  for (i = next_idx_; i < kMaxNumPPFeatures; i++) {
    if (feature_[i]) {
      *feature = feature_[i];
      next_idx_ = i + 1;
      break;
    }
  }

  if (i == kMaxNumPPFeatures) {
    ret = kErrorParameters;
    next_idx_ = 0;
  }

  return ret;
}

FeatureInterface* GetPostedStartFeatureCheckIntf(HWInterface *intf, PPFeaturesConfig *config) {
  return new ColorFeatureCheckingImpl(intf, config);
}

DisplayError STCIntfClient::Init(const std::string &panel_name) {
  lock_guard<mutex> obj(lock_);
  if (stc_intf_) {
    DLOGI("stc interface is already instantiated");
    return kErrorNone;
  }

  if (!stc_intf_lib_.Open(kStcIntfLib_)) {
    DLOGW("STC library is not present");
    return kErrorNotSupported;
  }

  if (!stc_intf_lib_.Sym("GetScPostBlendInterface",
                    reinterpret_cast<void **>(&GetScPostBlendInterface))) {
    DLOGE("GetHdrInterface symbol not found!, error = %s", dlerror());
    stc_intf_lib_.~DynLib();
    return kErrorNotSupported;
  }

  uint32_t major_version = 1;
  uint32_t minor_version = 0;

  stc_intf_ = GetScPostBlendInterface(major_version, minor_version);
  if (!stc_intf_) {
    DLOGE("Failed to get STC Interface!");
    stc_intf_lib_.~DynLib();
    return kErrorNotSupported;
  }

  int ret = stc_intf_->Init(panel_name);
  if (ret) {
    DLOGE("STC Interface init failed!, error = %d", ret);
    stc_intf_lib_.~DynLib();
    return kErrorNotSupported;
  }

  return kErrorNone;;
}

DisplayError STCIntfClient::DeInit() {
  lock_guard<mutex> obj(lock_);
  if (stc_intf_) {
    stc_intf_->DeInit();
  }
  stc_intf_lib_.~DynLib();
  return kErrorNone;
}

DisplayError STCIntfClient::SetProperty(const ScPayload &payload) {
  lock_guard<mutex> obj(lock_);
  int ret;

  if (!stc_intf_) {
    return kErrorNotSupported;
  }
  ret = stc_intf_->SetProperty(payload);
  if (ret) {
    DLOGE("Failed to SetProperty: %d, length = %d, ret = %d", payload.prop, payload.len, ret);
    return kErrorNotSupported;
  }

  return kErrorNone;
}

DisplayError STCIntfClient::GetProperty(ScPayload *payload) {
  lock_guard<mutex> obj(lock_);
  int ret = 0;

  if (!stc_intf_) {
    return kErrorNotSupported;
  }
  if (!payload) {
    DLOGE("Invalid parameters");
    return kErrorParameters;
  }

  ret = stc_intf_->GetProperty(payload);
  if (ret) {
    DLOGE("Failed to GetProperty: %d, length = %d, ret = %d", payload->prop, payload->len, ret);
    return kErrorNotSupported;
  }

  return kErrorNone;
}

DisplayError STCIntfClient::ProcessOps(const ScOps op, const ScPayload &input, ScPayload *output) {
  lock_guard<mutex> obj(lock_);
  int ret;

  if (!stc_intf_) {
    return kErrorNotSupported;
  }
  if (!output) {
    DLOGE("Invalid parameters");
    return kErrorParameters;
  }

  DisplayError error = kErrorNone;
  ret = stc_intf_->ProcessOps(op, input, output);
  if (ret) {
    DLOGE("Failed to ProcessOps: %d, ret = %d", op, ret);
    return kErrorNotSupported;
  }

  return error;
}

DisplayError ColorManagerProxy::Init(const HWResourceInfo &hw_res_info) {
  DisplayError error = kErrorNone;

  // Load color service library and retrieve its entry points.
  if (color_lib_.Open(COLORMGR_LIBRARY_NAME)) {
    if (!color_lib_.Sym(CREATE_COLOR_INTERFACE_NAME, reinterpret_cast<void **>(&create_intf_)) ||
        !color_lib_.Sym(DESTROY_COLOR_INTERFACE_NAME, reinterpret_cast<void **>(&destroy_intf_))) {
      DLOGW("Fail to retrieve = %s from %s", CREATE_COLOR_INTERFACE_NAME, COLORMGR_LIBRARY_NAME);
      error = kErrorResources;
    }
  } else {
    DLOGW("Fail to load = %s", COLORMGR_LIBRARY_NAME);
    error = kErrorResources;
  }

  hw_res_info_ = hw_res_info;

  return error;
}

void ColorManagerProxy::Deinit() {
  color_lib_.~DynLib();
}

ColorManagerProxy::ColorManagerProxy(int32_t id, DisplayType type, HWInterface *intf,
                                     const HWDisplayAttributes &attr,
                                     const HWPanelInfo &info)
    : display_id_(id), device_type_(type), pp_hw_attributes_(), hw_intf_(intf),
      color_intf_(NULL), pp_features_(), feature_intf_(NULL) {
  int32_t enable_posted_start_dyn = 0;
  Debug::Get()->GetProperty(ENABLE_POSTED_START_DYN_PROP, &enable_posted_start_dyn);
  if (enable_posted_start_dyn && info.mode == kModeCommand) {
    feature_intf_ = GetPostedStartFeatureCheckIntf(intf, &pp_features_);
    if (!feature_intf_) {
      DLOGI("Failed to create feature interface");
    } else {
      DisplayError err = feature_intf_->Init();
      if (err) {
        DLOGE("Failed to init feature interface");
        delete feature_intf_;
        feature_intf_ = NULL;
      }
    }
  }

  convert_[kPbGamut] = &ColorManagerProxy::ConvertToGamut;
  convert_[kPbIgc] = &ColorManagerProxy::ConvertToIgc;
  convert_[kPbGC] = &ColorManagerProxy::ConvertToGc;
}

ColorManagerProxy *ColorManagerProxy::CreateColorManagerProxy(DisplayType type,
                                                              HWInterface *hw_intf,
                                                              const HWDisplayAttributes &attribute,
                                                              const HWPanelInfo &panel_info,
                                                              DppsControlInterface *dpps_intf) {
  DisplayError error = kErrorNone;
  PPFeatureVersion versions;
  int32_t display_id = -1;
  ColorManagerProxy *color_manager_proxy = NULL;

  // check if all resources are available before invoking factory method from libsdm-color.so.
  if (!color_lib_ || !create_intf_ || !destroy_intf_) {
    DLOGW("Information for %s isn't available!", COLORMGR_LIBRARY_NAME);
    return NULL;
  }

  hw_intf->GetDisplayId(&display_id);
  color_manager_proxy = new ColorManagerProxy(display_id, type, hw_intf, attribute, panel_info);

  if (color_manager_proxy) {
    // 1. need query post-processing feature version from HWInterface.
    error = color_manager_proxy->hw_intf_->GetPPFeaturesVersion(&versions);
    PPHWAttributes &hw_attr = color_manager_proxy->pp_hw_attributes_;
    if (error != kErrorNone) {
      DLOGW("Fail to get DSPP feature versions");
    } else {
      hw_attr.Set(hw_res_info_, panel_info, attribute, versions, dpps_intf);
      DLOGI("PAV2 version is versions = %d, version = %d ",
            hw_attr.version.version[kGlobalColorFeaturePaV2],
            versions.version[kGlobalColorFeaturePaV2]);
    }

    // 2. instantiate concrete ColorInterface from libsdm-color.so, pass all hardware info in.
    error = create_intf_(COLOR_VERSION_TAG, color_manager_proxy->display_id_,
                         color_manager_proxy->device_type_, hw_attr,
                         &color_manager_proxy->color_intf_);
    if (error != kErrorNone) {
      DLOGW("Unable to instantiate concrete ColorInterface from %s", COLORMGR_LIBRARY_NAME);
      delete color_manager_proxy;
      color_manager_proxy = NULL;
      return color_manager_proxy;
    }

    color_manager_proxy->stc_intf_client_ = new STCIntfClient();
    if (!color_manager_proxy->stc_intf_client_) {
      DLOGW("Unable to instantiate concrete StcInterface");
      return color_manager_proxy;
    }

    error = color_manager_proxy->stc_intf_client_->Init(hw_attr.panel_name);
    if (error != kErrorNone) {
      DLOGW("Failed to init StcInterface");
      delete color_manager_proxy->stc_intf_client_;
      color_manager_proxy->stc_intf_client_ = NULL;
      return color_manager_proxy;
    }
    color_manager_proxy->support_stc_tonemap_ = color_manager_proxy->GetSupportStcTonemap();
  }

  return color_manager_proxy;
}

ColorManagerProxy::~ColorManagerProxy() {
  if (destroy_intf_)
    destroy_intf_(display_id_);
  color_intf_ = NULL;
  if (feature_intf_) {
    feature_intf_->Deinit();
    delete feature_intf_;
    feature_intf_ = NULL;
  }
  if (stc_intf_client_) {
    stc_intf_client_->DeInit();
  }
}

DisplayError ColorManagerProxy::ColorSVCRequestRoute(const PPDisplayAPIPayload &in_payload,
                                                     PPDisplayAPIPayload *out_payload,
                                                     PPPendingParams *pending_action) {
  DisplayError ret = kErrorNone;

  // On completion, dspp_features_ will be populated and mark dirty with all resolved dspp
  // feature list with paramaters being transformed into target requirement.
  ret = color_intf_->ColorSVCRequestRoute(in_payload, out_payload, &pp_features_, pending_action);

  return ret;
}

DisplayError ColorManagerProxy::ApplyDefaultDisplayMode(void) {
  DisplayError ret = kErrorNone;

  // On POR, will be invoked from prepare<> request once bootanimation is done.
  ret = color_intf_->ApplyDefaultDisplayMode(&pp_features_);

  return ret;
}

bool ColorManagerProxy::NeedsPartialUpdateDisable() {
  Locker &locker(pp_features_.GetLocker());
  SCOPE_LOCK(locker);

  return pp_features_.IsDirty();
}

DisplayError ColorManagerProxy::Commit() {
  Locker &locker(pp_features_.GetLocker());
  SCOPE_LOCK(locker);

  DisplayError ret = kErrorNone;
  bool is_dirty = pp_features_.IsDirty();
  if (feature_intf_) {
    feature_intf_->SetParams(kFeatureSwitchMode, &is_dirty);
  }
  if (is_dirty) {
    ret = hw_intf_->SetPPFeatures(&pp_features_);
  }

  return ret;
}

void PPHWAttributes::Set(const HWResourceInfo &hw_res,
                         const HWPanelInfo &panel_info,
                         const DisplayConfigVariableInfo &attr,
                         const PPFeatureVersion &feature_ver,
                         DppsControlInterface *intf) {
  HWResourceInfo &res = *this;
  res = hw_res;
  HWPanelInfo &panel = *this;
  panel = panel_info;
  DisplayConfigVariableInfo &attributes = *this;
  attributes = attr;
  version = feature_ver;
  dpps_intf = intf;

  if (strlen(panel_info.panel_name)) {
    snprintf(&panel_name[0], sizeof(panel_name), "%s", &panel_info.panel_name[0]);
    char *tmp = panel_name;
    while ((tmp = strstr(tmp, " ")) != NULL)
      *tmp = '_';
    if ((tmp = strstr(panel_name, "\n")) != NULL)
      *tmp = '\0';
  }
}

DisplayError ColorManagerProxy::ColorMgrGetNumOfModes(uint32_t *mode_cnt) {
  return color_intf_->ColorIntfGetNumDisplayModes(&pp_features_, 0, mode_cnt);
}

DisplayError ColorManagerProxy::ColorMgrGetModes(uint32_t *mode_cnt,
                                                 SDEDisplayMode *modes) {
  return color_intf_->ColorIntfEnumerateDisplayModes(&pp_features_, 0, modes, mode_cnt);
}

DisplayError ColorManagerProxy::ColorMgrSetMode(int32_t color_mode_id) {
  return color_intf_->ColorIntfSetDisplayMode(&pp_features_, 0, color_mode_id);
}

DisplayError ColorManagerProxy::ColorMgrGetModeInfo(int32_t mode_id, AttrVal *query) {
  return color_intf_->ColorIntfGetModeInfo(&pp_features_, 0, mode_id, query);
}

DisplayError ColorManagerProxy::ColorMgrSetColorTransform(uint32_t length,
                                                          const double *trans_data) {
  return color_intf_->ColorIntfSetColorTransform(&pp_features_, 0, length, trans_data);
}

DisplayError ColorManagerProxy::ColorMgrGetDefaultModeID(int32_t *mode_id) {
  return color_intf_->ColorIntfGetDefaultModeID(&pp_features_, 0, mode_id);
}

DisplayError ColorManagerProxy::ColorMgrCombineColorModes() {
  return color_intf_->ColorIntfCombineColorModes();
}

DisplayError ColorManagerProxy::ColorMgrSetModeWithRenderIntent(int32_t color_mode_id,
                                         const PrimariesTransfer &blend_space, uint32_t intent) {
  cur_blend_space_ = blend_space;
  cur_intent_ = intent;
  cur_mode_id_ = color_mode_id;
  apply_mode_ = true;
  return kErrorNone;
}

DisplayError ColorManagerProxy::Validate(HWLayers *hw_layers) {
  DisplayError ret = kErrorNone;
  if (!hw_layers) {
    return ret;
  }

  bool updates = NeedHwassetsUpdate();
  bool valid_meta_data = false;
  bool update_meta_data = false, update_mode_Hwassets = false;
  Layer hdr_layer = {};
  bool hdr_present = false, hdr_plus_present = false;

  valid_meta_data = NeedsToneMap(hw_layers->info.hw_layers);
  if (valid_meta_data) {
    if (hw_layers->info.hdr_layer_info.in_hdr_mode &&
          hw_layers->info.hdr_layer_info.operation == HWHDRLayerInfo::kSet) {
      hdr_layer = *(hw_layers->info.stack->layers.at(
                                 UINT32(hw_layers->info.hdr_layer_info.layer_index)));
      hdr_present = true;
    }

    if (hdr_present) {
      if (hdr_layer.input_buffer.color_metadata.dynamicMetaDataValid &&
          hdr_layer.input_buffer.color_metadata.dynamicMetaDataLen) {
        hdr_plus_present = true;
      }
    }
  }

  if (apply_mode_) {
    update_mode_Hwassets = true;
    apply_mode_ = false;
  }

  if (updates) {
    update_mode_Hwassets = true;
  }

  if (hdr_present || hdr_plus_present) {
    update_meta_data = true;
    meta_data_ = hdr_layer.input_buffer.color_metadata;
    update_mode_Hwassets = true;
  }

  if (update_mode_Hwassets) {
    snapdragoncolor::ColorMode color_mode;
    color_mode = GetColorPrimaries(cur_blend_space_, cur_intent_);
    UpdateModeHwassets(cur_mode_id_, color_mode, update_meta_data, meta_data_);
    DumpColorMetaData(meta_data_);
  }

  return kErrorNone;
}

bool ColorManagerProxy::IsSupportStcTonemap() {
  return support_stc_tonemap_;
}

bool ColorManagerProxy::GameEnhanceSupported() {
  bool supported = false;

  if (color_intf_) {
    color_intf_->ColorIntfGameEnhancementSupported(&supported);
  }

  return supported;
}

DisplayError ColorManagerProxy::ConvertToPPFeatures(HwConfigOutputParams *params,
                                                    PPFeaturesConfig *out_data) {
  if (!params || !out_data) {
    DLOGE("Invalid input parameters");
    return kErrorParameters;
  }

  if (params->payload.empty()) {
    return kErrorNone;
  }

  DisplayError error = kErrorNone;
  for (auto it = params->payload.begin(); it != params->payload.end(); it++) {
    ConvertTable::const_iterator found = convert_.find(it->hw_asset);
    if (found == convert_.end()) {
      DLOGE("%s is not supported", it->hw_asset.c_str());
      return kErrorNotSupported;
    }

    ConvertProc func = found->second;
    error = (this->*func)(*it, out_data);
    if (error != kErrorNone) {
      DLOGE("Failer to convert %s, error = %d", it->hw_asset.c_str(), error);
      return error;
    }
  }

  return error;
}

DisplayError ColorManagerProxy::ConvertToIgc(const HwConfigPayload &in_data,
                                                   PPFeaturesConfig *out_data) {
  if (in_data.hw_payload_len != sizeof(GammaPostBlendConfig)) {
    DLOGE("Invalid parameters size = %d", in_data.hw_payload_len);
    return kErrorParameters;
  }
  DisplayError ret = kErrorNone;
  GammaPostBlendConfig *ptr = reinterpret_cast<GammaPostBlendConfig*>(in_data.hw_payload.get());
  if (!ptr) {
    DLOGE("Invalid parameters");
    return kErrorUndefined;
  }
  ret = color_intf_->ColorIntfConvertToPPFeature(out_data, UINT32(display_id_), ptr->enabled,
                           kPbIgc, reinterpret_cast<void *>(ptr));
  if (ret != kErrorNone) {
    DLOGE("Failed to convert Igc config date to PPFeture info: ret = %d", ret);
    return ret;
  }
  return ret;
}

DisplayError ColorManagerProxy::ConvertToGc(const HwConfigPayload &in_data,
                                                  PPFeaturesConfig *out_data) {
  if (in_data.hw_payload_len != sizeof(GammaPostBlendConfig)) {
    DLOGE("Invalid parameters size = %d", in_data.hw_payload_len);
    return kErrorParameters;
  }
  DisplayError ret = kErrorNone;
  GammaPostBlendConfig *ptr = reinterpret_cast<GammaPostBlendConfig*>(in_data.hw_payload.get());
  ret = color_intf_->ColorIntfConvertToPPFeature(out_data, UINT32(display_id_), ptr->enabled,
                           kPbGC, reinterpret_cast<void *>(ptr));
  if (ret != kErrorNone) {
    DLOGE("Failed to convert Gc config date to PPFeture info: ret = %d", ret);
    ret = kErrorUndefined;
    return ret;
  }
  return ret;
}

DisplayError ColorManagerProxy::ConvertToGamut(const HwConfigPayload &in_data,
                                                     PPFeaturesConfig *out_data) {
  if (in_data.hw_payload_len != sizeof(GamutConfig)) {
    DLOGE("Invalid parameters size = %d", in_data.hw_payload_len);
    return kErrorParameters;
  }

  DisplayError ret = kErrorNone;
  GamutConfig *ptr = reinterpret_cast<GamutConfig*>(in_data.hw_payload.get());
  ret = color_intf_->ColorIntfConvertToPPFeature(out_data, UINT32(display_id_), ptr->enabled,
                           kPbGamut, reinterpret_cast<void *>(&ptr->gamut_info));
  if (ret != kErrorNone) {
    DLOGE("Failed to convert Gamut config date to PPFeture info: ret = %d", ret);
    ret = kErrorUndefined;
    return ret;
  }
  return ret;
}

bool ColorManagerProxy::NeedHwassetsUpdate() {
  bool need_update = false;
  if (!stc_intf_client_) {
    return need_update;
  }
  ScPayload payload;

  payload.len = sizeof(need_update);
  payload.prop = kNeedsUpdate;
  payload.payload = reinterpret_cast<uint64_t>(&need_update);
  stc_intf_client_->GetProperty(&payload);
  return need_update;
}

DisplayError ColorManagerProxy::UpdateModeHwassets(int32_t mode_id,
                                  snapdragoncolor::ColorMode color_mode, bool valid_meta_data,
                                  const ColorMetaData &meta_data) {
  if (!stc_intf_client_) {
    return kErrorUndefined;
  }

  DisplayError error = kErrorNone;
  struct snapdragoncolor::ModeRenderInputParams mode_params = {};
  struct snapdragoncolor::HwConfigOutputParams hw_params = {};
  mode_params.valid_meta_data = valid_meta_data;
  mode_params.meta_data = meta_data;
  mode_params.color_mode = color_mode;
  mode_params.mode_id = mode_id;

  struct snapdragoncolor::HwConfigPayload payload = {};
  payload.hw_asset = kPbGamut;
  payload.hw_payload_len = sizeof(GamutConfig);
  payload.hw_payload = std::make_shared<GamutConfig>();
  hw_params.payload.push_back(payload);

  payload.hw_asset = kPbIgc;
  payload.hw_payload = std::make_shared<GammaPostBlendConfig>(LUT1D_ENTRIES_SIZE);
  payload.hw_payload_len = sizeof(GammaPostBlendConfig);
  hw_params.payload.push_back(payload);

  payload.hw_asset = kPbGC;
  payload.hw_payload_len = sizeof(GammaPostBlendConfig);
  payload.hw_payload = std::make_shared<GammaPostBlendConfig>(LUT3D_GC_ENTRIES_SIZE);
  hw_params.payload.push_back(payload);

  ScPayload in_data = {};
  ScPayload out_data = {};
  in_data.prop = kModeRenderInputParams;
  in_data.len = sizeof(mode_params);
  in_data.payload = reinterpret_cast<uint64_t>(&mode_params);

  out_data.prop = kHwConfigPayloadParam;
  out_data.len = sizeof(hw_params);
  out_data.payload = reinterpret_cast<uint64_t>(&hw_params);
  error = stc_intf_client_->ProcessOps(kScModeRenderIntent, in_data, &out_data);
  if (error != kErrorNone) {
    DLOGE("Failed to call ProcessOps, error = %d", error);
    return error;
  }

  error = ConvertToPPFeatures(&hw_params, &pp_features_);
  if (error != kErrorNone) {
    DLOGE("Failed to convert hw assets to PP features, error = %d", error);
    return kErrorUndefined;
  }
  pp_features_.MarkAsDirty();
  return error;
}

bool ColorManagerProxy::GetSupportStcTonemap() {
  bool support_tonemap = false;
  if (!stc_intf_client_) {
    return support_tonemap;
  }
  ScPayload payload;

  payload.len = sizeof(support_tonemap);
  payload.prop = kSupportToneMap;
  payload.payload = reinterpret_cast<uint64_t>(&support_tonemap);
  stc_intf_client_->GetProperty(&payload);
  return support_tonemap;
}

void ColorManagerProxy::DumpColorMetaData(const ColorMetaData &color_metadata) {
  DLOGI_IF(kTagResources, "Primaries = %d, Range = %d, Transfer = %d, Matrix Coeffs = %d",
           color_metadata.colorPrimaries, color_metadata.range, color_metadata.transfer,
           color_metadata.matrixCoefficients);

  for (uint32_t i = 0; i < 3; i++) {
    for (uint32_t j = 0; j < 2; j++) {
      DLOGV_IF(kTagResources, "RGB Primaries[%d][%d] = %d", i, j,
               color_metadata.masteringDisplayInfo.primaries.rgbPrimaries[i][j]);
    }
  }
  DLOGV_IF(kTagResources, "White Point[0] = %d White Point[1] = %d",
           color_metadata.masteringDisplayInfo.primaries.whitePoint[0],
           color_metadata.masteringDisplayInfo.primaries.whitePoint[1]);
  DLOGV_IF(kTagResources, "Max Disp Luminance = %d Min Disp Luminance= %d",
           color_metadata.masteringDisplayInfo.maxDisplayLuminance,
           color_metadata.masteringDisplayInfo.minDisplayLuminance);
  DLOGV_IF(kTagResources, "Max ContentLightLevel = %d Max AvgLightLevel = %d",
           color_metadata.contentLightLevel.maxContentLightLevel,
           color_metadata.contentLightLevel.minPicAverageLightLevel);
  DLOGV_IF(kTagResources, "DynamicMetaDataValid = %d DynamicMetaDataLen = %d",
           color_metadata.dynamicMetaDataValid,
           color_metadata.dynamicMetaDataLen);
}

snapdragoncolor::ColorMode ColorManagerProxy::GetColorPrimaries(
                        const PrimariesTransfer &blend_space, uint32_t intent) {
  snapdragoncolor::ColorMode mode = {};

  mode.intent = static_cast<snapdragoncolor::RenderIntent>(intent + 1);

  mode.gamut = blend_space.primaries;
  mode.gamma = blend_space.transfer;

  return mode;
}

ColorFeatureCheckingImpl::ColorFeatureCheckingImpl(HWInterface *hw_intf,
                                                   PPFeaturesConfig *pp_features)
  : hw_intf_(hw_intf), pp_features_(pp_features) {}

DisplayError ColorFeatureCheckingImpl::Init() {
  states_.at(kFrameTriggerDefault) = new FeatureStateDefaultTrigger(this);
  states_.at(kFrameTriggerSerialize) = new FeatureStateSerializedTrigger(this);
  states_.at(kFrameTriggerPostedStart) = new FeatureStatePostedStart(this);

  if (std::any_of(states_.begin(), states_.end(),
      [](const FeatureInterface *p) {
      if (!p) {
        return true;
      } else {
        return false;
      }})) {
    std::all_of(states_.begin(), states_.end(),
      [](const FeatureInterface *p) {
      if (p) {delete p;} return true;});
    states_.fill(NULL);
    curr_state_ = NULL;
  } else {
    curr_state_ = states_.at(kFrameTriggerDefault);
  }

  if (curr_state_) {
    single_buffer_feature_.clear();
    single_buffer_feature_.push_back(kGlobalColorFeatureIgc);
    single_buffer_feature_.push_back(kGlobalColorFeatureGamut);
  } else {
    DLOGE("Failed to create curr_state_");
    return kErrorMemory;
  }
  return kErrorNone;
}

DisplayError ColorFeatureCheckingImpl::Deinit() {
  std::all_of(states_.begin(), states_.end(),
    [](const FeatureInterface *p)
    {if (p) {delete p;} return true;});
  states_.fill(NULL);
  curr_state_ = NULL;
  single_buffer_feature_.clear();
  return kErrorNone;
}

DisplayError ColorFeatureCheckingImpl::SetParams(FeatureOps param_type,
                                                 void *payload) {
  DisplayError error = kErrorNone;
  FrameTriggerMode mode = kFrameTriggerDefault;

  if (!payload) {
    DLOGE("Invalid input payload");
    return kErrorParameters;
  }

  if (!curr_state_) {
    DLOGE("Invalid curr state");
    return kErrorParameters;
  }

  bool is_dirty = *reinterpret_cast<bool *>(payload);
  switch (param_type) {
  case kFeatureSwitchMode:
    if (is_dirty) {
      CheckColorFeature(&mode);
    } else {
      mode = kFrameTriggerPostedStart;
    }
    DLOGV_IF(kTagQDCM, "Set frame trigger mode %d", mode);
    error = curr_state_->SetParams(param_type, &mode);
    if (error) {
      DLOGE_IF(kTagQDCM, "Failed to set params to state, error %d", error);
    }
    break;
  default:
    DLOGW("unhandled param_type %d", param_type);
    error = kErrorNotSupported;
    break;
  }
  return error;
}

DisplayError ColorFeatureCheckingImpl::GetParams(FeatureOps param_type,
                                                 void *payload) {
  DisplayError error = kErrorNone;

  if (!payload) {
    DLOGE("Invalid input payload");
    return kErrorParameters;
  }

  if (!curr_state_) {
    DLOGE("Invalid curr state");
    return kErrorParameters;
  }

  switch (param_type) {
  case kFeatureSwitchMode:
    if (curr_state_) {
      curr_state_->GetParams(param_type, payload);
    } else {
      DLOGE_IF(kTagQDCM, "curr_state_ NULL");
      error = kErrorUndefined;
    }
    break;
  default:
    DLOGW("unhandled param_type %d", param_type);
    error = kErrorNotSupported;
    break;
  }
  return error;
}

// This function checks through the feature list for the single buffer features.
// If there is single buffer feature existed in the feature list, the posted start
// should be disabled.
void ColorFeatureCheckingImpl::CheckColorFeature(FrameTriggerMode *mode) {
  PPFeatureInfo *feature = NULL;
  PPGlobalColorFeatureID id = kMaxNumPPFeatures;

  if (!pp_features_) {
    DLOGW("Invalid pp features");
    *mode = kFrameTriggerPostedStart;
    return;
  }

  for (uint32_t i = 0; i < single_buffer_feature_.size(); i++) {
    id = single_buffer_feature_[i];
    feature = pp_features_->GetFeature(id);
    if (feature && (feature->enable_flags_ & kOpsEnable)) {
      *mode = kFrameTriggerDefault;
      return;
    }
  }

  *mode = kFrameTriggerPostedStart;
}

FeatureStatePostedStart::FeatureStatePostedStart(ColorFeatureCheckingImpl *obj)
  : obj_(obj) {}

DisplayError FeatureStatePostedStart::Init() {
  return kErrorNone;
}

DisplayError FeatureStatePostedStart::Deinit() {
  return kErrorNone;
}

DisplayError FeatureStatePostedStart::SetParams(FeatureOps param_type,
                                                void *payload) {
  DisplayError error = kErrorNone;
  FrameTriggerMode mode = kFrameTriggerPostedStart;

  if (!obj_) {
    DLOGE("Invalid param obj_");
    return kErrorParameters;
  }

  if (!payload) {
    DLOGE("Invalid payload");
    return kErrorParameters;
  }

  switch (param_type) {
  case kFeatureSwitchMode:
    mode = *(reinterpret_cast<FrameTriggerMode *>(payload));
    if (mode >= kFrameTriggerMax) {
      DLOGE("Invalid mode %d", mode);
      return kErrorParameters;
    }
    if (mode != kFrameTriggerPostedStart) {
      error = obj_->hw_intf_->SetFrameTrigger(mode);
      if (!error) {
        obj_->curr_state_ = obj_->states_.at(mode);
      }
    } else {
      DLOGV_IF(kTagQDCM, "Already in posted start mode");
    }
    break;
  default:
    DLOGW("unhandled param_type %d", param_type);
    error = kErrorNotSupported;
    break;
  }
  return error;
}

DisplayError FeatureStatePostedStart::GetParams(FeatureOps param_type,
                                                void *payload) {
  DisplayError error = kErrorNone;

  if (!obj_) {
    DLOGE("Invalid param obj_");
    return kErrorParameters;
  }

  if (!payload) {
    DLOGE("Invalid payload");
    return kErrorParameters;
  }

  switch (param_type) {
  case kFeatureSwitchMode:
    *(reinterpret_cast<FrameTriggerMode *>(payload)) = kFrameTriggerPostedStart;
    break;
  default:
    DLOGW("unhandled param_type %d", param_type);
    error = kErrorNotSupported;
    break;
  }

  return error;
}

FeatureStateDefaultTrigger::FeatureStateDefaultTrigger(ColorFeatureCheckingImpl *obj)
  : obj_(obj) {}

DisplayError FeatureStateDefaultTrigger::Init() {
  return kErrorNone;
}

DisplayError FeatureStateDefaultTrigger::Deinit() {
  return kErrorNone;
}

DisplayError FeatureStateDefaultTrigger::SetParams(FeatureOps param_type,
                                                   void *payload) {
  DisplayError error = kErrorNone;
  FrameTriggerMode mode = kFrameTriggerDefault;

  if (!obj_) {
    DLOGE("Invalid param obj_");
    return kErrorParameters;
  }

  if (!payload) {
    DLOGE("Invalid payload");
    return kErrorParameters;
  }

  switch (param_type) {
  case kFeatureSwitchMode:
    mode = *(reinterpret_cast<FrameTriggerMode *>(payload));
    if (mode >= kFrameTriggerMax) {
      DLOGE("Invalid mode %d", mode);
      return kErrorParameters;
    }
    if (mode != kFrameTriggerDefault) {
      error = obj_->hw_intf_->SetFrameTrigger(mode);
      if (!error) {
        obj_->curr_state_ = obj_->states_.at(mode);
      }
    } else {
      DLOGV_IF(kTagQDCM, "Already in default trigger mode");
    }
    break;
  default:
    DLOGW("unhandled param_type %d", param_type);
    error = kErrorNotSupported;
    break;
  }
  return error;
}

DisplayError FeatureStateDefaultTrigger::GetParams(FeatureOps param_type,
                                                   void *payload) {
  DisplayError error = kErrorNone;

  if (!obj_) {
    DLOGE("Invalid param obj_");
    return kErrorParameters;
  }

  if (!payload) {
    DLOGE("Invalid payload");
    return kErrorParameters;
  }

  switch (param_type) {
  case kFeatureSwitchMode:
    *(reinterpret_cast<FrameTriggerMode *>(payload)) = kFrameTriggerDefault;
    break;
  default:
    DLOGW("unhandled param_type %d", param_type);
    error = kErrorNotSupported;
    break;
  }

  return error;
}

FeatureStateSerializedTrigger::FeatureStateSerializedTrigger(ColorFeatureCheckingImpl *obj)
  : obj_(obj) {}

DisplayError FeatureStateSerializedTrigger::Init() {
  return kErrorNone;
}

DisplayError FeatureStateSerializedTrigger::Deinit() {
  return kErrorNone;
}

DisplayError FeatureStateSerializedTrigger::SetParams(FeatureOps param_type,
                                                      void *payload) {
  DisplayError error = kErrorNone;
  FrameTriggerMode mode = kFrameTriggerSerialize;

  if (!obj_) {
    DLOGE("Invalid param obj_");
    return kErrorParameters;
  }

  if (!payload) {
    DLOGE("Invalid payload");
    return kErrorParameters;
  }

  switch (param_type) {
  case kFeatureSwitchMode:
    mode = *(reinterpret_cast<FrameTriggerMode *>(payload));
    if (mode >= kFrameTriggerMax) {
      DLOGE("Invalid mode %d", mode);
      return kErrorParameters;
    }
    if (mode != kFrameTriggerSerialize) {
      error = obj_->hw_intf_->SetFrameTrigger(mode);
      if (!error) {
        obj_->curr_state_ = obj_->states_.at(mode);
      }
    } else {
      DLOGV_IF(kTagQDCM, "Already in serialized trigger mode");
    }
    break;
  default:
    DLOGW("unhandled param_type %d", param_type);
    error = kErrorNotSupported;
    break;
  }
  return error;
}

DisplayError FeatureStateSerializedTrigger::GetParams(FeatureOps param_type,
                                                      void *payload) {
  DisplayError error = kErrorNone;

  if (!obj_) {
    DLOGE("Invalid param obj_");
    return kErrorParameters;
  }

  if (!payload) {
    DLOGE("Invalid payload");
    return kErrorParameters;
  }

  switch (param_type) {
  case kFeatureSwitchMode:
    *(reinterpret_cast<FrameTriggerMode *>(payload)) = kFrameTriggerSerialize;
    break;
  default:
    DLOGW("unhandled param_type %d", param_type);
    error = kErrorNotSupported;
    break;
  }

  return error;
}

}  // namespace sdm
