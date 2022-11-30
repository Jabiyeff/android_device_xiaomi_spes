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

#ifndef __DRM_PANEL_FEATURE_MGR_H__
#define __DRM_PANEL_FEATURE_MGR_H__

#include <vector>
#include <mutex>

#include "drm_interface.h"
#include "drm_property.h"
#include "drm_panel_feature_mgr_intf.h"

namespace sde_drm {

class DRMPanelFeatureMgr : public DRMPanelFeatureMgrIntf {
 public:
  virtual ~DRMPanelFeatureMgr() {}
  void Init(int fd, drmModeRes* res);
  void DeInit();
  void GetPanelFeatureInfo(DRMPanelFeatureInfo *info);
  void CachePanelFeature(const DRMPanelFeatureInfo &info);
  void CommitPanelFeatures(drmModeAtomicReq *req, const DRMDisplayToken &tok);

 private:
  int InitObjectProps(int obj_id, int obj_type);
  void ParseCapabilities(uint32_t blob_id, char* value, uint32_t max_len, const std::string str);
  void ParseDsppCapabilities(uint32_t blob_id, std::vector<int> *values, uint32_t *size,
                             const std::string str);

  std::mutex lock_;
  int dev_fd_ = -1;
  drmModeRes* drm_res_ = nullptr;
  DRMPropertyManager prop_mgr_ {};
  std::array<std::pair<bool, struct DRMPanelFeatureInfo>,
          kDRMPanelFeatureMax> dirty_features_ {};
  std::map<DRMPanelFeatureID, DRMProperty> drm_property_map_ {};
  std::map<DRMPanelFeatureID, DRMPropType> drm_prop_type_map_ {};
  std::map<DRMPanelFeatureID, uint32_t> drm_prop_blob_ids_map_ {};
  std::array<DRMPanelFeatureInfo, kDRMPanelFeatureMax> feature_info_tbl_ {};
};

/*
Below structure is required for cross compatibility between
different kernel versions.
This structure needs to be in sync with kernel structure.
*/

#define RC_DATA_SIZE_MAX   2720
#define RC_CFG_SIZE_MAX       4

struct msm_rc_mask_cfg {
  __u64 flags;
  __u32 cfg_param_01;
  __u32 cfg_param_02;
  __u32 cfg_param_03;
  __u32 cfg_param_04[RC_CFG_SIZE_MAX];
  __u32 cfg_param_05[RC_CFG_SIZE_MAX];
  __u32 cfg_param_06[RC_CFG_SIZE_MAX];
  __u64 cfg_param_07;
  __u32 cfg_param_08;
  __u64 cfg_param_09[RC_DATA_SIZE_MAX];
};

}  // namespace sde_drm

#endif  // __DRM_PANEL_FEATURE_MGR_H__

