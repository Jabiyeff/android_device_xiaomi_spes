/*
* Copyright (c) 2019, 2021, The Linux Foundation. All rights reserved.
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

#include <drm_logger.h>

#include "drm_atomic_req.h"
#include "drm_connector.h"
#include "drm_crtc.h"
#include "drm_manager.h"
#include "drm_plane.h"
#include "string.h"

#define __CLASS__ "DRMAtomicReq"

namespace sde_drm {

DRMAtomicReq::DRMAtomicReq(int fd, DRMManager *drm_mgr) : drm_mgr_(drm_mgr), fd_(fd) {}

DRMAtomicReq::~DRMAtomicReq() {
  if (drm_atomic_req_) {
    drmModeAtomicFree(drm_atomic_req_);
    drm_atomic_req_ = nullptr;
  }
}

int DRMAtomicReq::Init(const DRMDisplayToken &tok) {
  token_ = tok;
  drm_atomic_req_ = drmModeAtomicAlloc();
  if (!drm_atomic_req_) {
    return -ENOMEM;
  }

  return 0;
}

int DRMAtomicReq::Perform(DRMOps opcode, uint32_t obj_id, ...) {
  va_list args;
  va_start(args, obj_id);
  switch (opcode) {
    case DRMOps::PLANE_SET_SRC_RECT:
    case DRMOps::PLANE_SET_DST_RECT:
    case DRMOps::PLANE_SET_ZORDER:
    case DRMOps::PLANE_SET_ROTATION:
    case DRMOps::PLANE_SET_ALPHA:
    case DRMOps::PLANE_SET_BLEND_TYPE:
    case DRMOps::PLANE_SET_H_DECIMATION:
    case DRMOps::PLANE_SET_V_DECIMATION:
    case DRMOps::PLANE_SET_FB_ID:
    case DRMOps::PLANE_SET_ROT_FB_ID:
    case DRMOps::PLANE_SET_CRTC:
    case DRMOps::PLANE_SET_SRC_CONFIG:
    case DRMOps::PLANE_SET_INPUT_FENCE:
    case DRMOps::PLANE_SET_SCALER_CONFIG:
    case DRMOps::PLANE_SET_FB_SECURE_MODE:
    case DRMOps::PLANE_SET_CSC_CONFIG:
    case DRMOps::PLANE_SET_MULTIRECT_MODE:
    case DRMOps::PLANE_SET_EXCL_RECT:
    case DRMOps::PLANE_SET_INVERSE_PMA:
    case DRMOps::PLANE_SET_DGM_CSC_CONFIG:
    case DRMOps::PLANE_SET_POST_PROC:
    case DRMOps::PLANE_SET_SSPP_LAYOUT: {
      drm_mgr_->GetPlaneMgr()->Perform(opcode, obj_id, drm_atomic_req_, args);
    } break;
    case DRMOps::CRTC_SET_POST_PROC:
    case DRMOps::CRTC_SET_MODE:
    case DRMOps::CRTC_SET_ACTIVE:
    case DRMOps::CRTC_SET_OUTPUT_FENCE_OFFSET:
    case DRMOps::CRTC_SET_CORE_CLK:
    case DRMOps::CRTC_SET_CORE_AB:
    case DRMOps::CRTC_SET_CORE_IB:
    case DRMOps::CRTC_SET_LLCC_AB:
    case DRMOps::CRTC_SET_LLCC_IB:
    case DRMOps::CRTC_SET_DRAM_AB:
    case DRMOps::CRTC_SET_DRAM_IB:
    case DRMOps::CRTC_SET_ROT_PREFILL_BW:
    case DRMOps::CRTC_SET_ROT_CLK:
    case DRMOps::CRTC_GET_RELEASE_FENCE:
    case DRMOps::CRTC_SET_ROI:
    case DRMOps::CRTC_SET_SECURITY_LEVEL:
    case DRMOps::CRTC_SET_SOLIDFILL_STAGES:
    case DRMOps::CRTC_SET_IDLE_TIMEOUT:
    case DRMOps::CRTC_SET_DEST_SCALER_CONFIG:
    case DRMOps::CRTC_SET_CAPTURE_MODE:
    case DRMOps::CRTC_SET_IDLE_PC_STATE: {
      drm_mgr_->GetCrtcMgr()->Perform(opcode, obj_id, drm_atomic_req_, args);
    } break;
    case DRMOps::CONNECTOR_SET_CRTC:
    case DRMOps::CONNECTOR_GET_RETIRE_FENCE:
    case DRMOps::CONNECTOR_SET_OUTPUT_RECT:
    case DRMOps::CONNECTOR_SET_OUTPUT_FB_ID:
    case DRMOps::CONNECTOR_SET_POWER_MODE:
    case DRMOps::CONNECTOR_SET_ROI:
    case DRMOps::CONNECTOR_SET_AUTOREFRESH:
    case DRMOps::CONNECTOR_SET_FB_SECURE_MODE:
    case DRMOps::CONNECTOR_SET_POST_PROC:
    case DRMOps::CONNECTOR_SET_HDR_METADATA:
    case DRMOps::CONNECTOR_SET_QSYNC_MODE:
    case DRMOps::CONNECTOR_SET_TOPOLOGY_CONTROL:
    case DRMOps::CONNECTOR_SET_FRAME_TRIGGER:
    case DRMOps::CONNECTOR_SET_COLORSPACE: {
      drm_mgr_->GetConnectorMgr()->Perform(opcode, obj_id, drm_atomic_req_, args);
    } break;
    case DRMOps::DPPS_CACHE_FEATURE: {
      drm_mgr_->GetDppsMgrIntf()->CacheDppsFeature(obj_id, args);
    } break;
    case DRMOps::DPPS_COMMIT_FEATURE: {
      drm_mgr_->GetDppsMgrIntf()->CommitDppsFeatures(drm_atomic_req_, token_);
    } break;
    case DRMOps::COMMIT_PANEL_FEATURES: {
      drm_mgr_->GetPanelFeatureMgrIntf()->CommitPanelFeatures(drm_atomic_req_, token_);
    } break;
    default:
      DRM_LOGE("Invalid opcode %d", opcode);
  }
  va_end(args);
  return 0;
}

int DRMAtomicReq::Validate() {
  // Call UnsetUnusedPlanes to find planes that need to be unset. Do not call CommitPlaneState,
  // because we just want to validate, not actually mark planes as removed
  drm_mgr_->GetPlaneMgr()->UnsetUnusedResources(token_.crtc_id, false/*is_commit*/,
                                                drm_atomic_req_);

  int ret = drmModeAtomicCommit(fd_, drm_atomic_req_,
                                DRM_MODE_ATOMIC_ALLOW_MODESET | DRM_MODE_ATOMIC_TEST_ONLY, nullptr);
  if (ret) {
    DRM_LOGE("drmModeAtomicCommit failed with error %d (%s).", errno, strerror(errno));
  }

  drm_mgr_->GetPlaneMgr()->PostValidate(token_.crtc_id, !ret);
  drm_mgr_->GetCrtcMgr()->PostValidate(token_.crtc_id, !ret);
  drmModeAtomicSetCursor(drm_atomic_req_, 0);

  return ret;
}

int DRMAtomicReq::Commit(bool synchronous, bool retain_planes) {
  DTRACE_SCOPED();
  if (retain_planes) {
    // It is not enough to simply avoid calling UnsetUnusedPlanes, since state transitons have to
    // be correct when CommitPlaneState is called
    drm_mgr_->GetPlaneMgr()->RetainPlanes(token_.crtc_id);
  }

  drm_mgr_->GetPlaneMgr()->UnsetUnusedResources(token_.crtc_id, true/*is_commit*/, drm_atomic_req_);

  uint32_t flags = DRM_MODE_ATOMIC_ALLOW_MODESET;

  if (!synchronous) {
    flags |= DRM_MODE_ATOMIC_NONBLOCK;
  }

  int ret = drmModeAtomicCommit(fd_, drm_atomic_req_, flags, nullptr);
  if (ret) {
    DRM_LOGE("drmModeAtomicCommit failed with error %d (%s).", errno, strerror(errno));
  }

  drm_mgr_->GetPlaneMgr()->PostCommit(token_.crtc_id, !ret);
  drm_mgr_->GetCrtcMgr()->PostCommit(token_.crtc_id, !ret);
  drmModeAtomicSetCursor(drm_atomic_req_, 0);

  return ret;
}

}  // namespace sde_drm
