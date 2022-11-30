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

#ifndef __DRM_PROPERTY_H__
#define __DRM_PROPERTY_H__

#include <stdint.h>
#include <string>

namespace sde_drm {

enum struct DRMProperty {
  INVALID,
  TYPE,
  FB_ID,
  CRTC_ID,
  CRTC_X,
  CRTC_Y,
  CRTC_W,
  CRTC_H,
  SRC_X,
  SRC_Y,
  SRC_W,
  SRC_H,
  ZPOS,
  ALPHA,
  EXCL_RECT,
  H_DECIMATE,
  V_DECIMATE,
  INPUT_FENCE,
  ROTATION,
  BLEND_OP,
  SRC_CONFIG,
  SCALER_V1,
  SCALER_V2,
  CSC_V1,
  CAPABILITIES,
  MODE_PROPERTIES,
  LUT_ED,
  LUT_CIR,
  LUT_SEP,
  ROTATOR_CAPS_V1,
  TRUE_INLINE_ROT_REV,
  FB_TRANSLATION_MODE,
  ACTIVE,
  MODE_ID,
  OUTPUT_FENCE_OFFSET,
  OUTPUT_FENCE,
  ROI_V1,
  CORE_CLK,
  CORE_AB,
  CORE_IB,
  LLCC_AB,
  LLCC_IB,
  DRAM_AB,
  DRAM_IB,
  ROT_PREFILL_BW,
  ROT_CLK,
  SECURITY_LEVEL,
  DIM_STAGES_V1,
  IDLE_TIME,
  RETIRE_FENCE,
  DST_X,
  DST_Y,
  DST_W,
  DST_H,
  LP,
  HDR_PROPERTIES,
  DEST_SCALER,
  DS_LUT_ED,
  DS_LUT_CIR,
  DS_LUT_SEP,
  SDE_DSPP_GAMUT_V3,
  SDE_DSPP_GAMUT_V4,
  SDE_DSPP_GAMUT_V5,
  SDE_DSPP_GC_V1,
  SDE_DSPP_GC_V2,
  SDE_DSPP_IGC_V2,
  SDE_DSPP_IGC_V3,
  SDE_DSPP_IGC_V4,
  SDE_DSPP_PCC_V3,
  SDE_DSPP_PCC_V4,
  SDE_DSPP_PCC_V5,
  SDE_DSPP_PA_HSIC_V1,
  SDE_DSPP_PA_HSIC_V2,
  SDE_DSPP_PA_SIXZONE_V1,
  SDE_DSPP_PA_SIXZONE_V2,
  SDE_DSPP_PA_MEMCOL_SKIN_V1,
  SDE_DSPP_PA_MEMCOL_SKIN_V2,
  SDE_DSPP_PA_MEMCOL_SKY_V1,
  SDE_DSPP_PA_MEMCOL_SKY_V2,
  SDE_DSPP_PA_MEMCOL_FOLIAGE_V1,
  SDE_DSPP_PA_MEMCOL_FOLIAGE_V2,
  SDE_DSPP_PA_MEMCOL_PROT_V1,
  SDE_DSPP_PA_MEMCOL_PROT_V2,
  AUTOREFRESH,
  EXT_HDR_PROPERTIES,
  HDR_METADATA,
  MULTIRECT_MODE,
  ROT_FB_ID,
  SDE_DSPP_PA_DITHER_V1,
  SDE_DSPP_PA_DITHER_V2,
  SDE_PP_DITHER_V1,
  SDE_PP_DITHER_V2,
  INVERSE_PMA,
  CSC_DMA_V1,
  SDE_DGM_1D_LUT_IGC_V5,
  SDE_DGM_1D_LUT_GC_V5,
  SDE_VIG_1D_LUT_IGC_V5,
  SDE_VIG_1D_LUT_IGC_V6,
  SDE_VIG_3D_LUT_GAMUT_V5,
  SDE_VIG_3D_LUT_GAMUT_V6,
  SDE_DSPP_AD4_MODE,
  SDE_DSPP_AD4_INIT,
  SDE_DSPP_AD4_CFG,
  SDE_DSPP_AD4_INPUT,
  SDE_DSPP_AD4_BACKLIGHT,
  SDE_DSPP_AD4_ROI,
  SDE_DSPP_AD4_ASSERTIVENESS,
  SDE_DSPP_AD4_STRENGTH,
  SDE_DSPP_ABA_HIST_CTRL,
  SDE_DSPP_ABA_HIST_IRQ,
  SDE_DSPP_ABA_LUT,
  SDE_DSPP_SV_BL_SCALE,
  SDE_DSPP_BL_SCALE,
  CAPTURE_MODE,
  QSYNC_MODE,
  IDLE_PC_STATE,
  TOPOLOGY_CONTROL,
  EDID,
  SDE_LTM_VERSION,
  SDE_LTM_INIT,
  SDE_LTM_CFG,
  SDE_LTM_NOISE_THRESH,
  SDE_LTM_HIST_CTRL,
  SDE_LTM_BUFFER_CTRL,
  SDE_LTM_QUEUE_BUFFER,
  SDE_LTM_QUEUE_BUFFER2,
  SDE_LTM_QUEUE_BUFFER3,
  SDE_LTM_VLUT,
  FRAME_TRIGGER,
  COLORSPACE,
  SUPPORTED_COLORSPACES,
  SDE_SSPP_LAYOUT,
  DSPP_RC_MASK_V1,
  DSPP_CAPABILITIES,

  // Insert above
  MAX
};

struct DRMPropertyManager {
  DRMProperty GetPropertyEnum(const std::string &name) const;

  void SetPropertyId(DRMProperty prop_enum, uint32_t prop_id) {
    properties_[(uint32_t)prop_enum] = prop_id;
  }

  uint32_t GetPropertyId(DRMProperty prop_enum) const {
    return properties_[(uint32_t)prop_enum];
  }

  bool IsPropertyAvailable(DRMProperty prop_enum) const {
    return !!properties_[(uint32_t)prop_enum];
  }

 private:
  uint32_t properties_[(uint32_t)DRMProperty::MAX] {};
};

}  // namespace sde_drm

#endif  // __DRM_PROPERTY_H__
