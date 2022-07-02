/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of The Linux Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#ifndef __RC_INTF_H__
#define __RC_INTF_H__

#include <core/display_interface.h>
#include <string>

#include "generic_intf.h"
#include "generic_payload.h"

namespace sdm {

struct RCInputConfig {
  int32_t display_id = -1;                 // Used only for mask dumping and logging.
  DisplayType display_type = kDisplayMax;  // Used only for mask dumping and logging.
  uint32_t display_xres = 0;
  uint32_t display_yres = 0;
  uint32_t max_mem_size = 0;
};

struct RCOutputConfig {
  int32_t top_width = 0;
  int32_t top_height = 0;
  int32_t bottom_width = 0;
  int32_t bottom_height = 0;
  bool rc_needs_full_roi = false;
};

// These value is to get the status of the mask
// kStatusIgnore: Either there is no mask layers or mask generation in progress.
// kStatusRcMaskStackHandled: Mask is successfully created by thread and mask layer got
//                            dropped in prepare.
// kStatusRcMaskStackDirty: Mask is successfully created by thread but need to call neteds validate,
//                          to make SF drop the mask layers.
enum RCMaskStackStatus {
  kStatusIgnore,
  kStatusRcMaskStackHandled,
  kStatusRcMaskStackDirty,
};

struct RCMaskCfgState {
  RCMaskStackStatus rc_mask_state = kStatusIgnore;
  bool rc_pu_full_roi = false;  // Unused, as driver is handling PU
};

// RC specific params as enum
enum RCFeatureParams {
  kRCFeatureDisplayId,
  kRCFeatureDisplayType,
  kRCFeatureDisplayXRes,
  kRCFeatureDisplayYRes,
  kRCFeatureResetHW,
  kRCFeatureQueryDspp,
  kRCFeatureParamMax,
};

// RC specific ops as enum
enum RCFeatureOps {
  kRCFeaturePrepare,
  kRCFeatureCommit,
  kRCFeatureOpsMax,
};

using RCIntf = GenericIntf<RCFeatureParams, RCFeatureOps, GenericPayload>;

}  // namespace sdm

#endif  // __RC_INTF_H__
