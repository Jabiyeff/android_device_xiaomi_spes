/*
 * Copyright (c) 2014-2020, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

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

#ifndef __HWC_DISPLAY_PLUGGABLE_H__
#define __HWC_DISPLAY_PLUGGABLE_H__

#include "hwc_display.h"
#include "display_null.h"
#include "hwc_display_event_handler.h"

namespace sdm {

class HWCDisplayPluggable : public HWCDisplay {
 public:
  static int Create(CoreInterface *core_intf, HWCBufferAllocator *buffer_allocator,
                    HWCCallbacks *callbacks, HWCDisplayEventHandler *event_handler,
                    qService::QService *qservice, hwc2_display_t id,
                    int32_t sdm_id, uint32_t primary_width, uint32_t primary_height,
                    bool use_primary_res, HWCDisplay **hwc_display);
  static void Destroy(HWCDisplay *hwc_display);
  virtual int Init();
  virtual HWC2::Error Validate(uint32_t *out_num_types, uint32_t *out_num_requests);
  virtual HWC2::Error Present(shared_ptr<Fence> *out_retire_fence);
  virtual int SetState(bool connected);
  virtual DisplayError Flush();
  virtual HWC2::Error GetColorModes(uint32_t *out_num_modes, ColorMode *out_modes);
  virtual HWC2::Error GetRenderIntents(ColorMode mode, uint32_t *out_num_intents,
                                       RenderIntent *out_intents);
  virtual HWC2::Error SetColorMode(ColorMode mode);
  virtual HWC2::Error SetColorModeWithRenderIntent(ColorMode mode, RenderIntent intent);
  virtual HWC2::Error SetColorTransform(const float *matrix, android_color_transform_t hint);
  virtual HWC2::Error UpdatePowerMode(HWC2::PowerMode mode);

 private:
  HWCDisplayPluggable(CoreInterface *core_intf, HWCBufferAllocator *buffer_allocator,
                      HWCCallbacks *callbacks, HWCDisplayEventHandler *event_handler,
                      qService::QService *qservice, hwc2_display_t id, int32_t sdm_id);
  void ApplyScanAdjustment(hwc_rect_t *display_frame);
  void GetUnderScanConfig();
  static void GetDownscaleResolution(uint32_t primary_width, uint32_t primary_height,
                                     uint32_t *virtual_width, uint32_t *virtual_height);

  DisplayNullExternal display_null_;
  int underscan_width_ = 0;
  int underscan_height_ = 0;
  bool has_color_tranform_ = false;
};

}  // namespace sdm

#endif  // __HWC_DISPLAY_PLUGGABLE_H__
