/*
 * Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
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

#ifndef __HWC_DISPLAY_DUMMY_H__
#define __HWC_DISPLAY_DUMMY_H__

#include "hwc_display.h"
#include "display_null.h"

namespace sdm {

class HWCDisplayDummy : public HWCDisplay {
 public:
  static int Create(CoreInterface *core_intf, BufferAllocator *buffer_allocator,
                    HWCCallbacks *callbacks, HWCDisplayEventHandler *event_handler,
                    qService::QService *qservice, hwc2_display_t id, int32_t sdm_id,
                    HWCDisplay **hwc_display);
  static void Destroy(HWCDisplay *hwc_display);
  virtual HWC2::Error Validate(uint32_t *out_num_types, uint32_t *out_num_requests);
  virtual HWC2::Error Present(shared_ptr<Fence> *out_retire_fence);
  virtual HWC2::Error GetActiveConfig(hwc2_config_t *out_config);
  virtual HWC2::Error UpdatePowerMode(HWC2::PowerMode mode);
  virtual HWC2::Error SetColorMode(ColorMode mode);
  virtual HWC2::Error SetVsyncEnabled(HWC2::Vsync enabled);
  virtual bool VsyncEnablePending();
  virtual HWC2::Error SetClientTarget(buffer_handle_t target, shared_ptr<Fence> acquire_fence,
                                      int32_t dataspace, hwc_region_t damage);
  virtual HWC2::Error GetDisplayVsyncPeriod(VsyncPeriodNanos *vsync_period);
  virtual HWC2::Error SetDisplayVsyncPeriod(VsyncPeriodNanos vsync_period);
 private:
  HWCDisplayDummy(CoreInterface *core_intf, BufferAllocator *buffer_allocator,
                  HWCCallbacks *callbacks, HWCDisplayEventHandler *event_handler,
                  qService::QService *qservice, hwc2_display_t id, int32_t sdm_id);
  DisplayNull display_null_;
  bool vsync_enable_ = false;
  VsyncPeriodNanos vsync_period_ = 16600000;
};

}  // namespace sdm

#endif  // __HWC_DISPLAY_DUMMY_H__
