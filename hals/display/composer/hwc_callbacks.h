/*
 * Copyright (c) 2016-2017, 2019-2020 The Linux Foundation. All rights reserved.
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
 *    * Neither the name of The Linux Foundation. nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
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

#ifndef __HWC_CALLBACKS_H__
#define __HWC_CALLBACKS_H__

#define HWC2_INCLUDE_STRINGIFICATION
#define HWC2_USE_CPP11
#include <utils/locker.h>
#include <hardware/hwcomposer2.h>
#undef HWC2_INCLUDE_STRINGIFICATION
#undef HWC2_USE_CPP11

namespace sdm {

class HWCCallbacks {
 public:
  static const int kNumBuiltIn = 4;
  static const int kNumPluggable = 4;
  static const int kNumVirtual = 4;
  // Add 1 primary display which can be either a builtin or pluggable.
  // Async powermode update requires dummy hwc displays.
  // Limit dummy displays to builtin/pluggable type for now.
  static const int kNumRealDisplays = 1 + kNumBuiltIn + kNumPluggable + kNumVirtual;
  static const int kNumDisplays = 1 + kNumBuiltIn + kNumPluggable + kNumVirtual +
                                    1 + kNumBuiltIn + kNumPluggable;

  HWC2::Error Hotplug(hwc2_display_t display, HWC2::Connection state);
  HWC2::Error Refresh(hwc2_display_t display);
  HWC2::Error Vsync(hwc2_display_t display, int64_t timestamp);
  HWC2::Error Vsync_2_4(hwc2_display_t display, int64_t timestamp, uint32_t period);
  HWC2::Error VsyncPeriodTimingChanged(hwc2_display_t display,
                                       hwc_vsync_period_change_timeline_t *updated_timeline);
  HWC2::Error SeamlessPossible(hwc2_display_t display);
  HWC2::Error Register(HWC2::Callback, hwc2_callback_data_t callback_data,
                       hwc2_function_pointer_t pointer);
  void UpdateVsyncSource(hwc2_display_t from) {
    vsync_source_ = from;
  }
  hwc2_display_t GetVsyncSource() { return vsync_source_; }

  bool VsyncCallbackRegistered() { return (vsync_ != nullptr && vsync_data_ != nullptr); }
  bool Vsync_2_4CallbackRegistered() { return (vsync_2_4_ != nullptr); }
  bool NeedsRefresh(hwc2_display_t display) { return pending_refresh_.test(UINT32(display)); }
  void ResetRefresh(hwc2_display_t display) { pending_refresh_.reset(UINT32(display)); }
  bool IsClientConnected() {
    SCOPE_LOCK(hotplug_lock_);
    return client_connected_;
  }

 private:
  hwc2_callback_data_t hotplug_data_ = nullptr;
  hwc2_callback_data_t refresh_data_ = nullptr;
  hwc2_callback_data_t vsync_data_ = nullptr;
  hwc2_callback_data_t vsync_2_4_data_ = nullptr;
  hwc2_callback_data_t vsync_period_timing_changed_data_ = nullptr;
  hwc2_callback_data_t seamless_possible_data_ = nullptr;

  HWC2_PFN_HOTPLUG hotplug_ = nullptr;
  HWC2_PFN_REFRESH refresh_ = nullptr;
  HWC2_PFN_VSYNC vsync_ = nullptr;
  HWC2_PFN_VSYNC_2_4 vsync_2_4_ = nullptr;
  HWC2_PFN_VSYNC_PERIOD_TIMING_CHANGED vsync_period_timing_changed_ = nullptr;
  HWC2_PFN_SEAMLESS_POSSIBLE seamless_possible_ = nullptr;

  hwc2_display_t vsync_source_ = HWC_DISPLAY_PRIMARY;   // hw vsync is active on this display
  std::bitset<kNumDisplays> pending_refresh_;         // Displays waiting to get refreshed

  Locker hotplug_lock_;
  Locker refresh_lock_;
  Locker vsync_lock_;
  Locker vsync_2_4_lock_;
  Locker vsync_period_timing_changed_lock_;
  Locker seamless_possible_lock_;
  bool client_connected_ = false;
};

}  // namespace sdm

#endif  // __HWC_CALLBACKS_H__
