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
#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/locker.h>
#include "hwc_callbacks.h"

#define __CLASS__ "HWCCallbacks"

namespace sdm {

HWC2::Error HWCCallbacks::Hotplug(hwc2_display_t display, HWC2::Connection state) {
  SCOPE_LOCK(hotplug_lock_);
  DTRACE_SCOPED();

  // If client has not registered hotplug, wait for it finitely:
  //   1. spurious wake up (!hotplug_), wait again
  //   2. error = ETIMEDOUT, return with warning 1
  //   3. error != ETIMEDOUT, return with warning 2
  //   4. error == NONE and no spurious wake up, then !hotplug_ is false, exit loop
  while (!hotplug_) {
    DLOGW("Attempting to send client a hotplug too early Display = %" PRIu64 " state = %d", display,
          state);
    int ret = hotplug_lock_.WaitFinite(5000);
    if (ret == ETIMEDOUT) {
      DLOGW("Client didn't connect on time, dropping hotplug!");
      return HWC2::Error::None;
    } else if (ret != 0) {
      DLOGW("Failed client connection wait. Error %s, dropping hotplug!", strerror(ret));
      return HWC2::Error::None;
    }
  }
  hotplug_(hotplug_data_, display, INT32(state));
  return HWC2::Error::None;
}

HWC2::Error HWCCallbacks::Refresh(hwc2_display_t display) {
  SCOPE_LOCK(refresh_lock_);
  // Do not lock, will cause hotplug deadlock
  DTRACE_SCOPED();
  // If client has not registered refresh, drop it
  if (!refresh_) {
    return HWC2::Error::NoResources;
  }
  refresh_(refresh_data_, display);
  pending_refresh_.set(UINT32(display));
  return HWC2::Error::None;
}

HWC2::Error HWCCallbacks::Vsync(hwc2_display_t display, int64_t timestamp) {
  SCOPE_LOCK(vsync_lock_);
  // Do not lock, may cause hotplug deadlock
  DTRACE_SCOPED();
  // If client has not registered vsync, drop it
  if (!vsync_) {
    return HWC2::Error::NoResources;
  }
  vsync_(vsync_data_, display, timestamp);
  return HWC2::Error::None;
}

HWC2::Error HWCCallbacks::Vsync_2_4(hwc2_display_t display, int64_t timestamp, uint32_t period) {
  SCOPE_LOCK(vsync_2_4_lock_);
  DTRACE_SCOPED();
  if (!vsync_2_4_) {
    return HWC2::Error::NoResources;
  }

  vsync_2_4_(vsync_2_4_data_, display, timestamp, period);
  return HWC2::Error::None;
}

HWC2::Error HWCCallbacks::VsyncPeriodTimingChanged(
    hwc2_display_t display, hwc_vsync_period_change_timeline_t *updated_timeline) {
  SCOPE_LOCK(vsync_period_timing_changed_lock_);
  DTRACE_SCOPED();
  if (!vsync_period_timing_changed_) {
    return HWC2::Error::NoResources;
  }

  vsync_period_timing_changed_(vsync_period_timing_changed_data_, display, updated_timeline);
  return HWC2::Error::None;
}

HWC2::Error HWCCallbacks::SeamlessPossible(hwc2_display_t display) {
  SCOPE_LOCK(seamless_possible_lock_);
  DTRACE_SCOPED();
  if (!seamless_possible_) {
    return HWC2::Error::NoResources;
  }

  seamless_possible_(seamless_possible_data_, display);
  return HWC2::Error::None;
}

HWC2::Error HWCCallbacks::Register(HWC2::Callback descriptor, hwc2_callback_data_t callback_data,
                                   hwc2_function_pointer_t pointer) {
  switch (descriptor) {
    case HWC2::Callback::Hotplug: {
      SCOPE_LOCK(hotplug_lock_);
      hotplug_data_ = callback_data;
      hotplug_ = reinterpret_cast<HWC2_PFN_HOTPLUG>(pointer);
      client_connected_ = true;
      hotplug_lock_.Broadcast();
      } break;
    case HWC2::Callback::Refresh: {
      SCOPE_LOCK(refresh_lock_);
      refresh_data_ = callback_data;
      refresh_ = reinterpret_cast<HWC2_PFN_REFRESH>(pointer);
      } break;
    case HWC2::Callback::Vsync: {
      SCOPE_LOCK(vsync_lock_);
      vsync_data_ = callback_data;
      vsync_ = reinterpret_cast<HWC2_PFN_VSYNC>(pointer);
      } break;
    case HWC2::Callback::Vsync_2_4: {
      SCOPE_LOCK(vsync_2_4_lock_);
      vsync_2_4_data_ = callback_data;
      vsync_2_4_ = reinterpret_cast<HWC2_PFN_VSYNC_2_4>(pointer);
      } break;
    case HWC2::Callback::VsyncPeriodTimingChanged: {
      SCOPE_LOCK(vsync_period_timing_changed_lock_);
      vsync_period_timing_changed_data_ = callback_data;
      vsync_period_timing_changed_ =
          reinterpret_cast<HWC2_PFN_VSYNC_PERIOD_TIMING_CHANGED>(pointer);
      } break;
    case HWC2::Callback::SeamlessPossible: {
      SCOPE_LOCK(seamless_possible_lock_);
      seamless_possible_data_ = callback_data;
      seamless_possible_ = reinterpret_cast<HWC2_PFN_SEAMLESS_POSSIBLE>(pointer);
      } break;
    default:
      return HWC2::Error::BadParameter;
  }

  return HWC2::Error::None;
}

}  // namespace sdm
