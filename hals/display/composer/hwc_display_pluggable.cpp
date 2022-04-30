/*
* Copyright (c) 2014-2020, The Linux Foundation. All rights reserved.
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
*/

#include <cutils/properties.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <algorithm>

#include "hwc_display_pluggable.h"
#include "hwc_debugger.h"

#define __CLASS__ "HWCDisplayPluggable"

namespace sdm {

int HWCDisplayPluggable::Create(CoreInterface *core_intf, HWCBufferAllocator *buffer_allocator,
                               HWCCallbacks *callbacks, HWCDisplayEventHandler *event_handler,
                               qService::QService *qservice, hwc2_display_t id, int32_t sdm_id,
                               uint32_t primary_width, uint32_t primary_height,
                               bool use_primary_res, HWCDisplay **hwc_display) {
  uint32_t pluggable_width = 0;
  uint32_t pluggable_height = 0;
  DisplayError error = kErrorNone;

  HWCDisplay *hwc_display_pluggable = new HWCDisplayPluggable(core_intf, buffer_allocator,
                                                  callbacks, event_handler, qservice, id, sdm_id);
  int status = hwc_display_pluggable->Init();
  if (status) {
    delete hwc_display_pluggable;
    return status;
  }

  error = hwc_display_pluggable->GetMixerResolution(&pluggable_width, &pluggable_height);
  if (error != kErrorNone) {
    Destroy(hwc_display_pluggable);
    return -EINVAL;
  }

  if (primary_width && primary_height) {
    // use_primary_res means HWCDisplayPluggable should directly set framebuffer resolution to the
    // provided primary_width and primary_height
    if (use_primary_res) {
      pluggable_width = primary_width;
      pluggable_height = primary_height;
    } else {
      int downscale_enabled = 0;
      HWCDebugHandler::Get()->GetProperty(ENABLE_EXTERNAL_DOWNSCALE_PROP, &downscale_enabled);
      if (downscale_enabled) {
        GetDownscaleResolution(primary_width, primary_height, &pluggable_width, &pluggable_height);
      }
    }
  }

  status = hwc_display_pluggable->SetFrameBufferResolution(pluggable_width, pluggable_height);
  if (status) {
    Destroy(hwc_display_pluggable);
    return status;
  }

  *hwc_display = hwc_display_pluggable;

  return status;
}

int HWCDisplayPluggable::Init() {
  int status = HWCDisplay::Init();
  if (status) {
    return status;
  }
  color_mode_ = new HWCColorMode(display_intf_);
  color_mode_->Init();

  return status;
}

void HWCDisplayPluggable::Destroy(HWCDisplay *hwc_display) {
  // Flush the display to have outstanding fences signaled.
  hwc_display->Flush();
  hwc_display->Deinit();
  delete hwc_display;
}

HWCDisplayPluggable::HWCDisplayPluggable(CoreInterface *core_intf,
                                         HWCBufferAllocator *buffer_allocator,
                                         HWCCallbacks *callbacks,
                                         HWCDisplayEventHandler *event_handler,
                                         qService::QService *qservice,
                                         hwc2_display_t id,
                                         int32_t sdm_id)
    : HWCDisplay(core_intf, buffer_allocator, callbacks, event_handler, qservice, kPluggable, id,
                 sdm_id, DISPLAY_CLASS_PLUGGABLE) {
}

HWC2::Error HWCDisplayPluggable::Validate(uint32_t *out_num_types, uint32_t *out_num_requests) {
  auto status = HWC2::Error::None;

  if (active_secure_sessions_[kSecureDisplay]) {
    MarkLayersForGPUBypass();
    return status;
  }

  BuildLayerStack();

  if (layer_set_.empty()) {
    flush_ = !client_connected_;
    validated_ = true;
    return status;
  }

  // Apply current Color Mode and Render Intent.
  status = color_mode_->ApplyCurrentColorModeWithRenderIntent(
                                                 static_cast<bool>(layer_stack_.flags.hdr_present));
  if (status != HWC2::Error::None || has_color_tranform_) {
    // Fallback to GPU Composition if Color Mode can't be applied or if a color tranform needs to be
    // applied.
    MarkLayersForClientComposition();
  }

  // TODO(user): SetRefreshRate need to follow new interface when added.

  status = PrepareLayerStack(out_num_types, out_num_requests);
  return status;
}

HWC2::Error HWCDisplayPluggable::Present(shared_ptr<Fence> *out_retire_fence) {
  auto status = HWC2::Error::None;

  if (!active_secure_sessions_[kSecureDisplay]) {
    status = HWCDisplay::CommitLayerStack();
    if (status == HWC2::Error::None) {
      status = HWCDisplay::PostCommitLayerStack(out_retire_fence);
    }
  }
  return status;
}

void HWCDisplayPluggable::ApplyScanAdjustment(hwc_rect_t *display_frame) {
  if ((underscan_width_ <= 0) || (underscan_height_ <= 0)) {
    return;
  }

  float width_ratio = FLOAT(underscan_width_) / 100.0f;
  float height_ratio = FLOAT(underscan_height_) / 100.0f;

  uint32_t mixer_width = 0;
  uint32_t mixer_height = 0;
  GetMixerResolution(&mixer_width, &mixer_height);

  if (mixer_width == 0 || mixer_height == 0) {
    DLOGV("Invalid mixer dimensions (%d, %d)", mixer_width, mixer_height);
    return;
  }

  uint32_t new_mixer_width = UINT32(mixer_width * FLOAT(1.0f - width_ratio));
  uint32_t new_mixer_height = UINT32(mixer_height * FLOAT(1.0f - height_ratio));

  int x_offset = INT((FLOAT(mixer_width) * width_ratio) / 2.0f);
  int y_offset = INT((FLOAT(mixer_height) * height_ratio) / 2.0f);

  display_frame->left = (display_frame->left * INT32(new_mixer_width) / INT32(mixer_width))
                        + x_offset;
  display_frame->top = (display_frame->top * INT32(new_mixer_height) / INT32(mixer_height)) +
                       y_offset;
  display_frame->right = ((display_frame->right * INT32(new_mixer_width)) / INT32(mixer_width)) +
                         x_offset;
  display_frame->bottom = ((display_frame->bottom * INT32(new_mixer_height)) / INT32(mixer_height))
                          + y_offset;
}

static void AdjustSourceResolution(uint32_t dst_width, uint32_t dst_height, uint32_t *src_width,
                                   uint32_t *src_height) {
  *src_height = (dst_width * (*src_height)) / (*src_width);
  *src_width = dst_width;
}

void HWCDisplayPluggable::GetDownscaleResolution(uint32_t primary_width, uint32_t primary_height,
                                                uint32_t *non_primary_width,
                                                uint32_t *non_primary_height) {
  uint32_t primary_area = primary_width * primary_height;
  uint32_t non_primary_area = (*non_primary_width) * (*non_primary_height);

  if (primary_area > non_primary_area) {
    if (primary_height > primary_width) {
      std::swap(primary_height, primary_width);
    }
    AdjustSourceResolution(primary_width, primary_height, non_primary_width, non_primary_height);
  }
}

int HWCDisplayPluggable::SetState(bool connected) {
  DisplayError error = kErrorNone;
  DisplayState state = kStateOff;
  DisplayConfigVariableInfo fb_config = {};

  if (connected) {
    if (display_null_.IsActive()) {
      error = core_intf_->CreateDisplay(type_, this, &display_intf_);
      if (error != kErrorNone) {
        DLOGE("Display create failed. Error = %d display_type %d event_handler %p disp_intf %p",
              error, type_, this, &display_intf_);
        return -EINVAL;
      }

      // Restore HDMI attributes when display is reconnected.
      // This is to ensure that surfaceflinger & sdm are in sync.
      display_null_.GetFrameBufferConfig(&fb_config);
      int status = SetFrameBufferResolution(fb_config.x_pixels, fb_config.y_pixels);
      if (status) {
        DLOGW("Set frame buffer config failed. Error = %d", error);
        return -1;
      }
      shared_ptr<Fence> release_fence = nullptr;
      display_null_.GetDisplayState(&state);
      display_intf_->SetDisplayState(state, false /* teardown */, &release_fence);
      validated_ = false;

      SetVsyncEnabled(HWC2::Vsync::Enable);

      display_null_.SetActive(false);
      DLOGI("Display is connected successfully.");
    } else {
      DLOGI("Display is already connected.");
    }
  } else {
    if (!display_null_.IsActive()) {
      shared_ptr<Fence> release_fence = nullptr;
      // Preserve required attributes of HDMI display that surfaceflinger sees.
      // Restore HDMI attributes when display is reconnected.
      display_intf_->GetDisplayState(&state);
      display_null_.SetDisplayState(state, false /* teardown */, &release_fence);

      error = display_intf_->GetFrameBufferConfig(&fb_config);
      if (error != kErrorNone) {
        DLOGW("Get frame buffer config failed. Error = %d", error);
        return -1;
      }
      display_null_.SetFrameBufferConfig(fb_config);

      SetVsyncEnabled(HWC2::Vsync::Disable);
      core_intf_->DestroyDisplay(display_intf_);
      display_intf_ = &display_null_;

      display_null_.SetActive(true);
      DLOGI("Display is disconnected successfully.");
    } else {
      DLOGI("Display is already disconnected.");
    }
  }

  return 0;
}

void HWCDisplayPluggable::GetUnderScanConfig() {
  if (!display_intf_->IsUnderscanSupported()) {
    // Read user defined underscan width and height
    HWCDebugHandler::Get()->GetProperty(EXTERNAL_ACTION_SAFE_WIDTH_PROP, &underscan_width_);
    HWCDebugHandler::Get()->GetProperty(EXTERNAL_ACTION_SAFE_HEIGHT_PROP, &underscan_height_);
  }
}

DisplayError HWCDisplayPluggable::Flush() {
  return display_intf_->Flush(&layer_stack_);
}

HWC2::Error HWCDisplayPluggable::GetColorModes(uint32_t *out_num_modes, ColorMode *out_modes) {
  if (out_modes == nullptr) {
    *out_num_modes = color_mode_->GetColorModeCount();
  } else {
    color_mode_->GetColorModes(out_num_modes, out_modes);
  }
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayPluggable::GetRenderIntents(ColorMode mode, uint32_t *out_num_intents,
                                                 RenderIntent *out_intents) {
  if (out_intents == nullptr) {
    *out_num_intents = color_mode_->GetRenderIntentCount(mode);
  } else {
    color_mode_->GetRenderIntents(mode, out_num_intents, out_intents);
  }
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayPluggable::SetColorMode(ColorMode mode) {
  return SetColorModeWithRenderIntent(mode, RenderIntent::COLORIMETRIC);
}

HWC2::Error HWCDisplayPluggable::SetColorModeWithRenderIntent(ColorMode mode, RenderIntent intent) {
  auto status = color_mode_->CacheColorModeWithRenderIntent(mode, intent);
  if (status != HWC2::Error::None) {
    DLOGE("failed for mode = %d intent = %d", mode, intent);
    return status;
  }

  callbacks_->Refresh(id_);
  validated_ = false;

  return status;
}

HWC2::Error HWCDisplayPluggable::UpdatePowerMode(HWC2::PowerMode mode) {
  current_power_mode_ = mode;
  validated_ = false;
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayPluggable::SetColorTransform(const float *matrix,
                                                   android_color_transform_t hint) {
  if (HAL_COLOR_TRANSFORM_IDENTITY == hint) {
    has_color_tranform_ = false;
    // From 2.1 IComposerClient.hal:
    // If the device is not capable of either using the hint or the matrix to apply the desired
    // color transform, it must force all layers to client composition during VALIDATE_DISPLAY.
  } else {
    // Also, interpret HAL_COLOR_TRANSFORM_ARBITRARY_MATRIX hint as non-identity matrix.
    has_color_tranform_ = true;
  }

  callbacks_->Refresh(id_);
  validated_ = false;

  return HWC2::Error::None;
}

}  // namespace sdm
