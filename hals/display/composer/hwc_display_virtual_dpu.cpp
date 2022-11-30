/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
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

#include <hwc_display_virtual_dpu.h>

#define __CLASS__ "HWCDisplayVirtualDPU"

namespace sdm {

HWCDisplayVirtualDPU::HWCDisplayVirtualDPU(CoreInterface *core_intf, HWCBufferAllocator
                                           *buffer_allocator, HWCCallbacks *callbacks,
                                           hwc2_display_t id, int32_t sdm_id, uint32_t width,
                                           uint32_t height, float min_lum, float max_lum)
  : HWCDisplayVirtual(core_intf, buffer_allocator, callbacks, id, sdm_id, width, height),
    min_lum_(min_lum), max_lum_(max_lum) {
}

int HWCDisplayVirtualDPU::Init() {
  int status = HWCDisplay::Init();
  if (status) {
    DLOGE("Init failed: %d", status);
    return status;
  }

  if (max_lum_ != -1.0 || min_lum_ != -1.0) {
    SetPanelLuminanceAttributes(min_lum_, max_lum_);
  }

  status = SetConfig(width_, height_);
  if (status) {
    DLOGE("Failed to set width: %d height: %d", width_, height_);
    return status;
  }

  status = INT32(SetPowerMode(HWC2::PowerMode::On, false /* teardown */));
  if (status) {
    DLOGW("Failed to set power mode on virtual display");
    return status;
  }

  // TODO(user): Validate that we support this width/height
  status = SetFrameBufferResolution(width_, height_);
  if (status) {
    DLOGW("Failed to set FrameBuffer resolution on virtual display");
    return status;
  }

  return HWCDisplayVirtual::Init();
}

int HWCDisplayVirtualDPU::SetConfig(uint32_t width, uint32_t height) {
  DisplayConfigVariableInfo variable_info;
  variable_info.x_pixels = width;
  variable_info.y_pixels = height;
  // TODO(user): Need to get the framerate of primary display and update it.
  variable_info.fps = 60;

  DisplayError err = display_intf_->SetActiveConfig(&variable_info);
  if (err != kErrorNone) {
    return -EINVAL;
  }
  return 0;
}

HWC2::Error HWCDisplayVirtualDPU::SetOutputBuffer(buffer_handle_t buf,
                                                  shared_ptr<Fence> release_fence) {
  HWC2::Error error = HWCDisplayVirtual::SetOutputBuffer(buf, release_fence);
  if (error != HWC2::Error::None) {
    return error;
  }

  const private_handle_t *output_handle = static_cast<const private_handle_t *>(buf);
  if (output_handle) {
    int output_handle_format = output_handle->format;
    int active_aligned_w, active_aligned_h;
    int new_width, new_height;
    int new_aligned_w, new_aligned_h;
    uint32_t active_width, active_height;

    GetMixerResolution(&active_width, &active_height);
    buffer_allocator_->GetCustomWidthAndHeight(output_handle, &new_width, &new_height);
    buffer_allocator_->GetAlignedWidthAndHeight(INT(new_width), INT(new_height),
                                                output_handle_format, 0, &new_aligned_w,
                                                &new_aligned_h);
    buffer_allocator_->GetAlignedWidthAndHeight(INT(active_width), INT(active_height),
                                                output_handle_format, 0, &active_aligned_w,
                                                &active_aligned_h);
    if (new_aligned_w != active_aligned_w  || new_aligned_h != active_aligned_h) {
      int status = SetConfig(UINT32(new_width), UINT32(new_height));
      if (status) {
        DLOGE("SetConfig failed custom WxH %dx%d", new_width, new_height);
        return HWC2::Error::BadParameter;
      }
      validated_ = false;
    }

    output_buffer_.width = UINT32(new_aligned_w);
    output_buffer_.height = UINT32(new_aligned_h);
    output_buffer_.unaligned_width = UINT32(new_width);
    output_buffer_.unaligned_height = UINT32(new_height);
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayVirtualDPU::Validate(uint32_t *out_num_types, uint32_t *out_num_requests) {
  if (NeedsGPUBypass()) {
    MarkLayersForGPUBypass();
    return HWC2::Error::None;
  }

  BuildLayerStack();
  layer_stack_.output_buffer = &output_buffer_;
  // If Output buffer of Virtual Display is not secure, set SKIP flag on the secure layers.
  if (!output_buffer_.flags.secure && layer_stack_.flags.secure_present) {
    for (auto hwc_layer : layer_set_) {
      Layer *layer = hwc_layer->GetSDMLayer();
      if (layer->input_buffer.flags.secure) {
        layer_stack_.flags.skip_present = true;
        layer->flags.skip = true;
      }
    }
  }

  return PrepareLayerStack(out_num_types, out_num_requests);
}

HWC2::Error HWCDisplayVirtualDPU::Present(shared_ptr<Fence> *out_retire_fence) {
  auto status = HWC2::Error::None;

  if (!output_buffer_.buffer_id) {
    return HWC2::Error::NoResources;
  }

  if (active_secure_sessions_.any()) {
    return status;
  }

  layer_stack_.output_buffer = &output_buffer_;
  if (display_paused_) {
    validated_ = false;
    flush_ = true;
  }

  status = HWCDisplay::CommitLayerStack();
  if (status != HWC2::Error::None) {
    return status;
  }

  DumpVDSBuffer();

  status = HWCDisplay::PostCommitLayerStack(out_retire_fence);

  return status;
}

HWC2::Error HWCDisplayVirtualDPU::SetPanelLuminanceAttributes(float min_lum, float max_lum) {
  DisplayError err = display_intf_->SetPanelLuminanceAttributes(min_lum, max_lum);
  if (err != kErrorNone) {
    return HWC2::Error::BadParameter;
  }
  return HWC2::Error::None;
}

}  // namespace sdm

