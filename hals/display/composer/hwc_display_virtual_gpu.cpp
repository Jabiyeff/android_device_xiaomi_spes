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

#include "hwc_display_virtual_gpu.h"
#include "hwc_session.h"

#include <qdMetaData.h>

#define __CLASS__ "HWCDisplayVirtualGPU"

namespace sdm {

int HWCDisplayVirtualGPU::Init() {
  // Create client target.
  client_target_ = new HWCLayer(id_, buffer_allocator_);

  // Calls into SDM need to be dropped. Create Null Display interface.
  display_intf_ = new DisplayNull();

  disable_animation_ = Debug::IsExtAnimDisabled();

  return HWCDisplayVirtual::Init();
}

int HWCDisplayVirtualGPU::Deinit() {
  // Destory color convert instance. This destroys thread and underlying GL resources.
  if (gl_color_convert_) {
    color_convert_task_.PerformTask(ColorConvertTaskCode::kCodeDestroyInstance, nullptr);
  }

  delete static_cast<DisplayNull *>(display_intf_);
  delete client_target_;

  for (auto hwc_layer : layer_set_) {
    delete hwc_layer;
  }

  return 0;
}

HWCDisplayVirtualGPU::HWCDisplayVirtualGPU(CoreInterface *core_intf, HWCBufferAllocator
                                           *buffer_allocator, HWCCallbacks *callbacks,
                                           hwc2_display_t id, int32_t sdm_id, uint32_t width,
                                           uint32_t height, float min_lum, float max_lum) :
  HWCDisplayVirtual(core_intf, buffer_allocator, callbacks, id, sdm_id, width, height),
  color_convert_task_(*this) {
}

HWC2::Error HWCDisplayVirtualGPU::Validate(uint32_t *out_num_types, uint32_t *out_num_requests) {
  DTRACE_SCOPED();

  // Reset previous changes.
  layer_changes_.clear();
  layer_requests_.clear();

  // Mark all layers to GPU if there is no need to bypass.
  bool needs_gpu_bypass = NeedsGPUBypass() || FreezeScreen();
  for (auto hwc_layer : layer_set_) {
    auto layer = hwc_layer->GetSDMLayer();
    layer->composition = needs_gpu_bypass ? kCompositionSDE : kCompositionGPU;

    if (needs_gpu_bypass) {
      if (hwc_layer->GetClientRequestedCompositionType() == HWC2::Composition::Client) {
       layer_changes_[hwc_layer->GetId()] = HWC2::Composition::Device;
       layer_requests_[hwc_layer->GetId()] = HWC2::LayerRequest::ClearClientTarget;
      }
    } else {
      if (hwc_layer->GetClientRequestedCompositionType() != HWC2::Composition::Client) {
       layer_changes_[hwc_layer->GetId()] = HWC2::Composition::Client;
      }
    }
  }

  // Derive client target dataspace based on the color mode - bug/115482728
  int32_t client_target_dataspace = GetDataspaceFromColorMode(GetCurrentColorMode());
  SetClientTargetDataSpace(client_target_dataspace);

  *out_num_types = UINT32(layer_changes_.size());
  *out_num_requests = UINT32(layer_requests_.size());;
  has_client_composition_ = !needs_gpu_bypass;
  client_target_->ResetValidation();

  validated_ = true;

  return ((*out_num_types > 0) ? HWC2::Error::HasChanges : HWC2::Error::None);
}

HWC2::Error HWCDisplayVirtualGPU::SetOutputBuffer(buffer_handle_t buf,
                                                  shared_ptr<Fence> release_fence) {
  HWC2::Error error = HWCDisplayVirtual::SetOutputBuffer(buf, release_fence);
  if (error != HWC2::Error::None) {
    return error;
  }

  const private_handle_t *hnd = static_cast<const private_handle_t *>(buf);
  output_buffer_.width = hnd->width;
  output_buffer_.height = hnd->height;
  output_buffer_.unaligned_width = width_;
  output_buffer_.unaligned_height = height_;

  // Update active dimensions.
  BufferDim_t buffer_dim;
  if (getMetaData(const_cast<private_handle_t *>(hnd), GET_BUFFER_GEOMETRY, &buffer_dim) == 0) {
    output_buffer_.unaligned_width = buffer_dim.sliceWidth;
    output_buffer_.unaligned_height = buffer_dim.sliceHeight;
    color_convert_task_.PerformTask(ColorConvertTaskCode::kCodeReset, nullptr);
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayVirtualGPU::Present(shared_ptr<Fence> *out_retire_fence) {
  DTRACE_SCOPED();

  auto status = HWC2::Error::None;

  if (!validated_) {
    return HWC2::Error::NotValidated;
  }

  if (!output_buffer_.buffer_id) {
    return HWC2::Error::NoResources;
  }

  if (active_secure_sessions_.any() || layer_set_.empty()) {
    return status;
  }

  layer_stack_.output_buffer = &output_buffer_;
  if (display_paused_) {
    validated_ = false;
  }

  // Ensure that blit is initialized.
  // GPU context gets in secure or non-secure mode depending on output buffer provided.
  if (!gl_color_convert_) {
    // Get instance.
    color_convert_task_.PerformTask(ColorConvertTaskCode::kCodeGetInstance, nullptr);
    if (gl_color_convert_ == nullptr) {
      DLOGE("Failed to get Color Convert Instance");
      return HWC2::Error::NoResources;
    } else {
      DLOGI("Created ColorConvert instance: %p", gl_color_convert_);
    }
  }

  ColorConvertBlitContext ctx = {};

  Layer *sdm_layer = client_target_->GetSDMLayer();
  LayerBuffer &input_buffer = sdm_layer->input_buffer;
  ctx.src_hnd = reinterpret_cast<const private_handle_t *>(input_buffer.buffer_id);
  ctx.dst_hnd = reinterpret_cast<const private_handle_t *>(output_handle_);
  ctx.dst_rect = {0, 0, FLOAT(output_buffer_.unaligned_width),
                  FLOAT(output_buffer_.unaligned_height)};
  ctx.src_acquire_fence = input_buffer.acquire_fence;
  ctx.dst_acquire_fence = output_buffer_.acquire_fence;

  color_convert_task_.PerformTask(ColorConvertTaskCode::kCodeBlit, &ctx);

  // todo blit
  DumpVDSBuffer();

  *out_retire_fence = ctx.release_fence;

  return status;
}

void HWCDisplayVirtualGPU::OnTask(const ColorConvertTaskCode &task_code,
                                  SyncTask<ColorConvertTaskCode>::TaskContext *task_context) {
  switch (task_code) {
    case ColorConvertTaskCode::kCodeGetInstance: {
        gl_color_convert_ = GLColorConvert::GetInstance(kTargetYUV, output_buffer_.flags.secure);
      }
      break;
    case ColorConvertTaskCode::kCodeBlit: {
        DTRACE_SCOPED();
        ColorConvertBlitContext* ctx = reinterpret_cast<ColorConvertBlitContext*>(task_context);
        gl_color_convert_->Blit(ctx->src_hnd, ctx->dst_hnd, ctx->src_rect, ctx->dst_rect,
                                ctx->src_acquire_fence, ctx->dst_acquire_fence,
                                &(ctx->release_fence));
      }
      break;
    case ColorConvertTaskCode::kCodeReset: {
        DTRACE_SCOPED();
        if (gl_color_convert_) {
          gl_color_convert_->Reset();
        }
      }
      break;
    case ColorConvertTaskCode::kCodeDestroyInstance: {
        if (gl_color_convert_) {
          GLColorConvert::Destroy(gl_color_convert_);
        }
      }
      break;
  }
}

bool HWCDisplayVirtualGPU::FreezeScreen() {
  if (!disable_animation_) {
    return false;
  }

  bool freeze_screen = false;
  if (animating_ && !animation_in_progress_) {
    // Start of animation. GPU comp is needed.
    animation_in_progress_ = true;
  } else if (!animating_ && animation_in_progress_) {
    // End of animation. Start composing.
    animation_in_progress_ = false;
  } else if (animating_ && animation_in_progress_) {
    // Animation in progress...
    freeze_screen = true;
  }

  return freeze_screen;
}

}  // namespace sdm

