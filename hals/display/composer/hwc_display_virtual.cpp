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

#include <utils/constants.h>
#include <utils/debug.h>
#include <sync/sync.h>
#include <stdarg.h>

#include "hwc_display_virtual.h"
#include "hwc_debugger.h"

#define __CLASS__ "HWCDisplayVirtual"

namespace sdm {

void HWCDisplayVirtual::Destroy(HWCDisplay *hwc_display) {
  hwc_display->Deinit();
  delete hwc_display;
}

HWCDisplayVirtual::HWCDisplayVirtual(CoreInterface *core_intf, HWCBufferAllocator *buffer_allocator,
                                     HWCCallbacks *callbacks, hwc2_display_t id, int32_t sdm_id,
                                     uint32_t width, uint32_t height) :
      HWCDisplay(core_intf, buffer_allocator, callbacks, nullptr, nullptr, kVirtual, id, sdm_id,
                 DISPLAY_CLASS_VIRTUAL), width_(width), height_(height)  {
}

int HWCDisplayVirtual::Init() {
  flush_on_error_ = true;
  return 0;
}

int HWCDisplayVirtual::Deinit() {
  return HWCDisplay::Deinit();
}

bool HWCDisplayVirtual::NeedsGPUBypass() {
  return display_paused_ || active_secure_sessions_.any() || layer_set_.empty();
}

HWC2::Error HWCDisplayVirtual::Present(shared_ptr<Fence> *out_retire_fence) {
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayVirtual::DumpVDSBuffer() {
  if (dump_frame_count_ && !flush_ && dump_output_layer_) {
    if (output_handle_) {
      BufferInfo buffer_info;
      const private_handle_t *output_handle =
        reinterpret_cast<const private_handle_t *>(output_buffer_.buffer_id);
      DisplayError error = kErrorNone;
      if (!output_handle->base) {
        error = buffer_allocator_->MapBuffer(output_handle, nullptr);
        if (error != kErrorNone) {
          DLOGE("Failed to map output buffer, error = %d", error);
          return HWC2::Error::BadParameter;
        }
      }
      buffer_info.buffer_config.width = static_cast<uint32_t>(output_handle->width);
      buffer_info.buffer_config.height = static_cast<uint32_t>(output_handle->height);
      buffer_info.buffer_config.format =
      HWCLayer::GetSDMFormat(output_handle->format, output_handle->flags);
      buffer_info.alloc_buffer_info.size = static_cast<uint32_t>(output_handle->size);
      DumpOutputBuffer(buffer_info, reinterpret_cast<void *>(output_handle->base),
                       layer_stack_.retire_fence);

      int release_fence = -1;
      error = buffer_allocator_->UnmapBuffer(output_handle, &release_fence);
      if (error != kErrorNone) {
        DLOGE("Failed to unmap buffer, error = %d", error);
        return HWC2::Error::BadParameter;
      }
    }
  }

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayVirtual::SetOutputBuffer(buffer_handle_t buf,
                                               shared_ptr<Fence> release_fence) {
  if (buf == nullptr) {
    return HWC2::Error::BadParameter;
  }
  const private_handle_t *output_handle = static_cast<const private_handle_t *>(buf);

  if (output_handle) {
    int output_handle_format = output_handle->format;
    ColorMetaData color_metadata = {};

    if (output_handle_format == HAL_PIXEL_FORMAT_RGBA_8888) {
      output_handle_format = HAL_PIXEL_FORMAT_RGBX_8888;
    }

    LayerBufferFormat new_sdm_format =
        HWCLayer::GetSDMFormat(output_handle_format, output_handle->flags);
    if (new_sdm_format == kFormatInvalid) {
      return HWC2::Error::BadParameter;
    }

    if (sdm::SetCSC(output_handle, &color_metadata) != kErrorNone) {
      return HWC2::Error::BadParameter;
    }

    output_buffer_.flags.secure = 0;
    output_buffer_.flags.video = 0;
    output_buffer_.buffer_id = reinterpret_cast<uint64_t>(output_handle);
    output_buffer_.format = new_sdm_format;
    output_buffer_.color_metadata = color_metadata;
    output_handle_ = output_handle;

    // TZ Protected Buffer - L1
    if (output_handle->flags & private_handle_t::PRIV_FLAGS_SECURE_BUFFER) {
      output_buffer_.flags.secure = 1;
    }

    // ToDo: Need to extend for non-RGB formats
    output_buffer_.planes[0].fd = output_handle->fd;
    output_buffer_.planes[0].offset = output_handle->offset;
    output_buffer_.planes[0].stride = UINT32(output_handle->width);
  }

  output_buffer_.acquire_fence = release_fence;

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayVirtual::SetFrameDumpConfig(uint32_t count, uint32_t bit_mask_layer_type,
                                                  int32_t format, bool post_processed) {
  HWCDisplay::SetFrameDumpConfig(count, bit_mask_layer_type, format, post_processed);
  dump_output_layer_ = ((bit_mask_layer_type & (1 << OUTPUT_LAYER_DUMP)) != 0);

  DLOGI("output_layer_dump_enable %d", dump_output_layer_);
  return HWC2::Error::None;
}

HWC2::Error HWCDisplayVirtual::GetDisplayType(int32_t *out_type) {
  if (out_type == nullptr) {
    return HWC2::Error::BadParameter;
  }

  *out_type = HWC2_DISPLAY_TYPE_VIRTUAL;

  return HWC2::Error::None;
}

HWC2::Error HWCDisplayVirtual::SetColorMode(ColorMode mode) {
  return HWC2::Error::None;
}

}  // namespace sdm
