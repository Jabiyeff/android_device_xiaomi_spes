/*
 * Copyright (c) 2016-2018, 2020, The Linux Foundation. All rights reserved.

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

#ifndef __GR_BUF_DESCRIPTOR_H__
#define __GR_BUF_DESCRIPTOR_H__

#include <atomic>
#include <string>

#include "gr_utils.h"

namespace gralloc {
using android::hardware::hidl_vec;
const uint32_t kBufferDescriptorSize = 7;
const uint32_t kBufferDescriptorSizeV4 = 42;
const uint32_t kMagicVersion = 0x76312E30;  // v1.0

class BufferDescriptor {
 public:
  BufferDescriptor() {}
  explicit BufferDescriptor(uint64_t id) : id_(id) {}
  void SetUsage(uint64_t usage) { usage_ |= usage; }

  void SetDimensions(int w, int h) {
    width_ = w;
    height_ = h;
  }

  void SetColorFormat(int format) { format_ = format; }

  void SetLayerCount(uint32_t layer_count) { layer_count_ = layer_count; }

  void SetName(std::string name) { name_ = name; }

  void SetReservedSize(uint64_t reserved_size) { reserved_size_ = reserved_size; }

  uint64_t GetUsage() const { return usage_; }

  int GetWidth() const { return width_; }

  int GetHeight() const { return height_; }

  int GetFormat() const { return format_; }

  uint32_t GetLayerCount() const { return layer_count_; }

  uint64_t GetId() const { return id_; }

  uint64_t GetReservedSize() const { return reserved_size_; }

  std::string GetName() const { return name_; }

 private:
  std::string name_ = "";
  int width_ = -1;
  int height_ = -1;
  int format_ = -1;
  uint32_t layer_count_ = 1;
  uint64_t usage_ = 0;
  const uint64_t id_ = 0;
  uint64_t reserved_size_ = 0;
};
};      // namespace gralloc
#endif  // __GR_BUF_DESCRIPTOR_H__
