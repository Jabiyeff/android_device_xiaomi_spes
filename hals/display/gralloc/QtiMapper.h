/*
 * Copyright (c) 2018-2020 The Linux Foundation. All rights reserved.
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

#ifndef __QTIMAPPER_H__
#define __QTIMAPPER_H__

#include <hidl/MQDescriptor.h>
#include <hidl/Status.h>
#include <vendor/qti/hardware/display/mapper/3.0/IQtiMapper.h>

#include "QtiMapperExtensions.h"
#include "gr_buf_mgr.h"
#include "gr_utils.h"

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace mapper {
namespace V3_0 {
namespace implementation {

using ::android::sp;
using ::android::hardware::hidl_array;
using ::android::hardware::hidl_handle;
using ::android::hardware::hidl_memory;
using ::android::hardware::hidl_string;
using ::android::hardware::hidl_vec;
using ::android::hardware::Return;
using ::android::hardware::Void;
using ::android::hardware::graphics::common::V1_2::PixelFormat;
using ::android::hardware::graphics::mapper::V3_0::Error;
using ::android::hardware::graphics::mapper::V3_0::IMapper;
using ::android::hardware::graphics::mapper::V3_0::YCbCrLayout;
using ::android::hidl::base::V1_0::DebugInfo;
using ::android::hidl::base::V1_0::IBase;
using gralloc::BufferManager;
using ::vendor::qti::hardware::display::mapperextensions::V1_1::IQtiMapperExtensions;
using ::vendor::qti::hardware::display::mapperextensions::V1_1::implementation::QtiMapperExtensions;
using ::vendor::qti::hardware::display::mapper::V3_0::IQtiMapper;

using IMapper_3_0 = android::hardware::graphics::mapper::V3_0::IMapper;
using BufferDescriptorInfo_3_0 =
    android::hardware::graphics::mapper::V3_0::IMapper::BufferDescriptorInfo;
using IMapperBufferDescriptor = android::hardware::graphics::mapper::V3_0::BufferDescriptor;
using IMapper_3_0_Error = ::android::hardware::graphics::mapper::V3_0::Error;

class QtiMapper : public IQtiMapper {
 public:
  QtiMapper();
  // Methods from ::android::hardware::graphics::mapper::V2_0::IMapper follow.
  Return<void> createDescriptor(const BufferDescriptorInfo_3_0 &descriptor_info,
                                createDescriptor_cb hidl_cb) override;
  Return<void> importBuffer(const hidl_handle &raw_handle, importBuffer_cb hidl_cb) override;
  Return<Error> freeBuffer(void *buffer) override;
  Return<void> lock(void *buffer, uint64_t cpu_usage, const IMapper::Rect &access_region,
                    const hidl_handle &acquire_fence, lock_cb hidl_cb) override;
  Return<void> lockYCbCr(void *buffer, uint64_t cpu_usage, const IMapper::Rect &access_region,
                         const hidl_handle &acquire_fence, lockYCbCr_cb hidl_cb) override;
  Return<void> unlock(void *buffer, unlock_cb hidl_cb) override;

  // Methods from ::android::hardware::graphics::mapper::V2_1::IMapper follow.
  Return<Error> validateBufferSize(void *buffer, const BufferDescriptorInfo_3_0 &descriptorInfo,
                                   uint32_t stride) override;
  Return<void> getTransportSize(void *buffer, IMapper_3_0::getTransportSize_cb hidl_cb) override;

  Return<void> isSupported(const BufferDescriptorInfo_3_0 &descriptor_info,
                           IMapper_3_0::isSupported_cb hidl_cb) override;

  Return<void> getMapperExtensions(getMapperExtensions_cb hidl_cb);
  sp<mapperextensions::V1_1::IQtiMapperExtensions> extensions_ = nullptr;
  hidl_vec<uint32_t> Encode(const IMapper_3_0::BufferDescriptorInfo &bd_info) {
    hidl_vec<uint32_t> out;
    out.resize(gralloc::kBufferDescriptorSize);
    out[0] = gralloc::kMagicVersion;
    out[1] = bd_info.width;
    out[2] = bd_info.height;
    out[3] = bd_info.layerCount;
    out[4] = static_cast<uint32_t>(bd_info.format);
    out[5] = static_cast<uint32_t>(bd_info.usage);
    out[6] = static_cast<uint32_t>(bd_info.usage >> 32);
    return out;
  }
  static gralloc::Error Decode(const hidl_vec<uint32_t> &in,
                               gralloc::BufferDescriptor *buf_descriptor) {
    if (in.size() != gralloc::kBufferDescriptorSize || in[0] != gralloc::kMagicVersion) {
      return gralloc::Error::BAD_DESCRIPTOR;
    }
    uint32_t width = static_cast<int32_t>(in[1]);
    uint32_t height = static_cast<int32_t>(in[2]);
    buf_descriptor->SetDimensions(width, height);
    uint32_t layer_count = in[3];
    buf_descriptor->SetLayerCount(layer_count);
    uint32_t format = static_cast<int32_t>(in[4]);
    buf_descriptor->SetColorFormat(format);
    uint64_t usage = static_cast<uint64_t>(in[6]) << 32 | in[5];
    buf_descriptor->SetUsage(usage);
    return gralloc::Error::NONE;
  }

 private:
  BufferManager *buf_mgr_ = nullptr;
  Error CreateDescriptor(const BufferDescriptorInfo_3_0 &descriptor_info,
                         IMapperBufferDescriptor *descriptor);
  bool ValidDescriptor(const IMapper::BufferDescriptorInfo &bd);
  bool GetFenceFd(const hidl_handle &fence_handle, int *outFenceFd);
  void WaitFenceFd(int fence_fd);
  Error LockBuffer(void *buffer, uint64_t usage, const hidl_handle &acquire_fence);
};

}  // namespace implementation
}  // namespace V3_0
}  // namespace mapper
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor

#endif  // __QTIMAPPER_H__
