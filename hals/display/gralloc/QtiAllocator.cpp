/*
 * Copyright (c) 2018-2020 The Linux Foundation. All rights reserved.
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

#define DEBUG 0
#include "QtiAllocator.h"

#include <cutils/properties.h>
#include <log/log.h>
#include <vendor/qti/hardware/display/mapper/3.0/IQtiMapper.h>
#include <vendor/qti/hardware/display/mapper/4.0/IQtiMapper.h>

#include <vector>

#include "QtiMapper.h"
#include "QtiMapper4.h"
#include "gr_utils.h"

static void get_properties(gralloc::GrallocProperties *props) {
  props->use_system_heap_for_sensors =
      property_get_bool("vendor.gralloc.use_system_heap_for_sensors", 1);

  props->ubwc_disable = property_get_bool("vendor.gralloc.disable_ubwc", 0);

  props->ahardware_buffer_disable = property_get_bool("vendor.gralloc.disable_ahardware_buffer", 0);
}

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace allocator {
namespace V3_0 {
namespace implementation {

using android::hardware::hidl_handle;
using gralloc::BufferDescriptor;
using IMapper_3_0_Error = android::hardware::graphics::mapper::V3_0::Error;
using gralloc::Error;

QtiAllocator::QtiAllocator() {
  gralloc::GrallocProperties properties;
  get_properties(&properties);
  buf_mgr_ = BufferManager::GetInstance();
  buf_mgr_->SetGrallocDebugProperties(properties);
}

// Methods from ::android::hardware::graphics::allocator::V2_0::IAllocator follow.
Return<void> QtiAllocator::dumpDebugInfo(dumpDebugInfo_cb hidl_cb) {
  std::ostringstream os;
  buf_mgr_->Dump(&os);
  hidl_string reply;
  reply.setToExternal(os.str().c_str(), os.str().length());
  hidl_cb(reply);
  return Void();
}

Return<void> QtiAllocator::allocate(const hidl_vec<uint32_t> &descriptor, uint32_t count,
                                    allocate_cb hidl_cb) {
  ALOGD_IF(DEBUG, "Allocating buffers count: %d", count);
  gralloc::BufferDescriptor desc;

  auto err = ::vendor::qti::hardware::display::mapper::V3_0::implementation::QtiMapper::Decode(
      descriptor, &desc);
  if (err != Error::NONE) {
    hidl_cb(static_cast<IMapper_3_0_Error>(err), 0, hidl_vec<hidl_handle>());
    return Void();
  }

  std::vector<hidl_handle> buffers;
  buffers.reserve(count);
  for (uint32_t i = 0; i < count; i++) {
    buffer_handle_t buffer;
    ALOGD_IF(DEBUG, "buffer: %p", &buffer);
    err = buf_mgr_->AllocateBuffer(desc, &buffer);
    if (err != Error::NONE) {
      break;
    }
    buffers.emplace_back(hidl_handle(buffer));
  }

  uint32_t stride = 0;
  hidl_vec<hidl_handle> hidl_buffers;
  if (err == Error::NONE && buffers.size() > 0) {
    stride = static_cast<uint32_t>(PRIV_HANDLE_CONST(buffers[0].getNativeHandle())->width);
    hidl_buffers.setToExternal(buffers.data(), buffers.size());
  }
  hidl_cb(static_cast<IMapper_3_0_Error>(err), stride, hidl_buffers);

  for (const auto &b : buffers) {
    buf_mgr_->ReleaseBuffer(PRIV_HANDLE_CONST(b.getNativeHandle()));
  }

  return Void();
}

}  // namespace implementation
}  // namespace V3_0
}  // namespace allocator
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace allocator {
namespace V4_0 {
namespace implementation {

using android::hardware::hidl_handle;
using gralloc::BufferDescriptor;
using IMapper_4_0_Error = android::hardware::graphics::mapper::V4_0::Error;
using gralloc::Error;

QtiAllocator::QtiAllocator() {
  gralloc::GrallocProperties properties;
  get_properties(&properties);
  buf_mgr_ = BufferManager::GetInstance();
  buf_mgr_->SetGrallocDebugProperties(properties);
}

Return<void> QtiAllocator::allocate(const hidl_vec<uint8_t> &descriptor, uint32_t count,
                                    allocate_cb hidl_cb) {
  ALOGD_IF(DEBUG, "Allocating buffers count: %d", count);
  gralloc::BufferDescriptor desc;

  auto err = ::vendor::qti::hardware::display::mapper::V4_0::implementation::QtiMapper::Decode(
      descriptor, &desc);
  if (err != Error::NONE) {
    hidl_cb(static_cast<IMapper_4_0_Error>(err), 0, hidl_vec<hidl_handle>());
    return Void();
  }

  std::vector<hidl_handle> buffers;
  buffers.reserve(count);
  for (uint32_t i = 0; i < count; i++) {
    buffer_handle_t buffer;
    ALOGD_IF(DEBUG, "buffer: %p", &buffer);
    err = buf_mgr_->AllocateBuffer(desc, &buffer);
    if (err != Error::NONE) {
      break;
    }
    buffers.emplace_back(hidl_handle(buffer));
  }

  uint32_t stride = 0;
  hidl_vec<hidl_handle> hidl_buffers;
  if (err == Error::NONE && buffers.size() > 0) {
    stride = static_cast<uint32_t>(PRIV_HANDLE_CONST(buffers[0].getNativeHandle())->width);
    hidl_buffers.setToExternal(buffers.data(), buffers.size());
  }
  hidl_cb(static_cast<IMapper_4_0_Error>(err), stride, hidl_buffers);

  for (const auto &b : buffers) {
    buf_mgr_->ReleaseBuffer(PRIV_HANDLE_CONST(b.getNativeHandle()));
  }

  return Void();
}

}  // namespace implementation
}  // namespace V4_0
}  // namespace allocator
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
