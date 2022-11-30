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

#ifndef __QTIALLOCATOR_H__
#define __QTIALLOCATOR_H__

#include <hidl/MQDescriptor.h>
#include <hidl/Status.h>
#include <vendor/qti/hardware/display/allocator/3.0/IQtiAllocator.h>
#include <vendor/qti/hardware/display/allocator/4.0/IQtiAllocator.h>

#include "gr_buf_mgr.h"
#include "gr_utils.h"

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace allocator {
namespace V3_0 {
namespace implementation {

using ::android::sp;
using ::android::hardware::hidl_array;
using ::android::hardware::hidl_memory;
using ::android::hardware::hidl_string;
using ::android::hardware::hidl_vec;
using ::android::hardware::Return;
using ::android::hardware::Void;
using android::hardware::graphics::allocator::V3_0::IAllocator;
using ::android::hidl::base::V1_0::DebugInfo;
using ::android::hidl::base::V1_0::IBase;
using gralloc::BufferManager;
using vendor::qti::hardware::display::allocator::V3_0::IQtiAllocator;

class QtiAllocator : public IQtiAllocator {
 public:
  QtiAllocator();

  // Methods from ::android::hardware::graphics::allocator::V2_0::IAllocator follow.
  Return<void> dumpDebugInfo(dumpDebugInfo_cb _hidl_cb) override;
  Return<void> allocate(const hidl_vec<uint32_t> &descriptor, uint32_t count,
                        allocate_cb _hidl_cb) override;

  // Methods from ::android::hidl::base::V1_0::IBase follow.
 private:
  BufferManager *buf_mgr_ = nullptr;
};

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

using ::android::sp;
using ::android::hardware::hidl_array;
using ::android::hardware::hidl_memory;
using ::android::hardware::hidl_string;
using ::android::hardware::hidl_vec;
using ::android::hardware::Return;
using ::android::hardware::Void;
using android::hardware::graphics::allocator::V4_0::IAllocator;
using ::android::hidl::base::V1_0::DebugInfo;
using ::android::hidl::base::V1_0::IBase;
using gralloc::BufferManager;
using vendor::qti::hardware::display::allocator::V4_0::IQtiAllocator;

class QtiAllocator : public IQtiAllocator {
 public:
  QtiAllocator();

  // Methods from ::android::hardware::graphics::allocator::V4_0::IAllocator follow.
  Return<void> allocate(const hidl_vec<uint8_t> &descriptor, uint32_t count,
                        allocate_cb _hidl_cb) override;

  // Methods from ::android::hidl::base::V1_0::IBase follow.
 private:
  BufferManager *buf_mgr_ = nullptr;
};

}  // namespace implementation
}  // namespace V4_0
}  // namespace allocator
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor

#endif  // __QTIALLOCATOR_H__
