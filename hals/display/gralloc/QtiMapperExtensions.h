/*
 * Copyright (c) 2019 The Linux Foundation. All rights reserved.
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

#ifndef __QTIMAPPEREXTENSIONS_H__
#define __QTIMAPPEREXTENSIONS_H__

#include <hidl/MQDescriptor.h>
#include <hidl/Status.h>
#include <vendor/qti/hardware/display/mapperextensions/1.1/IQtiMapperExtensions.h>

#include "gr_buf_mgr.h"
namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace mapperextensions {
namespace V1_1 {
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
using ::android::hidl::base::V1_0::DebugInfo;
using ::android::hidl::base::V1_0::IBase;
using gralloc::BufferManager;
using ::vendor::qti::hardware::display::mapperextensions::V1_1::IQtiMapperExtensions;
using ::vendor::qti::hardware::display::mapperextensions::V1_0::Error;
using ::vendor::qti::hardware::display::mapperextensions::V1_0::PlaneLayout;
using ::vendor::qti::hardware::display::mapperextensions::V1_0::YCbCrLayout;

class QtiMapperExtensions : public IQtiMapperExtensions {
 public:
  QtiMapperExtensions();
  Return<void> getMapSecureBufferFlag(void *buffer, getMapSecureBufferFlag_cb _hidl_cb) override;
  Return<void> getInterlacedFlag(void *buffer, getInterlacedFlag_cb _hidl_cb) override;
  Return<void> getCustomDimensions(void *buffer, getCustomDimensions_cb _hidl_cb) override;
  Return<void> getRgbDataAddress(void *buffer, getRgbDataAddress_cb _hidl_cb) override;
  Return<void> calculateBufferAttributes(int32_t width, int32_t height, int32_t format,
                                         uint64_t usage,
                                         calculateBufferAttributes_cb _hidl_cb) override;
  Return<void> getCustomFormatFlags(int32_t format, uint64_t usage,
                                    getCustomFormatFlags_cb _hidl_cb) override;
  Return<void> getColorSpace(void *buffer, getColorSpace_cb _hidl_cb) override;
  Return<void> getYuvPlaneInfo(void *buffer, getYuvPlaneInfo_cb _hidl_cb) override;
  Return<Error> setSingleBufferMode(void *buffer, bool enable) override;
  Return<void> getFd(void *buffer, getFd_cb _hidl_cb) override;
  Return<void> getWidth(void *buffer, getWidth_cb _hidl_cb) override;
  Return<void> getHeight(void *buffer, getHeight_cb _hidl_cb) override;
  Return<void> getOffset(void *buffer, getOffset_cb _hidl_cb) override;
  Return<void> getSize(void *buffer, getSize_cb _hidl_cb) override;
  Return<void> getFormat(void *buffer, getFormat_cb _hidl_cb) override;
  Return<void> getPrivateFlags(void *buffer, getPrivateFlags_cb _hidl_cb) override;
  Return<void> getUnalignedWidth(void *buffer, getUnalignedWidth_cb _hidl_cb) override;
  Return<void> getUnalignedHeight(void *buffer, getUnalignedHeight_cb _hidl_cb) override;
  Return<void> getLayerCount(void *buffer, getLayerCount_cb _hidl_cb) override;
  Return<void> getId(void *buffer, getId_cb _hidl_cb) override;
  Return<void> getUsageFlags(void *buffer, getUsageFlags_cb _hidl_cb) override;
  Return<void> getSurfaceMetadata(void *buffer, getSurfaceMetadata_cb _hidl_cb) override;
  Return<void> getFormatLayout(int32_t format, uint64_t usage, int32_t flags, int32_t width,
                               int32_t height, getFormatLayout_cb hidl_cb) override;
  Return<Error> getSurfaceMetadata_V1(void *buffer, void *metadata) override;
};

}  // namespace implementation
}  // namespace V1_1
}  // namespace mapperextensions
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor

#endif  // __QTIMAPPEREXTENSIONS_H__
