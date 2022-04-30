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

#ifndef __QTIMAPPER4_H__
#define __QTIMAPPER4_H__

#include <QtiGralloc.h>
#include <gralloctypes/Gralloc4.h>
#include <hidl/MQDescriptor.h>
#include <hidl/Status.h>
#include <vendor/qti/hardware/display/mapper/4.0/IQtiMapper.h>

#include <algorithm>
#include <string>

#include "QtiMapperExtensions.h"
#include "gr_buf_mgr.h"
namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace mapper {
namespace V4_0 {
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
using ::android::hardware::graphics::mapper::V4_0::Error;
using ::android::hardware::graphics::mapper::V4_0::IMapper;
using ::android::hidl::base::V1_0::DebugInfo;
using ::android::hidl::base::V1_0::IBase;
using gralloc::BufferManager;
using ::vendor::qti::hardware::display::mapper::V4_0::IQtiMapper;
using ::vendor::qti::hardware::display::mapperextensions::V1_1::IQtiMapperExtensions;
using ::vendor::qti::hardware::display::mapperextensions::V1_1::implementation::QtiMapperExtensions;

using android::hardware::graphics::mapper::V4_0::IMapper;
using BufferDescriptorInfo_4_0 =
    android::hardware::graphics::mapper::V4_0::IMapper::BufferDescriptorInfo;
using IMapperBufferDescriptor = android::hardware::graphics::mapper::V4_0::BufferDescriptor;
using MetadataType = ::android::hardware::graphics::mapper::V4_0::IMapper::MetadataType;
using MetadataTypeDescription =
    ::android::hardware::graphics::mapper::V4_0::IMapper::MetadataTypeDescription;
using IMapper_4_0_Error = ::android::hardware::graphics::mapper::V4_0::Error;

class QtiMapper : public IQtiMapper {
 public:
  QtiMapper();
  // Methods from ::android::hardware::graphics::mapper::V2_0::IMapper follow.
  Return<void> createDescriptor(const BufferDescriptorInfo_4_0 &descriptor_info,
                                createDescriptor_cb hidl_cb) override;
  Return<void> importBuffer(const hidl_handle &raw_handle, importBuffer_cb hidl_cb) override;
  Return<Error> freeBuffer(void *buffer) override;
  Return<void> lock(void *buffer, uint64_t cpu_usage, const IMapper::Rect &access_region,
                    const hidl_handle &acquire_fence, lock_cb hidl_cb) override;
  Return<void> unlock(void *buffer, unlock_cb hidl_cb) override;

  // Methods from ::android::hardware::graphics::mapper::V2_1::IMapper follow.
  Return<Error> validateBufferSize(void *buffer, const BufferDescriptorInfo_4_0 &descriptorInfo,
                                   uint32_t stride) override;
  Return<void> getTransportSize(void *buffer, getTransportSize_cb hidl_cb) override;

  Return<void> isSupported(const BufferDescriptorInfo_4_0 &descriptor_info,
                           isSupported_cb hidl_cb) override;

  Return<void> getMapperExtensions(getMapperExtensions_cb hidl_cb);
  sp<mapperextensions::V1_1::IQtiMapperExtensions> extensions_ = nullptr;

  // Methods from ::android::hardware::graphics::mapper::V4:0::IMapper follow.
  Return<void> get(void *buffer, const MetadataType &metadataType, get_cb hidl_cb) override;
  Return<Error> set(void *buffer, const MetadataType &metadataType,
                    const hidl_vec<uint8_t> &metadata) override;
  Return<void> getFromBufferDescriptorInfo(const BufferDescriptorInfo &description,
                                           const MetadataType &metadataType,
                                           getFromBufferDescriptorInfo_cb hidl_cb) override;
  Return<void> flushLockedBuffer(void *buffer, flushLockedBuffer_cb hidl_cb);
  Return<Error> rereadLockedBuffer(void *buffer);
  Return<void> listSupportedMetadataTypes(listSupportedMetadataTypes_cb hidl_cb);
  Return<void> getReservedRegion(void *buffer, getReservedRegion_cb _hidl_cb);
  Return<void> dumpBuffer(void *buffer, dumpBuffer_cb _hidl_cb);
  Return<void> dumpBuffers(dumpBuffers_cb _hidl_cb);

  hidl_vec<uint8_t> Encode(const BufferDescriptorInfo_4_0 &bd_info) {
    hidl_vec<uint8_t> out;

    uint64_t name_size = bd_info.name.size();

    /* Name length is variable, need to store string prepended with size
     * The rest of the packet size is constant
     */
    out.resize(gralloc::kBufferDescriptorSizeV4 + sizeof(name_size) + name_size);

    int index = 0;
    uint32_t magic_version = gralloc::kMagicVersion;
    std::memcpy(&out[index], &magic_version, sizeof(magic_version));
    index += sizeof(magic_version);

    std::memcpy(&out[index], &name_size, sizeof(name_size));
    index += sizeof(name_size);

    std::memcpy(&out[index], bd_info.name.c_str(), bd_info.name.size());
    index += name_size;

    std::memcpy(&out[index], &bd_info.width, sizeof(bd_info.width));
    index += sizeof(bd_info.width);

    std::memcpy(&out[index], &bd_info.height, sizeof(bd_info.height));
    index += sizeof(bd_info.height);

    std::memcpy(&out[index], &bd_info.layerCount, sizeof(bd_info.layerCount));
    index += sizeof(bd_info.layerCount);

    std::memcpy(&out[index], &bd_info.format, sizeof(bd_info.format));
    index += sizeof(bd_info.format);

    std::memcpy(&out[index], &bd_info.usage, sizeof(bd_info.usage));
    index += sizeof(bd_info.usage);

    std::memcpy(&out[index], &bd_info.reservedSize, sizeof(bd_info.reservedSize));

    return out;
  }
  static gralloc::Error Decode(const hidl_vec<uint8_t> &in,
                               gralloc::BufferDescriptor *buf_descriptor) {
    // First check is to avoid dereferencing if the vector is too short
    if (in.size() < gralloc::kBufferDescriptorSizeV4) {
      return gralloc::Error::BAD_DESCRIPTOR;
    }

    int index = 0;
    uint32_t magic_version;
    std::memcpy(&magic_version, &in[index], sizeof(magic_version));
    index += sizeof(magic_version);

    uint64_t name_size;
    std::memcpy(&name_size, &in[index], sizeof(name_size));
    index += sizeof(name_size);

    // The second check validates that the size and magic version are correct
    if (in.size() != (gralloc::kBufferDescriptorSizeV4 + name_size + sizeof(name_size)) ||
        magic_version != gralloc::kMagicVersion) {
      return gralloc::Error::BAD_DESCRIPTOR;
    }

    std::string name;

    name.resize(name_size);
    std::memcpy(name.data(), &in[index], name.size());
    index += name_size;
    buf_descriptor->SetName(name);

    uint32_t width, height;
    std::memcpy(&width, &in[index], sizeof(width));

    index += sizeof(width);
    std::memcpy(&height, &in[index], sizeof(height));
    index += sizeof(height);
    buf_descriptor->SetDimensions(width, height);

    uint32_t layer_count;
    std::memcpy(&layer_count, &in[index], sizeof(layer_count));
    index += sizeof(layer_count);
    buf_descriptor->SetLayerCount(layer_count);

    uint32_t format;
    std::memcpy(&format, &in[index], sizeof(format));
    index += sizeof(format);
    buf_descriptor->SetColorFormat(format);

    uint64_t usage;
    std::memcpy(&usage, &in[index], sizeof(usage));
    index += sizeof(usage);
    buf_descriptor->SetUsage(usage);

    uint64_t reserved_size;
    std::memcpy(&reserved_size, &in[index], sizeof(reserved_size));
    index += sizeof(reserved_size);

    buf_descriptor->SetReservedSize(reserved_size);
    return gralloc::Error::NONE;
  }

 private:
  BufferManager *buf_mgr_ = nullptr;
  Error CreateDescriptor(const BufferDescriptorInfo_4_0 &descriptor_info,
                         IMapperBufferDescriptor *descriptor);
  bool ValidDescriptor(const IMapper::BufferDescriptorInfo &bd);
  bool GetFenceFd(const hidl_handle &fence_handle, int *outFenceFd);
  void WaitFenceFd(int fence_fd);
  Error LockBuffer(void *buffer, uint64_t usage, const hidl_handle &acquire_fence,
                   const IMapper::Rect &access_region);

  Error DumpBufferMetadata(const private_handle_t *buffer, BufferDump *outBufferDump);

  hidl_vec<MetadataTypeDescription> metadata_type_descriptions_ = {
      // MetadataType, description, gettable, settable
      {android::gralloc4::MetadataType_BufferId, "", true, false},
      {android::gralloc4::MetadataType_Name, "", true, false},
      {android::gralloc4::MetadataType_Width, "", true, false},
      {android::gralloc4::MetadataType_Height, "", true, false},
      {android::gralloc4::MetadataType_LayerCount, "", true, false},
      {android::gralloc4::MetadataType_PixelFormatRequested, "", true, false},
      {android::gralloc4::MetadataType_PixelFormatFourCC, "", true, false},
      {android::gralloc4::MetadataType_PixelFormatModifier, "", true, false},
      {android::gralloc4::MetadataType_Usage, "", true, false},
      {android::gralloc4::MetadataType_AllocationSize, "", true, false},
      {android::gralloc4::MetadataType_ProtectedContent, "", true, false},
      {android::gralloc4::MetadataType_ChromaSiting, "", true, false},
      {android::gralloc4::MetadataType_Compression, "", true, false},
      {android::gralloc4::MetadataType_Interlaced, "", true, false},
      {android::gralloc4::MetadataType_PlaneLayouts, "", true, false},
      {android::gralloc4::MetadataType_Dataspace, "", true, true},
      {android::gralloc4::MetadataType_BlendMode, "", true, true},
      {android::gralloc4::MetadataType_Smpte2086, "", true, true},
      {android::gralloc4::MetadataType_Cta861_3, "", true, true},
      {android::gralloc4::MetadataType_Smpte2094_40, "", true, true},
      {android::gralloc4::MetadataType_Crop, "", true, true},
      {qtigralloc::MetadataType_VTTimestamp, "VT Timestamp", true, true},
      {qtigralloc::MetadataType_ColorMetadata, "Color metadata", true, true},
      {qtigralloc::MetadataType_PPParamInterlaced, "Interlaced", true, true},
      {qtigralloc::MetadataType_VideoPerfMode, "Video perf mode", true, true},
      {qtigralloc::MetadataType_GraphicsMetadata, "Graphics metadata", true, true},
      {qtigralloc::MetadataType_UBWCCRStatsInfo, "UBWC stats", true, true},
      {qtigralloc::MetadataType_RefreshRate, "Refresh rate", true, true},
      {qtigralloc::MetadataType_MapSecureBuffer, "Secure buffer mappable", true, true},
      {qtigralloc::MetadataType_LinearFormat, "Linear format", true, true},
      {qtigralloc::MetadataType_SingleBufferMode, "Single buffer mode flag", true, true},
      {qtigralloc::MetadataType_CVPMetadata, "CVP metadata", true, true},
      {qtigralloc::MetadataType_VideoHistogramStats, "Video histogram stats", true, true},
      {qtigralloc::MetadataType_FD, "fd from private_handle_t", true, false},
      {qtigralloc::MetadataType_PrivateFlags, "Flags in private_handle_t", true, false},
      {qtigralloc::MetadataType_AlignedWidthInPixels, "width in private_handle_t", true, false},
      {qtigralloc::MetadataType_AlignedHeightInPixels, "height in private_handle_t", true, false},
#ifdef METADATA_V2
      {qtigralloc::MetadataType_StandardMetadataStatus, "Is standard metadata set", true, false},
      {qtigralloc::MetadataType_VendorMetadataStatus, "Is vendor metadata set", true, false},
#endif
#ifdef QTI_BUFFER_TYPE
      {qtigralloc::MetadataType_BufferType, "Buffer type from private_handle_t", true, false},
#endif
  };
};

}  // namespace implementation
}  // namespace V4_0
}  // namespace mapper
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor

#endif  // __QTIMAPPER4_H__
