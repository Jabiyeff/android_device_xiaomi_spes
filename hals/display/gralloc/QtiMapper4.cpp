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

#define ATRACE_TAG (ATRACE_TAG_GRAPHICS | ATRACE_TAG_HAL)
#define DEBUG 0
#include "QtiMapper4.h"

#include <cutils/trace.h>
#include <qdMetaData.h>
#include <sync/sync.h>

#include <vector>

#include "gr_utils.h"

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace mapper {
namespace V4_0 {
namespace implementation {

using gralloc::BufferInfo;

using aidl::android::hardware::graphics::common::StandardMetadataType;
QtiMapper::QtiMapper() {
  extensions_ = new QtiMapperExtensions();
  buf_mgr_ = BufferManager::GetInstance();
  ALOGD_IF(DEBUG, "Created QtiMapper instance");
}

bool QtiMapper::ValidDescriptor(const BufferDescriptorInfo_4_0 &bd) {
  if (bd.width == 0 || bd.height == 0 || (static_cast<int32_t>(bd.format) <= 0) ||
      bd.layerCount <= 0) {
    return false;
  }

  return true;
}

Error QtiMapper::CreateDescriptor(const BufferDescriptorInfo_4_0 &descriptor_info,
                                  IMapperBufferDescriptor *descriptor) {
  ALOGD_IF(DEBUG,
           "BufferDescriptorInfo: name %s wxh: %dx%d usage: 0x%" PRIu64
           " format: %d layer_count: %d",
           descriptor_info.name.c_str(), descriptor_info.width, descriptor_info.height,
           descriptor_info.usage, static_cast<uint32_t>(descriptor_info.format),
           descriptor_info.layerCount);

  if (ValidDescriptor(descriptor_info)) {
    auto vec = Encode(descriptor_info);
    *descriptor = vec;
    return Error::NONE;
  } else {
    return Error::BAD_VALUE;
  }
}

// Methods from ::android::hardware::graphics::mapper::V2_0::IMapper follow.
Return<void> QtiMapper::createDescriptor(const BufferDescriptorInfo_4_0 &descriptor_info,
                                         createDescriptor_cb hidl_cb) {
  IMapperBufferDescriptor descriptor;
  auto info_4_0 = BufferDescriptorInfo_4_0{descriptor_info.name,
                                           descriptor_info.width,
                                           descriptor_info.height,
                                           descriptor_info.layerCount,
                                           static_cast<PixelFormat>(descriptor_info.format),
                                           descriptor_info.usage,
                                           descriptor_info.reservedSize};
  auto err = CreateDescriptor(info_4_0, &descriptor);
  hidl_cb(err, descriptor);
  return Void();
}

Return<void> QtiMapper::importBuffer(const hidl_handle &raw_handle, importBuffer_cb hidl_cb) {
  if (!raw_handle.getNativeHandle()) {
    ALOGE("%s: Unable to import handle", __FUNCTION__);
    hidl_cb(Error::BAD_BUFFER, nullptr);
    return Void();
  }

  native_handle_t *buffer_handle = native_handle_clone(raw_handle.getNativeHandle());
  if (!buffer_handle) {
    ALOGE("%s: Unable to clone handle", __FUNCTION__);
    hidl_cb(Error::NO_RESOURCES, nullptr);
    return Void();
  }

  auto error =
      static_cast<IMapper_4_0_Error>(buf_mgr_->RetainBuffer(PRIV_HANDLE_CONST(buffer_handle)));
  if (error != Error::NONE) {
    ALOGE("%s: Unable to retain handle: %p", __FUNCTION__, buffer_handle);
    native_handle_close(buffer_handle);
    native_handle_delete(buffer_handle);

    hidl_cb(error, nullptr);
    return Void();
  }
  ALOGD_IF(DEBUG, "Imported handle: %p id: %" PRIu64, buffer_handle,
           PRIV_HANDLE_CONST(buffer_handle)->id);
  hidl_cb(Error::NONE, buffer_handle);
  return Void();
}

Return<Error> QtiMapper::freeBuffer(void *buffer) {
  if (!buffer) {
    return Error::BAD_BUFFER;
  }
  return static_cast<IMapper_4_0_Error>(buf_mgr_->ReleaseBuffer(PRIV_HANDLE_CONST(buffer)));
}

bool QtiMapper::GetFenceFd(const hidl_handle &fence_handle, int *outFenceFd) {
  auto handle = fence_handle.getNativeHandle();
  if (handle && handle->numFds > 1) {
    ALOGE("invalid fence handle with %d fds", handle->numFds);
    return false;
  }

  *outFenceFd = (handle && handle->numFds == 1) ? handle->data[0] : -1;
  return true;
}

void QtiMapper::WaitFenceFd(int fence_fd) {
  if (fence_fd < 0) {
    return;
  }

  const int timeout = 3000;
  ATRACE_BEGIN("fence wait");
  const int error = sync_wait(fence_fd, timeout);
  ATRACE_END();
  if (error < 0) {
    ALOGE("QtiMapper: lock fence %d didn't signal in %u ms -  error: %s", fence_fd, timeout,
          strerror(errno));
  }
}

Error QtiMapper::LockBuffer(void *buffer, uint64_t usage, const hidl_handle &acquire_fence,
                            const IMapper::Rect &access_region) {
  if (!buffer) {
    return Error::BAD_BUFFER;
  }

  int fence_fd;
  if (!GetFenceFd(acquire_fence, &fence_fd)) {
    return Error::BAD_VALUE;
  }

  if (fence_fd > 0) {
    WaitFenceFd(fence_fd);
  }

  auto hnd = PRIV_HANDLE_CONST(buffer);

  if (access_region.top < 0 || access_region.left < 0 || access_region.width < 0 ||
      access_region.height < 0 || access_region.width > hnd->width ||
      access_region.height > hnd->height) {
    return Error::BAD_VALUE;
  }
  return static_cast<IMapper_4_0_Error>(buf_mgr_->LockBuffer(hnd, usage));
}

Return<void> QtiMapper::lock(void *buffer, uint64_t cpu_usage, const IMapper::Rect &access_region,
                             const hidl_handle &acquire_fence, lock_cb hidl_cb) {
  auto err = LockBuffer(buffer, cpu_usage, acquire_fence, access_region);
  if (err != Error::NONE) {
    hidl_cb(err, nullptr);
    return Void();
  }

  auto hnd = PRIV_HANDLE_CONST(buffer);
  auto *out_data = reinterpret_cast<void *>(hnd->base);

  hidl_cb(err, out_data);
  return Void();
}

Return<void> QtiMapper::unlock(void *buffer, unlock_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  if (buffer != nullptr) {
    err = static_cast<IMapper_4_0_Error>(buf_mgr_->UnlockBuffer(PRIV_HANDLE_CONST(buffer)));
  }
  // We don't have a release fence
  hidl_cb(err, hidl_handle(nullptr));
  return Void();
}

Return<Error> QtiMapper::validateBufferSize(void *buffer,
                                            const BufferDescriptorInfo_4_0 &descriptor_info,
                                            uint32_t /*stride*/) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (static_cast<IMapper_4_0_Error>(buf_mgr_->IsBufferImported(hnd)) != Error::NONE) {
      return Error::BAD_BUFFER;
    }
    auto info = gralloc::BufferInfo(descriptor_info.width, descriptor_info.height,
                                    static_cast<uint32_t>(descriptor_info.format),
                                    static_cast<uint64_t>(descriptor_info.usage));
    info.layer_count = descriptor_info.layerCount;
    err = static_cast<IMapper_4_0_Error>(buf_mgr_->ValidateBufferSize(hnd, info));
  }
  return err;
}

Return<void> QtiMapper::getTransportSize(void *buffer, getTransportSize_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  uint32_t num_fds = 0, num_ints = 0;
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (static_cast<IMapper_4_0_Error>(buf_mgr_->IsBufferImported(hnd)) != Error::NONE) {
      hidl_cb(err, num_fds, num_ints);
      return Void();
    }
    num_fds = 2;
    // TODO(user): reduce to transported values;
    num_ints = static_cast<uint32_t>(hnd->numInts);
    err = Error::NONE;
  }
  ALOGD_IF(DEBUG, "GetTransportSize: num fds: %d num ints: %d err:%d", num_fds, num_ints, err);
  hidl_cb(err, num_fds, num_ints);
  return Void();
}

Return<void> QtiMapper::get(void *buffer, const MetadataType &metadataType, get_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  hidl_vec<uint8_t> metadata;
  if (buffer != nullptr) {
    if (metadataType.name != GRALLOC4_STANDARD_METADATA_TYPE &&
        metadataType.name != qtigralloc::VENDOR_QTI) {
      hidl_cb(Error::UNSUPPORTED, metadata);
      return Void();
    }
    auto hnd = static_cast<private_handle_t *>(buffer);
    err = static_cast<IMapper_4_0_Error>(buf_mgr_->GetMetadata(hnd, metadataType.value, &metadata));
  }
  hidl_cb(err, metadata);
  return Void();
}

Return<Error> QtiMapper::set(void *buffer, const MetadataType &metadataType,
                             const hidl_vec<uint8_t> &metadata) {
  auto err = Error::BAD_BUFFER;
  if (buffer != nullptr) {
    auto hnd = static_cast<private_handle_t *>(buffer);
    err = static_cast<IMapper_4_0_Error>(buf_mgr_->SetMetadata(hnd, metadataType.value, metadata));
  }
  return err;
}

Return<void> QtiMapper::getFromBufferDescriptorInfo(const BufferDescriptorInfo &description,
                                                    const MetadataType &metadataType,
                                                    getFromBufferDescriptorInfo_cb hidl_cb) {
  hidl_vec<uint8_t> out;
  auto err = Error::UNSUPPORTED;
  switch (metadataType.value) {
    case static_cast<int64_t>(StandardMetadataType::NAME):
      err = static_cast<IMapper_4_0_Error>(android::gralloc4::encodeName(description.name, &out));
      break;
    case static_cast<int64_t>(StandardMetadataType::WIDTH):
      err = static_cast<IMapper_4_0_Error>(android::gralloc4::encodeWidth(description.width, &out));
      break;
    case static_cast<int64_t>(StandardMetadataType::HEIGHT):
      err =
          static_cast<IMapper_4_0_Error>(android::gralloc4::encodeHeight(description.height, &out));
      break;
    case static_cast<int64_t>(StandardMetadataType::LAYER_COUNT):
      err = static_cast<IMapper_4_0_Error>(
          android::gralloc4::encodeLayerCount(description.layerCount, &out));
      break;
    case static_cast<int64_t>(StandardMetadataType::PIXEL_FORMAT_REQUESTED):
      err = static_cast<IMapper_4_0_Error>(
          android::gralloc4::encodePixelFormatRequested(description.format, &out));
      break;
    case static_cast<int64_t>(StandardMetadataType::USAGE):
      err = static_cast<IMapper_4_0_Error>(android::gralloc4::encodeUsage(description.usage, &out));
      break;
    case static_cast<int64_t>(StandardMetadataType::COMPRESSION): {
      int format =
          gralloc::GetImplDefinedFormat(description.usage, static_cast<int>(description.format));
      if (gralloc::IsUBwcEnabled(format, description.usage)) {
        err = static_cast<IMapper_4_0_Error>(
            android::gralloc4::encodeCompression(qtigralloc::Compression_QtiUBWC, &out));
      } else {
        err = static_cast<IMapper_4_0_Error>(
            android::gralloc4::encodeCompression(android::gralloc4::Compression_None, &out));
      }
      break;
    }
    case static_cast<int64_t>(StandardMetadataType::PROTECTED_CONTENT): {
      uint64_t protected_content = 0;
      if (description.usage & GRALLOC_USAGE_PROTECTED &&
          !(description.usage & GRALLOC_USAGE_SW_READ_MASK) &&
          !(description.usage & GRALLOC_USAGE_SW_WRITE_MASK)) {
        protected_content = 1;
      }
      err = static_cast<IMapper_4_0_Error>(
          android::gralloc4::encodeProtectedContent(protected_content, &out));
      break;
    }
    case static_cast<int64_t>(StandardMetadataType::PIXEL_FORMAT_FOURCC):
    case static_cast<int64_t>(StandardMetadataType::PIXEL_FORMAT_MODIFIER): {
      int format =
          gralloc::GetImplDefinedFormat(description.usage, static_cast<int>(description.format));
      uint32_t drm_format;
      uint64_t drm_format_modifier;
      if (gralloc::IsUBwcEnabled(format, description.usage)) {
        gralloc::GetDRMFormat(format, private_handle_t::PRIV_FLAGS_UBWC_ALIGNED, &drm_format,
                              &drm_format_modifier);
      } else {
        gralloc::GetDRMFormat(format, 0, &drm_format, &drm_format_modifier);
      }
      if (metadataType.value == static_cast<int64_t>(StandardMetadataType::PIXEL_FORMAT_FOURCC)) {
        err = static_cast<IMapper_4_0_Error>(
            android::gralloc4::encodePixelFormatFourCC(drm_format, &out));
      } else {
        err = static_cast<IMapper_4_0_Error>(
            android::gralloc4::encodePixelFormatModifier(drm_format_modifier, &out));
      }
      break;
    }
    default:
      break;
  }

  hidl_cb(err, out);
  return Void();
}
Return<void> QtiMapper::flushLockedBuffer(void *buffer, flushLockedBuffer_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  if (buffer != nullptr) {
    err = static_cast<IMapper_4_0_Error>(buf_mgr_->FlushBuffer(PRIV_HANDLE_CONST(buffer)));
  }
  // We don't have a release fence
  hidl_cb(err, hidl_handle(nullptr));
  return Void();
}

Return<Error> QtiMapper::rereadLockedBuffer(void *buffer) {
  auto err = Error::BAD_BUFFER;
  if (buffer != nullptr) {
    err = static_cast<IMapper_4_0_Error>(buf_mgr_->RereadBuffer(PRIV_HANDLE_CONST(buffer)));
  }
  return err;
}

Return<void> QtiMapper::getReservedRegion(void *buffer, getReservedRegion_cb hidl_cb) {
  auto hnd = static_cast<private_handle_t *>(buffer);
  void *reserved_region = nullptr;
  uint64_t reserved_size = 0;
  if (static_cast<IMapper_4_0_Error>(buf_mgr_->IsBufferImported(hnd)) != Error::NONE) {
    hidl_cb(Error::BAD_BUFFER, reserved_region, reserved_size);
  }
  auto err = static_cast<IMapper_4_0_Error>(
      buf_mgr_->GetReservedRegion(hnd, &reserved_region, &reserved_size));

  hidl_cb(err, reserved_region, reserved_size);
  return Void();
}

Error QtiMapper::DumpBufferMetadata(const private_handle_t *buffer, BufferDump *outBufferDump) {
  outBufferDump->metadataDump.resize(metadata_type_descriptions_.size());
  for (int i = 0; i < static_cast<int>(metadata_type_descriptions_.size()); i++) {
    auto type = metadata_type_descriptions_[i].metadataType;
    hidl_vec<uint8_t> metadata;
    if (static_cast<IMapper_4_0_Error>(buf_mgr_->GetMetadata(
            const_cast<private_handle_t *>(buffer), type.value, &metadata)) == Error::BAD_BUFFER) {
      // If buffer is deleted during metadata dump, return BAD_BUFFER
      return Error::BAD_BUFFER;
    }
    MetadataDump metadata_dump = {type, metadata};
    outBufferDump->metadataDump[i] = metadata_dump;
  }
  return Error::NONE;
}
Return<void> QtiMapper::dumpBuffer(void *buffer, dumpBuffer_cb hidl_cb) {
  BufferDump buffer_dump;
  auto hnd = PRIV_HANDLE_CONST(buffer);
  if (buffer != nullptr) {
    if (DumpBufferMetadata(hnd, &buffer_dump) == Error::NONE) {
      hidl_cb(Error::NONE, buffer_dump);
      return Void();
    }
  }
  hidl_cb(Error::BAD_BUFFER, buffer_dump);
  return Void();
}
Return<void> QtiMapper::dumpBuffers(dumpBuffers_cb hidl_cb) {
  hidl_vec<BufferDump> buffers_dump;
  std::vector<const private_handle_t *> handle_list;
  if (static_cast<IMapper_4_0_Error>(buf_mgr_->GetAllHandles(&handle_list)) != Error::NONE) {
    hidl_cb(Error::NO_RESOURCES, buffers_dump);
  }
  buffers_dump.resize(handle_list.size());
  for (int i = 0; i < handle_list.size(); i++) {
    BufferDump buffer_dump;
    if (DumpBufferMetadata(handle_list[i], &buffer_dump) != Error::NONE) {
      continue;
    }
    buffers_dump[i] = buffer_dump;
  }
  hidl_cb(Error::NONE, buffers_dump);
  return Void();
}

Return<void> QtiMapper::listSupportedMetadataTypes(listSupportedMetadataTypes_cb hidl_cb) {
  hidl_cb(Error::NONE, metadata_type_descriptions_);
  return Void();
}

Return<void> QtiMapper::isSupported(const BufferDescriptorInfo_4_0 &descriptor_info,
                                    isSupported_cb hidl_cb) {
  IMapperBufferDescriptor descriptor;
  auto err = CreateDescriptor(descriptor_info, &descriptor);
  if (err != Error::NONE) {
    hidl_cb(err, false);
    return Void();
  }

  gralloc::BufferDescriptor desc;
  err = static_cast<Error>(Decode(descriptor, &desc));
  if (err != Error::NONE) {
    hidl_cb(err, false);
    return Void();
  }

  buffer_handle_t buffer;
  err = static_cast<IMapper_4_0_Error>(buf_mgr_->AllocateBuffer(desc, &buffer, 0, true));
  if (err != Error::NONE) {
    hidl_cb(err, false);
  } else {
    hidl_cb(err, true);
  }

  return Void();
}

Return<void> QtiMapper::getMapperExtensions(QtiMapper::getMapperExtensions_cb hidl_cb) {
  if (extensions_ != nullptr) {
    hidl_cb(Error::NONE, extensions_);
  } else {
    hidl_cb(Error::UNSUPPORTED, extensions_);
  }
  return Void();
}

// Methods from ::android::hidl::base::V1_0::IBase follow.

// When we are in passthrough mode, this method is used
// by hidl to obtain the SP HAL object
extern "C" IMapper *HIDL_FETCH_IMapper(const char * /* name */) {
  ALOGD_IF(DEBUG, "Fetching IMapper from QtiMapper");
  auto mapper = new QtiMapper();
  return static_cast<IMapper *>(mapper);
}

extern "C" IQtiMapper *HIDL_FETCH_IQtiMapper(const char * /* name */) {
  ALOGD_IF(DEBUG, "Fetching QtiMapper");
  return new QtiMapper();
}

}  // namespace implementation
}  // namespace V4_0
}  // namespace mapper
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
