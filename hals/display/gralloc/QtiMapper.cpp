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
#include "QtiMapper.h"
#include <cutils/trace.h>
#include <qdMetaData.h>
#include <sync/sync.h>
#include "gr_utils.h"

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace mapper {
namespace V3_0 {
namespace implementation {

using gralloc::BufferInfo;

QtiMapper::QtiMapper() {
  extensions_ = new QtiMapperExtensions();
  buf_mgr_ = BufferManager::GetInstance();
  ALOGD_IF(DEBUG, "Created QtiMapper instance");
}

bool QtiMapper::ValidDescriptor(const BufferDescriptorInfo_3_0 &bd) {
  if (bd.width == 0 || bd.height == 0 || (static_cast<int32_t>(bd.format) <= 0) ||
      bd.layerCount <= 0) {
    return false;
  }

  return true;
}

Error QtiMapper::CreateDescriptor(const BufferDescriptorInfo_3_0 &descriptor_info,
                                  IMapperBufferDescriptor *descriptor) {
  ALOGD_IF(DEBUG,
           "BufferDescriptorInfo: wxh: %dx%d usage: 0x%" PRIu64 " format: %d layer_count: %d",
           descriptor_info.width, descriptor_info.height, descriptor_info.usage,
           static_cast<uint32_t>(descriptor_info.format), descriptor_info.layerCount);

  if (ValidDescriptor(descriptor_info)) {
    auto vec = Encode(descriptor_info);
    *descriptor = vec;
    return Error::NONE;
  } else {
    return Error::BAD_VALUE;
  }
}

// Methods from ::android::hardware::graphics::mapper::V2_0::IMapper follow.
Return<void> QtiMapper::createDescriptor(const BufferDescriptorInfo_3_0 &descriptor_info,
                                         createDescriptor_cb hidl_cb) {
  IMapperBufferDescriptor descriptor;
  auto info_3_0 = BufferDescriptorInfo_3_0{
      descriptor_info.width,
      descriptor_info.height,
      descriptor_info.layerCount,
      static_cast<PixelFormat>(descriptor_info.format),
      descriptor_info.usage,
  };
  auto err = CreateDescriptor(info_3_0, &descriptor);
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
      static_cast<IMapper_3_0_Error>(buf_mgr_->RetainBuffer(PRIV_HANDLE_CONST(buffer_handle)));
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
  return static_cast<IMapper_3_0_Error>(buf_mgr_->ReleaseBuffer(PRIV_HANDLE_CONST(buffer)));
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

Error QtiMapper::LockBuffer(void *buffer, uint64_t usage, const hidl_handle &acquire_fence) {
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

  return static_cast<IMapper_3_0_Error>(buf_mgr_->LockBuffer(hnd, usage));
}

Return<void> QtiMapper::lock(void *buffer, uint64_t cpu_usage,
                             const IMapper::Rect & /*access_region*/,
                             const hidl_handle &acquire_fence, lock_cb hidl_cb) {
  auto err = LockBuffer(buffer, cpu_usage, acquire_fence);
  if (err != Error::NONE) {
    hidl_cb(err, nullptr, -1, -1);
    return Void();
  }

  auto hnd = PRIV_HANDLE_CONST(buffer);
  auto *out_data = reinterpret_cast<void *>(hnd->base);
  hidl_cb(err, out_data, gralloc::GetBpp(hnd->format),
          (hnd->width) * (gralloc::GetBpp(hnd->format)));
  return Void();
}

Return<void> QtiMapper::lockYCbCr(void *buffer, uint64_t cpu_usage,
                                  const IMapper::Rect & /*access_region*/,
                                  const hidl_handle &acquire_fence, lockYCbCr_cb hidl_cb) {
  YCbCrLayout layout = {};
  auto err = LockBuffer(buffer, cpu_usage, acquire_fence);
  if (err != Error::NONE) {
    hidl_cb(err, layout);
    return Void();
  }

  auto hnd = PRIV_HANDLE_CONST(buffer);
  android_ycbcr yuv_plane_info[2];
  if (gralloc::GetYUVPlaneInfo(hnd, yuv_plane_info) != 0) {
    hidl_cb(Error::BAD_VALUE, layout);
  }
  layout.y = yuv_plane_info[0].y;
  layout.cr = yuv_plane_info[0].cr;
  layout.cb = yuv_plane_info[0].cb;
  layout.yStride = static_cast<uint32_t>(yuv_plane_info[0].ystride);
  layout.cStride = static_cast<uint32_t>(yuv_plane_info[0].cstride);
  layout.chromaStep = static_cast<uint32_t>(yuv_plane_info[0].chroma_step);
  hidl_cb(Error::NONE, layout);
  return Void();
}

Return<void> QtiMapper::unlock(void *buffer, unlock_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  if (buffer != nullptr) {
    err = static_cast<IMapper_3_0_Error>(buf_mgr_->UnlockBuffer(PRIV_HANDLE_CONST(buffer)));
  }
  // We don't have a release fence
  hidl_cb(err, hidl_handle(nullptr));
  return Void();
}

Return<Error> QtiMapper::validateBufferSize(void *buffer,
                                            const BufferDescriptorInfo_3_0 &descriptor_info,
                                            uint32_t /*stride*/) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (static_cast<IMapper_3_0_Error>(buf_mgr_->IsBufferImported(hnd)) != Error::NONE) {
      return Error::BAD_BUFFER;
    }
    auto info = gralloc::BufferInfo(descriptor_info.width, descriptor_info.height,
                                    static_cast<uint32_t>(descriptor_info.format),
                                    static_cast<uint64_t>(descriptor_info.usage));
    info.layer_count = descriptor_info.layerCount;
    err = static_cast<IMapper_3_0_Error>(buf_mgr_->ValidateBufferSize(hnd, info));
  }
  return err;
}

Return<void> QtiMapper::getTransportSize(void *buffer, IMapper_3_0::getTransportSize_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  uint32_t num_fds = 0, num_ints = 0;
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (static_cast<IMapper_3_0_Error>(buf_mgr_->IsBufferImported(hnd)) != Error::NONE) {
      hidl_cb(err, num_fds, num_ints);
      return Void();
    }
    num_fds = 2;
    // TODO(user): reduce to transported values;
    num_ints = static_cast<uint32_t >(hnd->numInts);
    err = Error::NONE;
  }
  ALOGD_IF(DEBUG, "GetTransportSize: num fds: %d num ints: %d err:%d", num_fds, num_ints, err);
  hidl_cb(err, num_fds, num_ints);
  return Void();
}

Return<void> QtiMapper::isSupported(const BufferDescriptorInfo_3_0 &descriptor_info,
                                    IMapper_3_0::isSupported_cb hidl_cb) {
  IMapperBufferDescriptor descriptor;
  auto err = CreateDescriptor(descriptor_info, &descriptor);
  if (err != Error::NONE) {
    hidl_cb(err, false);
    return Void();
  }

  gralloc::BufferDescriptor desc;
  err = static_cast<IMapper_3_0_Error>(Decode(descriptor, &desc));
  if (err != Error::NONE) {
    hidl_cb(err, false);
    return Void();
  }

  buffer_handle_t buffer;
  err = static_cast<IMapper_3_0_Error>(buf_mgr_->AllocateBuffer(desc, &buffer, 0, true));
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
extern "C" IMapper_3_0 *HIDL_FETCH_IMapper(const char * /* name */) {
  ALOGD_IF(DEBUG, "Fetching IMapper from QtiMapper");
  auto mapper = new QtiMapper();
  return static_cast<IMapper_3_0 *>(mapper);
}

extern "C" IQtiMapper *HIDL_FETCH_IQtiMapper(const char * /* name */) {
  ALOGD_IF(DEBUG, "Fetching QtiMapper");
  return new QtiMapper();
}

}  // namespace implementation
}  // namespace V3_0
}  // namespace mapper
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
