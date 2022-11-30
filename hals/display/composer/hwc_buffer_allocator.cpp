/*
 * Copyright (c) 2015-2020, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of The Linux Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

#include <gralloc_priv.h>

#include <core/buffer_allocator.h>
#include <utils/constants.h>
#include <utils/debug.h>

#include "gr_utils.h"
#include "hwc_buffer_allocator.h"
#include "hwc_debugger.h"
#include "hwc_layers.h"

#define __CLASS__ "HWCBufferAllocator"

using android::hardware::graphics::mapper::V2_0::Error;
using MapperV3Error = android::hardware::graphics::mapper::V3_0::Error;
using android::hardware::graphics::mapper::V2_0::BufferDescriptor;
using MapperV3BufferDescriptor = android::hardware::graphics::mapper::V3_0::BufferDescriptor;
using android::hardware::hidl_handle;
using android::hardware::hidl_vec;

namespace sdm {

DisplayError HWCBufferAllocator::GetGrallocInstance() {
  // Lazy initialization of gralloc HALs
  if (mapper_V3_ != nullptr || mapper_V2_ != nullptr || allocator_V3_ != nullptr ||
      allocator_V2_ != nullptr) {
    return kErrorNone;
  }

  allocator_V3_ = IAllocatorV3::getService();
  if (allocator_V3_ == nullptr) {
    allocator_V2_ = IAllocatorV2::getService();
    if (allocator_V2_ == nullptr) {
      DLOGE("Unable to get allocator");
      return kErrorCriticalResource;
    }
  }

  mapper_V3_ = IMapperV3::getService();
  if (mapper_V3_ == nullptr) {
    mapper_V2_ = IMapperV2::getService();
    if (mapper_V2_ == nullptr) {
      DLOGE("Unable to get mapper");
      return kErrorCriticalResource;
    }
  }

  return kErrorNone;
}

DisplayError HWCBufferAllocator::AllocateBuffer(BufferInfo *buffer_info) {
  auto err = GetGrallocInstance();
  if (err != kErrorNone) {
    return err;
  }
  const BufferConfig &buffer_config = buffer_info->buffer_config;
  AllocatedBufferInfo *alloc_buffer_info = &buffer_info->alloc_buffer_info;
  int format;
  uint64_t alloc_flags = 0;
  int error = SetBufferInfo(buffer_config.format, &format, &alloc_flags);
  if (error != 0) {
    return kErrorParameters;
  }

  if (buffer_config.secure) {
    alloc_flags |= BufferUsage::PROTECTED;
  }

  if (buffer_config.secure_camera) {
    alloc_flags |= BufferUsage::CAMERA_OUTPUT;
  }

  if (!buffer_config.cache) {
    // Allocate uncached buffers
    alloc_flags |= GRALLOC_USAGE_PRIVATE_UNCACHED;
  }

  if (buffer_config.gfx_client) {
    alloc_flags |= BufferUsage::GPU_TEXTURE;
  }

  alloc_flags |= BufferUsage::COMPOSER_OVERLAY;

  const native_handle_t *buf = nullptr;

  if (mapper_V3_ != nullptr) {
    IMapperV3::BufferDescriptorInfo descriptor_info;
    descriptor_info.width = buffer_config.width;
    descriptor_info.height = buffer_config.height;
    descriptor_info.layerCount = 1;
    descriptor_info.format =
        static_cast<android::hardware::graphics::common::V1_2::PixelFormat>(format);
    descriptor_info.usage = alloc_flags;

    auto hidl_err = MapperV3Error::NONE;

    auto descriptor = BufferDescriptor();
    mapper_V3_->createDescriptor(descriptor_info, [&](const auto &_error, const auto &_descriptor) {
      hidl_err = _error;
      if (hidl_err != MapperV3Error::NONE) {
        return;
      }
      descriptor = _descriptor;
    });

    if (hidl_err != MapperV3Error::NONE) {
      DLOGE("Failed to create descriptor");
      return kErrorMemory;
    }

    hidl_handle raw_handle = nullptr;

    allocator_V3_->allocate(descriptor, 1,
                            [&](const auto &_error, const auto &_stride, const auto &_buffers) {
                              hidl_err = _error;
                              if (hidl_err != MapperV3Error::NONE) {
                                return;
                              }
                              raw_handle = _buffers[0];
                            });

    if (hidl_err != MapperV3Error::NONE) {
      DLOGE("Failed to allocate buffer");
      return kErrorMemory;
    }

    mapper_V3_->importBuffer(raw_handle, [&](const auto &_error, const auto &_buffer) {
      hidl_err = _error;
      if (hidl_err != MapperV3Error::NONE) {
        return;
      }
      buf = static_cast<const native_handle_t *>(_buffer);
    });

    if (hidl_err != MapperV3Error::NONE) {
      DLOGE("Failed to import buffer into HWC");
      return kErrorMemory;
    }
  } else {
    IMapperV2::BufferDescriptorInfo descriptor_info;
    descriptor_info.width = buffer_config.width;
    descriptor_info.height = buffer_config.height;
    descriptor_info.layerCount = 1;
    descriptor_info.format =
        static_cast<android::hardware::graphics::common::V1_0::PixelFormat>(format);
    descriptor_info.usage = alloc_flags;

    auto hidl_err = Error::NONE;

    auto descriptor = BufferDescriptor();
    mapper_V2_->createDescriptor(descriptor_info, [&](const auto &_error, const auto &_descriptor) {
      hidl_err = _error;
      if (hidl_err != Error::NONE) {
        return;
      }
      descriptor = _descriptor;
    });

    if (hidl_err != Error::NONE) {
      DLOGE("Failed to create descriptor");
      return kErrorMemory;
    }

    hidl_handle raw_handle = nullptr;

    allocator_V2_->allocate(descriptor, 1,
                            [&](const auto &_error, const auto &_stride, const auto &_buffers) {
                              hidl_err = _error;
                              if (hidl_err != Error::NONE) {
                                return;
                              }
                              raw_handle = _buffers[0];
                            });

    if (hidl_err != Error::NONE) {
      DLOGE("Failed to allocate buffer");
      return kErrorMemory;
    }

    mapper_V2_->importBuffer(raw_handle, [&](const auto &_error, const auto &_buffer) {
      hidl_err = _error;
      if (hidl_err != Error::NONE) {
        return;
      }
      buf = static_cast<const native_handle_t *>(_buffer);
    });

    if (hidl_err != Error::NONE) {
      DLOGE("Failed to import buffer into HWC");
      return kErrorMemory;
    }
  }

  private_handle_t *hnd = nullptr;
  hnd = (private_handle_t *)buf;  // NOLINT
  alloc_buffer_info->fd = hnd->fd;
  alloc_buffer_info->stride = UINT32(hnd->width);
  alloc_buffer_info->aligned_width = UINT32(hnd->width);
  alloc_buffer_info->aligned_height = UINT32(hnd->height);
  alloc_buffer_info->size = hnd->size;
  alloc_buffer_info->id = hnd->id;
  alloc_buffer_info->format = HWCLayer::GetSDMFormat(hnd->format, hnd->flags);

  buffer_info->private_data = reinterpret_cast<void *>(hnd);
  return kErrorNone;
}

DisplayError HWCBufferAllocator::FreeBuffer(BufferInfo *buffer_info) {
  DisplayError err = kErrorNone;
  auto hnd = reinterpret_cast<void *>(buffer_info->private_data);
  if (mapper_V3_ != nullptr) {
    mapper_V3_->freeBuffer(hnd);
  } else {
    mapper_V2_->freeBuffer(hnd);
  }
  AllocatedBufferInfo &alloc_buffer_info = buffer_info->alloc_buffer_info;

  alloc_buffer_info.fd = -1;
  alloc_buffer_info.stride = 0;
  alloc_buffer_info.size = 0;
  buffer_info->private_data = NULL;
  return err;
}

void HWCBufferAllocator::GetCustomWidthAndHeight(const private_handle_t *handle, int *width,
                                                 int *height) {
  *width = handle->width;
  *height = handle->height;
  gralloc::GetCustomDimensions(const_cast<private_handle_t *>(handle), width, height);
}

void HWCBufferAllocator::GetAlignedWidthAndHeight(int width, int height, int format,
                                                  uint32_t alloc_type, int *aligned_width,
                                                  int *aligned_height) {
  uint64_t usage = 0;
  if (alloc_type & GRALLOC_USAGE_HW_FB) {
    usage |= BufferUsage::COMPOSER_CLIENT_TARGET;
  }
  if (alloc_type & GRALLOC_USAGE_PRIVATE_ALLOC_UBWC) {
    usage |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
  }
  uint32_t aligned_w = UINT(width);
  uint32_t aligned_h = UINT(height);
  gralloc::BufferInfo info(width, height, format, usage);
  gralloc::GetAlignedWidthAndHeight(info, &aligned_w, &aligned_h);
  *aligned_width = INT(aligned_w);
  *aligned_height = INT(aligned_h);
}

uint32_t HWCBufferAllocator::GetBufferSize(BufferInfo *buffer_info) {
  const BufferConfig &buffer_config = buffer_info->buffer_config;
  uint64_t alloc_flags = GRALLOC_USAGE_PRIVATE_IOMMU_HEAP;

  int width = INT(buffer_config.width);
  int height = INT(buffer_config.height);
  int format;

  if (buffer_config.secure) {
    alloc_flags |= INT(GRALLOC_USAGE_PROTECTED);
  }

  if (!buffer_config.cache) {
    // Allocate uncached buffers
    alloc_flags |= GRALLOC_USAGE_PRIVATE_UNCACHED;
  }

  if (SetBufferInfo(buffer_config.format, &format, &alloc_flags) < 0) {
    return 0;
  }

  uint32_t aligned_width = 0, aligned_height = 0, buffer_size = 0;
  gralloc::BufferInfo info(width, height, format, alloc_flags);
  int ret = GetBufferSizeAndDimensions(info, &buffer_size, &aligned_width, &aligned_height);
  if (ret < 0) {
    return 0;
  }
  return buffer_size;
}

int HWCBufferAllocator::SetBufferInfo(LayerBufferFormat format, int *target, uint64_t *flags) {
  switch (format) {
    case kFormatRGBA8888:
      *target = HAL_PIXEL_FORMAT_RGBA_8888;
      break;
    case kFormatRGBX8888:
      *target = HAL_PIXEL_FORMAT_RGBX_8888;
      break;
    case kFormatRGB888:
      *target = HAL_PIXEL_FORMAT_RGB_888;
      break;
    case kFormatRGB565:
      *target = HAL_PIXEL_FORMAT_RGB_565;
      break;
    case kFormatBGR565:
      *target = HAL_PIXEL_FORMAT_BGR_565;
      break;
    case kFormatBGR888:
      *target = HAL_PIXEL_FORMAT_BGR_888;
      break;
    case kFormatBGRA8888:
      *target = HAL_PIXEL_FORMAT_BGRA_8888;
      break;
    case kFormatYCrCb420PlanarStride16:
      *target = HAL_PIXEL_FORMAT_YV12;
      break;
    case kFormatYCrCb420SemiPlanar:
      *target = HAL_PIXEL_FORMAT_YCrCb_420_SP;
      break;
    case kFormatYCbCr420SemiPlanar:
      *target = HAL_PIXEL_FORMAT_YCbCr_420_SP;
      break;
    case kFormatYCbCr422H2V1Packed:
      *target = HAL_PIXEL_FORMAT_YCbCr_422_I;
      break;
    case kFormatCbYCrY422H2V1Packed:
      *target = HAL_PIXEL_FORMAT_CbYCrY_422_I;
      break;
    case kFormatYCbCr422H2V1SemiPlanar:
      *target = HAL_PIXEL_FORMAT_YCbCr_422_SP;
      break;
    case kFormatYCbCr420SemiPlanarVenus:
      *target = HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS;
      break;
    case kFormatYCrCb420SemiPlanarVenus:
      *target = HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS;
      break;
    case kFormatYCbCr420SPVenusUbwc:
    case kFormatYCbCr420SPVenusTile:
      *target = HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatRGBA5551:
      *target = HAL_PIXEL_FORMAT_RGBA_5551;
      break;
    case kFormatRGBA4444:
      *target = HAL_PIXEL_FORMAT_RGBA_4444;
      break;
    case kFormatRGBA1010102:
      *target = HAL_PIXEL_FORMAT_RGBA_1010102;
      break;
    case kFormatARGB2101010:
      *target = HAL_PIXEL_FORMAT_ARGB_2101010;
      break;
    case kFormatRGBX1010102:
      *target = HAL_PIXEL_FORMAT_RGBX_1010102;
      break;
    case kFormatXRGB2101010:
      *target = HAL_PIXEL_FORMAT_XRGB_2101010;
      break;
    case kFormatBGRA1010102:
      *target = HAL_PIXEL_FORMAT_BGRA_1010102;
      break;
    case kFormatABGR2101010:
      *target = HAL_PIXEL_FORMAT_ABGR_2101010;
      break;
    case kFormatBGRX1010102:
      *target = HAL_PIXEL_FORMAT_BGRX_1010102;
      break;
    case kFormatXBGR2101010:
      *target = HAL_PIXEL_FORMAT_XBGR_2101010;
      break;
    case kFormatYCbCr420P010:
      *target = HAL_PIXEL_FORMAT_YCbCr_420_P010;
      break;
    case kFormatYCbCr420TP10Ubwc:
    case kFormatYCbCr420TP10Tile:
      *target = HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatYCbCr420P010Ubwc:
    case kFormatYCbCr420P010Tile:
      *target = HAL_PIXEL_FORMAT_YCbCr_420_P010_UBWC;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatYCbCr420P010Venus:
      *target = HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS;
      break;
    case kFormatRGBA8888Ubwc:
      *target = HAL_PIXEL_FORMAT_RGBA_8888;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatRGBX8888Ubwc:
      *target = HAL_PIXEL_FORMAT_RGBX_8888;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatBGR565Ubwc:
      *target = HAL_PIXEL_FORMAT_BGR_565;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatRGBA1010102Ubwc:
      *target = HAL_PIXEL_FORMAT_RGBA_1010102;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    case kFormatRGBX1010102Ubwc:
      *target = HAL_PIXEL_FORMAT_RGBX_1010102;
      *flags |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
      break;
    default:
      DLOGW("Unsupported format = 0x%x", format);
      return -EINVAL;
  }
  return 0;
}

DisplayError HWCBufferAllocator::GetAllocatedBufferInfo(
    const BufferConfig &buffer_config, AllocatedBufferInfo *allocated_buffer_info) {
  // TODO(user): This API should pass the buffer_info of the already allocated buffer
  // The private_data can then be typecast to the private_handle and used directly.
  uint64_t alloc_flags = GRALLOC_USAGE_PRIVATE_IOMMU_HEAP;

  int width = INT(buffer_config.width);
  int height = INT(buffer_config.height);
  int format;

  if (buffer_config.secure) {
    alloc_flags |= INT(GRALLOC_USAGE_PROTECTED);
  }

  if (!buffer_config.cache) {
    // Allocate uncached buffers
    alloc_flags |= GRALLOC_USAGE_PRIVATE_UNCACHED;
  }

  if (SetBufferInfo(buffer_config.format, &format, &alloc_flags) < 0) {
    return kErrorParameters;
  }

  uint32_t aligned_width = 0, aligned_height = 0, buffer_size = 0;
  gralloc::BufferInfo info(width, height, format, alloc_flags);
  int ret = GetBufferSizeAndDimensions(info, &buffer_size, &aligned_width, &aligned_height);
  if (ret < 0) {
    return kErrorParameters;
  }
  allocated_buffer_info->stride = UINT32(aligned_width);
  allocated_buffer_info->aligned_width = UINT32(aligned_width);
  allocated_buffer_info->aligned_height = UINT32(aligned_height);
  allocated_buffer_info->size = UINT32(buffer_size);

  return kErrorNone;
}

DisplayError HWCBufferAllocator::GetBufferLayout(const AllocatedBufferInfo &buf_info,
                                                 uint32_t stride[4], uint32_t offset[4],
                                                 uint32_t *num_planes) {
  // TODO(user): Transition APIs to not need a private handle
  private_handle_t hnd(-1, 0, 0, 0, 0, 0, 0);
  int format = HAL_PIXEL_FORMAT_RGBA_8888;
  uint64_t flags = 0;

  SetBufferInfo(buf_info.format, &format, &flags);
  // Setup only the required stuff, skip rest
  hnd.format = format;
  hnd.width = INT32(buf_info.aligned_width);
  hnd.height = INT32(buf_info.aligned_height);
  if (flags & GRALLOC_USAGE_PRIVATE_ALLOC_UBWC) {
    hnd.flags = private_handle_t::PRIV_FLAGS_UBWC_ALIGNED;
  }

  int ret = gralloc::GetBufferLayout(&hnd, stride, offset, num_planes);
  if (ret < 0) {
    DLOGE("GetBufferLayout failed");
    return kErrorParameters;
  }

  return kErrorNone;
}

DisplayError HWCBufferAllocator::MapBuffer(const private_handle_t *handle,
                                           shared_ptr<Fence> acquire_fence) {
  auto err = GetGrallocInstance();
  if (err != kErrorNone) {
    return err;
  }

  Fence::ScopedRef scoped_ref;
  NATIVE_HANDLE_DECLARE_STORAGE(acquire_fence_storage, 1, 0);
  hidl_handle acquire_fence_handle;
  if (acquire_fence) {
    auto h = native_handle_init(acquire_fence_storage, 1, 0);
    h->data[0] = scoped_ref.Get(acquire_fence);
    acquire_fence_handle = h;
  }

  auto hnd = const_cast<private_handle_t *>(handle);
  void *buffer_ptr = NULL;
  if (mapper_V3_ != nullptr) {
    const IMapperV3::Rect access_region = {.left = 0, .top = 0, .width = 0, .height = 0};
    mapper_V3_->lock(
        reinterpret_cast<void *>(hnd), (uint64_t)BufferUsage::CPU_READ_OFTEN, access_region,
        acquire_fence_handle,
        [&](const auto &_error, const auto &_buffer, const auto &_bpp, const auto &_stride) {
          if (_error == MapperV3Error::NONE) {
            buffer_ptr = _buffer;
          }
        });
  } else {
    const IMapperV2::Rect access_region = {.left = 0, .top = 0, .width = 0, .height = 0};
    mapper_V2_->lock(reinterpret_cast<void *>(hnd), (uint64_t)BufferUsage::CPU_READ_OFTEN,
                     access_region, acquire_fence_handle,
                     [&](const auto &_error, const auto &_buffer) {
                       if (_error == Error::NONE) {
                         buffer_ptr = _buffer;
                       }
                     });
  }
  if (!buffer_ptr) {
    return kErrorUndefined;
  }
  return kErrorNone;
}

DisplayError HWCBufferAllocator::UnmapBuffer(const private_handle_t *handle, int *release_fence) {
  DisplayError err = kErrorNone;
  *release_fence = -1;
  auto hnd = const_cast<private_handle_t *>(handle);
  if (mapper_V3_ != nullptr) {
    mapper_V3_->unlock(reinterpret_cast<void *>(hnd),
                       [&](const auto &_error, const auto &_release_fence) {
                         if (_error != MapperV3Error::NONE) {
                           err = kErrorUndefined;
                         }
                       });
  } else {
    mapper_V2_->unlock(reinterpret_cast<void *>(hnd),
                       [&](const auto &_error, const auto &_release_fence) {
                         if (_error != Error::NONE) {
                           err = kErrorUndefined;
                         }
                       });
  }
  return err;
}

}  // namespace sdm
