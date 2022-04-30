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

#define ATRACE_TAG (ATRACE_TAG_GRAPHICS | ATRACE_TAG_HAL)
#define DEBUG 0
#include "QtiMapperExtensions.h"
#include <cutils/trace.h>
#include <qdMetaData.h>
#include <sync/sync.h>
#include "gr_utils.h"

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace mapperextensions {
namespace V1_1 {
namespace implementation {

using gralloc::BufferInfo;

QtiMapperExtensions::QtiMapperExtensions() {}

Return<void> QtiMapperExtensions::getMapSecureBufferFlag(void *buffer,
                                                         getMapSecureBufferFlag_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  int map_secure_buffer = 0;
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (getMetaData(hnd, GET_MAP_SECURE_BUFFER, &map_secure_buffer) != 0) {
      map_secure_buffer = 0;
    } else {
      err = Error::NONE;
    }
  }
  hidl_cb(err, map_secure_buffer != 0);
  return Void();
}

Return<void> QtiMapperExtensions::getInterlacedFlag(void *buffer, getInterlacedFlag_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  int interlaced_flag = 0;
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (getMetaData(hnd, GET_PP_PARAM_INTERLACED, &interlaced_flag) != 0) {
      interlaced_flag = 0;
    } else {
      err = Error::NONE;
    }
  }
  hidl_cb(err, interlaced_flag != 0);
  return Void();
}

Return<void> QtiMapperExtensions::getCustomDimensions(void *buffer,
                                                      getCustomDimensions_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  int stride = 0;
  int height = 0;
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    stride = hnd->width;
    height = hnd->height;
    gralloc::GetCustomDimensions(hnd, &stride, &height);
    err = Error::NONE;
  }
  hidl_cb(err, stride, height);
  return Void();
}

Return<void> QtiMapperExtensions::getRgbDataAddress(void *buffer, getRgbDataAddress_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  void *rgb_data = nullptr;
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (gralloc::GetRgbDataAddress(hnd, &rgb_data) == 0) {
      err = Error::NONE;
    }
  }
  hidl_cb(err, rgb_data);
  return Void();
}

Return<void> QtiMapperExtensions::calculateBufferAttributes(int32_t width, int32_t height,
                                                            int32_t format, uint64_t usage,
                                                            calculateBufferAttributes_cb hidl_cb) {
  unsigned int alignedw, alignedh;
  BufferInfo info(width, height, format, usage);
  gralloc::GetAlignedWidthAndHeight(info, &alignedw, &alignedh);
  bool ubwc_enabled = gralloc::IsUBwcEnabled(format, usage);
  hidl_cb(Error::NONE, alignedw, alignedh, ubwc_enabled);
  return Void();
}

Return<void> QtiMapperExtensions::getCustomFormatFlags(int32_t format, uint64_t usage,
                                                       getCustomFormatFlags_cb hidl_cb) {
  uint64_t priv_flags = 0;
  auto err = Error::NONE;
  int32_t custom_format = format;
  if (gralloc::GetCustomFormatFlags(format, usage, &custom_format, &priv_flags) != 0) {
    err = Error::UNSUPPORTED;
  }
  hidl_cb(err, custom_format, priv_flags);
  return Void();
}

Return<void> QtiMapperExtensions::getColorSpace(void *buffer, getColorSpace_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  int color_space = 0;
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    gralloc::GetColorSpaceFromMetadata(hnd, &color_space);
    err = Error::NONE;
  }
  hidl_cb(err, color_space);
  return Void();
}

Return<void> QtiMapperExtensions::getYuvPlaneInfo(void *buffer, getYuvPlaneInfo_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  hidl_vec<YCbCrLayout> layout;
  layout.resize(2);
  android_ycbcr yuv_plane_info[2];
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (gralloc::GetYUVPlaneInfo(hnd, yuv_plane_info) == 0) {
      err = Error::NONE;
      for (int i = 0; i < 2; i++) {
        layout[i].y = yuv_plane_info[i].y;
        layout[i].cr = yuv_plane_info[i].cr;
        layout[i].cb = yuv_plane_info[i].cb;
        layout[i].yStride = static_cast<uint32_t>(yuv_plane_info[i].ystride);
        layout[i].cStride = static_cast<uint32_t>(yuv_plane_info[i].cstride);
        layout[i].chromaStep = static_cast<uint32_t>(yuv_plane_info[i].chroma_step);
      }
    }
  }
  hidl_cb(err, layout);
  return Void();
}

Return<Error> QtiMapperExtensions::setSingleBufferMode(void *buffer, bool enable) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (setMetaData(hnd, SET_SINGLE_BUFFER_MODE, &enable) != 0) {
      err = Error::UNSUPPORTED;
    } else {
      err = Error::NONE;
    }
  }
  return err;
}

Return<void> QtiMapperExtensions::getFd(void *buffer, getFd_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int fd = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    fd = hnd->fd;
  }
  hidl_cb(err, fd);
  return Void();
}

Return<void> QtiMapperExtensions::getWidth(void *buffer, getWidth_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int width = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    width = hnd->width;
  }
  hidl_cb(err, width);
  return Void();
}

Return<void> QtiMapperExtensions::getHeight(void *buffer, getHeight_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int height = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    height = hnd->height;
  }
  hidl_cb(err, height);
  return Void();
}

Return<void> QtiMapperExtensions::getFormat(void *buffer, getFormat_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int format = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    format = hnd->format;
  }
  hidl_cb(err, format);
  return Void();
}

Return<void> QtiMapperExtensions::getPrivateFlags(void *buffer, getPrivateFlags_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int flags = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    flags = hnd->flags;
  }
  hidl_cb(err, flags);
  return Void();
}

Return<void> QtiMapperExtensions::getUnalignedWidth(void *buffer, getUnalignedWidth_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int unaligned_width = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    unaligned_width = hnd->unaligned_width;
  }
  hidl_cb(err, unaligned_width);
  return Void();
}

Return<void> QtiMapperExtensions::getUnalignedHeight(void *buffer, getUnalignedHeight_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  int unaligned_height = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    unaligned_height = hnd->unaligned_height;
  }
  hidl_cb(err, unaligned_height);
  return Void();
}

Return<void> QtiMapperExtensions::getLayerCount(void *buffer, getLayerCount_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  unsigned int layer_count = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    layer_count = hnd->layer_count;
  }
  hidl_cb(err, layer_count);
  return Void();
}

Return<void> QtiMapperExtensions::getId(void *buffer, getId_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  uint64_t id = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    id = hnd->id;
  }
  hidl_cb(err, id);
  return Void();
}

Return<void> QtiMapperExtensions::getUsageFlags(void *buffer, getUsageFlags_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  uint64_t usage = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    usage = hnd->usage;
  }
  hidl_cb(err, usage);
  return Void();
}

Return<void> QtiMapperExtensions::getSize(void *buffer, getSize_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  unsigned int size = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    size = hnd->size;
  }
  hidl_cb(err, size);
  return Void();
}

Return<void> QtiMapperExtensions::getOffset(void *buffer, getOffset_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  unsigned int offset = 0;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    err = Error::NONE;
    offset = hnd->offset;
  }
  hidl_cb(err, offset);
  return Void();
}

Return<void> QtiMapperExtensions::getSurfaceMetadata(void *buffer, getSurfaceMetadata_cb hidl_cb) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  GraphicsMetadata surface_metadata;
  if (buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (getMetaData(hnd, GET_GRAPHICS_METADATA, &surface_metadata) == 0) {
      err = Error::NONE;
    }
  }
  if (err != Error::NONE) {
    hidl_cb(err, nullptr);
  } else {
    hidl_cb(err, &surface_metadata);
  }
  return Void();
}

// It will return size for single layer only i.e. layer count is always 1.
Return<void> QtiMapperExtensions::getFormatLayout(int32_t format, uint64_t usage, int32_t flags,
                                                  int32_t width, int32_t height,
                                                  getFormatLayout_cb hidl_cb) {
  ALOGD_IF(DEBUG, "%s: Input parameters - wxh: %dx%d usage: 0x%" PRIu64 " format: %d", __FUNCTION__,
           width, height, usage, format);
  auto err = Error::NONE;
  hidl_vec<PlaneLayout> plane_info;
  unsigned int alignedw = 0, alignedh = 0;
  int plane_count = 0;
  uint32_t size = 0;
  int custom_format = gralloc::GetImplDefinedFormat(usage, format);
  BufferInfo info(width, height, custom_format, usage);
  int ret = gralloc::GetBufferSizeAndDimensions(info, &size, &alignedw, &alignedh);
  if (ret) {
    err = Error::BAD_BUFFER;
    hidl_cb(err, size, plane_info);
    return Void();
  }
  gralloc::PlaneLayoutInfo plane_layout[8] = {};
  ALOGD_IF(DEBUG, "%s: Aligned width and height - wxh: %ux%u custom_format = %d", __FUNCTION__,
           alignedw, alignedh, custom_format);
  if (gralloc::IsYuvFormat(custom_format)) {
    gralloc::GetYUVPlaneInfo(info, custom_format, alignedw, alignedh, flags, &plane_count,
                             plane_layout);
  } else if (gralloc::IsUncompressedRGBFormat(custom_format) ||
             gralloc::IsCompressedRGBFormat(custom_format)) {
    gralloc::GetRGBPlaneInfo(info, custom_format, alignedw, alignedh, flags, &plane_count,
                             plane_layout);
  } else {
    err = Error::BAD_BUFFER;
    hidl_cb(err, size, plane_info);
    return Void();
  }
  ALOGD_IF(DEBUG, "%s: Number of plane - %d, custom_format - %d", __FUNCTION__, plane_count,
           custom_format);
  plane_info.resize(plane_count);
  for (int i = 0; i < plane_count; i++) {
    plane_info[i].component = plane_layout[i].component;
    plane_info[i].h_subsampling = plane_layout[i].h_subsampling;
    plane_info[i].v_subsampling = plane_layout[i].v_subsampling;
    plane_info[i].offset = plane_layout[i].offset;
    plane_info[i].pixel_increment = plane_layout[i].step;
    plane_info[i].stride = plane_layout[i].stride;
    plane_info[i].stride_bytes = plane_layout[i].stride_bytes;
    plane_info[i].scanlines = plane_layout[i].scanlines;
    plane_info[i].size = plane_layout[i].size;
    ALOGD_IF(DEBUG, "%s: plane info: component - %d", __FUNCTION__, plane_info[i].component);
    ALOGD_IF(DEBUG, "h_subsampling - %u, v_subsampling - %u, offset - %u, pixel_increment - %d",
             plane_info[i].h_subsampling, plane_info[i].v_subsampling, plane_info[i].offset,
             plane_info[i].pixel_increment);
    ALOGD_IF(DEBUG, "stride_pixel - %d, stride_bytes - %d, scanlines - %d, size - %u",
             plane_info[i].stride, plane_info[i].stride_bytes, plane_info[i].scanlines,
             plane_info[i].size);
  }
  hidl_cb(err, size, plane_info);
  return Void();
}

Return<Error> QtiMapperExtensions::getSurfaceMetadata_V1(void *buffer, void *metadata) {
  auto err = Error::BAD_BUFFER;
  auto hnd = static_cast<private_handle_t *>(buffer);
  if (metadata != nullptr && buffer != nullptr && private_handle_t::validate(hnd) == 0) {
    if (getMetaData(hnd, GET_GRAPHICS_METADATA, metadata) == 0) {
      err = Error::NONE;
    } else {
      err = Error::UNSUPPORTED;
    }
  } else {
    ALOGE("%s: buffer pointer: %p, metadata pointer: %p ", __FUNCTION__, buffer, metadata);
  }
  return err;
}

}  // namespace implementation
}  // namespace V1_1
}  // namespace mapperextensions
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
