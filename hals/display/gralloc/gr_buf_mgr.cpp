/*
 * Copyright (c) 2011-2018, 2020 The Linux Foundation. All rights reserved.
 * Not a Contribution
 *
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define DEBUG 0

#include "gr_buf_mgr.h"

#include <QtiGralloc.h>
#include <QtiGrallocPriv.h>
#include <gralloctypes/Gralloc4.h>
#include <sys/mman.h>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <fstream>
#include "gr_adreno_info.h"
#include "gr_buf_descriptor.h"
#include "gr_priv_handle.h"
#include "gr_utils.h"
#include "qdMetaData.h"
#include "qd_utils.h"

namespace gralloc {

using aidl::android::hardware::graphics::common::BlendMode;
using aidl::android::hardware::graphics::common::Cta861_3;
using aidl::android::hardware::graphics::common::Dataspace;
using aidl::android::hardware::graphics::common::PlaneLayout;
using aidl::android::hardware::graphics::common::PlaneLayoutComponent;
using aidl::android::hardware::graphics::common::Rect;
using aidl::android::hardware::graphics::common::Smpte2086;
using aidl::android::hardware::graphics::common::StandardMetadataType;
using aidl::android::hardware::graphics::common::XyColor;
using ::android::hardware::graphics::common::V1_2::PixelFormat;

static BufferInfo GetBufferInfo(const BufferDescriptor &descriptor) {
  return BufferInfo(descriptor.GetWidth(), descriptor.GetHeight(), descriptor.GetFormat(),
                    descriptor.GetUsage());
}

static uint64_t getMetaDataSize(uint64_t reserved_region_size) {
// Only include the reserved region size when using Metadata_t V2
#ifndef METADATA_V2
  reserved_region_size = 0;
#endif
  return static_cast<uint64_t>(ROUND_UP_PAGESIZE(sizeof(MetaData_t) + reserved_region_size));
}

static void unmapAndReset(private_handle_t *handle, uint64_t reserved_region_size = 0) {
  if (private_handle_t::validate(handle) == 0 && handle->base_metadata) {
    munmap(reinterpret_cast<void *>(handle->base_metadata), getMetaDataSize(reserved_region_size));
    handle->base_metadata = 0;
  }
}

static int validateAndMap(private_handle_t *handle, uint64_t reserved_region_size = 0) {
  if (private_handle_t::validate(handle)) {
    ALOGE("%s: Private handle is invalid - handle:%p", __func__, handle);
    return -1;
  }
  if (handle->fd_metadata < 0) {
    // Silently return, metadata cannot be used
    return -1;
  }

  if (!handle->base_metadata) {
    uint64_t size = getMetaDataSize(reserved_region_size);
    void *base = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, handle->fd_metadata, 0);
    if (base == reinterpret_cast<void *>(MAP_FAILED)) {
      ALOGE("%s: metadata mmap failed - handle:%p fd: %d err: %s", __func__, handle,
            handle->fd_metadata, strerror(errno));
      return -1;
    }
    handle->base_metadata = (uintptr_t)base;
#ifdef METADATA_V2
    // The allocator process gets the reserved region size from the BufferDescriptor.
    // When importing to another process, the reserved size is unknown until mapping the metadata,
    // hence the re-mapping below
    auto metadata = reinterpret_cast<MetaData_t *>(handle->base_metadata);
    if (reserved_region_size == 0 && metadata->reservedSize) {
      size = getMetaDataSize(metadata->reservedSize);
      unmapAndReset(handle);
      void *new_base = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, handle->fd_metadata, 0);
      if (new_base == reinterpret_cast<void *>(MAP_FAILED)) {
        ALOGE("%s: metadata mmap failed - handle:%p fd: %d err: %s", __func__, handle,
              handle->fd_metadata, strerror(errno));
        return -1;
      }
      handle->base_metadata = (uintptr_t)new_base;
    }
#endif
  }
  return 0;
}

static Error dataspaceToColorMetadata(Dataspace dataspace, ColorMetaData *color_metadata) {
  ColorMetaData out;
  uint32_t primaries = (uint32_t)dataspace & (uint32_t)Dataspace::STANDARD_MASK;
  uint32_t transfer = (uint32_t)dataspace & (uint32_t)Dataspace::TRANSFER_MASK;
  uint32_t range = (uint32_t)dataspace & (uint32_t)Dataspace::RANGE_MASK;

  switch (primaries) {
    case (uint32_t)Dataspace::STANDARD_BT709:
      out.colorPrimaries = ColorPrimaries_BT709_5;
      break;
    // TODO(tbalacha): verify this is equivalent
    case (uint32_t)Dataspace::STANDARD_BT470M:
      out.colorPrimaries = ColorPrimaries_BT470_6M;
      break;
    case (uint32_t)Dataspace::STANDARD_BT601_625:
    case (uint32_t)Dataspace::STANDARD_BT601_625_UNADJUSTED:
      out.colorPrimaries = ColorPrimaries_BT601_6_625;
      break;
    case (uint32_t)Dataspace::STANDARD_BT601_525:
    case (uint32_t)Dataspace::STANDARD_BT601_525_UNADJUSTED:
      out.colorPrimaries = ColorPrimaries_BT601_6_525;
      break;
    case (uint32_t)Dataspace::STANDARD_FILM:
      out.colorPrimaries = ColorPrimaries_GenericFilm;
      break;
    case (uint32_t)Dataspace::STANDARD_BT2020:
      out.colorPrimaries = ColorPrimaries_BT2020;
      break;
    case (uint32_t)Dataspace::STANDARD_ADOBE_RGB:
      out.colorPrimaries = ColorPrimaries_AdobeRGB;
      break;
    case (uint32_t)Dataspace::STANDARD_DCI_P3:
      out.colorPrimaries = ColorPrimaries_DCIP3;
      break;
    default:
      return Error::UNSUPPORTED;
      /*
       ColorPrimaries_SMPTE_240M;
       ColorPrimaries_SMPTE_ST428;
       ColorPrimaries_EBU3213;
      */
  }

  switch (transfer) {
    case (uint32_t)Dataspace::TRANSFER_SRGB:
      out.transfer = Transfer_sRGB;
      break;
    case (uint32_t)Dataspace::TRANSFER_GAMMA2_2:
      out.transfer = Transfer_Gamma2_2;
      break;
    case (uint32_t)Dataspace::TRANSFER_GAMMA2_8:
      out.transfer = Transfer_Gamma2_8;
      break;
    case (uint32_t)Dataspace::TRANSFER_SMPTE_170M:
      out.transfer = Transfer_SMPTE_170M;
      break;
    case (uint32_t)Dataspace::TRANSFER_LINEAR:
      out.transfer = Transfer_Linear;
      break;
    case (uint32_t)Dataspace::TRANSFER_HLG:
      out.transfer = Transfer_HLG;
      break;
    default:
      return Error::UNSUPPORTED;
      /*
      Transfer_SMPTE_240M
      Transfer_Log
      Transfer_Log_Sqrt
      Transfer_XvYCC
      Transfer_BT1361
      Transfer_sYCC
      Transfer_BT2020_2_1
      Transfer_BT2020_2_2
      Transfer_SMPTE_ST2084
      Transfer_ST_428
      */
  }

  switch (range) {
    case (uint32_t)Dataspace::RANGE_FULL:
      out.range = Range_Full;
      break;
    case (uint32_t)Dataspace::RANGE_LIMITED:
      out.range = Range_Limited;
      break;
    case (uint32_t)Dataspace::RANGE_EXTENDED:
      out.range = Range_Extended;
      break;
    default:
      return Error::UNSUPPORTED;
  }

  color_metadata->colorPrimaries = out.colorPrimaries;
  color_metadata->transfer = out.transfer;
  color_metadata->range = out.range;
  return Error::NONE;
}
static Error colorMetadataToDataspace(ColorMetaData color_metadata, Dataspace *dataspace) {
  Dataspace primaries, transfer, range = Dataspace::UNKNOWN;

  switch (color_metadata.colorPrimaries) {
    case ColorPrimaries_BT709_5:
      primaries = Dataspace::STANDARD_BT709;
      break;
    // TODO(tbalacha): verify this is equivalent
    case ColorPrimaries_BT470_6M:
      primaries = Dataspace::STANDARD_BT470M;
      break;
    case ColorPrimaries_BT601_6_625:
      primaries = Dataspace::STANDARD_BT601_625;
      break;
    case ColorPrimaries_BT601_6_525:
      primaries = Dataspace::STANDARD_BT601_525;
      break;
    case ColorPrimaries_GenericFilm:
      primaries = Dataspace::STANDARD_FILM;
      break;
    case ColorPrimaries_BT2020:
      primaries = Dataspace::STANDARD_BT2020;
      break;
    case ColorPrimaries_AdobeRGB:
      primaries = Dataspace::STANDARD_ADOBE_RGB;
      break;
    case ColorPrimaries_DCIP3:
      primaries = Dataspace::STANDARD_DCI_P3;
      break;
    default:
      return Error::UNSUPPORTED;
      /*
       ColorPrimaries_SMPTE_240M;
       ColorPrimaries_SMPTE_ST428;
       ColorPrimaries_EBU3213;
      */
  }

  switch (color_metadata.transfer) {
    case Transfer_sRGB:
      transfer = Dataspace::TRANSFER_SRGB;
      break;
    case Transfer_Gamma2_2:
      transfer = Dataspace::TRANSFER_GAMMA2_2;
      break;
    case Transfer_Gamma2_8:
      transfer = Dataspace::TRANSFER_GAMMA2_8;
      break;
    case Transfer_SMPTE_170M:
      transfer = Dataspace::TRANSFER_SMPTE_170M;
      break;
    case Transfer_Linear:
      transfer = Dataspace::TRANSFER_LINEAR;
      break;
    case Transfer_HLG:
      transfer = Dataspace::TRANSFER_HLG;
      break;
    default:
      return Error::UNSUPPORTED;
      /*
      Transfer_SMPTE_240M
      Transfer_Log
      Transfer_Log_Sqrt
      Transfer_XvYCC
      Transfer_BT1361
      Transfer_sYCC
      Transfer_BT2020_2_1
      Transfer_BT2020_2_2
      Transfer_SMPTE_ST2084
      Transfer_ST_428
      */
  }

  switch (color_metadata.range) {
    case Range_Full:
      range = Dataspace::RANGE_FULL;
      break;
    case Range_Limited:
      range = Dataspace::RANGE_LIMITED;
      break;
    case Range_Extended:
      range = Dataspace::RANGE_EXTENDED;
      break;
    default:
      return Error::UNSUPPORTED;
  }

  *dataspace = (Dataspace)((uint32_t)primaries | (uint32_t)transfer | (uint32_t)range);
  return Error::NONE;
}

static Error getComponentSizeAndOffset(int32_t format, PlaneLayoutComponent &comp) {
  switch (format) {
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGBA_8888):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGBX_8888):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGB_888):
      comp.sizeInBits = 8;
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.offsetInBits = 8;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.offsetInBits = 16;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value &&
                 format != HAL_PIXEL_FORMAT_RGB_888) {
        comp.offsetInBits = 24;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGB_565):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.offsetInBits = 0;
        comp.sizeInBits = 5;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.offsetInBits = 5;
        comp.sizeInBits = 6;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.offsetInBits = 11;
        comp.sizeInBits = 5;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_BGR_565):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.offsetInBits = 11;
        comp.sizeInBits = 5;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.offsetInBits = 5;
        comp.sizeInBits = 6;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.offsetInBits = 0;
        comp.sizeInBits = 5;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_BGRA_8888):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_BGRX_8888):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_BGR_888):
      comp.sizeInBits = 8;
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.offsetInBits = 16;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.offsetInBits = 8;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value &&
                 format != HAL_PIXEL_FORMAT_BGR_888) {
        comp.offsetInBits = 24;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGBA_5551):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.sizeInBits = 5;
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.sizeInBits = 5;
        comp.offsetInBits = 5;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.sizeInBits = 5;
        comp.offsetInBits = 10;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value) {
        comp.sizeInBits = 1;
        comp.offsetInBits = 15;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGBA_4444):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.sizeInBits = 4;
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.sizeInBits = 4;
        comp.offsetInBits = 4;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.sizeInBits = 4;
        comp.offsetInBits = 8;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value) {
        comp.sizeInBits = 4;
        comp.offsetInBits = 12;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_R_8):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RG_88):
      comp.sizeInBits = 8;
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value &&
                 format != HAL_PIXEL_FORMAT_R_8) {
        comp.offsetInBits = 8;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGBA_1010102):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGBX_1010102):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 10;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 20;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value) {
        comp.sizeInBits = 2;
        comp.offsetInBits = 30;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_ARGB_2101010):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_XRGB_2101010):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 2;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 12;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 22;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value) {
        comp.sizeInBits = 2;
        comp.offsetInBits = 0;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_BGRA_1010102):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_BGRX_1010102):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 20;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 10;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value) {
        comp.sizeInBits = 2;
        comp.offsetInBits = 30;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_ABGR_2101010):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_XBGR_2101010):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 22;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 12;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.sizeInBits = 10;
        comp.offsetInBits = 2;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value) {
        comp.sizeInBits = 2;
        comp.offsetInBits = 0;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RGBA_FP16):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_R.value) {
        comp.sizeInBits = 16;
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_G.value) {
        comp.sizeInBits = 16;
        comp.offsetInBits = 16;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_B.value) {
        comp.sizeInBits = 16;
        comp.offsetInBits = 32;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_A.value) {
        comp.sizeInBits = 16;
        comp.offsetInBits = 48;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YCbCr_420_SP):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YCbCr_422_SP):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_NV12_ENCODEABLE):
      comp.sizeInBits = 8;
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_Y.value ||
          comp.type.value == android::gralloc4::PlaneLayoutComponentType_CB.value) {
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_CR.value) {
        comp.offsetInBits = 8;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YCrCb_420_SP):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YCrCb_422_SP):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YCrCb_420_SP_ADRENO):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YCrCb_420_SP_VENUS):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_NV21_ZSL):
      comp.sizeInBits = 8;
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_Y.value ||
          comp.type.value == android::gralloc4::PlaneLayoutComponentType_CR.value) {
        comp.offsetInBits = 0;
      } else if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_CB.value) {
        comp.offsetInBits = 8;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_Y16):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_Y.value) {
        comp.offsetInBits = 0;
        comp.sizeInBits = 16;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YV12):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_Y.value ||
          comp.type.value == android::gralloc4::PlaneLayoutComponentType_CB.value ||
          comp.type.value == android::gralloc4::PlaneLayoutComponentType_CR.value) {
        comp.offsetInBits = 0;
        comp.sizeInBits = 8;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_Y8):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_Y.value) {
        comp.offsetInBits = 0;
        comp.sizeInBits = 8;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_YCbCr_420_P010):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_Y.value ||
          comp.type.value == android::gralloc4::PlaneLayoutComponentType_CB.value ||
          comp.type.value == android::gralloc4::PlaneLayoutComponentType_CR.value) {
        comp.offsetInBits = 0;
        comp.sizeInBits = 10;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RAW16):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_RAW.value) {
        comp.offsetInBits = 0;
        comp.sizeInBits = 16;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RAW12):
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RAW10):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_RAW.value) {
        comp.offsetInBits = 0;
        comp.sizeInBits = -1;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    case static_cast<int32_t>(HAL_PIXEL_FORMAT_RAW8):
      if (comp.type.value == android::gralloc4::PlaneLayoutComponentType_RAW.value) {
        comp.offsetInBits = 0;
        comp.sizeInBits = 8;
      } else {
        return Error::BAD_VALUE;
      }
      break;
    default:
      ALOGI_IF(DEBUG, "Offset and size in bits unknown for format %d", format);
      return Error::UNSUPPORTED;
  }
  return Error::NONE;
}

static void grallocToStandardPlaneLayoutComponentType(uint32_t in,
                                                      std::vector<PlaneLayoutComponent> *components,
                                                      int32_t format) {
  PlaneLayoutComponent comp;
  comp.offsetInBits = -1;
  comp.sizeInBits = -1;

  if (in & PLANE_COMPONENT_Y) {
    comp.type = android::gralloc4::PlaneLayoutComponentType_Y;
    if (getComponentSizeAndOffset(format, comp) == Error::NONE)
      components->push_back(comp);
  }

  if (in & PLANE_COMPONENT_Cb) {
    comp.type = android::gralloc4::PlaneLayoutComponentType_CB;
    if (getComponentSizeAndOffset(format, comp) == Error::NONE)
      components->push_back(comp);
  }

  if (in & PLANE_COMPONENT_Cr) {
    comp.type = android::gralloc4::PlaneLayoutComponentType_CR;
    if (getComponentSizeAndOffset(format, comp) == Error::NONE)
      components->push_back(comp);
  }

  if (in & PLANE_COMPONENT_R) {
    comp.type = android::gralloc4::PlaneLayoutComponentType_R;
    if (getComponentSizeAndOffset(format, comp) == Error::NONE)
      components->push_back(comp);
  }

  if (in & PLANE_COMPONENT_G) {
    comp.type = android::gralloc4::PlaneLayoutComponentType_G;
    if (getComponentSizeAndOffset(format, comp) == Error::NONE)
      components->push_back(comp);
  }

  if (in & PLANE_COMPONENT_B) {
    comp.type = android::gralloc4::PlaneLayoutComponentType_B;
    if (getComponentSizeAndOffset(format, comp) == Error::NONE)
      components->push_back(comp);
  }

  if (in & PLANE_COMPONENT_A) {
    comp.type = android::gralloc4::PlaneLayoutComponentType_A;
    if (getComponentSizeAndOffset(format, comp) == Error::NONE)
      components->push_back(comp);
  }

  if (in & PLANE_COMPONENT_RAW) {
    comp.type = android::gralloc4::PlaneLayoutComponentType_RAW;
    if (getComponentSizeAndOffset(format, comp) == Error::NONE)
      components->push_back(comp);
  }

  if (in & PLANE_COMPONENT_META) {
    comp.type = qtigralloc::PlaneLayoutComponentType_Meta;
    components->push_back(comp);
  }
}

static Error getFormatLayout(private_handle_t *handle, std::vector<PlaneLayout> *out) {
  std::vector<PlaneLayout> plane_info;
  int plane_count = 0;
  BufferInfo info(handle->unaligned_width, handle->unaligned_height, handle->format, handle->usage);

  gralloc::PlaneLayoutInfo plane_layout[8] = {};
  if (gralloc::IsYuvFormat(handle->format)) {
    gralloc::GetYUVPlaneInfo(info, handle->format, handle->width, handle->height, handle->flags,
                             &plane_count, plane_layout);
  } else if (gralloc::IsUncompressedRGBFormat(handle->format) ||
             gralloc::IsCompressedRGBFormat(handle->format)) {
    gralloc::GetRGBPlaneInfo(info, handle->format, handle->width, handle->height, handle->flags,
                             &plane_count, plane_layout);
  } else {
    return Error::BAD_BUFFER;
  }
  plane_info.resize(plane_count);
  for (int i = 0; i < plane_count; i++) {
    std::vector<PlaneLayoutComponent> components;
    grallocToStandardPlaneLayoutComponentType(plane_layout[i].component, &plane_info[i].components,
                                              handle->format);
    plane_info[i].horizontalSubsampling = (1ull << plane_layout[i].h_subsampling);
    plane_info[i].verticalSubsampling = (1ull << plane_layout[i].v_subsampling);
    plane_info[i].offsetInBytes = static_cast<int64_t>(plane_layout[i].offset);
    plane_info[i].sampleIncrementInBits = static_cast<int64_t>(plane_layout[i].step * 8);
    plane_info[i].strideInBytes = static_cast<int64_t>(plane_layout[i].stride_bytes);
    plane_info[i].totalSizeInBytes = static_cast<int64_t>(plane_layout[i].size);
    plane_info[i].widthInSamples = handle->unaligned_width >> plane_layout[i].h_subsampling;
    plane_info[i].heightInSamples = handle->unaligned_height >> plane_layout[i].v_subsampling;
  }
  *out = plane_info;
  return Error::NONE;
}

BufferManager::BufferManager() : next_id_(0) {
  handles_map_.clear();
  allocator_ = new Allocator();
  allocator_->Init();
}

BufferManager *BufferManager::GetInstance() {
  static BufferManager *instance = new BufferManager();
  return instance;
}

BufferManager::~BufferManager() {
  if (allocator_) {
    delete allocator_;
  }
}

void BufferManager::SetGrallocDebugProperties(gralloc::GrallocProperties props) {
  allocator_->SetProperties(props);
  AdrenoMemInfo::GetInstance()->AdrenoSetProperties(props);
}

Error BufferManager::FreeBuffer(std::shared_ptr<Buffer> buf) {
  auto hnd = buf->handle;
  ALOGD_IF(DEBUG, "FreeBuffer handle:%p", hnd);

  if (private_handle_t::validate(hnd) != 0) {
    ALOGE("FreeBuffer: Invalid handle: %p", hnd);
    return Error::BAD_BUFFER;
  }

  auto meta_size = getMetaDataSize(buf->reserved_size);

  if (allocator_->FreeBuffer(reinterpret_cast<void *>(hnd->base), hnd->size, hnd->offset, hnd->fd,
                             buf->ion_handle_main) != 0) {
    return Error::BAD_BUFFER;
  }

  if (allocator_->FreeBuffer(reinterpret_cast<void *>(hnd->base_metadata), meta_size,
                             hnd->offset_metadata, hnd->fd_metadata, buf->ion_handle_meta) != 0) {
    return Error::BAD_BUFFER;
  }

  private_handle_t *handle = const_cast<private_handle_t *>(hnd);
  handle->fd = -1;
  handle->fd_metadata = -1;
  if (!(handle->flags & private_handle_t::PRIV_FLAGS_CLIENT_ALLOCATED)) {
    delete handle;
  }
  return Error::NONE;
}

Error BufferManager::ValidateBufferSize(private_handle_t const *hnd, BufferInfo info) {
  unsigned int size, alignedw, alignedh;
  info.format = GetImplDefinedFormat(info.usage, info.format);
  int ret = GetBufferSizeAndDimensions(info, &size, &alignedw, &alignedh);
  if (ret < 0) {
    return Error::BAD_BUFFER;
  }
  auto ion_fd_size = static_cast<unsigned int>(lseek(hnd->fd, 0, SEEK_END));
  if (size != ion_fd_size) {
    return Error::BAD_VALUE;
  }
  return Error::NONE;
}

void BufferManager::RegisterHandleLocked(const private_handle_t *hnd, int ion_handle,
                                         int ion_handle_meta) {
  auto buffer = std::make_shared<Buffer>(hnd, ion_handle, ion_handle_meta);

  if (hnd->base_metadata) {
    auto metadata = reinterpret_cast<MetaData_t *>(hnd->base_metadata);
#ifdef METADATA_V2
    buffer->reserved_size = metadata->reservedSize;
    if (buffer->reserved_size > 0) {
      buffer->reserved_region_ptr =
          reinterpret_cast<void *>(hnd->base_metadata + sizeof(MetaData_t));
    } else {
      buffer->reserved_region_ptr = nullptr;
    }
#else
    buffer->reserved_region_ptr = reinterpret_cast<void *>(&(metadata->reservedRegion.data));
    buffer->reserved_size = metadata->reservedRegion.size;
#endif
  }

  handles_map_.emplace(std::make_pair(hnd, buffer));
}

Error BufferManager::ImportHandleLocked(private_handle_t *hnd) {
  if (private_handle_t::validate(hnd) != 0) {
    ALOGE("ImportHandleLocked: Invalid handle: %p", hnd);
    return Error::BAD_BUFFER;
  }
  ALOGD_IF(DEBUG, "Importing handle:%p id: %" PRIu64, hnd, hnd->id);
  int ion_handle = allocator_->ImportBuffer(hnd->fd);
  if (ion_handle < 0) {
    ALOGE("Failed to import ion buffer: hnd: %p, fd:%d, id:%" PRIu64, hnd, hnd->fd, hnd->id);
    return Error::BAD_BUFFER;
  }
  int ion_handle_meta = allocator_->ImportBuffer(hnd->fd_metadata);
  if (ion_handle_meta < 0) {
    ALOGE("Failed to import ion metadata buffer: hnd: %p, fd:%d, id:%" PRIu64, hnd, hnd->fd,
          hnd->id);
    return Error::BAD_BUFFER;
  }
  // Initialize members that aren't transported
  hnd->size = static_cast<unsigned int>(lseek(hnd->fd, 0, SEEK_END));
  hnd->offset = 0;
  hnd->offset_metadata = 0;
  hnd->base = 0;
  hnd->base_metadata = 0;
  hnd->gpuaddr = 0;

  if (validateAndMap(hnd)) {
    ALOGE("Failed to map metadata: hnd: %p, fd:%d, id:%" PRIu64, hnd, hnd->fd, hnd->id);
    return Error::BAD_BUFFER;
  }

  RegisterHandleLocked(hnd, ion_handle, ion_handle_meta);
  allocated_ += hnd->size;
  if (allocated_ >=  kAllocThreshold) {
    kAllocThreshold += kMemoryOffset;
    BuffersDump();
  }
  return Error::NONE;
}

std::shared_ptr<BufferManager::Buffer> BufferManager::GetBufferFromHandleLocked(
    const private_handle_t *hnd) {
  auto it = handles_map_.find(hnd);
  if (it != handles_map_.end()) {
    return it->second;
  } else {
    return nullptr;
  }
}

Error BufferManager::MapBuffer(private_handle_t const *handle) {
  private_handle_t *hnd = const_cast<private_handle_t *>(handle);
  ALOGD_IF(DEBUG, "Map buffer handle:%p id: %" PRIu64, hnd, hnd->id);

  hnd->base = 0;
  if (allocator_->MapBuffer(reinterpret_cast<void **>(&hnd->base), hnd->size, hnd->offset,
                            hnd->fd) != 0) {
    return Error::BAD_BUFFER;
  }
  return Error::NONE;
}

Error BufferManager::IsBufferImported(const private_handle_t *hnd) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  auto buf = GetBufferFromHandleLocked(hnd);
  if (buf != nullptr) {
    return Error::NONE;
  }
  return Error::BAD_BUFFER;
}

Error BufferManager::RetainBuffer(private_handle_t const *hnd) {
  ALOGD_IF(DEBUG, "Retain buffer handle:%p id: %" PRIu64, hnd, hnd->id);
  auto err = Error::NONE;
  std::lock_guard<std::mutex> lock(buffer_lock_);
  auto buf = GetBufferFromHandleLocked(hnd);
  if (buf != nullptr) {
    buf->IncRef();
  } else {
    private_handle_t *handle = const_cast<private_handle_t *>(hnd);
    err = ImportHandleLocked(handle);
  }
  return err;
}

Error BufferManager::ReleaseBuffer(private_handle_t const *hnd) {
  ALOGD_IF(DEBUG, "Release buffer handle:%p", hnd);
  std::lock_guard<std::mutex> lock(buffer_lock_);
  auto buf = GetBufferFromHandleLocked(hnd);
  if (buf == nullptr) {
    ALOGE("Could not find handle: %p id: %" PRIu64, hnd, hnd->id);
    return Error::BAD_BUFFER;
  } else {
    if (buf->DecRef()) {
      handles_map_.erase(hnd);
      // Unmap, close ion handle and close fd
      if (allocated_ >= hnd->size) {
        allocated_ -= hnd->size;
      }
      FreeBuffer(buf);
    }
  }
  return Error::NONE;
}

Error BufferManager::LockBuffer(const private_handle_t *hnd, uint64_t usage) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  auto err = Error::NONE;
  ALOGD_IF(DEBUG, "LockBuffer buffer handle:%p id: %" PRIu64, hnd, hnd->id);

  // If buffer is not meant for CPU return err
  if (!CpuCanAccess(usage)) {
    return Error::BAD_VALUE;
  }

  auto buf = GetBufferFromHandleLocked(hnd);
  if (buf == nullptr) {
    return Error::BAD_BUFFER;
  }

  if (hnd->base == 0) {
    // we need to map for real
    err = MapBuffer(hnd);
  }

  // Invalidate if CPU reads in software and there are non-CPU
  // writers. No need to do this for the metadata buffer as it is
  // only read/written in software.

  // todo use handle here
  if (err == Error::NONE && (hnd->flags & private_handle_t::PRIV_FLAGS_USES_ION) &&
      (hnd->flags & private_handle_t::PRIV_FLAGS_CACHED)) {
    if (allocator_->CleanBuffer(reinterpret_cast<void *>(hnd->base), hnd->size, hnd->offset,
                                buf->ion_handle_main, CACHE_INVALIDATE, hnd->fd)) {
      return Error::BAD_BUFFER;
    }
  }

  // Mark the buffer to be flushed after CPU write.
  if (err == Error::NONE && CpuCanWrite(usage)) {
    private_handle_t *handle = const_cast<private_handle_t *>(hnd);
    handle->flags |= private_handle_t::PRIV_FLAGS_NEEDS_FLUSH;
  }

  return err;
}

Error BufferManager::FlushBuffer(const private_handle_t *handle) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  auto status = Error::NONE;

  private_handle_t *hnd = const_cast<private_handle_t *>(handle);
  auto buf = GetBufferFromHandleLocked(hnd);
  if (buf == nullptr) {
    return Error::BAD_BUFFER;
  }

  if (allocator_->CleanBuffer(reinterpret_cast<void *>(hnd->base), hnd->size, hnd->offset,
                              buf->ion_handle_main, CACHE_CLEAN, hnd->fd) != 0) {
    status = Error::BAD_BUFFER;
  }

  return status;
}

Error BufferManager::RereadBuffer(const private_handle_t *handle) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  auto status = Error::NONE;

  private_handle_t *hnd = const_cast<private_handle_t *>(handle);
  auto buf = GetBufferFromHandleLocked(hnd);
  if (buf == nullptr) {
    return Error::BAD_BUFFER;
  }

  if (allocator_->CleanBuffer(reinterpret_cast<void *>(hnd->base), hnd->size, hnd->offset,
                              buf->ion_handle_main, CACHE_INVALIDATE, hnd->fd) != 0) {
    status = Error::BAD_BUFFER;
  }

  return status;
}

Error BufferManager::UnlockBuffer(const private_handle_t *handle) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  auto status = Error::NONE;

  private_handle_t *hnd = const_cast<private_handle_t *>(handle);
  auto buf = GetBufferFromHandleLocked(hnd);
  if (buf == nullptr) {
    return Error::BAD_BUFFER;
  }

  if (hnd->flags & private_handle_t::PRIV_FLAGS_NEEDS_FLUSH) {
    if (allocator_->CleanBuffer(reinterpret_cast<void *>(hnd->base), hnd->size, hnd->offset,
                                buf->ion_handle_main, CACHE_CLEAN, hnd->fd) != 0) {
      status = Error::BAD_BUFFER;
    }
    hnd->flags &= ~private_handle_t::PRIV_FLAGS_NEEDS_FLUSH;
  } else {
    if (allocator_->CleanBuffer(reinterpret_cast<void *>(hnd->base), hnd->size, hnd->offset,
                                buf->ion_handle_main, CACHE_READ_DONE, hnd->fd) != 0) {
      status = Error::BAD_BUFFER;
    }
  }

  return status;
}

Error BufferManager::AllocateBuffer(const BufferDescriptor &descriptor, buffer_handle_t *handle,
                                    unsigned int bufferSize, bool testAlloc) {
  if (!handle)
    return Error::BAD_BUFFER;
  std::lock_guard<std::mutex> buffer_lock(buffer_lock_);

  uint64_t usage = descriptor.GetUsage();
  int format = GetImplDefinedFormat(usage, descriptor.GetFormat());
  uint32_t layer_count = descriptor.GetLayerCount();

  unsigned int size;
  unsigned int alignedw, alignedh;
  int err = 0;

  int buffer_type = GetBufferType(format);
  BufferInfo info = GetBufferInfo(descriptor);
  info.format = format;
  info.layer_count = layer_count;

  GraphicsMetadata graphics_metadata = {};
  err = GetBufferSizeAndDimensions(info, &size, &alignedw, &alignedh, &graphics_metadata);
  if (err < 0) {
    return Error::BAD_DESCRIPTOR;
  }

  if (testAlloc) {
    return Error::NONE;
  }

  size = (bufferSize >= size) ? bufferSize : size;
  uint64_t flags = 0;
  auto page_size = UINT(getpagesize());
  AllocData data;
  data.align = GetDataAlignment(format, usage);
  data.size = size;
  data.handle = (uintptr_t)handle;
  data.uncached = UseUncached(format, usage);

  // Allocate buffer memory
  err = allocator_->AllocateMem(&data, usage, format);
  if (err) {
    ALOGE("gralloc failed to allocate err=%s format %d size %d WxH %dx%d usage %" PRIu64,
          strerror(-err), format, size, alignedw, alignedh, usage);
    return Error::NO_RESOURCES;
  }

  // Allocate memory for MetaData
  AllocData e_data;
  e_data.size = static_cast<unsigned int>(getMetaDataSize(descriptor.GetReservedSize()));
  e_data.handle = data.handle;
  e_data.align = page_size;

  err = allocator_->AllocateMem(&e_data, 0, 0);
  if (err) {
    ALOGE("gralloc failed to allocate metadata error=%s", strerror(-err));
    return Error::NO_RESOURCES;
  }

  flags = GetHandleFlags(format, usage);
  flags |= data.alloc_type;

  // Create handle
  private_handle_t *hnd = new private_handle_t(
      data.fd, e_data.fd, INT(flags), INT(alignedw), INT(alignedh), descriptor.GetWidth(),
      descriptor.GetHeight(), format, buffer_type, data.size, usage);

  hnd->id = ++next_id_;
  hnd->base = 0;
  hnd->base_metadata = 0;
  hnd->layer_count = layer_count;

  bool use_adreno_for_size = CanUseAdrenoForSize(buffer_type, usage);
  if (use_adreno_for_size) {
    setMetaDataAndUnmap(hnd, SET_GRAPHICS_METADATA, reinterpret_cast<void *>(&graphics_metadata));
  }

#ifdef METADATA_V2
  auto error = validateAndMap(hnd, descriptor.GetReservedSize());
#else
  auto error = validateAndMap(hnd);
#endif

  if (error != 0) {
    ALOGE("validateAndMap failed");
    return Error::BAD_BUFFER;
  }
  auto metadata = reinterpret_cast<MetaData_t *>(hnd->base_metadata);
  auto nameLength = std::min(descriptor.GetName().size(), size_t(MAX_NAME_LEN - 1));
  nameLength = descriptor.GetName().copy(metadata->name, nameLength);
  metadata->name[nameLength] = '\0';

#ifdef METADATA_V2
  metadata->reservedSize = descriptor.GetReservedSize();
#else
  metadata->reservedRegion.size =
      std::min(descriptor.GetReservedSize(), (uint64_t)RESERVED_REGION_SIZE);
#endif
  metadata->crop.top = 0;
  metadata->crop.left = 0;
  metadata->crop.right = hnd->width;
  metadata->crop.bottom = hnd->height;

  unmapAndReset(hnd, descriptor.GetReservedSize());

  *handle = hnd;

  RegisterHandleLocked(hnd, data.ion_handle, e_data.ion_handle);
  ALOGD_IF(DEBUG, "Allocated buffer handle: %p id: %" PRIu64, hnd, hnd->id);
  if (DEBUG) {
    private_handle_t::Dump(hnd);
  }
  return Error::NONE;
}

void BufferManager:: BuffersDump() {
  char timeStamp[32];
  char hms[32];
  uint64_t millis;
  struct timeval tv;
  struct tm ptm;

  gettimeofday(&tv, NULL);
  localtime_r(&tv.tv_sec, &ptm);
  strftime (hms, sizeof (hms), "%H:%M:%S", &ptm);
  millis = tv.tv_usec / 1000;
  snprintf(timeStamp, sizeof(timeStamp), "Timestamp: %s.%03" PRIu64, hms, millis);

  std::fstream fs;
  fs.open(file_dump_.kDumpFile, std::ios::app);
  if (!fs) {
    return;
  }
  fs << "============================" << std::endl;
  fs << timeStamp << std::endl;
  fs << "Total layers = " << handles_map_.size() << std::endl;
  uint64_t totalAllocationSize = 0;
  for (auto it : handles_map_) {
    auto buf = it.second;
    auto hnd = buf->handle;
    auto metadata = reinterpret_cast<MetaData_t *>(hnd->base_metadata);
    fs  << std::setw(80) << "Client:" << (metadata ? metadata->name: "No name");
    fs  << std::setw(20) << "WxH:" << std::setw(4) << hnd->width << " x "
        << std::setw(4) << hnd->height;
    fs  << std::setw(20) << "Size: " << std::setw(9) << hnd->size <<  std::endl;
    totalAllocationSize += hnd->size;
  }
  fs << "Total allocation  = " << totalAllocationSize/1024 << "KiB" << std::endl;
  file_dump_.position = fs.tellp();
  if (file_dump_.position > (20 * 1024 * 1024)) {
    file_dump_.position = 0;
  }
  fs.close();
}

Error BufferManager::Dump(std::ostringstream *os) {
  std::lock_guard<std::mutex> buffer_lock(buffer_lock_);
  for (auto it : handles_map_) {
    auto buf = it.second;
    auto hnd = buf->handle;
    *os << "handle id: " << std::setw(4) << hnd->id;
    *os << " fd: " << std::setw(3) << hnd->fd;
    *os << " fd_meta: " << std::setw(3) << hnd->fd_metadata;
    *os << " wxh: " << std::setw(4) << hnd->width << " x " << std::setw(4) << hnd->height;
    *os << " uwxuh: " << std::setw(4) << hnd->unaligned_width << " x ";
    *os << std::setw(4) << hnd->unaligned_height;
    *os << " size: " << std::setw(9) << hnd->size;
    *os << std::hex << std::setfill('0');
    *os << " priv_flags: "
        << "0x" << std::setw(8) << hnd->flags;
    *os << " usage: "
        << "0x" << std::setw(8) << hnd->usage;
    // TODO(user): get format string from qdutils
    *os << " format: "
        << "0x" << std::setw(8) << hnd->format;
    *os << std::dec << std::setfill(' ') << std::endl;
  }
  return Error::NONE;
}

// Get list of private handles in handles_map_
Error BufferManager::GetAllHandles(std::vector<const private_handle_t *> *out_handle_list) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  if (handles_map_.empty()) {
    return Error::NO_RESOURCES;
  }
  out_handle_list->reserve(handles_map_.size());
  for (auto handle : handles_map_) {
    out_handle_list->push_back(handle.first);
  }
  return Error::NONE;
}

Error BufferManager::GetReservedRegion(private_handle_t *handle, void **reserved_region,
                                       uint64_t *reserved_region_size) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  if (!handle)
    return Error::BAD_BUFFER;

  auto buf = GetBufferFromHandleLocked(handle);
  if (buf == nullptr)
    return Error::BAD_BUFFER;
  if (!handle->base_metadata) {
    return Error::BAD_BUFFER;
  }

  *reserved_region = buf->reserved_region_ptr;
  *reserved_region_size = buf->reserved_size;

  return Error::NONE;
}

Error BufferManager::GetMetadata(private_handle_t *handle, int64_t metadatatype_value,
                                 hidl_vec<uint8_t> *out) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  if (!handle)
    return Error::BAD_BUFFER;
  auto buf = GetBufferFromHandleLocked(handle);
  if (buf == nullptr)
    return Error::BAD_BUFFER;

  if (!handle->base_metadata) {
    return Error::BAD_BUFFER;
  }

  auto metadata = reinterpret_cast<MetaData_t *>(handle->base_metadata);

  Error error = Error::NONE;
  switch (metadatatype_value) {
    case (int64_t)StandardMetadataType::BUFFER_ID:
      android::gralloc4::encodeBufferId((uint64_t)handle->id, out);
      break;
    case (int64_t)StandardMetadataType::NAME: {
      std::string name(metadata->name);
      android::gralloc4::encodeName(name, out);
      break;
    }
    case (int64_t)StandardMetadataType::WIDTH:
      android::gralloc4::encodeWidth((uint64_t)handle->unaligned_width, out);
      break;
    case (int64_t)StandardMetadataType::HEIGHT:
      android::gralloc4::encodeHeight((uint64_t)handle->unaligned_height, out);
      break;
    case (int64_t)StandardMetadataType::LAYER_COUNT:
      android::gralloc4::encodeLayerCount((uint64_t)handle->layer_count, out);
      break;
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_REQUESTED:
      // TODO(tbalacha): need to return IMPLEMENTATION_DEFINED,
      // which wouldn't be known from private_handle_t
      android::gralloc4::encodePixelFormatRequested((PixelFormat)handle->format, out);
      break;
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_FOURCC: {
      uint32_t drm_format = 0;
      uint64_t drm_format_modifier = 0;
      GetDRMFormat(handle->format, handle->flags, &drm_format, &drm_format_modifier);
      android::gralloc4::encodePixelFormatFourCC(drm_format, out);
      break;
    }
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_MODIFIER: {
      uint32_t drm_format = 0;
      uint64_t drm_format_modifier = 0;
      GetDRMFormat(handle->format, handle->flags, &drm_format, &drm_format_modifier);
      android::gralloc4::encodePixelFormatModifier(drm_format_modifier, out);
      break;
    }
    case (int64_t)StandardMetadataType::USAGE:
      android::gralloc4::encodeUsage((uint64_t)handle->usage, out);
      break;
    case (int64_t)StandardMetadataType::ALLOCATION_SIZE:
      android::gralloc4::encodeAllocationSize((uint64_t)handle->size, out);
      break;
    case (int64_t)StandardMetadataType::PROTECTED_CONTENT: {
      uint64_t protected_content = (handle->flags & qtigralloc::PRIV_FLAGS_SECURE_BUFFER) ? 1 : 0;
      android::gralloc4::encodeProtectedContent(protected_content, out);
      break;
    }
    case (int64_t)StandardMetadataType::CHROMA_SITING:
      android::gralloc4::encodeChromaSiting(android::gralloc4::ChromaSiting_None, out);
      break;
    case (int64_t)StandardMetadataType::DATASPACE:
#ifdef METADATA_V2
      if (metadata->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(metadatatype_value)]) {
#endif
        Dataspace dataspace;
        colorMetadataToDataspace(metadata->color, &dataspace);
        android::gralloc4::encodeDataspace(dataspace, out);
#ifdef METADATA_V2
      } else {
        android::gralloc4::encodeDataspace(Dataspace::UNKNOWN, out);
      }
#endif
      break;
    case (int64_t)StandardMetadataType::INTERLACED:
      if (metadata->interlaced > 0) {
        android::gralloc4::encodeInterlaced(qtigralloc::Interlaced_Qti, out);
      } else {
        android::gralloc4::encodeInterlaced(android::gralloc4::Interlaced_None, out);
      }
      break;
    case (int64_t)StandardMetadataType::COMPRESSION:
      if (handle->flags & qtigralloc::PRIV_FLAGS_UBWC_ALIGNED ||
          handle->flags & qtigralloc::PRIV_FLAGS_UBWC_ALIGNED_PI) {
        android::gralloc4::encodeCompression(qtigralloc::Compression_QtiUBWC, out);
      } else {
        android::gralloc4::encodeCompression(android::gralloc4::Compression_None, out);
      }
      break;
    case (int64_t)StandardMetadataType::PLANE_LAYOUTS: {
      std::vector<PlaneLayout> plane_layouts;
      getFormatLayout(handle, &plane_layouts);
      android::gralloc4::encodePlaneLayouts(plane_layouts, out);
      break;
    }
    case (int64_t)StandardMetadataType::BLEND_MODE:
      android::gralloc4::encodeBlendMode((BlendMode)metadata->blendMode, out);
      break;
    case (int64_t)StandardMetadataType::SMPTE2086: {
      if (metadata->color.masteringDisplayInfo.colorVolumeSEIEnabled) {
        Smpte2086 mastering_display_values;
        mastering_display_values.primaryRed = {
            static_cast<float>(metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[0][0]) /
                50000.0f,
            static_cast<float>(metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[0][1]) /
                50000.0f};
        mastering_display_values.primaryGreen = {
            static_cast<float>(metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[1][0]) /
                50000.0f,
            static_cast<float>(metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[1][1]) /
                50000.0f};
        mastering_display_values.primaryBlue = {
            static_cast<float>(metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[2][0]) /
                50000.0f,
            static_cast<float>(metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[2][1]) /
                50000.0f};
        mastering_display_values.whitePoint = {
            static_cast<float>(metadata->color.masteringDisplayInfo.primaries.whitePoint[0]) /
                50000.0f,
            static_cast<float>(metadata->color.masteringDisplayInfo.primaries.whitePoint[1]) /
                50000.0f};
        mastering_display_values.maxLuminance =
            static_cast<float>(metadata->color.masteringDisplayInfo.maxDisplayLuminance);
        mastering_display_values.minLuminance =
            static_cast<float>(metadata->color.masteringDisplayInfo.minDisplayLuminance) / 10000.0f;
        android::gralloc4::encodeSmpte2086(mastering_display_values, out);
      } else {
        android::gralloc4::encodeSmpte2086(std::nullopt, out);
      }
      break;
    }
    case (int64_t)StandardMetadataType::CTA861_3: {
      if (metadata->color.contentLightLevel.lightLevelSEIEnabled) {
        Cta861_3 content_light_level;
        content_light_level.maxContentLightLevel =
            static_cast<float>(metadata->color.contentLightLevel.maxContentLightLevel);
        content_light_level.maxFrameAverageLightLevel =
            static_cast<float>(metadata->color.contentLightLevel.minPicAverageLightLevel) /
            10000.0f;
        android::gralloc4::encodeCta861_3(content_light_level, out);
      } else {
        android::gralloc4::encodeCta861_3(std::nullopt, out);
      }
      break;
    }
    case (int64_t)StandardMetadataType::SMPTE2094_40: {
      if (metadata->color.dynamicMetaDataValid &&
          metadata->color.dynamicMetaDataLen <= HDR_DYNAMIC_META_DATA_SZ) {
        std::vector<uint8_t> dynamic_metadata_payload;
        dynamic_metadata_payload.resize(metadata->color.dynamicMetaDataLen);
        dynamic_metadata_payload.assign(
            metadata->color.dynamicMetaDataPayload,
            metadata->color.dynamicMetaDataPayload + metadata->color.dynamicMetaDataLen);
        android::gralloc4::encodeSmpte2094_40(dynamic_metadata_payload, out);
      } else {
        android::gralloc4::encodeSmpte2094_40(std::nullopt, out);
      }
      break;
    }
    case (int64_t)StandardMetadataType::CROP: {
      // Crop is the same for all planes
      std::vector<Rect> out_crop = {{metadata->crop.left, metadata->crop.top, metadata->crop.right,
                                     metadata->crop.bottom}};
      android::gralloc4::encodeCrop(out_crop, out);
      break;
    }
    case QTI_VT_TIMESTAMP:
      android::gralloc4::encodeUint64(qtigralloc::MetadataType_VTTimestamp, metadata->vtTimeStamp,
                                      out);
      break;
    case QTI_COLOR_METADATA:
      qtigralloc::encodeColorMetadata(metadata->color, out);
      break;
    case QTI_PP_PARAM_INTERLACED:
      android::gralloc4::encodeInt32(qtigralloc::MetadataType_PPParamInterlaced,
                                     metadata->interlaced, out);
      break;
    case QTI_VIDEO_PERF_MODE:
      android::gralloc4::encodeUint32(qtigralloc::MetadataType_VideoPerfMode,
                                      metadata->isVideoPerfMode, out);
      break;
    case QTI_GRAPHICS_METADATA:
      qtigralloc::encodeGraphicsMetadata(metadata->graphics_metadata, out);
      break;
    case QTI_UBWC_CR_STATS_INFO:
      qtigralloc::encodeUBWCStats(metadata->ubwcCRStats, out);
      break;
    case QTI_REFRESH_RATE:
      android::gralloc4::encodeFloat(qtigralloc::MetadataType_RefreshRate, metadata->refreshrate,
                                     out);
      break;
    case QTI_MAP_SECURE_BUFFER:
      android::gralloc4::encodeInt32(qtigralloc::MetadataType_MapSecureBuffer,
                                     metadata->mapSecureBuffer, out);
      break;
    case QTI_LINEAR_FORMAT:
      android::gralloc4::encodeUint32(qtigralloc::MetadataType_LinearFormat, metadata->linearFormat,
                                      out);
      break;
    case QTI_SINGLE_BUFFER_MODE:
      android::gralloc4::encodeUint32(qtigralloc::MetadataType_SingleBufferMode,
                                      metadata->isSingleBufferMode, out);
      break;
    case QTI_CVP_METADATA:
      qtigralloc::encodeCVPMetadata(metadata->cvpMetadata, out);
      break;
    case QTI_VIDEO_HISTOGRAM_STATS:
      qtigralloc::encodeVideoHistogramMetadata(metadata->video_histogram_stats, out);
      break;
    case QTI_FD:
      android::gralloc4::encodeInt32(qtigralloc::MetadataType_FD, handle->fd, out);
      break;
    case QTI_PRIVATE_FLAGS:
      android::gralloc4::encodeInt32(qtigralloc::MetadataType_PrivateFlags, handle->flags, out);
      break;
    case QTI_ALIGNED_WIDTH_IN_PIXELS:
      android::gralloc4::encodeUint32(qtigralloc::MetadataType_AlignedWidthInPixels, handle->width,
                                      out);
      break;
    case QTI_ALIGNED_HEIGHT_IN_PIXELS:
      android::gralloc4::encodeUint32(qtigralloc::MetadataType_AlignedHeightInPixels,
                                      handle->height, out);
      break;
#ifdef METADATA_V2
    case QTI_STANDARD_METADATA_STATUS:
      qtigralloc::encodeMetadataState(metadata->isStandardMetadataSet, out);
      break;
    case QTI_VENDOR_METADATA_STATUS:
      qtigralloc::encodeMetadataState(metadata->isVendorMetadataSet, out);
      break;
#endif
#ifdef QTI_BUFFER_TYPE
    case QTI_BUFFER_TYPE:
      android::gralloc4::encodeUint32(qtigralloc::MetadataType_BufferType, handle->buffer_type,
                                      out);
      break;
#endif
    default:
      error = Error::UNSUPPORTED;
  }

  return error;
}

Error BufferManager::SetMetadata(private_handle_t *handle, int64_t metadatatype_value,
                                 hidl_vec<uint8_t> in) {
  std::lock_guard<std::mutex> lock(buffer_lock_);
  if (!handle)
    return Error::BAD_BUFFER;

  auto buf = GetBufferFromHandleLocked(handle);
  if (buf == nullptr)
    return Error::BAD_BUFFER;

  if (!handle->base_metadata) {
    return Error::BAD_BUFFER;
  }
  if (in.size() == 0) {
    return Error::UNSUPPORTED;
  }

  auto metadata = reinterpret_cast<MetaData_t *>(handle->base_metadata);

#ifdef METADATA_V2
  // By default, set these to true
  // Reset to false for special cases below
  if (IS_VENDOR_METADATA_TYPE(metadatatype_value)) {
    metadata->isVendorMetadataSet[GET_VENDOR_METADATA_STATUS_INDEX(metadatatype_value)] = true;
  } else if (GET_STANDARD_METADATA_STATUS_INDEX(metadatatype_value) < METADATA_SET_SIZE) {
    metadata->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(metadatatype_value)] = true;
  }
#endif

  switch (metadatatype_value) {
    // These are constant (unchanged after allocation)
    case (int64_t)StandardMetadataType::BUFFER_ID:
    case (int64_t)StandardMetadataType::NAME:
    case (int64_t)StandardMetadataType::WIDTH:
    case (int64_t)StandardMetadataType::HEIGHT:
    case (int64_t)StandardMetadataType::LAYER_COUNT:
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_REQUESTED:
    case (int64_t)StandardMetadataType::USAGE:
      return Error::BAD_VALUE;
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_FOURCC:
    case (int64_t)StandardMetadataType::PIXEL_FORMAT_MODIFIER:
    case (int64_t)StandardMetadataType::PROTECTED_CONTENT:
    case (int64_t)StandardMetadataType::ALLOCATION_SIZE:
    case (int64_t)StandardMetadataType::PLANE_LAYOUTS:
    case (int64_t)StandardMetadataType::CHROMA_SITING:
    case (int64_t)StandardMetadataType::INTERLACED:
    case (int64_t)StandardMetadataType::COMPRESSION:
    case QTI_FD:
    case QTI_PRIVATE_FLAGS:
    case QTI_ALIGNED_WIDTH_IN_PIXELS:
    case QTI_ALIGNED_HEIGHT_IN_PIXELS:
      return Error::UNSUPPORTED;
    case (int64_t)StandardMetadataType::DATASPACE:
      Dataspace dataspace;
      android::gralloc4::decodeDataspace(in, &dataspace);
      dataspaceToColorMetadata(dataspace, &metadata->color);
      break;
    case (int64_t)StandardMetadataType::BLEND_MODE:
      BlendMode mode;
      android::gralloc4::decodeBlendMode(in, &mode);
      metadata->blendMode = (int32_t)mode;
      break;
    case (int64_t)StandardMetadataType::SMPTE2086: {
      std::optional<Smpte2086> mastering_display_values;
      android::gralloc4::decodeSmpte2086(in, &mastering_display_values);
      if (mastering_display_values != std::nullopt) {
        metadata->color.masteringDisplayInfo.colorVolumeSEIEnabled = true;

        metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[0][0] =
            static_cast<uint32_t>(mastering_display_values->primaryRed.x * 50000.0f);
        metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[0][1] =
            static_cast<uint32_t>(mastering_display_values->primaryRed.y * 50000.0f);

        metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[1][0] =
            static_cast<uint32_t>(mastering_display_values->primaryGreen.x * 50000.0f);
        metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[1][1] =
            static_cast<uint32_t>(mastering_display_values->primaryGreen.y * 50000.0f);

        metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[2][0] =
            static_cast<uint32_t>(mastering_display_values->primaryBlue.x * 50000.0f);
        metadata->color.masteringDisplayInfo.primaries.rgbPrimaries[2][1] =
            static_cast<uint32_t>(mastering_display_values->primaryBlue.y * 50000.0f);

        metadata->color.masteringDisplayInfo.primaries.whitePoint[0] =
            static_cast<uint32_t>(mastering_display_values->whitePoint.x * 50000.0f);
        metadata->color.masteringDisplayInfo.primaries.whitePoint[1] =
            static_cast<uint32_t>(mastering_display_values->whitePoint.y * 50000.0f);

        metadata->color.masteringDisplayInfo.maxDisplayLuminance =
            static_cast<uint32_t>(mastering_display_values->maxLuminance);
        metadata->color.masteringDisplayInfo.minDisplayLuminance =
            static_cast<uint32_t>(mastering_display_values->minLuminance * 10000.0f);
      } else {
#ifdef METADATA_V2
        metadata->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(metadatatype_value)] =
            false;
#endif
        metadata->color.masteringDisplayInfo.colorVolumeSEIEnabled = false;
      }
      break;
    }
    case (int64_t)StandardMetadataType::CTA861_3: {
      std::optional<Cta861_3> content_light_level;
      android::gralloc4::decodeCta861_3(in, &content_light_level);
      if (content_light_level != std::nullopt) {
        metadata->color.contentLightLevel.lightLevelSEIEnabled = true;
        metadata->color.contentLightLevel.maxContentLightLevel =
            static_cast<uint32_t>(content_light_level->maxContentLightLevel);
        metadata->color.contentLightLevel.minPicAverageLightLevel =
            static_cast<uint32_t>(content_light_level->maxFrameAverageLightLevel * 10000.0f);
      } else {
#ifdef METADATA_V2
        metadata->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(metadatatype_value)] =
            false;
#endif
        metadata->color.contentLightLevel.lightLevelSEIEnabled = false;
      }
      break;
    }
    case (int64_t)StandardMetadataType::SMPTE2094_40: {
      std::optional<std::vector<uint8_t>> dynamic_metadata_payload;
      android::gralloc4::decodeSmpte2094_40(in, &dynamic_metadata_payload);
      if (dynamic_metadata_payload != std::nullopt) {
        if (dynamic_metadata_payload->size() > HDR_DYNAMIC_META_DATA_SZ)
          return Error::BAD_VALUE;

        metadata->color.dynamicMetaDataLen = dynamic_metadata_payload->size();
        std::copy(dynamic_metadata_payload->begin(), dynamic_metadata_payload->end(),
                  metadata->color.dynamicMetaDataPayload);
        metadata->color.dynamicMetaDataValid = true;
      } else {
        // Reset metadata by passing in std::nullopt
#ifdef METADATA_V2
        metadata->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(metadatatype_value)] =
            false;
#endif
        metadata->color.dynamicMetaDataValid = false;
      }
      break;
    }
    case (int64_t)StandardMetadataType::CROP: {
      std::vector<Rect> in_crop;
      android::gralloc4::decodeCrop(in, &in_crop);
      if (in_crop.size() != 1)
        return Error::UNSUPPORTED;

      metadata->crop.left = in_crop[0].left;
      metadata->crop.top = in_crop[0].top;
      metadata->crop.right = in_crop[0].right;
      metadata->crop.bottom = in_crop[0].bottom;
      break;
    }
    case QTI_VT_TIMESTAMP:
      android::gralloc4::decodeUint64(qtigralloc::MetadataType_VTTimestamp, in,
                                      &metadata->vtTimeStamp);
      break;
    case QTI_COLOR_METADATA:
      ColorMetaData color;
      qtigralloc::decodeColorMetadata(in, &color);
      metadata->color = color;
      break;
    case QTI_PP_PARAM_INTERLACED:
      android::gralloc4::decodeInt32(qtigralloc::MetadataType_PPParamInterlaced, in,
                                     &metadata->interlaced);
      break;
    case QTI_VIDEO_PERF_MODE:
      android::gralloc4::decodeUint32(qtigralloc::MetadataType_VideoPerfMode, in,
                                      &metadata->isVideoPerfMode);
      break;
    case QTI_GRAPHICS_METADATA:
      qtigralloc::decodeGraphicsMetadata(in, &metadata->graphics_metadata);
      break;
    case QTI_UBWC_CR_STATS_INFO:
      qtigralloc::decodeUBWCStats(in, &metadata->ubwcCRStats[0]);
      break;
    case QTI_REFRESH_RATE:
      android::gralloc4::decodeFloat(qtigralloc::MetadataType_RefreshRate, in,
                                     &metadata->refreshrate);
      break;
    case QTI_MAP_SECURE_BUFFER:
      android::gralloc4::decodeInt32(qtigralloc::MetadataType_MapSecureBuffer, in,
                                     &metadata->mapSecureBuffer);
      break;
    case QTI_LINEAR_FORMAT:
      android::gralloc4::decodeUint32(qtigralloc::MetadataType_LinearFormat, in,
                                      &metadata->linearFormat);
      break;
    case QTI_SINGLE_BUFFER_MODE:
      android::gralloc4::decodeUint32(qtigralloc::MetadataType_SingleBufferMode, in,
                                      &metadata->isSingleBufferMode);
      break;
    case QTI_CVP_METADATA:
      qtigralloc::decodeCVPMetadata(in, &metadata->cvpMetadata);
      break;
    case QTI_VIDEO_HISTOGRAM_STATS:
      qtigralloc::decodeVideoHistogramMetadata(in, &metadata->video_histogram_stats);
      break;
    default:
#ifdef METADATA_V2
      if (IS_VENDOR_METADATA_TYPE(metadatatype_value)) {
        metadata->isVendorMetadataSet[GET_VENDOR_METADATA_STATUS_INDEX(metadatatype_value)] = false;
      } else if (GET_STANDARD_METADATA_STATUS_INDEX(metadatatype_value) < METADATA_SET_SIZE) {
        metadata->isStandardMetadataSet[GET_STANDARD_METADATA_STATUS_INDEX(metadatatype_value)] =
            false;
      }
#endif
      return Error::BAD_VALUE;
  }

  return Error::NONE;
}

}  //  namespace gralloc
