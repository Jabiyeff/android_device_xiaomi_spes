/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of The Linux Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#include <dlfcn.h>
#include <log/log.h>
#include <mutex>

#include "gr_camera_info.h"
#include "gralloc_priv.h"

using std::lock_guard;
using std::mutex;

namespace gralloc {

CameraInfo *CameraInfo::s_instance = nullptr;

CameraInfo *CameraInfo::GetInstance() {
  static mutex s_lock;
  lock_guard<mutex> obj(s_lock);
  if (!s_instance) {
    s_instance = new CameraInfo();
  }

  return s_instance;
}

CameraInfo::CameraInfo() {
  libcamera_utils_ = ::dlopen("libcamxexternalformatutils.so", RTLD_NOW);
  if (libcamera_utils_) {
    *reinterpret_cast<void **>(&LINK_camera_get_stride_in_bytes) =
        ::dlsym(libcamera_utils_, "CamxFormatUtil_GetStrideInBytes");
    *reinterpret_cast<void **>(&LINK_camera_get_stride_in_pixels) =
        ::dlsym(libcamera_utils_, "CamxFormatUtil_GetStrideInPixels");
    *reinterpret_cast<void **>(&LINK_camera_get_scanline) =
        ::dlsym(libcamera_utils_, "CamxFormatUtil_GetScanline");
    *reinterpret_cast<void **>(&LINK_camera_get_plane_size) =
        ::dlsym(libcamera_utils_, "CamxFormatUtil_GetPlaneSize");
    *reinterpret_cast<void **>(&LINK_camera_get_buffer_size) =
        ::dlsym(libcamera_utils_, "CamxFormatUtil_GetBufferSize");
    *reinterpret_cast<void **>(&LINK_camera_get_ubwc_info) =
        ::dlsym(libcamera_utils_, "CamxFormatUtil_GetUBWCInfo");
    *reinterpret_cast<void **>(&LINK_camera_get_plane_alignment) =
        ::dlsym(libcamera_utils_, "CamxFormatUtil_GetPlaneAlignment");
    *reinterpret_cast<void **>(&LINK_camera_get_plane_offset) =
        ::dlsym(libcamera_utils_, "CamxFormatUtil_GetPlaneOffset");
    *reinterpret_cast<void **>(&LINK_camera_is_per_plane_fd_needed) =
        ::dlsym(libcamera_utils_, "CamxFormatUtil_IsPerPlaneFdNeeded");
    *reinterpret_cast<void **>(&LINK_camera_get_bpp) =
        ::dlsym(libcamera_utils_, "CamxFormatUtil_GetBpp");
    *reinterpret_cast<void **>(&LINK_camera_get_per_plane_bpp) =
        ::dlsym(libcamera_utils_, "CamxFormatUtil_GetPerPlaneBpp");
    *reinterpret_cast<void **>(&LINK_camera_get_subsampling_factor) =
        ::dlsym(libcamera_utils_, "CamxFormatUtil_GetSubsamplingFactor");
    *reinterpret_cast<void **>(&LINK_camera_get_plane_count) =
        ::dlsym(libcamera_utils_, "CamxFormatUtil_GetPlaneCount");
    *reinterpret_cast<void **>(&LINK_camera_get_plane_types) =
        ::dlsym(libcamera_utils_, "CamxFormatUtil_GetPlaneTypes");
    *reinterpret_cast<void **>(&LINK_camera_get_pixel_increment) =
        ::dlsym(libcamera_utils_, "CamxFormatUtil_GetPixelIncrement");
    *reinterpret_cast<void **>(&LINK_camera_get_plane_start_address_alignment) =
        ::dlsym(libcamera_utils_, "CamxFormatUtil_GetPlaneStartAddressAlignment");
  } else {
    ALOGE("%s: Failed to load libcamxexternalformatutils.so - %s", __FUNCTION__, strerror(errno));
  }
}

CameraInfo::~CameraInfo() {
  if (libcamera_utils_) {
    ::dlclose(libcamera_utils_);
  }
}

CamxPixelFormat CameraInfo::GetCameraPixelFormat(int hal_format) {
  CamxPixelFormat format = (CamxPixelFormat)0;
  switch (hal_format) {
    case HAL_PIXEL_FORMAT_NV21_ZSL:
      format = CAMERA_PIXEL_FORMAT_NV21_ZSL;
      break;
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX:
      format = CAMERA_PIXEL_FORMAT_UBWC_FLEX;
      break;
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH:
      format = CAMERA_PIXEL_FORMAT_UBWC_FLEX_2_BATCH;
      break;
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH:
      format = CAMERA_PIXEL_FORMAT_UBWC_FLEX_4_BATCH;
      break;
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH:
      format = CAMERA_PIXEL_FORMAT_UBWC_FLEX_8_BATCH;
      break;
    case HAL_PIXEL_FORMAT_MULTIPLANAR_FLEX:
      format = CAMERA_PIXEL_FORMAT_MULTIPLANAR_FLEX;
      break;
    case HAL_PIXEL_FORMAT_RAW_OPAQUE:
      format = CAMERA_PIXEL_FORMAT_RAW_OPAQUE;
      break;
    case HAL_PIXEL_FORMAT_RAW10:
      format = CAMERA_PIXEL_FORMAT_RAW10;
      break;
    case HAL_PIXEL_FORMAT_RAW12:
      format = CAMERA_PIXEL_FORMAT_RAW12;
      break;
    default:
      ALOGE("%s: No map for format: 0x%x", __FUNCTION__, hal_format);
      break;
  }

  return format;
}

int CameraInfo::GetBufferSize(int format, int width, int height, unsigned int *size) {
  CamxFormatResult result = (CamxFormatResult)-1;
  if (LINK_camera_get_buffer_size) {
    result = LINK_camera_get_buffer_size(GetCameraPixelFormat(format), width, height, size);
    if (result != 0) {
      ALOGE("%s: Failed to get the buffer size. Error code: %d", __FUNCTION__, result);
    }
  } else {
    ALOGW("%s: Failed to link CamxFormatUtil_GetBufferSize. Error code : %d", __FUNCTION__, result);
  }

  return result;
}

int CameraInfo::GetStrideInBytes(int format, int plane_type, int width, int *stride_bytes) {
  CamxFormatResult result = (CamxFormatResult)-1;
  if (LINK_camera_get_stride_in_bytes) {
    result = LINK_camera_get_stride_in_bytes(GetCameraPixelFormat(format),
                                             GetCamxPlaneType(plane_type), width, stride_bytes);
    if (result != 0) {
      ALOGE("%s: Failed to get the stride in bytes. Error code: %d", __FUNCTION__, result);
    }
  } else {
    ALOGW("%s: Failed to link CamxFormatUtil_GetStrideInBytes. Error code : %d", __FUNCTION__,
          result);
  }

  return result;
}

int CameraInfo::GetStrideInPixels(int format, int plane_type, int width, float *stride_pixel) {
  CamxFormatResult result = (CamxFormatResult)-1;
  if (LINK_camera_get_stride_in_pixels) {
    result = LINK_camera_get_stride_in_pixels(GetCameraPixelFormat(format),
                                              GetCamxPlaneType(plane_type), width, stride_pixel);
    if (result != 0) {
      ALOGE("%s: Failed to get the stride in pixels. Error code: %d", __FUNCTION__, result);
    }
  } else {
    ALOGW("%s: Failed to link CamxFormatUtil_GetStrideInPixels. Error code : %d", __FUNCTION__,
          result);
  }

  return result;
}

int CameraInfo::GetPixelIncrement(int format, int plane_type, int *pixel_increment) {
  CamxFormatResult result = (CamxFormatResult)-1;
  if (LINK_camera_get_pixel_increment) {
    result = LINK_camera_get_pixel_increment(GetCameraPixelFormat(format),
                                             GetCamxPlaneType(plane_type), pixel_increment);
    if (result != 0) {
      ALOGE("%s: Failed to get pixel increment. Error code: %d", __FUNCTION__, result);
    }
  } else {
    ALOGW("%s: Failed to link CamxFormatUtil_GetPixelIncrement. Error code : %d", __FUNCTION__,
          result);
  }

  return result;
}

int CameraInfo::GetPlaneOffset(int format, int plane_type, int width, int height, int *offset) {
  CamxFormatResult result = (CamxFormatResult)-1;
  if (LINK_camera_get_plane_offset) {
    result = LINK_camera_get_plane_offset(GetCameraPixelFormat(format),
                                          GetCamxPlaneType(plane_type), offset, width, height);
    if (result != 0) {
      ALOGE("%s: Failed to get the plane offset. Error code: %d", __FUNCTION__, result);
    }
  } else {
    ALOGW("%s: Failed to link CamxFormatUtil_GetPlaneOffset. Error code : %d", __FUNCTION__,
          result);
  }

  return result;
}

int CameraInfo::GetSubsamplingFactor(int format, int plane_type, bool isHorizontal,
                                     int *subsampling_factor) {
  CamxFormatResult result = (CamxFormatResult)-1;
  if (LINK_camera_get_subsampling_factor) {
    result = LINK_camera_get_subsampling_factor(GetCameraPixelFormat(format),
                                                GetCamxPlaneType(plane_type), isHorizontal,
                                                subsampling_factor);
    if (result != 0) {
      ALOGE("%s: Failed to get the sub-sampling factor. Error code: %d", __FUNCTION__, result);
    }
  } else {
    ALOGW("%s: Failed to link CamxFormatUtil_GetSubsamplingFactor. Error code : %d", __FUNCTION__,
          result);
  }

  return result;
}

int CameraInfo::GetPlaneTypes(int format, PlaneComponent *plane_component_array, int *plane_count) {
  CamxPlaneType plane_types_array[8] = {};
  CamxFormatResult result = (CamxFormatResult)-1;
  if (LINK_camera_get_plane_types) {
    result =
        LINK_camera_get_plane_types(GetCameraPixelFormat(format), plane_types_array, plane_count);
    if (result == 0) {
      for (int plane = 0; plane < *plane_count; plane++) {
        plane_component_array[plane] = GetPlaneComponent(plane_types_array[plane]);
      }
    } else {
      ALOGE("%s: Failed to get the plane types. Error code: %d", __FUNCTION__, result);
    }
  } else {
    ALOGW("%s: Failed to link CamxFormatUtil_GetPlaneTypes. Error code : %d", __FUNCTION__, result);
  }

  return result;
}

int CameraInfo::GetScanline(int format, int plane_type, int height, int *scanlines) {
  CamxFormatResult result = (CamxFormatResult)-1;
  if (LINK_camera_get_scanline) {
    result = LINK_camera_get_scanline(GetCameraPixelFormat(format), GetCamxPlaneType(plane_type),
                                      height, scanlines);
    if (result != 0) {
      ALOGE("%s: Failed to get the scanlines. Error code: %d", __FUNCTION__, result);
    }
  } else {
    ALOGW("%s: Failed to link CamxFormatUtil_GetScanline. Error code : %d", __FUNCTION__, result);
  }

  return result;
}

int CameraInfo::GetPlaneSize(int format, int plane_type, int width, int height,
                             unsigned int *size) {
  CamxFormatResult result = (CamxFormatResult)-1;
  if (LINK_camera_get_plane_size) {
    result = LINK_camera_get_plane_size(GetCameraPixelFormat(format), GetCamxPlaneType(plane_type),
                                        width, height, size);
    if (result != 0) {
      ALOGE("%s: Failed to get the plane size. Error code: %d", __FUNCTION__, result);
    }
  } else {
    ALOGW("%s: Failed to link CamxFormatUtil_GetPlaneSize. Error code : %d", __FUNCTION__, result);
  }

  return result;
}

int CameraInfo::GetCameraFormatPlaneInfo(int format, int width, int height, int *plane_count,
                                         PlaneLayoutInfo *plane_info) {
  int h_subsampling = 0;
  int v_subsampling = 0;
  int offset = 0;
  int pixel_increment = 0;
  float stride_pixel = 0;
  int stride_bytes = 0;
  int scanlines = 0;
  unsigned int plane_size = 0;
  PlaneComponent plane_type[8] = {};
  int result;
  result = GetPlaneTypes(format, plane_type, plane_count);
  if (result != 0) {
    ALOGE("%s: Failed to get the plane types. Error code : %d", __FUNCTION__, result);
    return result;
  }

  for (int i = 0; i < *plane_count; i++) {
    plane_info[i].component = plane_type[i];
    result = GetSubsamplingFactor(format, plane_type[i], true, &h_subsampling);
    if (result != 0) {
      ALOGE("%s: Failed to get horizontal subsampling factor. plane_type = %d, Error code : %d",
            __FUNCTION__, plane_type[i], result);
      break;
    }
    plane_info[i].h_subsampling = (uint32_t)h_subsampling;

    result = GetSubsamplingFactor(format, plane_type[i], false, &v_subsampling);
    if (result != 0) {
      ALOGE("%s: Failed to get vertical subsampling factor. plane_type = %d, Error code : %d",
            __FUNCTION__, plane_type[i], result);
      break;
    }
    plane_info[i].v_subsampling = (uint32_t)v_subsampling;

    result = GetPlaneOffset(format, plane_type[i], width, height, &offset);
    if (result != 0) {
      ALOGE("%s: Failed to get plane offset. plane_type = %d, Error code : %d", __FUNCTION__,
            plane_type[i], result);
      break;
    }
    plane_info[i].offset = (uint32_t)offset;

    result = GetPixelIncrement(format, plane_type[i], &pixel_increment);
    if (result != 0) {
      ALOGE("%s: Failed to get pixel increment. plane_type = %d, Error code : %d", __FUNCTION__,
            plane_type[i], result);
      break;
    }
    plane_info[i].step = (int32_t)pixel_increment;

    result = GetStrideInPixels(format, plane_type[i], width, &stride_pixel);
    if (result != 0) {
      ALOGE("%s: Failed to get stride in pixel. plane_type = %d, Error code : %d", __FUNCTION__,
            plane_type[i], result);
      break;
    }
    plane_info[i].stride = (int32_t)stride_pixel;

    result = GetStrideInBytes(format, plane_type[i], width, &stride_bytes);
    if (result != 0) {
      ALOGE("%s: Failed to get stride in bytes. plane_type = %d, Error code : %d", __FUNCTION__,
            plane_type[i], result);
      break;
    }
    plane_info[i].stride_bytes = (int32_t)stride_bytes;

    result = GetScanline(format, plane_type[i], height, &scanlines);
    if (result != 0) {
      ALOGE("%s: Failed to get scanlines. plane_type = %d, Error code : %d", __FUNCTION__,
            plane_type[i], result);
      break;
    }
    plane_info[i].scanlines = (int32_t)scanlines;

    result = GetPlaneSize(format, plane_type[i], width, height, &plane_size);
    if (result != 0) {
      ALOGE("%s: Failed to get plane size. plane_type = %d, Error code : %d", __FUNCTION__,
            plane_type[i], result);
      break;
    }
    plane_info[i].size = (uint32_t)plane_size;
  }

  return result;
}

int CameraInfo::GetUBWCInfo(int format, bool *is_supported, bool *is_pi, int *version) {
  CamxFormatResult result = (CamxFormatResult)-1;
  if (LINK_camera_get_ubwc_info) {
    result = LINK_camera_get_ubwc_info(GetCameraPixelFormat(format), is_supported, is_pi, version);
    if (result != 0) {
      ALOGE("%s: Failed to get the UBWC info. Error code: %d", __FUNCTION__, result);
    }
  } else {
    ALOGW("%s: Failed to link CamxFormatUtil_GetUBWCInfo. Error code : %d", __FUNCTION__, result);
  }

  return result;
}

int CameraInfo::GetPlaneAlignment(int format, int plane_type, unsigned int *alignment) {
  CamxFormatResult result = (CamxFormatResult)-1;
  if (LINK_camera_get_plane_alignment) {
    result = LINK_camera_get_plane_alignment(GetCameraPixelFormat(format),
                                             GetCamxPlaneType(plane_type), alignment);
    if (result != 0) {
      ALOGE("%s: Failed to get the plane alignment. Error code: %d", __FUNCTION__, result);
    }
  } else {
    ALOGW("%s: Failed to link CamxFormatUtil_GetPlaneAlignment. Error code : %d", __FUNCTION__,
          result);
  }

  return result;
}

int CameraInfo::IsPerPlaneFdNeeded(int format, bool *is_per_plane_fd_needed) {
  CamxFormatResult result = (CamxFormatResult)-1;
  if (LINK_camera_is_per_plane_fd_needed) {
    result =
        LINK_camera_is_per_plane_fd_needed(GetCameraPixelFormat(format), is_per_plane_fd_needed);
    if (result != 0) {
      ALOGE("%s: Failed to get per_plane_fd flag. Error code: %d", __FUNCTION__, result);
    }
  } else {
    ALOGW("%s: Failed to link CamxFormatUtil_IsPerPlaneFdNeeded. Error code : %d", __FUNCTION__,
          result);
  }

  return result;
}

int CameraInfo::GetBpp(int format, int *bpp) {
  CamxFormatResult result = (CamxFormatResult)-1;
  if (LINK_camera_get_bpp) {
    result = LINK_camera_get_bpp(GetCameraPixelFormat(format), bpp);
    if (result != 0) {
      ALOGE("%s: Failed to get the bpp. Error code: %d", __FUNCTION__, result);
    }
  } else {
    ALOGW("%s: Failed to link CamxFormatUtil_GetBpp. Error code : %d", __FUNCTION__, result);
  }

  return result;
}

int CameraInfo::GetPerPlaneBpp(int format, int plane_type, int *bpp) {
  CamxFormatResult result = (CamxFormatResult)-1;
  if (LINK_camera_get_per_plane_bpp) {
    result = LINK_camera_get_per_plane_bpp(GetCameraPixelFormat(format),
                                           GetCamxPlaneType(plane_type), bpp);
    if (result != 0) {
      ALOGE("%s: Failed to get the per plane bpp. Error code: %d", __FUNCTION__, result);
    }
  } else {
    ALOGW("%s: Failed to link CamxFormatUtil_GetPerPlaneBpp. Error code : %d", __FUNCTION__,
          result);
  }

  return result;
}

int CameraInfo::GetPlaneStartAddressAlignment(int format, int plane_type, int *alignment) {
  CamxFormatResult result = (CamxFormatResult)-1;
  if (LINK_camera_get_plane_start_address_alignment) {
    result = LINK_camera_get_plane_start_address_alignment(GetCameraPixelFormat(format),
                                                           GetCamxPlaneType(plane_type), alignment);
    if (result != 0) {
      ALOGE("%s: Failed to get the plane star address alignment. Error code: %d", __FUNCTION__,
            result);
    }
  } else {
    ALOGW("%s: Failed to link CamxFormatUtil_GetPlaneStartAddressAlignment. Error code : %d",
          __FUNCTION__, result);
  }

  return result;
}

PlaneComponent CameraInfo::GetPlaneComponent(CamxPlaneType plane_type) {
  PlaneComponent plane_component = (PlaneComponent)0;
  switch (plane_type) {
    case CAMERA_PLANE_TYPE_RAW:
      plane_component = (PlaneComponent)PLANE_COMPONENT_RAW;
      break;
    case CAMERA_PLANE_TYPE_Y:
      plane_component = (PlaneComponent)PLANE_COMPONENT_Y;
      break;
    case CAMERA_PLANE_TYPE_UV:
      plane_component = (PlaneComponent)(PLANE_COMPONENT_Cb | PLANE_COMPONENT_Cr);
      break;
    case CAMERA_PLANE_TYPE_U:
      plane_component = (PlaneComponent)PLANE_COMPONENT_Cb;
      break;
    case CAMERA_PLANE_TYPE_V:
      plane_component = (PlaneComponent)PLANE_COMPONENT_Cr;
      break;
    case CAMERA_PLANE_TYPE_META_Y:
      plane_component = (PlaneComponent)(PLANE_COMPONENT_META | PLANE_COMPONENT_Y);
      break;
    case CAMERA_PLANE_TYPE_META_VU:
      plane_component =
          (PlaneComponent)(PLANE_COMPONENT_META | PLANE_COMPONENT_Cb | PLANE_COMPONENT_Cr);
      break;
    default:
      ALOGE("%s: No PlaneComponent mapping for plane_type: %d", __FUNCTION__, plane_type);
      break;
  }

  return plane_component;
}

CamxPlaneType CameraInfo::GetCamxPlaneType(int plane_type) {
  CamxPlaneType camx_plane_type = (CamxPlaneType)0;
  switch (plane_type) {
    case PLANE_COMPONENT_RAW:
      camx_plane_type = CAMERA_PLANE_TYPE_RAW;
      break;
    case PLANE_COMPONENT_Y:
      camx_plane_type = CAMERA_PLANE_TYPE_Y;
      break;
    case (PLANE_COMPONENT_Cb | PLANE_COMPONENT_Cr):
      camx_plane_type = CAMERA_PLANE_TYPE_UV;
      break;
    case PLANE_COMPONENT_Cb:
      camx_plane_type = CAMERA_PLANE_TYPE_U;
      break;
    case PLANE_COMPONENT_Cr:
      camx_plane_type = CAMERA_PLANE_TYPE_V;
      break;
    case (PLANE_COMPONENT_META | PLANE_COMPONENT_Y):
      camx_plane_type = CAMERA_PLANE_TYPE_META_Y;
      break;
    case (PLANE_COMPONENT_META | PLANE_COMPONENT_Cr | PLANE_COMPONENT_Cb):
      camx_plane_type = CAMERA_PLANE_TYPE_META_VU;
      break;
    default:
      ALOGE("%s: No CamxPlane for plane_type: %d", __FUNCTION__, plane_type);
      break;
  }

  return camx_plane_type;
}

}  // namespace gralloc
