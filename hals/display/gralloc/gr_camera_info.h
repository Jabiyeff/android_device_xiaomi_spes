/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 *
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
 *
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __GR_CAMERA_INFO_H__
#define __GR_CAMERA_INFO_H__

#include "gr_utils.h"

// Plane types supported by the camera format
typedef enum {
  CAMERA_PLANE_TYPE_RAW,  // RAW plane for Single planar formats including UYVY and thier variants
  CAMERA_PLANE_TYPE_Y,    // Y only
  CAMERA_PLANE_TYPE_UV,   // UV, VU, Cb, Cr planes for YUV variants
  CAMERA_PLANE_TYPE_U,    // U plane only
  CAMERA_PLANE_TYPE_V,    // V plane only
  CAMERA_PLANE_TYPE_META_Y,   // Metadata plane for Y
  CAMERA_PLANE_TYPE_META_VU,  // Metadata plane for VU and UV
} CamxPlaneType;

// External camera pixel formats that are allocated by gralloc
typedef enum : unsigned int {
  CAMERA_PIXEL_FORMAT_NV21_ZSL = 0x113,  // NV21 format with alignment requirements for
                                         // YUV reprocessing
  CAMERA_PIXEL_FORMAT_YUV_FLEX = 0x125,  // YUV format with fliexible alignment defined by
                                         // individual APIs
  CAMERA_PIXEL_FORMAT_UBWC_FLEX = 0x126,  // YUV format with fliexible alignment defined by
                                          // individual APIs
  CAMERA_PIXEL_FORMAT_MULTIPLANAR_FLEX = 0x127,   // YUV format with fliexible alignment defined by
                                                  // individual APIs
  CAMERA_PIXEL_FORMAT_UBWC_FLEX_2_BATCH = 0x128,  // YUV format with fliexible alignment defined by
                                                  // individual APIs
  CAMERA_PIXEL_FORMAT_UBWC_FLEX_4_BATCH = 0x129,  // YUV format with fliexible alignment defined by
                                                  // individual APIs
  CAMERA_PIXEL_FORMAT_UBWC_FLEX_8_BATCH = 0x130,  // YUV format with fliexible alignment defined by
                                                  // individual APIs
  CAMERA_PIXEL_FORMAT_NV12_VENUS = 0x7FA30C04,           // NV12 video format
  CAMERA_PIXEL_FORMAT_NV12_HEIF = 0x00000116,            // HEIF video YUV420 format
  CAMERA_PIXEL_FORMAT_YCbCr_420_SP_UBWC = 0x7FA30C06,    // 8 bit YUV 420 semi-planar UBWC format
  CAMERA_PIXEL_FORMAT_YCbCr_420_TP10_UBWC = 0x7FA30C09,  // TP10 YUV 420 semi-planar UBWC format
  CAMERA_PIXEL_FORMAT_YCbCr_420_P010_UBWC = 0x124,       // P010 YUV 420 semi-planar UBWC format
  CAMERA_PIXEL_FORMAT_RAW_OPAQUE = 0x24,                 // Opaque RAW format
  CAMERA_PIXEL_FORMAT_RAW10 = 0x25,                      // Opaque RAW10 bit format
  CAMERA_PIXEL_FORMAT_RAW12 = 0x26,                      // Opaque RAW12 bit format
} CamxPixelFormat;

// Camera Result Codes
typedef enum : int {
  CamxFormatResultSuccess = 0,           // Operation was successful
  CamxFormatResultEFailed = 1,           // Operation encountered unspecified error
  CamxFormatResultEUnsupported = 2,      // Operation is not supported
  CamxFormatResultEInvalidState = 3,     // Invalid state
  CamxFormatResultEInvalidArg = 4,       // Invalid argument
  CamxFormatResultEInvalidPointer = 5,   // Invalid memory pointer
  CamxFormatResultENoSuch = 6,           // No such item exists or is valid
  CamxFormatResultEOutOfBounds = 7,      // Out of bounds
  CamxFormatResultENoMemory = 8,         // Out of memory
  CamxFormatResultENoMore = 10,          // No more items available
  CamxFormatResultENeedMore = 11,        // Operation requires more
  CamxFormatResultEPrivLevel = 13,       // Privileges are insufficient for requested operation
  CamxFormatResultENotImplemented = 26,  // Function or method is not implemented
} CamxFormatResult;

namespace gralloc {

class CameraInfo {
 public:
  bool IsCameraUtilsPresent() { return libcamera_utils_ != nullptr; }

  int GetUBWCInfo(int format, bool *is_Supported, bool *is_PI, int *version);

  int GetPlaneAlignment(int format, int plane_type, unsigned int *alignment);

  int IsPerPlaneFdNeeded(int format, bool *is_per_plane_fd_needed);

  int GetBpp(int format, int *bpp);

  int GetPerPlaneBpp(int format, int plane_type, int *bpp);

  int GetPlaneStartAddressAlignment(int format, int plane_type, int *alignment);

  int GetBufferSize(int format, int width, int height, unsigned int *size);

  int GetStrideInBytes(int format, int plane_type, int width, int *stride_bytes);

  int GetStrideInPixels(int format, int plane_type, int width, float *stride_pixel);

  int GetPixelIncrement(int format, int plane_type, int *pixel_increment);

  int GetPlaneOffset(int format, int plane_type, int width, int height, int *offset);

  int GetSubsamplingFactor(int format, int plane_type, bool isHorizontal, int *subsampling_factor);

  int GetPlaneTypes(int format, PlaneComponent *plane_component_array, int *plane_count);

  int GetScanline(int format, int plane_type, int height, int *scanlines);

  int GetPlaneSize(int format, int plane_type, int width, int height, unsigned int *size);

  int GetCameraFormatPlaneInfo(int format, int width, int height, int *plane_count,
                               PlaneLayoutInfo *plane_info);

  CamxPixelFormat GetCameraPixelFormat(int hal_format);

  static CameraInfo *GetInstance();

 private:
  CameraInfo();
  ~CameraInfo();

  PlaneComponent GetPlaneComponent(CamxPlaneType plane_type);

  CamxPlaneType GetCamxPlaneType(int plane_type);

  CamxFormatResult (*LINK_camera_get_stride_in_bytes)(CamxPixelFormat format,
                                                      CamxPlaneType plane_type, int width,
                                                      int *stride) = nullptr;

  CamxFormatResult (*LINK_camera_get_stride_in_pixels)(CamxPixelFormat format,
                                                       CamxPlaneType plane_type, int width,
                                                       float *stride) = nullptr;

  CamxFormatResult (*LINK_camera_get_scanline)(CamxPixelFormat format, CamxPlaneType plane_type,
                                               int height, int *scanLine) = nullptr;

  CamxFormatResult (*LINK_camera_get_plane_size)(CamxPixelFormat format, CamxPlaneType plane_type,
                                                 int width, int height,
                                                 unsigned int *aligned_size) = nullptr;

  CamxFormatResult (*LINK_camera_get_buffer_size)(CamxPixelFormat format, int width, int height,
                                                  unsigned int *buffer_size) = nullptr;

  CamxFormatResult (*LINK_camera_get_ubwc_info)(CamxPixelFormat format, bool *isSupported,
                                                bool *isPI, int *version) = nullptr;

  CamxFormatResult (*LINK_camera_get_plane_alignment)(CamxPixelFormat format,
                                                      CamxPlaneType plane_type,
                                                      unsigned int *alignment) = nullptr;

  CamxFormatResult (*LINK_camera_get_plane_offset)(CamxPixelFormat format, CamxPlaneType plane_type,
                                                   int *offset, int width, int height) = nullptr;

  CamxFormatResult (*LINK_camera_get_plane_types)(CamxPixelFormat format,
                                                  CamxPlaneType *plane_types_array,
                                                  int *plane_count) = nullptr;

  CamxFormatResult (*LINK_camera_is_per_plane_fd_needed)(CamxPixelFormat format,
                                                         bool *is_perplane_fd_needed) = nullptr;

  CamxFormatResult (*LINK_camera_get_bpp)(CamxPixelFormat format, int *bpp) = nullptr;

  CamxFormatResult (*LINK_camera_get_per_plane_bpp)(CamxPixelFormat format,
                                                    CamxPlaneType plane_type, int *bpp) = nullptr;

  CamxFormatResult (*LINK_camera_get_subsampling_factor)(CamxPixelFormat format,
                                                         CamxPlaneType plane_type,
                                                         bool is_horizontal,
                                                         int *subsampling_factor) = nullptr;

  CamxFormatResult (*LINK_camera_get_plane_count)(CamxPixelFormat format,
                                                  int *plane_count) = nullptr;

  CamxFormatResult (*LINK_camera_get_pixel_increment)(CamxPixelFormat format,
                                                      CamxPlaneType plane_type,
                                                      int *pixel_increment) = nullptr;

  CamxFormatResult (*LINK_camera_get_plane_start_address_alignment)(CamxPixelFormat format,
                                                                    CamxPlaneType planeType,
                                                                    int *pAlignment) = nullptr;

  void *libcamera_utils_ = nullptr;
  static CameraInfo *s_instance;
};

}  // namespace gralloc

#endif  // __GR_CAMERA_INFO_H__
