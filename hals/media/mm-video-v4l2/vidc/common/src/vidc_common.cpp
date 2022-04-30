/*--------------------------------------------------------------------------
Copyright (c) 2017-2020 The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------*/
#define LOG_TAG "OMX_COMMON"

#include <utils/Log.h>
#include "vidc_debug.h"
#include "vidc_common.h"
#include "OMX_Core.h"
#include "OMX_QCOMExtns.h"
#include "OMX_VideoExt.h"
#include "OMX_IndexExt.h"
#include <linux/videodev2.h>

int debug_level = PRIO_ERROR;


pl_map profile_level_converter::profile_avc_omx_to_v4l2 ({
            {QOMX_VIDEO_AVCProfileConstrainedBaseline,
                        V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE},
            {QOMX_VIDEO_AVCProfileBaseline,
                        V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE},
            {QOMX_VIDEO_AVCProfileMain,
                        V4L2_MPEG_VIDEO_H264_PROFILE_MAIN},
            {QOMX_VIDEO_AVCProfileConstrainedHigh,
                        V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_HIGH},
            {QOMX_VIDEO_AVCProfileHigh,
                        V4L2_MPEG_VIDEO_H264_PROFILE_HIGH}
        });

pl_map profile_level_converter::profile_avc_v4l2_to_omx ({});

pl_map profile_level_converter::profile_hevc_omx_to_v4l2 ({
            {OMX_VIDEO_HEVCProfileMain,
                        V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN},
            {OMX_VIDEO_HEVCProfileMain10,
                        V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10},
            {OMX_VIDEO_HEVCProfileMainStill,
                        V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE},
            {OMX_VIDEO_HEVCProfileMain10HDR10,
                        V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10},
            {OMX_VIDEO_HEVCProfileMain10HDR10Plus,
                        V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10},
        });

pl_map profile_level_converter::profile_hevc_v4l2_to_omx ({});

pl_map profile_level_converter::profile_mpeg2_omx_to_v4l2 ({
            {OMX_VIDEO_MPEG2ProfileSimple,
                        V4L2_MPEG_VIDC_VIDEO_MPEG2_PROFILE_SIMPLE},
            {OMX_VIDEO_MPEG2ProfileMain,
                        V4L2_MPEG_VIDC_VIDEO_MPEG2_PROFILE_MAIN},
        });

pl_map profile_level_converter::profile_mpeg2_v4l2_to_omx ({});

pl_map profile_level_converter::profile_vp8_omx_to_v4l2 ({
            {OMX_VIDEO_VP8ProfileMain, V4L2_MPEG_VIDEO_VP8_PROFILE_0},
        });

pl_map profile_level_converter::profile_vp8_v4l2_to_omx ({});

pl_map profile_level_converter::profile_vp9_omx_to_v4l2 ({
            {OMX_VIDEO_VP9Profile0, V4L2_MPEG_VIDEO_VP9_PROFILE_0},
            {OMX_VIDEO_VP9Profile2, V4L2_MPEG_VIDEO_VP9_PROFILE_2},
            {OMX_VIDEO_VP9Profile2HDR, V4L2_MPEG_VIDEO_VP9_PROFILE_2},
            {OMX_VIDEO_VP9Profile2HDR10Plus, V4L2_MPEG_VIDEO_VP9_PROFILE_2},
        });

pl_map profile_level_converter::profile_vp9_v4l2_to_omx ({});

pl_map profile_level_converter::level_avc_omx_to_v4l2 ({
            {OMX_VIDEO_AVCLevel1, V4L2_MPEG_VIDEO_H264_LEVEL_1_0},
            {OMX_VIDEO_AVCLevel11, V4L2_MPEG_VIDEO_H264_LEVEL_1_1},
            {OMX_VIDEO_AVCLevel12, V4L2_MPEG_VIDEO_H264_LEVEL_1_2},
            {OMX_VIDEO_AVCLevel13, V4L2_MPEG_VIDEO_H264_LEVEL_1_3},
            {OMX_VIDEO_AVCLevel1b, V4L2_MPEG_VIDEO_H264_LEVEL_1B},
            {OMX_VIDEO_AVCLevel2, V4L2_MPEG_VIDEO_H264_LEVEL_2_0},
            {OMX_VIDEO_AVCLevel21, V4L2_MPEG_VIDEO_H264_LEVEL_2_1},
            {OMX_VIDEO_AVCLevel22, V4L2_MPEG_VIDEO_H264_LEVEL_2_2},
            {OMX_VIDEO_AVCLevel3, V4L2_MPEG_VIDEO_H264_LEVEL_3_0},
            {OMX_VIDEO_AVCLevel31, V4L2_MPEG_VIDEO_H264_LEVEL_3_1},
            {OMX_VIDEO_AVCLevel32, V4L2_MPEG_VIDEO_H264_LEVEL_3_2},
            {OMX_VIDEO_AVCLevel4, V4L2_MPEG_VIDEO_H264_LEVEL_4_0},
            {OMX_VIDEO_AVCLevel41, V4L2_MPEG_VIDEO_H264_LEVEL_4_1},
            {OMX_VIDEO_AVCLevel42, V4L2_MPEG_VIDEO_H264_LEVEL_4_2},
            {OMX_VIDEO_AVCLevel5, V4L2_MPEG_VIDEO_H264_LEVEL_5_0},
            {OMX_VIDEO_AVCLevel51, V4L2_MPEG_VIDEO_H264_LEVEL_5_1},
            {OMX_VIDEO_AVCLevel52, V4L2_MPEG_VIDEO_H264_LEVEL_5_2},
            {OMX_VIDEO_AVCLevel6, V4L2_MPEG_VIDEO_H264_LEVEL_6_0},
            {OMX_VIDEO_AVCLevel61, V4L2_MPEG_VIDEO_H264_LEVEL_6_1},
            {OMX_VIDEO_AVCLevel62, V4L2_MPEG_VIDEO_H264_LEVEL_6_2},
        });

pl_map profile_level_converter::level_avc_v4l2_to_omx ({});

pl_map profile_level_converter::level_hevc_omx_to_v4l2 ({
            {OMX_VIDEO_HEVCMainTierLevel1,  V4L2_MPEG_VIDEO_HEVC_LEVEL_1},
            {OMX_VIDEO_HEVCMainTierLevel2,  V4L2_MPEG_VIDEO_HEVC_LEVEL_2},
            {OMX_VIDEO_HEVCMainTierLevel21, V4L2_MPEG_VIDEO_HEVC_LEVEL_2_1},
            {OMX_VIDEO_HEVCMainTierLevel3,  V4L2_MPEG_VIDEO_HEVC_LEVEL_3},
            {OMX_VIDEO_HEVCMainTierLevel31, V4L2_MPEG_VIDEO_HEVC_LEVEL_3_1},
            {OMX_VIDEO_HEVCMainTierLevel4,  V4L2_MPEG_VIDEO_HEVC_LEVEL_4},
            {OMX_VIDEO_HEVCMainTierLevel41, V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1},
            {OMX_VIDEO_HEVCMainTierLevel5,  V4L2_MPEG_VIDEO_HEVC_LEVEL_5},
            {OMX_VIDEO_HEVCMainTierLevel51, V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1},
            {OMX_VIDEO_HEVCMainTierLevel52, V4L2_MPEG_VIDEO_HEVC_LEVEL_5_2},
            {OMX_VIDEO_HEVCMainTierLevel6,  V4L2_MPEG_VIDEO_HEVC_LEVEL_6},
            {OMX_VIDEO_HEVCMainTierLevel61, V4L2_MPEG_VIDEO_HEVC_LEVEL_6_1},
            {OMX_VIDEO_HEVCMainTierLevel62, V4L2_MPEG_VIDEO_HEVC_LEVEL_6_2},
        });

pl_map profile_level_converter::level_hevc_v4l2_to_omx ({});

pl_map profile_level_converter::level_vp8_omx_to_v4l2 ({
            {OMX_VIDEO_VP8Level_Version0, V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_0},
            {OMX_VIDEO_VP8Level_Version1, V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_1},
            {OMX_VIDEO_VP8Level_Version2, V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_2},
            {OMX_VIDEO_VP8Level_Version3, V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_3},
        });

pl_map profile_level_converter::level_vp8_v4l2_to_omx ({});

pl_map profile_level_converter::level_mpeg2_omx_to_v4l2 ({
            {OMX_VIDEO_MPEG2LevelLL, V4L2_MPEG_VIDC_VIDEO_MPEG2_LEVEL_0},
            {OMX_VIDEO_MPEG2LevelML, V4L2_MPEG_VIDC_VIDEO_MPEG2_LEVEL_1},
            {OMX_VIDEO_MPEG2LevelHL, V4L2_MPEG_VIDC_VIDEO_MPEG2_LEVEL_2},
        });

pl_map profile_level_converter::level_mpeg2_v4l2_to_omx ({});

pl_map profile_level_converter::level_vp9_omx_to_v4l2 ({
            {OMX_VIDEO_VP9Level1, V4L2_MPEG_VIDC_VIDEO_VP9_LEVEL_1},
            {OMX_VIDEO_VP9Level11, V4L2_MPEG_VIDC_VIDEO_VP9_LEVEL_11},
            {OMX_VIDEO_VP9Level2, V4L2_MPEG_VIDC_VIDEO_VP9_LEVEL_2},
            {OMX_VIDEO_VP9Level21, V4L2_MPEG_VIDC_VIDEO_VP9_LEVEL_21},
            {OMX_VIDEO_VP9Level3, V4L2_MPEG_VIDC_VIDEO_VP9_LEVEL_3},
            {OMX_VIDEO_VP9Level31, V4L2_MPEG_VIDC_VIDEO_VP9_LEVEL_31},
            {OMX_VIDEO_VP9Level4, V4L2_MPEG_VIDC_VIDEO_VP9_LEVEL_4},
            {OMX_VIDEO_VP9Level41, V4L2_MPEG_VIDC_VIDEO_VP9_LEVEL_41},
            {OMX_VIDEO_VP9Level5, V4L2_MPEG_VIDC_VIDEO_VP9_LEVEL_5},
            {OMX_VIDEO_VP9Level51, V4L2_MPEG_VIDC_VIDEO_VP9_LEVEL_51},
            {OMX_VIDEO_VP9Level6, V4L2_MPEG_VIDC_VIDEO_VP9_LEVEL_6},
            {OMX_VIDEO_VP9Level61, V4L2_MPEG_VIDC_VIDEO_VP9_LEVEL_61},
        });

pl_map profile_level_converter::level_vp9_v4l2_to_omx ({});

codec_map profile_level_converter::profile_omx_to_v4l2_map ({
            {V4L2_PIX_FMT_H264, &profile_avc_omx_to_v4l2},
            {V4L2_PIX_FMT_HEVC, &profile_hevc_omx_to_v4l2},
            {V4L2_PIX_FMT_MPEG2, &profile_mpeg2_omx_to_v4l2},
            {V4L2_PIX_FMT_VP8, &profile_vp8_omx_to_v4l2},
            {V4L2_PIX_FMT_VP9, &profile_vp9_omx_to_v4l2},
        });

codec_map profile_level_converter::profile_v4l2_to_omx_map ({
            {V4L2_PIX_FMT_H264, &profile_avc_v4l2_to_omx},
            {V4L2_PIX_FMT_HEVC, &profile_hevc_v4l2_to_omx},
            {V4L2_PIX_FMT_MPEG2, &profile_mpeg2_v4l2_to_omx},
            {V4L2_PIX_FMT_VP8, &profile_vp8_v4l2_to_omx},
            {V4L2_PIX_FMT_VP9, &profile_vp9_v4l2_to_omx},
        });

codec_map profile_level_converter::level_omx_to_v4l2_map ({
            {V4L2_PIX_FMT_H264, &level_avc_omx_to_v4l2},
            {V4L2_PIX_FMT_HEVC, &level_hevc_omx_to_v4l2},
            {V4L2_PIX_FMT_MPEG2, &level_mpeg2_omx_to_v4l2},
            {V4L2_PIX_FMT_VP8, &level_vp8_omx_to_v4l2},
            {V4L2_PIX_FMT_VP9, &level_vp9_omx_to_v4l2},
        });

codec_map profile_level_converter::level_v4l2_to_omx_map ({
            {V4L2_PIX_FMT_H264, &level_avc_v4l2_to_omx},
            {V4L2_PIX_FMT_HEVC, &level_hevc_v4l2_to_omx},
            {V4L2_PIX_FMT_MPEG2, &level_mpeg2_v4l2_to_omx},
            {V4L2_PIX_FMT_VP8, &level_vp8_v4l2_to_omx},
            {V4L2_PIX_FMT_VP9, &level_vp9_v4l2_to_omx},
        });

void reverse_map(pl_map source_map, pl_map &dest_map)
{
    pl_map::iterator it;

    for(it = source_map.begin(); it != source_map.end(); it++) {
        dest_map[it->second] = it->first;
    }
    return;
}

void profile_level_converter::init()
{
    reverse_map(profile_avc_omx_to_v4l2, profile_avc_v4l2_to_omx);
    reverse_map(profile_hevc_omx_to_v4l2, profile_hevc_v4l2_to_omx);
    reverse_map(profile_mpeg2_omx_to_v4l2, profile_mpeg2_v4l2_to_omx);
    reverse_map(profile_vp8_omx_to_v4l2, profile_vp8_v4l2_to_omx);
    reverse_map(profile_vp9_omx_to_v4l2, profile_vp9_v4l2_to_omx);
    reverse_map(level_avc_omx_to_v4l2, level_avc_v4l2_to_omx);
    reverse_map(level_hevc_omx_to_v4l2, level_hevc_v4l2_to_omx);
    reverse_map(level_vp8_omx_to_v4l2, level_vp8_v4l2_to_omx);
    reverse_map(level_mpeg2_omx_to_v4l2, level_mpeg2_v4l2_to_omx);
    reverse_map(level_vp9_omx_to_v4l2, level_vp9_v4l2_to_omx);
}

bool profile_level_converter::find_map(const codec_map &map, int key, pl_map **value_map)
{
    auto map_it = map.find (key);
    if (map_it == map.end()) {
        DEBUG_PRINT_ERROR(" Invalid codec : %d Cannot find map for this codec", key);
        return false;
    }
    *value_map = map_it->second;
    return true;
}

bool profile_level_converter::find_item(const pl_map &map, int key, int *value)
{
    auto it = map.find (key);
    if (it == map.end()) {
        DEBUG_PRINT_ERROR(" Invalid key : %d Cannot find key in map ", key);
        return false;
    }
    *value = it->second;
    return true;
}

bool profile_level_converter::convert_v4l2_profile_to_omx(int codec, int v4l2_profile, int *omx_profile)
{
    pl_map *profile_map;

    if (!find_map(profile_v4l2_to_omx_map, codec, &profile_map))
        return false;

    return find_item(*profile_map, v4l2_profile, omx_profile);
}

bool profile_level_converter::convert_omx_profile_to_v4l2(int codec, int omx_profile, int *v4l2_profile)
{
    pl_map *profile_map;

    if (!find_map(profile_omx_to_v4l2_map, codec, &profile_map))
        return false;

    return find_item(*profile_map, omx_profile, v4l2_profile);
}

bool profile_level_converter::convert_v4l2_level_to_omx(int codec, int v4l2_level, int *omx_level)
{
    pl_map *level_map;

    if (!find_map(level_v4l2_to_omx_map, codec, &level_map))
        return false;

    return find_item(*level_map, v4l2_level, omx_level);
}

bool profile_level_converter::find_tier(int codec, int omx_level, unsigned int *tier)
{
    /* Default is HIGH tier */
    *tier = V4L2_MPEG_VIDEO_HEVC_TIER_HIGH;

    if(codec == V4L2_PIX_FMT_HEVC) {
        unsigned int level_main_mask = OMX_VIDEO_HEVCMainTierLevel1  |
                                       OMX_VIDEO_HEVCMainTierLevel2  |
                                       OMX_VIDEO_HEVCMainTierLevel21 |
                                       OMX_VIDEO_HEVCMainTierLevel3  |
                                       OMX_VIDEO_HEVCMainTierLevel31 |
                                       OMX_VIDEO_HEVCMainTierLevel4  |
                                       OMX_VIDEO_HEVCMainTierLevel41 |
                                       OMX_VIDEO_HEVCMainTierLevel5  |
                                       OMX_VIDEO_HEVCMainTierLevel51 |
                                       OMX_VIDEO_HEVCMainTierLevel52 |
                                       OMX_VIDEO_HEVCMainTierLevel6  |
                                       OMX_VIDEO_HEVCMainTierLevel61 |
                                       OMX_VIDEO_HEVCMainTierLevel62;
        unsigned int bit_set = level_main_mask & (unsigned int)omx_level;
        if (bit_set && (bit_set & (bit_set-1)) == 0) {
          *tier = V4L2_MPEG_VIDEO_HEVC_TIER_MAIN;
        }
    }
    return true;
}

bool profile_level_converter::convert_omx_level_to_v4l2(int codec, int omx_level, int *v4l2_level)
{
    pl_map *level_map;

    if (!find_map(level_omx_to_v4l2_map, codec, &level_map))
        return false;

    return find_item(*level_map, omx_level, v4l2_level);
}

void get_gralloc_format_as_string(char * buf, int buf_len, int format) {
    switch (format) {
        case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
            snprintf(buf, buf_len, "HAL_PIXEL_FORMAT_NV12_ENCODEABLE");
            break;
        case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
            snprintf(buf, buf_len, "HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC");
            break;
        case HAL_PIXEL_FORMAT_RGBA_8888:
            snprintf(buf, buf_len, "HAL_PIXEL_FORMAT_RGBA_8888");
            break;
        case QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m:
            snprintf(buf, buf_len, "QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m");
            break;
        case HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC:
            snprintf(buf, buf_len, "HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC");
            break;
        case HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS:
            snprintf(buf, buf_len, "HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS");
            break;
        case QOMX_COLOR_FormatYVU420SemiPlanar:
            snprintf(buf, buf_len, "QOMX_COLOR_FormatYVU420SemiPlanar");
            break;
        default:
            snprintf(buf, buf_len, "no match found for gralloc format 0x%d",
                    format);
            return;
    }
}

void get_v4l2_color_format_as_string(char * buf, int buf_len, unsigned long v4l2Pixformat) {
    switch (v4l2Pixformat) {
        case V4L2_PIX_FMT_RGB32:
            snprintf(buf, buf_len, "V4L2_PIX_FMT_RGB32");
            break;
        case V4L2_PIX_FMT_RGBA8888_UBWC:
            snprintf(buf, buf_len, "V4L2_PIX_FMT_RGBA8888_UBWC");
            break;
        case V4L2_PIX_FMT_NV12:
            snprintf(buf, buf_len, "V4L2_PIX_FMT_NV12");
            break;
        case V4L2_PIX_FMT_NV12_512:
            snprintf(buf, buf_len, "V4L2_PIX_FMT_NV12_512");
            break;
        case V4L2_PIX_FMT_NV12_UBWC:
            snprintf(buf, buf_len, "V4L2_PIX_FMT_NV12_UBWC");
            break;
        case V4L2_PIX_FMT_NV12_TP10_UBWC:
            snprintf(buf, buf_len, "V4L2_PIX_FMT_NV12_TP10_UBWC");
            break;
        case V4L2_PIX_FMT_SDE_Y_CBCR_H2V2_P010_VENUS:
            snprintf(buf, buf_len, "V4L2_PIX_FMT_SDE_Y_CBCR_H2V2_P010_VENUS");
            break;
        case V4L2_PIX_FMT_NV21:
            snprintf(buf, buf_len, "V4L2_PIX_FMT_NV21");
            break;
        case V4L2_PIX_FMT_NV12_P010_UBWC:
            snprintf(buf, buf_len, "V4L2_PIX_FMT_NV12_P010_UBWC");
            break;
        default:
            snprintf(buf, buf_len, "no match found for v4l2 pix format 0x%lx",
                    v4l2Pixformat);
            return;
    }
}

IvfFileHeader:: IvfFileHeader() :
    signature{'D','K','I','F'},
    version(),
    size(32),
    fourCC{'V','P','8','0'},
    width(0),
    height(0),
    rate(0),
    scale(0),
    frameCount(0),
    unused()
{
}

IvfFileHeader:: IvfFileHeader(bool isVp9, int width, int height,
                    int rate, int scale, int frameCount) :
    IvfFileHeader() {
    this->width = width;
    this->height = height;
    this->rate = rate;
    this->scale = scale;
    this->frameCount = frameCount;
    fourCC[2] = isVp9 ? '9' : '8';
}

IvfFrameHeader:: IvfFrameHeader(): filledLen(), timeStamp() {}

IvfFrameHeader:: IvfFrameHeader(uint32_t filledLen, uint64_t timeStamp) :
    filledLen(filledLen),
    timeStamp(timeStamp) {
}

void do_sync_ioctl(int fd, struct dma_buf_sync* sync) {
#ifdef USE_ION
    int rc = ioctl(fd, DMA_BUF_IOCTL_SYNC, sync);
    if (rc < 0) {
        DEBUG_PRINT_ERROR("Failed DMA_BUF_IOCTL_SYNC");
        return;
    }
#else
    (void)fd, (void)sync;
#endif
}
