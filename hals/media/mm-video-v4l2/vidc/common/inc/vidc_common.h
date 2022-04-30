/*--------------------------------------------------------------------------
Copyright (c) 2017, The Linux Foundation. All rights reserved.

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

#ifndef __VIDC_COMMON_H__
#define __VIDC_COMMON_H__

#include <unordered_map>

#include "OMX_QCOMExtns.h"
#include <linux/dma-buf.h>
#ifdef _ANDROID_
#include <gralloc_priv.h>
#endif
#ifdef _ANDROID_
#include <cutils/properties.h>
#else
#define PROPERTY_VALUE_MAX 92
#endif

// BitMask Management logic
#define BITS_PER_INDEX                          64
#define BITMASK_FLAG(mIndex)                    ((uint64_t)1 \
    << ((mIndex) % BITS_PER_INDEX))
#define BITMASK_CLEAR(pBufferMask_64,mIndex)    ((*pBufferMask_64) \
    &= ~(BITMASK_FLAG(mIndex)))
#define BITMASK_SET(pBufferMask_64,mIndex)      ((*pBufferMask_64) \
    |= BITMASK_FLAG(mIndex))
#define BITMASK_PRESENT(pBufferMask_64,mIndex)  ((*pBufferMask_64) \
    & BITMASK_FLAG(mIndex))
#define BITMASK_ABSENT(pBufferMask_64,mIndex)   (((*pBufferMask_64) \
    & BITMASK_FLAG(mIndex)) == 0x0)

using pl_map = std::unordered_map<int, int>;
using codec_map = std::unordered_map<int, pl_map *>;

class profile_level_converter {
    static pl_map        profile_avc_omx_to_v4l2;
    static pl_map        profile_hevc_omx_to_v4l2;
    static pl_map        profile_mpeg2_omx_to_v4l2;
    static pl_map        profile_vp8_omx_to_v4l2;
    static pl_map        profile_vp9_omx_to_v4l2;
    static pl_map        level_avc_omx_to_v4l2;
    static pl_map        level_hevc_omx_to_v4l2;
    static pl_map        level_vp8_omx_to_v4l2;
    static pl_map        level_mpeg2_omx_to_v4l2;
    static pl_map        level_vp9_omx_to_v4l2;
    static pl_map        profile_avc_v4l2_to_omx;
    static pl_map        profile_hevc_v4l2_to_omx;
    static pl_map        profile_mpeg2_v4l2_to_omx;
    static pl_map        profile_vp9_v4l2_to_omx;
    static pl_map        level_avc_v4l2_to_omx;
    static pl_map        level_hevc_v4l2_to_omx;
    static pl_map        level_vp8_v4l2_to_omx;
    static pl_map        level_mpeg2_v4l2_to_omx;
    static pl_map        profile_vp8_v4l2_to_omx;
    static pl_map        level_vp9_v4l2_to_omx;
    static codec_map     profile_omx_to_v4l2_map;
    static codec_map     profile_v4l2_to_omx_map;
    static codec_map     level_omx_to_v4l2_map;
    static codec_map     level_v4l2_to_omx_map;

    //Constructor that initializes and performs the mapping
    profile_level_converter() = delete;
    static bool find_item(const pl_map &map, int key, int *value);
    static bool find_map(const codec_map &map, int key, pl_map **value_map);

    public:
    static void init();
    static bool convert_v4l2_profile_to_omx(int codec, int v4l2_profile, int *omx_profile);
    static bool convert_omx_profile_to_v4l2(int codec, int omx_profile, int *v4l2_profile);
    static bool convert_v4l2_level_to_omx(int codec, int v4l2_level, int *omx_level);
    static bool find_tier(int codec, int omx_level, unsigned int *tire);
    static bool convert_omx_level_to_v4l2(int codec, int omx_level, int *v4l2_level);
};

void get_gralloc_format_as_string(char * buf, int buf_len, int format);
void get_v4l2_color_format_as_string(char * buf, int buf_len, unsigned long v4l2Pixformat);

void do_sync_ioctl(int fd, struct dma_buf_sync* sync);

static inline void sync_start_write(int fd) {
    struct dma_buf_sync sync = {0};
    sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE;
    do_sync_ioctl(fd, &sync);
}

static inline void sync_end_write(int fd) {
    struct dma_buf_sync sync = {0};
    sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE;
    do_sync_ioctl(fd, &sync);
}

static inline void sync_start_read(int fd) {
    struct dma_buf_sync sync = {0};
    sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
    do_sync_ioctl(fd, &sync);
}

static inline void sync_end_read(int fd) {
    struct dma_buf_sync sync = {0};
    sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
    do_sync_ioctl(fd, &sync);
}

static inline void sync_start_rw(int fd) {
    struct dma_buf_sync sync = {0};
    sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;
    do_sync_ioctl(fd, &sync);
}

static inline void sync_end_rw(int fd) {
    struct dma_buf_sync sync = {0};
    sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;
    do_sync_ioctl(fd, &sync);
}

static inline void cache_clean(int fd) {
    sync_start_write(fd);
    sync_end_write(fd);
}

static inline void cache_invalidate(int fd) {
    sync_start_write(fd);
    sync_end_read(fd);
}

static inline void cache_clean_invalidate(int fd) {
    sync_start_rw(fd);
    sync_end_rw(fd);
}
#endif // __VIDC_COMMON_H__
