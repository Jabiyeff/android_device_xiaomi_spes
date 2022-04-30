/*--------------------------------------------------------------------------
Copyright (c) 2013 - 2020, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of The Linux Foundation nor
      the names of its contributors may be used to endorse or promote
      products derived from this software without specific prior written
      permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------*/

#ifndef __VIDC_DEBUG_H__
#define __VIDC_DEBUG_H__

#ifdef _ANDROID_

#include <cstdio>
#include <string.h>
#include <pthread.h>
#include <sys/mman.h>
extern "C" {
#include <utils/Log.h>
}

#include <linux/videodev2.h>
#include "OMX_Core.h"
#include "vidc_common.h"
#include <color_metadata.h>
#define STRINGIFY_ENUMS
#include "media/hardware/VideoAPI.h"
#include "media/msm_vidc_utils.h"

using android::ColorAspects;
using android::HDRStaticInfo;

enum {
   PRIO_ERROR=0x1,
   PRIO_INFO=0x1,
   PRIO_HIGH=0x2,
   PRIO_LOW=0x4,
   PRIO_TRACE_HIGH = 0x10,
   PRIO_TRACE_LOW = 0x20,
};

extern int debug_level;

#undef DEBUG_PRINT_ERROR
#define DEBUG_PRINT_ERROR(fmt, args...) ({ \
      if (debug_level & PRIO_ERROR) \
          ALOGE(fmt,##args); \
      })
#undef DEBUG_PRINT_INFO
#define DEBUG_PRINT_INFO(fmt, args...) ({ \
      if (debug_level & PRIO_INFO) \
          ALOGI(fmt,##args); \
      })
#undef DEBUG_PRINT_LOW
#define DEBUG_PRINT_LOW(fmt, args...) ({ \
      if (debug_level & PRIO_LOW) \
          ALOGD(fmt,##args); \
      })
#undef DEBUG_PRINT_HIGH
#define DEBUG_PRINT_HIGH(fmt, args...) ({ \
      if (debug_level & PRIO_HIGH) \
          ALOGD(fmt,##args); \
      })
#else
#define DEBUG_PRINT_ERROR printf
#define DEBUG_PRINT_INFO printf
#define DEBUG_PRINT_LOW printf
#define DEBUG_PRINT_HIGH printf
#endif

struct debug_cap {
    bool in_buffer_log;
    bool out_buffer_log;
    bool out_cc_buffer_log;
    bool out_meta_buffer_log;
    char infile_name[PROPERTY_VALUE_MAX + 36];
    char outfile_name[PROPERTY_VALUE_MAX + 36];
    char ccoutfile_name[PROPERTY_VALUE_MAX + 36];
    char out_ymetafile_name[PROPERTY_VALUE_MAX + 36];
    char out_uvmetafile_name[PROPERTY_VALUE_MAX + 36];
    char log_loc[PROPERTY_VALUE_MAX];
    FILE *infile;
    FILE *outfile;
    FILE *ccoutfile;
    FILE *out_ymeta_file;
    FILE *out_uvmeta_file;
    int64_t session_id;
    int seq_count;
};

#define VALIDATE_OMX_PARAM_DATA(ptr, paramType)                                \
    {                                                                          \
        if (ptr == NULL) { return OMX_ErrorBadParameter; }                     \
        paramType *p = reinterpret_cast<paramType *>(ptr);                     \
        if (p->nSize < sizeof(paramType)) {                                    \
            ALOGE("Insufficient object size(%u) v/s expected(%zu) for type %s",\
                    (unsigned int)p->nSize, sizeof(paramType), #paramType);    \
            return OMX_ErrorBadParameter;                                      \
        }                                                                      \
    }                                                                          \

/*
 * Validate OMX_CONFIG_ANDROID_VENDOR_EXTENSIONTYPE type param
 * *assumes* VALIDATE_OMX_PARAM_DATA checks have passed
 * Checks for nParamCount cannot be generalized here. it is imperative that
 *  the calling code handles it.
 */
#define VALIDATE_OMX_VENDOR_EXTENSION_PARAM_DATA(ext)                                             \
    {                                                                                             \
        if (ext->nParamSizeUsed < 1 || ext->nParamSizeUsed > OMX_MAX_ANDROID_VENDOR_PARAMCOUNT) { \
            ALOGE("VendorExtension: sub-params(%u) not in expected range(%u - %u)",               \
                    ext->nParamSizeUsed, 1, OMX_MAX_ANDROID_VENDOR_PARAMCOUNT);                   \
            return OMX_ErrorBadParameter;                                                         \
        }                                                                                         \
        OMX_U32 expectedSize = (OMX_U32)sizeof(OMX_CONFIG_ANDROID_VENDOR_EXTENSIONTYPE) +         \
                ((ext->nParamSizeUsed - 1) * (OMX_U32)sizeof(OMX_CONFIG_ANDROID_VENDOR_PARAMTYPE));\
        if (ext->nSize < expectedSize) {                                                          \
            ALOGE("VendorExtension: Insifficient size(%u) v/s expected(%u)",                      \
                    ext->nSize, expectedSize);                                                    \
            return OMX_ErrorBadParameter;                                                         \
        }                                                                                         \
    }                                                                                             \

void print_debug_color_aspects(ColorAspects *a, const char *prefix);
void print_debug_hdr_color_info(HDRStaticInfo *hdr_info, const char *prefix);
void print_debug_hdr_color_info_mdata(ColorMetaData* color_mdata);
void print_debug_hdr10plus_metadata(ColorMetaData& color_mdata);

static inline void print_omx_buffer(const char *str, OMX_BUFFERHEADERTYPE *pHeader)
{
    if (!pHeader)
        return;

    DEBUG_PRINT_HIGH("%s: Header %p buffer %p alloclen %d offset %d filledlen %d timestamp %lld flags %#x",
        str, pHeader, pHeader->pBuffer, pHeader->nAllocLen,
        pHeader->nOffset, pHeader->nFilledLen,
        pHeader->nTimeStamp, pHeader->nFlags);
}

static inline void print_v4l2_buffer(const char *str, struct v4l2_buffer *v4l2)
{
    if (!v4l2)
        return;

    if (v4l2->length == 1)
        DEBUG_PRINT_HIGH(
            "%s: %s: idx %2d userptr %#lx fd %d off %d size %d filled %d flags %#x\n",
            str, v4l2->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ?
            "OUTPUT" : "CAPTURE", v4l2->index,
            v4l2->m.planes[0].m.userptr, v4l2->m.planes[0].reserved[MSM_VIDC_BUFFER_FD],
            v4l2->m.planes[0].reserved[MSM_VIDC_DATA_OFFSET], v4l2->m.planes[0].length,
            v4l2->m.planes[0].bytesused, v4l2->flags);
    else
        DEBUG_PRINT_HIGH(
            "%s: %s: idx %2d userptr %#lx fd %d off %d size %d filled %d flags %#x, extradata: fd %d off %d size %d filled %d\n",
            str, v4l2->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ?
            "OUTPUT" : "CAPTURE", v4l2->index,
            v4l2->m.planes[0].m.userptr, v4l2->m.planes[0].reserved[MSM_VIDC_BUFFER_FD],
            v4l2->m.planes[0].reserved[MSM_VIDC_DATA_OFFSET], v4l2->m.planes[0].length,
            v4l2->m.planes[0].bytesused, v4l2->flags, v4l2->m.planes[1].reserved[MSM_VIDC_BUFFER_FD],
            v4l2->m.planes[1].reserved[MSM_VIDC_DATA_OFFSET], v4l2->m.planes[1].length,
            v4l2->m.planes[1].bytesused);
}

class auto_lock {
    public:
        auto_lock(pthread_mutex_t &lock)
            : mLock(lock) {
                pthread_mutex_lock(&mLock);
            }
        ~auto_lock() {
            pthread_mutex_unlock(&mLock);
        }
    private:
        pthread_mutex_t &mLock;
};

class Signal {
    bool signalled;
    pthread_mutex_t mutex;
    pthread_cond_t condition;
public:
    Signal() {
        pthread_condattr_t attr;
        signalled = false;
        pthread_condattr_init(&attr);
        pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
        pthread_cond_init(&condition, &attr);
        pthread_mutex_init(&mutex, NULL);
    }

    ~Signal() {
            pthread_cond_destroy(&condition);
            pthread_mutex_destroy(&mutex);
    }

    void signal() {
        pthread_mutex_lock(&mutex);
        signalled = true;
        pthread_cond_signal(&condition);
        pthread_mutex_unlock(&mutex);
    }

    int wait(uint64_t timeout_nsec) {
        struct timespec ts;

        pthread_mutex_lock(&mutex);
        if (signalled) {
            signalled = false;
            pthread_mutex_unlock(&mutex);
            return 0;
        }
        clock_gettime(CLOCK_MONOTONIC, &ts);
        ts.tv_sec += timeout_nsec / 1000000000;
        ts.tv_nsec += timeout_nsec % 1000000000;
        if (ts.tv_nsec >= 1000000000) {
            ts.tv_nsec -= 1000000000;
            ts.tv_sec  += 1;
        }
        int ret = pthread_cond_timedwait(&condition, &mutex, &ts);
        //as the mutex lock is released inside timedwait first
        //the singalled variant maybe changed by the main thread in some rare cases
        //meanwhile still returns wait time out
        //need to double check it and return 0 to process the last cmd/event during time out
        if (signalled)
            ret = 0;
        signalled = false;
        pthread_mutex_unlock(&mutex);
        return ret;
    }
};

#ifdef _ANDROID_
#define ATRACE_TAG ATRACE_TAG_VIDEO
#include <cutils/trace.h>
#include <utils/Trace.h>

class AutoTracer {
    int mPrio;
public:
    AutoTracer(int prio, const char* msg)
        : mPrio(prio) {
        if (debug_level & prio) {
            ATRACE_BEGIN(msg);
        }
    }
    ~AutoTracer() {
        if (debug_level & mPrio) {
            ATRACE_END();
        }
    }
};

struct __attribute__((packed)) IvfFileHeader {
    uint8_t signature[4];
    uint16_t version;
    uint16_t size;
    uint8_t fourCC[4];
    uint16_t width;
    uint16_t height;
    uint32_t rate;
    uint32_t scale;
    uint32_t frameCount;
    uint32_t unused;

    IvfFileHeader();
    IvfFileHeader(bool isVp9, int width, int height,
                int rate, int scale, int nFrameCount);
};

struct __attribute__((packed)) IvfFrameHeader {
    uint32_t filledLen;
    uint64_t timeStamp;

    IvfFrameHeader();
    IvfFrameHeader(uint32_t size, uint64_t timeStamp);
};

#define VIDC_TRACE_NAME_LOW(_name) AutoTracer _tracer(PRIO_TRACE_LOW, _name);
#define VIDC_TRACE_NAME_HIGH(_name) AutoTracer _tracer(PRIO_TRACE_HIGH, _name);

#define VIDC_TRACE_INT_LOW(_name, _int) \
    if (debug_level & PRIO_TRACE_LOW) { \
        ATRACE_INT(_name, _int);        \
    }

#define VIDC_TRACE_INT_HIGH(_name, _int) \
    if (debug_level & PRIO_TRACE_HIGH) { \
        ATRACE_INT(_name, _int);        \
    }

#else // _ANDROID_

#define VIDC_TRACE_NAME_LOW(_name)
#define VIDC_TRACE_NAME_HIGH(_name)
#define VIDC_TRACE_INT_LOW(_name, _int)
#define VIDC_TRACE_INT_HIGH(_name, _int)

#endif // !_ANDROID_

#endif
