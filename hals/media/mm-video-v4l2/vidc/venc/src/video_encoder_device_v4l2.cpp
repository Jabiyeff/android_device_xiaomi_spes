/*--------------------------------------------------------------------------
Copyright (c) 2010-2020 The Linux Foundation. All rights reserved.

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

#include <string.h>
#include <sys/ioctl.h>
#include <sys/prctl.h>
#include <sys/eventfd.h>
#include <unistd.h>
#include <fcntl.h>
#include "video_encoder_device_v4l2.h"
#include "omx_video_encoder.h"
#include "media/msm_vidc_utils.h"
#ifdef USE_ION
#include <linux/msm_ion.h>
#endif
#include<linux/v4l2-controls.h>

#include <math.h>
#include <media/msm_media_info.h>
#include <cutils/properties.h>
#include <media/hardware/HardwareAPI.h>

#ifdef _ANDROID_
#include <media/hardware/HardwareAPI.h>
#include <gralloc_priv.h>
#endif

#ifdef _USE_GLIB_
#include <glib.h>
#define strlcpy g_strlcpy
#endif

#include <qdMetaData.h>
#include <color_metadata.h>
#include "PlatformConfig.h"

#define ATRACE_TAG ATRACE_TAG_VIDEO
#include <utils/Trace.h>

#define YUV_STATS_LIBRARY_NAME "libgpustats.so" // UBWC case: use GPU library

#undef ALIGN
#define ALIGN(x, to_align) ((((unsigned long) x) + (to_align - 1)) & ~(to_align - 1))
#define EXTRADATA_IDX(__num_planes) ((__num_planes) ? (__num_planes) - 1 : 0)
#define MAXDPB 16
#ifndef MIN
#define MIN(x,y) (((x) < (y)) ? (x) : (y))
#endif
#ifndef MAX
#define MAX(x,y) (((x) > (y)) ? (x) : (y))
#endif

#define ROUND(__sz, __align) (((__sz) + ((__align>>1))) & (~(__align-1)))
#define MAX_PROFILE_PARAMS 6
#define HEVC_MAIN_START 0
#define HEVC_MAIN10_START (HEVC_MAIN_START + 13)
#define POLL_TIMEOUT 1000
#define MAX_SUPPORTED_SLICES_PER_FRAME 28 /* Max supported slices with 32 output buffers */

#define SZ_4K 0x1000
#define SZ_1M 0x100000


#define Log2(number, power)  { OMX_U32 temp = number; power = 0; while( (0 == (temp & 0x1)) &&  power < 16) { temp >>=0x1; power++; } }
#define Q16ToFraction(q,num,den) { OMX_U32 power; Log2(q,power);  num = q >> power; den = 0x1 << (16 - power); }

//TODO: remove once gerrit : 2680400 is merged
#define VIDC_HAL_PIXEL_FORMAT_NV12_LINEAR_FLEX  HAL_PIXEL_FORMAT_NV12_LINEAR_FLEX
#define VIDC_HAL_PIXEL_FORMAT_NV12_UBWC_FLEX HAL_PIXEL_FORMAT_NV12_UBWC_FLEX

#define BUFFER_LOG_LOC "/data/vendor/media"

#undef LOG_TAG
#define LOG_TAG "OMX-VENC: venc_dev"

#define LUMINANCE_MULTIPLICATION_FACTOR 10000

//constructor
venc_dev::venc_dev(class omx_venc *venc_class)
{
    //nothing to do
    int i = 0;
    venc_handle = venc_class;
    etb = ebd = ftb = fbd = 0;
    m_poll_efd = -1;

    struct v4l2_control control;
    for (i = 0; i < MAX_PORT; i++)
        streaming[i] = false;

    stopped = 1;
    paused = false;
    async_thread_created = false;
    async_thread_force_stop = false;
    color_format = 0;
    hw_overload = false;
    mBatchSize = 0;
    m_roi_enabled = false;
    m_roi_type = ROI_NONE;
    m_cvp_meta_enabled = false;
    m_cvp_first_metadata = false;
    low_latency_mode = false;
    m_bDimensionsNeedFlip = false;
    pthread_mutex_init(&m_roilock, NULL);
    pthread_mutex_init(&m_configlock, NULL);
    pthread_mutex_init(&pause_resume_mlock, NULL);
    pthread_cond_init(&pause_resume_cond, NULL);
    memset(&input_extradata_info, 0, sizeof(input_extradata_info));
    memset(&output_extradata_info, 0, sizeof(output_extradata_info));
    for (int i = 0; i < VIDEO_MAX_FRAME; i++) {
        input_extradata_info.ion[i].data_fd = -1;
        output_extradata_info.ion[i].data_fd = -1;
    }
    memset(&idrperiod, 0, sizeof(idrperiod));
    memset(&multislice, 0, sizeof(multislice));
    memset(&m_sVenc_cfg, 0, sizeof(m_sVenc_cfg));
    memset(&rate_ctrl, 0, sizeof(rate_ctrl));
    memset(&bitrate, 0, sizeof(bitrate));
    memset(&intra_period, 0, sizeof(intra_period));
    memset(&codec_profile, 0, sizeof(codec_profile));
    memset(&set_param, 0, sizeof(set_param));
    memset(&time_inc, 0, sizeof(time_inc));
    memset(&m_sInput_buff_property, 0, sizeof(m_sInput_buff_property));
    memset(&m_sOutput_buff_property, 0, sizeof(m_sOutput_buff_property));
    memset(&session_qp, 0, sizeof(session_qp));
    memset(&session_ipb_qp_values, 0, sizeof(session_ipb_qp_values));
    memset(&entropy, 0, sizeof(entropy));
    memset(&dbkfilter, 0, sizeof(dbkfilter));
    memset(&intra_refresh, 0, sizeof(intra_refresh));
    memset(&hec, 0, sizeof(hec));
    memset(&voptimecfg, 0, sizeof(voptimecfg));
    memset(&capability, 0, sizeof(capability));
    memset(&m_debug,0,sizeof(m_debug));
    sess_priority.priority = 1;
    operating_rate = 30;
    memset(&color_space, 0x0, sizeof(color_space));
    memset(&temporal_layers_config, 0x0, sizeof(temporal_layers_config));
    bframe_implicitly_enabled = false;
    intra_period.num_pframes = 29;
    intra_period.num_bframes = 0;
    m_hdr10meta_enabled = false;

    Platform::Config::getInt32(Platform::vidc_enc_log_in,
            (int32_t *)&m_debug.in_buffer_log, 0);
    Platform::Config::getInt32(Platform::vidc_enc_log_out,
            (int32_t *)&m_debug.out_buffer_log, 0);
    Platform::Config::getInt32(Platform::vidc_enc_csc_custom_matrix,
            (int32_t *)&is_csc_custom_matrix_enabled, 0);
    Platform::Config::getInt32(Platform::vidc_enc_auto_blur_disable,
            (int32_t *)&is_auto_blur_disabled, 0);
    Platform::Config::getInt32(Platform::vidc_disable_hdr,
            (int32_t *)&m_disable_hdr, 0);

    char property_value[PROPERTY_VALUE_MAX] = {0};

    property_get("vendor.vidc.enc.log.in", property_value, "0");
    m_debug.in_buffer_log |= atoi(property_value);

    property_value[0] = '\0';
    property_get("vendor.vidc.enc.log.out", property_value, "0");
    m_debug.out_buffer_log |= atoi(property_value);

    property_get("vendor.vidc.enc.log.extradata", property_value, "0");
    m_debug.extradata_log = atoi(property_value);

    property_get("vendor.vidc.cvp.log.in", property_value, "0");
    m_debug.cvp_log |= atoi(property_value);

#ifdef _UBWC_
    property_get("vendor.gralloc.disable_ubwc", property_value, "0");
    if(!(strncmp(property_value, "1", PROPERTY_VALUE_MAX)) ||
        !(strncmp(property_value, "true", PROPERTY_VALUE_MAX))) {
        is_gralloc_source_ubwc = 0;
    } else {
        is_gralloc_source_ubwc = 1;
    }
#else
    is_gralloc_source_ubwc = 0;
#endif

     property_value[0] = '\0';
     property_get("vendor.vidc.log.loc", property_value, BUFFER_LOG_LOC);
     if (*property_value)
         strlcpy(m_debug.log_loc, property_value, PROPERTY_VALUE_MAX);

    mUseAVTimerTimestamps = false;
    mIsGridset = false;
    Platform::Config::getInt32(Platform::vidc_enc_linear_color_format,
            (int32_t *)&mUseLinearColorFormat, 0);
    Platform::Config::getInt32(Platform::vidc_enc_bitrate_savings_enable,
            (int32_t *)&mBitrateSavingsEnable, 3);
    Platform::Config::getInt32(Platform::vidc_enc_quality_boost_enable,
            (int32_t *)&mQualityBoostRequested, 0);
    mQualityBoostEligible = false;

    profile_level_converter::init();
}

venc_dev::~venc_dev()
{
    if (m_roi_enabled) {
        std::list<roidata>::iterator iter;
        pthread_mutex_lock(&m_roilock);
        for (iter = m_roilist.begin(); iter != m_roilist.end(); iter++) {
            DEBUG_PRINT_HIGH("roidata with timestamp (%lld) should have been removed already",
                iter->info.nTimeStamp);
        }
        m_roilist.clear();
        mRoiRegionList.clear();
        pthread_mutex_unlock(&m_roilock);
    }
    pthread_mutex_destroy(&m_roilock);

    std::list<dynamicConfig>::iterator iter;
    pthread_mutex_lock(&m_configlock);
    for (iter = m_configlist.begin(); iter != m_configlist.end(); iter++) {
        DEBUG_PRINT_HIGH("dynamic config data with timestamp (%lld) should have been removed already",
            iter->timestamp);
    }
    m_configlist.clear();
    pthread_mutex_unlock(&m_configlock);
    pthread_mutex_destroy(&m_configlock);
}

void* venc_dev::async_venc_message_thread (void *input)
{
    struct venc_msg venc_msg;
    omx_video* omx_venc_base = NULL;
    omx_venc *omx = reinterpret_cast<omx_venc*>(input);
    omx_venc_base = reinterpret_cast<omx_video*>(input);
    OMX_BUFFERHEADERTYPE* omxhdr = NULL;

    prctl(PR_SET_NAME, (unsigned long)"VideoEncCallBackThread", 0, 0, 0);
    struct v4l2_plane plane[VIDEO_MAX_PLANES];
    struct pollfd pfds[2];
    struct v4l2_buffer v4l2_buf;
    struct v4l2_event dqevent;
    struct statistics stats;
    pfds[0].events = POLLIN | POLLRDNORM | POLLOUT | POLLWRNORM | POLLRDBAND | POLLPRI;
    pfds[1].events = POLLIN | POLLERR;
    pfds[0].fd = omx->handle->m_nDriver_fd;
    pfds[1].fd = omx->handle->m_poll_efd;
    int error_code = 0,rc=0;

    memset(&stats, 0, sizeof(statistics));
    memset(&v4l2_buf, 0, sizeof(v4l2_buf));

    while (!omx->handle->async_thread_force_stop) {
        pthread_mutex_lock(&omx->handle->pause_resume_mlock);

        if (omx->handle->paused) {
            venc_msg.msgcode = VEN_MSG_PAUSE;
            venc_msg.statuscode = VEN_S_SUCCESS;

            if (omx->async_message_process(input, &venc_msg) < 0) {
                DEBUG_PRINT_ERROR("ERROR: Failed to process pause msg");
                pthread_mutex_unlock(&omx->handle->pause_resume_mlock);
                break;
            }

            /* Block here until the IL client resumes us again */
            pthread_cond_wait(&omx->handle->pause_resume_cond,
                    &omx->handle->pause_resume_mlock);

            venc_msg.msgcode = VEN_MSG_RESUME;
            venc_msg.statuscode = VEN_S_SUCCESS;

            if (omx->async_message_process(input, &venc_msg) < 0) {
                DEBUG_PRINT_ERROR("ERROR: Failed to process resume msg");
                pthread_mutex_unlock(&omx->handle->pause_resume_mlock);
                break;
            }
            memset(&stats, 0, sizeof(statistics));
        }

        pthread_mutex_unlock(&omx->handle->pause_resume_mlock);

        rc = poll(pfds, 2, POLL_TIMEOUT);

        if (!rc) {
            DEBUG_PRINT_HIGH("Poll timedout, pipeline stalled due to client/firmware ETB: %d, EBD: %d, FTB: %d, FBD: %d",
                    omx->handle->etb, omx->handle->ebd, omx->handle->ftb, omx->handle->fbd);
            continue;
        } else if (rc < 0 && errno != EINTR && errno != EAGAIN) {
            DEBUG_PRINT_ERROR("Error while polling: %d, errno = %d", rc, errno);
            break;
        }

        if ((pfds[1].revents & POLLIN) || (pfds[1].revents & POLLERR)) {
            DEBUG_PRINT_ERROR("async_venc_message_thread interrupted to be exited");
            break;
        }

        if ((pfds[0].revents & POLLIN) || (pfds[0].revents & POLLRDNORM)) {
            v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            v4l2_buf.memory = V4L2_MEMORY_USERPTR;
            v4l2_buf.length = omx->handle->num_output_planes;
            v4l2_buf.m.planes = plane;

            while (!ioctl(pfds[0].fd, VIDIOC_DQBUF, &v4l2_buf)) {
                venc_msg.msgcode=VEN_MSG_OUTPUT_BUFFER_DONE;
                venc_msg.statuscode=VEN_S_SUCCESS;
                omxhdr=omx_venc_base->m_out_mem_ptr+v4l2_buf.index;
                venc_msg.buf.len= v4l2_buf.m.planes->bytesused;
                venc_msg.buf.offset = v4l2_buf.m.planes->data_offset;
                venc_msg.buf.flags = 0;
                venc_msg.buf.ptrbuffer = (OMX_U8 *)omx_venc_base->m_pOutput_pmem[v4l2_buf.index].buffer;
                venc_msg.buf.clientdata=(void*)omxhdr;
                venc_msg.buf.timestamp = (int64_t) v4l2_buf.timestamp.tv_sec * (int64_t) 1000000 + (int64_t) v4l2_buf.timestamp.tv_usec;

                /* TODO: ideally report other types of frames as well
                 * for now it doesn't look like IL client cares about
                 * other types
                 */
                if (v4l2_buf.flags & V4L2_BUF_FLAG_KEYFRAME) {
                    venc_msg.buf.flags |= QOMX_VIDEO_PictureTypeIDR;
                    venc_msg.buf.flags |= OMX_BUFFERFLAG_SYNCFRAME;
                }
                if (v4l2_buf.flags & V4L2_BUF_FLAG_PFRAME) {
                    venc_msg.buf.flags |= OMX_VIDEO_PictureTypeP;
                } else if (v4l2_buf.flags & V4L2_BUF_FLAG_BFRAME) {
                    venc_msg.buf.flags |= OMX_VIDEO_PictureTypeB;
                }

                if (v4l2_buf.flags & V4L2_BUF_FLAG_CODECCONFIG)
                    venc_msg.buf.flags |= OMX_BUFFERFLAG_CODECCONFIG;

                if (v4l2_buf.flags & V4L2_BUF_FLAG_EOS)
                    venc_msg.buf.flags |= OMX_BUFFERFLAG_EOS;

                if (omx->handle->num_output_planes > 1 && v4l2_buf.m.planes->bytesused)
                    venc_msg.buf.flags |= OMX_BUFFERFLAG_EXTRADATA;

                if (omxhdr->nFilledLen)
                    venc_msg.buf.flags |= OMX_BUFFERFLAG_ENDOFFRAME;

                omx->handle->fbd++;
                stats.bytes_generated += venc_msg.buf.len;

                if (omx->async_message_process(input,&venc_msg) < 0) {
                    DEBUG_PRINT_ERROR("ERROR: Wrong ioctl message");
                    break;
                }
            }
        }

        if ((pfds[0].revents & POLLOUT) || (pfds[0].revents & POLLWRNORM)) {
            v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
            v4l2_buf.memory = V4L2_MEMORY_USERPTR;
            v4l2_buf.m.planes = plane;
            v4l2_buf.length = omx->handle->num_input_planes;

            while (!ioctl(pfds[0].fd, VIDIOC_DQBUF, &v4l2_buf)) {
                venc_msg.msgcode=VEN_MSG_INPUT_BUFFER_DONE;
                venc_msg.statuscode=VEN_S_SUCCESS;
                omx->handle->ebd++;

                if (omx->handle->mBatchSize) {
                    int bufIndex = omx->handle->mBatchInfo.retrieveBufferAt(v4l2_buf.index);
                    if (bufIndex < 0) {
                        DEBUG_PRINT_ERROR("Retrieved invalid buffer %d", v4l2_buf.index);
                        break;
                    }
                    if (omx->handle->mBatchInfo.isPending(bufIndex)) {
                        DEBUG_PRINT_LOW(" EBD for %d [v4l2-id=%d].. batch still pending",
                                bufIndex, v4l2_buf.index);
                        //do not return to client yet
                        continue;
                    }
                    v4l2_buf.index = bufIndex;
                }
                if (omx_venc_base->mUseProxyColorFormat && !omx_venc_base->mUsesColorConversion)
                    omxhdr = &omx_venc_base->meta_buffer_hdr[v4l2_buf.index];
                else
                    omxhdr = &omx_venc_base->m_inp_mem_ptr[v4l2_buf.index];

                venc_msg.buf.clientdata=(void*)omxhdr;

                DEBUG_PRINT_LOW("sending EBD %p [id=%d]", omxhdr, v4l2_buf.index);
                if (omx->async_message_process(input,&venc_msg) < 0) {
                    DEBUG_PRINT_ERROR("ERROR: Wrong ioctl message");
                    break;
                }
            }
        }

        if (pfds[0].revents & POLLPRI) {
            rc = ioctl(pfds[0].fd, VIDIOC_DQEVENT, &dqevent);

            if (dqevent.type == V4L2_EVENT_MSM_VIDC_FLUSH_DONE) {
                venc_msg.msgcode = VEN_MSG_FLUSH_INPUT_DONE;
                venc_msg.statuscode = VEN_S_SUCCESS;

                if (omx->async_message_process(input,&venc_msg) < 0) {
                    DEBUG_PRINT_ERROR("ERROR: Wrong ioctl message");
                    break;
                }

                venc_msg.msgcode = VEN_MSG_FLUSH_OUPUT_DONE;
                venc_msg.statuscode = VEN_S_SUCCESS;

                if (omx->async_message_process(input,&venc_msg) < 0) {
                    DEBUG_PRINT_ERROR("ERROR: Wrong ioctl message");
                    break;
                }
            } else if (dqevent.type == V4L2_EVENT_MSM_VIDC_HW_OVERLOAD) {
                DEBUG_PRINT_ERROR("HW Overload received");
                venc_msg.statuscode = VEN_S_EFAIL;
                venc_msg.msgcode = VEN_MSG_HW_OVERLOAD;

                if (omx->async_message_process(input,&venc_msg) < 0) {
                    DEBUG_PRINT_ERROR("ERROR: Wrong ioctl message");
                    break;
                }
            } else if (dqevent.type == V4L2_EVENT_MSM_VIDC_SYS_ERROR){
                DEBUG_PRINT_ERROR("ERROR: Encoder is in bad state");
                venc_msg.msgcode = VEN_MSG_INDICATION;
                venc_msg.statuscode=VEN_S_EFAIL;

                if (omx->async_message_process(input,&venc_msg) < 0) {
                    DEBUG_PRINT_ERROR("ERROR: Wrong ioctl message");
                    break;
                }
            }
        }

        /* calc avg. fps, bitrate */
        OMX_U64 current_time;
        OMX_U64 prev_time;
        struct timespec curr_tv;
        clock_gettime(CLOCK_MONOTONIC, &curr_tv);
        current_time = (OMX_U64)curr_tv.tv_sec * 1000000000ULL + (OMX_U64)curr_tv.tv_nsec;
        prev_time = (OMX_U64)stats.prev_tv.tv_sec * 1000000000ULL + (OMX_U64)stats.prev_tv.tv_nsec;
        if (current_time < prev_time) {
            stats.prev_tv = curr_tv;
            stats.bytes_generated = 0;
            stats.prev_fbd = omx->handle->fbd;
        } else if (current_time - prev_time  >= 1000000000ULL) {    // timestamp is in nano_seconds
            OMX_U32 num_fbd = omx->handle->fbd - stats.prev_fbd;
            OMX_U64 time_diff = current_time - prev_time;
            if (stats.prev_tv.tv_sec && num_fbd && time_diff) {
                float framerate = ((OMX_U64)num_fbd * 1000000000ULL) / (float)time_diff;
                OMX_U64 bitrate = ((OMX_U64)stats.bytes_generated * 8 / (float)num_fbd) * framerate;
                DEBUG_PRINT_INFO("stats: avg. fps %0.2f, bitrate %llu",
                    framerate, bitrate);
            }
            stats.prev_tv = curr_tv;
            stats.bytes_generated = 0;
            stats.prev_fbd = omx->handle->fbd;
        }

    }

    DEBUG_PRINT_HIGH("omx_venc: Async Thread exit");
    return NULL;
}

static const int event_type[] = {
    V4L2_EVENT_MSM_VIDC_FLUSH_DONE,
    V4L2_EVENT_MSM_VIDC_SYS_ERROR
};

static OMX_ERRORTYPE subscribe_to_events(int fd)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    struct v4l2_event_subscription sub;
    int array_sz = sizeof(event_type)/sizeof(int);
    int i,rc;
    memset(&sub, 0, sizeof(sub));

    if (fd < 0) {
       DEBUG_PRINT_ERROR("Invalid input: %d", fd);
        return OMX_ErrorBadParameter;
    }

    for (i = 0; i < array_sz; ++i) {
        memset(&sub, 0, sizeof(sub));
        sub.type = event_type[i];
        rc = ioctl(fd, VIDIOC_SUBSCRIBE_EVENT, &sub);

        if (rc) {
           DEBUG_PRINT_ERROR("Failed to subscribe event: 0x%x", sub.type);
            break;
        }
    }

    if (i < array_sz) {
        for (--i; i >=0 ; i--) {
            memset(&sub, 0, sizeof(sub));
            sub.type = event_type[i];
            rc = ioctl(fd, VIDIOC_UNSUBSCRIBE_EVENT, &sub);

            if (rc)
               DEBUG_PRINT_ERROR("Failed to unsubscribe event: 0x%x", sub.type);
        }

        eRet = OMX_ErrorNotImplemented;
    }

    return eRet;
}

bool inline venc_dev::venc_query_cap(struct v4l2_queryctrl &cap) {

    if (ioctl(m_nDriver_fd, VIDIOC_QUERYCTRL, &cap)) {
        DEBUG_PRINT_ERROR("Query caps for id = %u failed", cap.id);
        return false;
    }
    return true;
}

bool venc_dev::venc_validate_range(OMX_S32 id, OMX_S32 val) {

    struct v4l2_queryctrl cap;
    memset(&cap, 0, sizeof(struct v4l2_queryctrl));

    cap.id = id;
    if (venc_query_cap(cap)) {
        if (val >= cap.minimum && val <= cap.maximum) {
            return true;
        } else {
            DEBUG_PRINT_INFO("id = %u, value = %u, min = %u, max = %u",
                cap.id, val, cap.minimum, cap.maximum);
        }
    }
    return false;
}

void venc_dev::get_roi_for_timestamp(struct roidata &roi, OMX_TICKS timestamp)
{
    std::list<roidata>::iterator iter;
    bool found = false;

    memset(&roi, 0, sizeof(struct roidata));
    roi.dirty = false;

    pthread_mutex_lock(&m_roilock);
    iter = m_roilist.begin();
    while (iter != m_roilist.end()) {
        if (iter->info.nTimeStamp < timestamp) {
            /* remove previous roi data */
            iter = m_roilist.erase(iter);
            /* iter++ is not required as erase would do it */
            continue;
        } else if (iter->info.nTimeStamp == timestamp){
            found = true;
            roi = *iter;
            break;
        }
        iter++;
    }
    if (found) {
        DEBUG_PRINT_LOW("found roidata with timestamp %lld us", roi.info.nTimeStamp);
    }
    pthread_mutex_unlock(&m_roilock);
}

bool venc_dev::handle_dynamic_config(OMX_BUFFERHEADERTYPE *bufferHdr)
{
    std::list<dynamicConfig>::iterator iter;
    OMX_TICKS timestamp = bufferHdr->nTimeStamp;
    bool ret = false;

    pthread_mutex_lock(&m_configlock);
    iter = m_configlist.begin();
    while (iter != m_configlist.end()) {
        if (iter->timestamp > timestamp) {
            iter++;
            continue;
        }
        DEBUG_PRINT_LOW("found dynamic config with timestamp %lld us", iter->timestamp);
        switch ((int)iter->type) {
            case OMX_QcomIndexConfigVideoLTRUse:
                DEBUG_PRINT_LOW("handle_dynamic_config: OMX_QcomIndexConfigVideoLTRUse");
                if (!venc_config_useLTR(&iter->config_data.useltr))
                    goto bailout;
                break;
            case OMX_QcomIndexConfigVideoLTRMark:
                DEBUG_PRINT_LOW("handle_dynamic_config: OMX_QcomIndexConfigVideoLTRMark");
                if (!venc_config_markLTR(&iter->config_data.markltr))
                    goto bailout;
                break;
            case OMX_IndexConfigVideoFramerate:
                DEBUG_PRINT_LOW("handle_dynamic_config: OMX_IndexConfigVideoFramerate");
                if (!venc_config_framerate(&iter->config_data.framerate))
                    goto bailout;
                break;
            case OMX_QcomIndexConfigQp:
                DEBUG_PRINT_LOW("handle_dynamic_config: OMX_QcomIndexConfigQp");
                if (!venc_config_qp(&iter->config_data.configqp))
                    goto bailout;
                break;
            case QOMX_IndexConfigVideoIntraperiod:
                DEBUG_PRINT_LOW("handle_dynamic_config:QOMX_IndexConfigVideoIntraperiod");
                if (!set_nP_frames(iter->config_data.intraperiod.nPFrames))
                    goto bailout;
                break;
            case OMX_IndexConfigVideoVp8ReferenceFrame:
                DEBUG_PRINT_LOW("handle_dynamic_config: OMX_IndexConfigVideoVp8ReferenceFrame");
                if (!venc_config_vp8refframe(&iter->config_data.vp8refframe))
                    goto bailout;
                break;
            case OMX_IndexConfigVideoIntraVOPRefresh:
                DEBUG_PRINT_LOW("handle_dynamic_config: OMX_IndexConfigVideoIntraVOPRefresh");
                if (!venc_config_intravoprefresh(&iter->config_data.intravoprefresh))
                    goto bailout;
                break;
            case OMX_IndexConfigVideoBitrate:
                DEBUG_PRINT_LOW("handle_dynamic_config: OMX_IndexConfigVideoBitrate");
                if (!venc_config_bitrate(&iter->config_data.bitrate))
                    goto bailout;
                break;
            case OMX_IndexConfigCommonMirror:
                DEBUG_PRINT_LOW("handle_dynamic_config: OMX_IndexConfigCommonMirror");
                if (!strncmp(venc_handle->m_platform, "bengal", 6) && (venc_handle->m_no_vpss)) {
                    DEBUG_PRINT_HIGH("Unsupported dynamic config on this target");
                    ret = true;
                    goto bailout;
                } else if (!venc_set_mirror(iter->config_data.mirror.eMirror)) {
                    goto bailout;
                }
                break;
            default:
                DEBUG_PRINT_ERROR("Unsupported dynamic config type %d with timestamp %lld us", iter->type, iter->timestamp);
                goto bailout;
        }
        iter = m_configlist.erase(iter);
    }
    ret = true;

bailout:
    pthread_mutex_unlock(&m_configlock);
    return ret;
}

bool venc_dev::handle_input_extradata(struct v4l2_buffer buf)
{
    unsigned int filled_len = 0;
    unsigned int index = buf.index;
    int height = m_sVenc_cfg.input_height;
    int width = m_sVenc_cfg.input_width;
    OMX_TICKS nTimeStamp = static_cast<OMX_TICKS>(buf.timestamp.tv_sec) * 1000000 + buf.timestamp.tv_usec;
    int fd = buf.m.planes[0].reserved[MSM_VIDC_BUFFER_FD];
    char *p_extradata = NULL;
    OMX_OTHER_EXTRADATATYPE *data = NULL;
    struct roidata roi;
    bool status = true;
    OMX_U32 packet_size = 0;
    OMX_U32 payload_size = 0;

    if (!EXTRADATA_IDX(num_input_planes)) {
        DEBUG_PRINT_LOW("Input extradata not enabled");
        return true;
    }

    if (!input_extradata_info.ion[index].uaddr) {
        DEBUG_PRINT_ERROR("Extradata buffers not allocated\n");
        return true;
    }

    DEBUG_PRINT_HIGH("Processing Extradata for Buffer = %lld", nTimeStamp); // Useful for debugging
    sync_start_rw(input_extradata_info.ion[index].data_fd);

    p_extradata = input_extradata_info.ion[index].uaddr;
    data = (struct OMX_OTHER_EXTRADATATYPE *)p_extradata;
    memset((void *)(data), 0, (input_extradata_info.buffer_size)); // clear stale data in current buffer

    if (m_cvp_meta_enabled && !(buf.flags & V4L2_BUF_FLAG_CVPMETADATA_SKIP)) {
        packet_size = sizeof(struct msm_vidc_extradata_header) - sizeof(unsigned int)
                           + cvpMetadata.size;

        if (filled_len + packet_size <= input_extradata_info.buffer_size) {
            struct  msm_vidc_enc_cvp_metadata_payload *payload_cvp;
            data->nSize = ALIGN(packet_size, 4);
            data->nVersion.nVersion = OMX_SPEC_VERSION;
            data->nPortIndex = PORT_INDEX_IN;
            data->eType = (OMX_EXTRADATATYPE)MSM_VIDC_EXTRADATA_CVP_METADATA;
            data->nDataSize = cvpMetadata.size;
            payload_cvp = (struct  msm_vidc_enc_cvp_metadata_payload *)(data->data);
            memcpy(payload_cvp->data, cvpMetadata.payload, cvpMetadata.size);
            filled_len += data->nSize;
            data = (OMX_OTHER_EXTRADATATYPE *)((char *)data + data->nSize);
        } else {
            DEBUG_PRINT_ERROR("Insufficient size for CVPMetadata: required %u buffer_size %lu filled_len %u",
               packet_size, input_extradata_info.buffer_size, filled_len);
            status = false;
            goto bailout;
        }
    }
    cvpMetadata.size = 0;

      /*
       * Below code is based on these points.
       * 1) As _PQ_ not defined in Napali :
       *     a) Send data to Venus as ROI.
       *     b) ROI enabled : Processed under unlocked context.
       *     c) ROI disabled : Nothing to fill.
       *     d) pq enabled : Not possible.
       * 2) Normal ROI handling.
       *     By this time if client sets next ROI, then we shouldn't process new ROI here.
       */

    memset(&roi, 0, sizeof(struct roidata));
    roi.dirty = false;
    if (m_roi_enabled) {
        get_roi_for_timestamp(roi, nTimeStamp);
    }

    if (roi.dirty) {
        OMX_U32 mbAlign = 16;
        if (m_codec == OMX_VIDEO_CodingHEVC) {
            mbAlign = 32;
        }
        OMX_U32 mbRow = ALIGN(width, mbAlign) / mbAlign;
        OMX_U32 mbRowAligned = ((mbRow + 7) >> 3) << 3;
        OMX_U32 mbCol = ALIGN(height, mbAlign) / mbAlign;
        OMX_U32 numBytes = mbRowAligned * mbCol * 2;
        OMX_U32 numBytesAligned = ALIGN(numBytes, 4);

        data->nDataSize = ALIGN(sizeof(struct msm_vidc_roi_deltaqp_payload),256)
                            + numBytesAligned;
        data->nSize = ALIGN(sizeof(OMX_OTHER_EXTRADATATYPE) + data->nDataSize, 4);
        if (data->nSize > input_extradata_info.buffer_size  - filled_len) {
           DEBUG_PRINT_ERROR("Buffer size (%lu) is less than ROI extradata size (%u)",
                             (input_extradata_info.buffer_size - filled_len) ,data->nSize);
           status = false;
           goto bailout;
        }

        data->nVersion.nVersion = OMX_SPEC_VERSION;
        data->nPortIndex = 0;
        data->eType = (OMX_EXTRADATATYPE)MSM_VIDC_EXTRADATA_ROI_QP;
        struct msm_vidc_roi_deltaqp_payload *roiData =
                (struct msm_vidc_roi_deltaqp_payload *)(data->data);
        roiData->b_roi_info = true;
        roiData->mbi_info_size = numBytesAligned;
        /* Video hardware expects ROI QP data to be aligned to 256,
         * And its offset should be available in roiData->data[0].
         *  -----------------
         *  | unsigned int b_roi_info;   |
         *  | int mbi_info_size;         |
         *  ----------------------------
         *  | data[0] = n                | => Contains Offset value to 256 aligned address
         *  | .                          |
         *  | .                          |
         *  | .                          |
         *  | data+n  <ROI data start>   | => 256 aligned address
         *  ----------------------------
         */
        roiData->data[0] = (unsigned int)(ALIGN(&roiData->data[1], 256) - (unsigned long)roiData->data);

        OMX_U8* tempBuf = (OMX_U8*)roi.info.pRoiMBInfo;
        OMX_U16* exDataBuf = (OMX_U16*)((OMX_U8*)roiData->data + roiData->data[0]);;
        OMX_U16* pBuf;
        OMX_U8 clientROI;

        /* Convert input extradata format to HW compatible format
         * Input        : 1byte per MB
         * HW Format    : 2bytes per MB. (1 << 11) | ((clientROI & 0x3F)<< 4)
         * MB Row must be aligned to 8
         */
        for (OMX_U32 j = 0;j < mbCol; j++)
        {
            pBuf = exDataBuf + j*mbRowAligned;
            for (OMX_U32 i = 0;i < mbRow; i++)
            {
                clientROI = *tempBuf++;
                *pBuf++ = (1 << 11) | ((clientROI & 0x3F)<< 4);
            }
        }
        filled_len += data->nSize;
        data = (OMX_OTHER_EXTRADATATYPE *)((char *)data + data->nSize);
    } else {
        // m_roilist didn't contain any ROI info with OMX_IndexConfigVideoRoiInfo.
        // then we can check mRoiRegionList which may contain the roi from vendor extension.
        OMX_U32 freeSize = input_extradata_info.buffer_size - filled_len;
        OMX_U32 appendSize = append_extradata_roi_region_qp_info(data, nTimeStamp, freeSize);
        filled_len += appendSize;
        data = (OMX_OTHER_EXTRADATATYPE *)((char *)data + appendSize);
    }

    if (m_roi_enabled) {
        if (roi.dirty) {
            DEBUG_PRINT_LOW("free roidata with timestamp %lld us", roi.info.nTimeStamp);
            roi.dirty = false;
        }
    }

    /* HDR10Plus MetaData information. Enabled by Default. */
    payload_size = sizeof(struct msm_vidc_hdr10plus_metadata_payload) - sizeof(unsigned int)
                                + colorData.dynamicMetaDataLen;
    packet_size = (sizeof(OMX_OTHER_EXTRADATATYPE) + payload_size + 3)&(~3);

    if (m_hdr10meta_enabled && (filled_len + packet_size <= input_extradata_info.buffer_size) && colorData.dynamicMetaDataLen > 0) {
        struct msm_vidc_hdr10plus_metadata_payload *payload;

        data->nSize = packet_size;
        data->nVersion.nVersion = OMX_SPEC_VERSION;
        data->nPortIndex = 0;
        data->eType = (OMX_EXTRADATATYPE)MSM_VIDC_EXTRADATA_HDR10PLUS_METADATA;
        data->nDataSize = payload_size;

        payload = (struct  msm_vidc_hdr10plus_metadata_payload *)(data->data);
        payload->size = colorData.dynamicMetaDataLen;
        memcpy(payload->data, colorData.dynamicMetaDataPayload, colorData.dynamicMetaDataLen);

        filled_len += data->nSize;
        data = (OMX_OTHER_EXTRADATATYPE *)((char *)data + data->nSize);
    } else if (colorData.dynamicMetaDataLen == 0) {
        DEBUG_PRINT_HIGH("DynamicMetaDataLength == 0 Skip writing metadata.");
    } else {
        if (m_hdr10meta_enabled) {
            DEBUG_PRINT_HIGH("Insufficient size for HDR10Metadata: Required %u Available %lu",
                             packet_size, (input_extradata_info.buffer_size - filled_len));
        }
    }

    data->nSize = sizeof(OMX_OTHER_EXTRADATATYPE);
    data->nVersion.nVersion = OMX_SPEC_VERSION;
    data->eType = OMX_ExtraDataNone;
    data->nDataSize = 0;
    data->data[0] = 0;

bailout:
    sync_end_rw(input_extradata_info.ion[index].data_fd);
    return status;
}

bool venc_dev::venc_handle_client_input_extradata(void *buffer)
{
    OMX_BUFFERHEADERTYPE *p_bufhdr = (OMX_BUFFERHEADERTYPE *)buffer;
    OMX_OTHER_EXTRADATATYPE *p_extra = (OMX_OTHER_EXTRADATATYPE*)p_bufhdr->pBuffer;
    while(p_extra->eType != OMX_ExtraDataNone) {
        switch((int)p_extra->eType) {
            case OMX_ExtraDataInputROIInfo:
            {
                OMX_QTI_VIDEO_CONFIG_ROIINFO* roiInfo = reinterpret_cast<OMX_QTI_VIDEO_CONFIG_ROIINFO*>(p_extra->data);
                struct roidata roi;
                if (!m_roi_enabled) {
                    DEBUG_PRINT_ERROR("ROI info not enabled");
                    return false;
                }

                if (!roiInfo) {
                    DEBUG_PRINT_ERROR("No ROI info present");
                    return false;
                }
                if (m_sVenc_cfg.codectype != V4L2_PIX_FMT_H264 &&
                m_sVenc_cfg.codectype != V4L2_PIX_FMT_HEVC) {
                    DEBUG_PRINT_ERROR("OMX_QTIIndexConfigVideoRoiInfo is not supported for %d codec", (OMX_U32) m_sVenc_cfg.codectype);
                    return false;
                }

                memset(&roi, 0, sizeof(struct roidata));

                roi.info.nRoiMBInfoCount = roiInfo->nRoiMBInfoCount;
                roi.info.nTimeStamp = roiInfo->nTimeStamp;
                memcpy(roi.info.pRoiMBInfo, &roiInfo->pRoiMBInfo, roiInfo->nRoiMBInfoCount);

                roi.dirty = true;

                pthread_mutex_lock(&m_roilock);
                DEBUG_PRINT_LOW("list add roidata with timestamp %lld us.", roi.info.nTimeStamp);
                m_roilist.push_back(roi);
                pthread_mutex_unlock(&m_roilock);
                break;
            }
        }
        p_extra = (OMX_OTHER_EXTRADATATYPE *)((char *)p_extra + p_extra->nSize);
    }
    return true;
}

bool venc_dev::handle_output_extradata(void *buffer, int index)
{
    OMX_BUFFERHEADERTYPE *p_bufhdr = (OMX_BUFFERHEADERTYPE *) buffer;
    OMX_OTHER_EXTRADATATYPE *p_clientextra = NULL;
    struct msm_vidc_extradata_header *p_extradata = NULL;
    OMX_U32 remaining_fw_extradata_size = 0;

    if(venc_handle->m_client_output_extradata_mem_ptr && venc_handle->m_sExtraData
        && venc_handle->m_client_out_extradata_info.getSize() >=
        output_extradata_info.buffer_size) {
        p_clientextra = (OMX_OTHER_EXTRADATATYPE * )
            ((venc_handle->m_client_output_extradata_mem_ptr + index) ->pBuffer);
    }
    if (p_clientextra == NULL) {
        DEBUG_PRINT_ERROR("Client Extradata buffers not allocated\n");
        return false;
    }

    if (!output_extradata_info.ion[index].uaddr) {
        DEBUG_PRINT_ERROR("Extradata buffers not allocated\n");
        return false;
    }
    p_extradata = (struct msm_vidc_extradata_header *)output_extradata_info.ion[index].uaddr;

    memcpy(p_clientextra, p_extradata, output_extradata_info.buffer_size);

    /* just for debugging */
    remaining_fw_extradata_size = output_extradata_info.buffer_size;
    while (remaining_fw_extradata_size >= sizeof(OMX_OTHER_EXTRADATATYPE) &&
            remaining_fw_extradata_size >= p_extradata->size &&
            p_extradata->type != MSM_VIDC_EXTRADATA_NONE) {

        switch (p_extradata->type) {
            case MSM_VIDC_EXTRADATA_METADATA_LTRINFO:
            {
                DEBUG_PRINT_LOW("LTRInfo Extradata = 0x%x", *((OMX_U32 *)p_extradata->data));
                break;
            }
            default:
                /* No idea what this stuff is, just skip over it */
                DEBUG_PRINT_HIGH("Found an unrecognised extradata (%x) ignoring it",
                        p_extradata->type);
                break;
        }

        remaining_fw_extradata_size -= p_extradata->size;
        p_extradata = (struct msm_vidc_extradata_header *) (
            ((char *)p_extradata) + p_extradata->size);
    }

    return true;
}

int venc_dev::venc_set_format(int format)
{
    int rc = true;

    if (format) {
        color_format = format;

        switch (color_format) {
        case NV12_128m:
        case NV12_512:
            return venc_set_color_format((OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m);
        case NV12_UBWC:
            return venc_set_color_format((OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420PackedSemiPlanar32mCompressed);
        case YCbCr420_VENUS_P010:
            return venc_set_color_format((OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420SemiPlanarP010Venus);
        default:
            return false;
        }

    } else {
        color_format = 0;
        rc = false;
    }

    return rc;
}

OMX_ERRORTYPE venc_dev::venc_get_supported_profile_level(OMX_VIDEO_PARAM_PROFILELEVELTYPE *profileLevelType)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    struct v4l2_queryctrl profile_cap, level_cap, tier_cap;
    int v4l2_profile;
    char platform_name[PROP_VALUE_MAX] = {0};
    char version[PROP_VALUE_MAX] = {0};

    int avc_profiles[5] = { QOMX_VIDEO_AVCProfileConstrainedBaseline,
                            QOMX_VIDEO_AVCProfileBaseline,
                            QOMX_VIDEO_AVCProfileMain,
                            QOMX_VIDEO_AVCProfileConstrainedHigh,
                            QOMX_VIDEO_AVCProfileHigh };
    int hevc_profiles[5] = { OMX_VIDEO_HEVCProfileMain,
                             OMX_VIDEO_HEVCProfileMain10,
                             OMX_VIDEO_HEVCProfileMainStill,
                             OMX_VIDEO_HEVCProfileMain10HDR10,
                             OMX_VIDEO_HEVCProfileMain10HDR10Plus };

    if (!profileLevelType)
        return OMX_ErrorBadParameter;

    memset(&tier_cap, 0, sizeof(struct v4l2_queryctrl));
    memset(&level_cap, 0, sizeof(struct v4l2_queryctrl));
    memset(&profile_cap, 0, sizeof(struct v4l2_queryctrl));

    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_H264) {
        level_cap.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL;
        profile_cap.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE;
    } else if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_VP8) {
        level_cap.id = V4L2_CID_MPEG_VIDC_VIDEO_VP8_PROFILE_LEVEL;
        profile_cap.id = V4L2_CID_MPEG_VIDEO_VP8_PROFILE;
    } else if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC) {
        tier_cap.id = V4L2_CID_MPEG_VIDEO_HEVC_TIER;
        level_cap.id = V4L2_CID_MPEG_VIDEO_HEVC_LEVEL;
        profile_cap.id = V4L2_CID_MPEG_VIDEO_HEVC_PROFILE;
    } else {
        DEBUG_PRINT_ERROR("get_parameter: OMX_IndexParamVideoProfileLevelQuerySupported Invalid codec");
        return OMX_ErrorInvalidComponent;
    }

    if (profile_cap.id) {
        if(!venc_query_cap(profile_cap)) {
            DEBUG_PRINT_ERROR("Getting capabilities for profile failed");
            return OMX_ErrorHardware;
        }
    }

    if (level_cap.id) {
        if(!venc_query_cap(level_cap)) {
            DEBUG_PRINT_ERROR("Getting capabilities for level failed");
            return OMX_ErrorHardware;
        }
    }

    if (tier_cap.id) {
        if(!venc_query_cap(tier_cap)) {
            DEBUG_PRINT_ERROR("Getting capabilities for tier failed");
            return OMX_ErrorHardware;
        }
    }

    /* Get the corresponding omx level from v4l2 level */
    if (!profile_level_converter::convert_v4l2_level_to_omx(m_sVenc_cfg.codectype, level_cap.maximum, (int *)&profileLevelType->eLevel)) {
        DEBUG_PRINT_ERROR("Invalid level, cannot find corresponding v4l2 level : %d ", level_cap.maximum);
        return OMX_ErrorHardware;
    }
    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC && tier_cap.maximum == V4L2_MPEG_VIDEO_HEVC_TIER_HIGH) {
        /* handle HEVC high tier */
        profileLevelType->eLevel <<= 1;
    }

    /* For given profile index get corresponding profile that needs to be supported */
    if (profileLevelType->nPortIndex != 1) {
        DEBUG_PRINT_ERROR("get_parameter: OMX_IndexParamVideoProfileLevelQuerySupported should be queried on output port only %u",
                            (unsigned int)profileLevelType->nPortIndex);
        return OMX_ErrorBadPortIndex;
    }

    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_H264) {
        if (profileLevelType->nProfileIndex < (sizeof(avc_profiles)/sizeof(int))) {
            profileLevelType->eProfile = avc_profiles[profileLevelType->nProfileIndex];
        } else {
            DEBUG_PRINT_LOW("AVC: get_parameter: OMX_IndexParamVideoProfileLevelQuerySupported nProfileIndex ret NoMore %u",
                    (unsigned int)profileLevelType->nProfileIndex);
            return OMX_ErrorNoMore;
        }
    } else if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_VP8) {
        if (profileLevelType->nProfileIndex == 0) {
            profileLevelType->eProfile = OMX_VIDEO_VP8ProfileMain;
        } else {
            DEBUG_PRINT_LOW("VP8: get_parameter: OMX_IndexParamVideoProfileLevelQuerySupported nProfileIndex ret NoMore %u",
                    (unsigned int)profileLevelType->nProfileIndex);
            return OMX_ErrorNoMore;
        }
    } else if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC) {
        if (profileLevelType->nProfileIndex < (sizeof(hevc_profiles)/sizeof(int))) {
            profileLevelType->eProfile =  hevc_profiles[profileLevelType->nProfileIndex];
        } else {
            DEBUG_PRINT_LOW("HEVC: get_parameter: OMX_IndexParamVideoProfileLevelQuerySupported nProfileIndex ret NoMore %u",
                    (unsigned int)profileLevelType->nProfileIndex);
            return OMX_ErrorNoMore;
        }
    }

    /* Check if the profile is supported by driver or not  */
    /* During query caps of profile driver sends a mask of */
    /* of all v4l2 profiles supported(in the flags field)  */
    if (!profile_level_converter::convert_omx_profile_to_v4l2(m_sVenc_cfg.codectype, profileLevelType->eProfile, &v4l2_profile)) {
        DEBUG_PRINT_ERROR("Invalid profile, cannot find corresponding omx profile");
        return OMX_ErrorHardware;
    }

    DEBUG_PRINT_INFO("v4l2 profile : %d flags : %d ", v4l2_profile, profile_cap.flags);
    if(!((profile_cap.flags >> v4l2_profile) & 0x1)) {
        DEBUG_PRINT_ERROR("%s: Invalid index corresponding profile not supported : %d ",__FUNCTION__, profileLevelType->eProfile);
        eRet = OMX_ErrorNoMore;
    }

    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC) {
        property_get("ro.board.platform", platform_name, "0");
        if (!strncmp(platform_name, "lito", 4)) {
            if (property_get("vendor.media.target.version", version, "0") && ((atoi(version) != 2) && (atoi(version) != 3))) {
                DEBUG_PRINT_LOW("Disabling main10 and above for saipan");
                m_disable_hdr = 0x2;
            }
        }
    }

    if (m_disable_hdr & ENC_HDR_DISABLE_FLAG) {
        if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC) {
            if (profileLevelType->eProfile == OMX_VIDEO_HEVCProfileMain10 ||
                profileLevelType->eProfile == OMX_VIDEO_HEVCProfileMain10HDR10 ||
                profileLevelType->eProfile == OMX_VIDEO_HEVCProfileMain10HDR10Plus) {
                DEBUG_PRINT_LOW("%s: HDR profile unsupported", __FUNCTION__);
                return OMX_ErrorHardware;
            }
        }
    }

    DEBUG_PRINT_LOW("get_parameter: OMX_IndexParamVideoProfileLevelQuerySupported for Input port returned Profile:%u, Level:%u",
            (unsigned int)profileLevelType->eProfile, (unsigned int)profileLevelType->eLevel);
    return eRet;
}

bool venc_dev::venc_get_supported_color_format(unsigned index, OMX_U32 *colorFormat) {
#ifdef _UBWC_
    //we support following formats
    //index 0 - Compressed (UBWC) Venus flavour of YUV420SP
    //index 1 - Venus flavour of YUV420SP
    //index 2 - Compressed (UBWC) TP10 (10bit packed)
    //index 3 - Compressed (UBWC) Venus flavour of RGBA8888
    //index 4 - Venus flavour of RGBA8888
    //index 5 - opaque which internally maps to YUV420SP.
    //index 6 - vannilla YUV420SP
    //this can be extended in the future
    int supportedFormats[] = {
        [0] = QOMX_COLOR_FORMATYUV420PackedSemiPlanar32mCompressed,
        [1] = QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m,
        [2] = QOMX_COLOR_FormatYVU420SemiPlanar,
        [3] = QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m10bitCompressed,
        [4] = QOMX_COLOR_FORMATYUV420SemiPlanarP010Venus,
        [5] = QOMX_COLOR_Format32bitRGBA8888Compressed,
        [6] = QOMX_COLOR_Format32bitRGBA8888,
        [7] = QOMX_COLOR_FormatAndroidOpaque,
        [8] = OMX_COLOR_FormatYUV420SemiPlanar,
    };
#else
    //we support two formats
    //index 0 - Venus flavour of YUV420SP
    //index 1 - opaque which internally maps to YUV420SP.
    //index 2 - vannilla YUV420SP
    //this can be extended in the future
    int supportedFormats[] = {
        [0] = QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m,
        [1] = QOMX_COLOR_FormatYVU420SemiPlanar,
        [2] = QOMX_COLOR_FormatAndroidOpaque,
        [3] = OMX_COLOR_FormatYUV420SemiPlanar,
    };
#endif
    if (index > (sizeof(supportedFormats)/sizeof(*supportedFormats) - 1))
        return false;
    *colorFormat = supportedFormats[index];
    return true;
}

OMX_ERRORTYPE venc_dev::allocate_extradata(struct extradata_buffer_info *extradata_info, int flags)
{
    if (extradata_info->allocated)
        return OMX_ErrorNone;

    if (!extradata_info->buffer_size || !extradata_info->count) {
        DEBUG_PRINT_ERROR("Invalid extradata buffer size(%lu) or count(%d) for port %d",
            extradata_info->buffer_size, extradata_info->count, extradata_info->port_index);
        return OMX_ErrorUndefined;
    }

#ifdef USE_ION

    for (int i = 0; i < extradata_info->count; i++) {
        if (extradata_info->ion[i].data_fd != -1) {
            venc_handle->ion_unmap(extradata_info->ion[i].data_fd,
                    (void *)extradata_info->ion[i].uaddr, extradata_info->buffer_size);
            venc_handle->free_ion_memory(&extradata_info->ion[i]);
        }

        // ION_IOC_MAP defined @ bionic/libc/kernel/uapi/linux/ion.h not in kernel,
        // and this ioctl uses "struct ion_fd_data" with handle member
        // Refer alloc_map_ion_memory definition @omx_video_base.cpp
        bool status = venc_handle->alloc_map_ion_memory(
                extradata_info->buffer_size, &extradata_info->ion[i], flags);

        if (status == false) {
            DEBUG_PRINT_ERROR("Failed to alloc extradata memory\n");
            return OMX_ErrorInsufficientResources;
        }

        extradata_info->ion[i].uaddr = (char *)venc_handle->ion_map(extradata_info->ion[i].data_fd,
                                                extradata_info->buffer_size);

        if (extradata_info->ion[i].uaddr == MAP_FAILED) {
            DEBUG_PRINT_ERROR("Failed to map extradata memory\n");
            venc_handle->free_ion_memory(&extradata_info->ion[i]);
            return OMX_ErrorInsufficientResources;
        } else {
            DEBUG_PRINT_HIGH("memset extradata buffer size %lu", extradata_info->buffer_size);
            memset((char *)extradata_info->ion[i].uaddr, 0, extradata_info->buffer_size);
        }
    }
#else
    (void)flags;
#endif
    extradata_info->allocated = OMX_TRUE;
    return OMX_ErrorNone;
}

void venc_dev::free_extradata(struct extradata_buffer_info *extradata_info)
{
#ifdef USE_ION

    if (extradata_info == NULL) {
        return;
    }

    for (int i = 0; i < extradata_info->count; i++) {
        if (extradata_info->ion[i].uaddr) {
            venc_handle->ion_unmap(extradata_info->ion[i].data_fd,
                    (void *)extradata_info->ion[i].uaddr, extradata_info->buffer_size);
            extradata_info->ion[i].uaddr = NULL;
            venc_handle->free_ion_memory(&extradata_info->ion[i]);
        }
        memset(&extradata_info->ion[i].alloc_data, 0, sizeof(struct ion_allocation_data));
    }
    extradata_info->buffer_size = 0;
    extradata_info->count = 0;
    extradata_info->allocated = OMX_FALSE;
#else
    (void)extradata_info;
#endif // USE_ION
}

void venc_dev::free_extradata_all()
{
    free_extradata(&output_extradata_info);
    free_extradata(&input_extradata_info);
}

bool venc_dev::venc_get_output_log_flag()
{
    return (m_debug.out_buffer_log == 1);
}

int venc_dev::venc_cvp_log_buffers(const char *metadataName, uint32_t buffer_len, uint8_t *buf)
{
    if (!m_debug.cvpfile && m_debug.cvp_log) {
        int size = 0;

        if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_H264 ||
            m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC) {
                size = snprintf(m_debug.cvpfile_name, PROPERTY_VALUE_MAX, "%s/enc_cvp_%lu_%lu_%p.bin",
                        m_debug.log_loc, m_sVenc_cfg.input_width, m_sVenc_cfg.input_height, this);
        }

        if ((size > PROPERTY_VALUE_MAX) && (size < 0)) {
            DEBUG_PRINT_ERROR("Failed to open cvp file: %s for logging size:%d",
                    m_debug.cvpfile_name, size);
        }

        m_debug.cvpfile = fopen(m_debug.cvpfile_name, "ab");
        if (!m_debug.cvpfile) {
            DEBUG_PRINT_ERROR("Failed to open cvp file: %s for logging errno:%d",
                            m_debug.cvpfile_name, errno);
            m_debug.cvpfile_name[0] = '\0';
            return -1;
        }
    }

    if (m_debug.cvpfile) {
        // Truncate or Zero-filled to match the string size to 5
        char name[6] = {0};
        for(int i=0; i<5 && i<strlen(metadataName); i++) {
            name[i] = metadataName[i];
        }
        fwrite(name, 5, 1, m_debug.cvpfile);                            // Metadata name
        fwrite(&buffer_len, sizeof(buffer_len), 1, m_debug.cvpfile);    // Blob size
        fwrite(buf, buffer_len, 1, m_debug.cvpfile);                    // Blob data
    }
    return 0;
}


int venc_dev::venc_output_log_buffers(const char *buffer_addr, int buffer_len, uint64_t timestamp)
{
    if (venc_handle->is_secure_session()) {
        DEBUG_PRINT_ERROR("logging secure output buffers is not allowed!");
        return -1;
    }

    if (!m_debug.outfile) {
        int size = 0;
        if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_H264) {
           size = snprintf(m_debug.outfile_name, PROPERTY_VALUE_MAX, "%s/output_enc_%lu_%lu_%p.264",
                           m_debug.log_loc, m_sVenc_cfg.input_width, m_sVenc_cfg.input_height, this);
        } else if(m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC) {
           size = snprintf(m_debug.outfile_name, PROPERTY_VALUE_MAX, "%s/output_enc_%ld_%ld_%p.265",
                           m_debug.log_loc, m_sVenc_cfg.input_width, m_sVenc_cfg.input_height, this);
        } else if(m_sVenc_cfg.codectype == V4L2_PIX_FMT_VP8) {
           size = snprintf(m_debug.outfile_name, PROPERTY_VALUE_MAX, "%s/output_enc_%lu_%lu_%p.ivf",
                           m_debug.log_loc, m_sVenc_cfg.input_width, m_sVenc_cfg.input_height, this);
        }

        if ((size > PROPERTY_VALUE_MAX) && (size < 0)) {
             DEBUG_PRINT_ERROR("Failed to open output file: %s for logging size:%d",
                                m_debug.outfile_name, size);
        }
        m_debug.outfile = fopen(m_debug.outfile_name, "ab");
        if (!m_debug.outfile) {
            DEBUG_PRINT_ERROR("Failed to open output file: %s for logging errno:%d",
                               m_debug.outfile_name, errno);
            m_debug.outfile_name[0] = '\0';
            return -1;
        }
        if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_VP8) {
            int fps = m_sVenc_cfg.fps_num / m_sVenc_cfg.fps_den;
            IvfFileHeader ivfFileHeader(false, m_sVenc_cfg.input_width,
                                        m_sVenc_cfg.input_height, fps, 1, 0);
            fwrite(&ivfFileHeader, sizeof(ivfFileHeader), 1, m_debug.outfile);
        }
    }
    if (m_debug.outfile && buffer_len) {
        if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_VP8) {
            IvfFrameHeader ivfFrameHeader(buffer_len, timestamp);
            fwrite(&ivfFrameHeader, sizeof(ivfFrameHeader), 1, m_debug.outfile);
        }
        DEBUG_PRINT_LOW("%s buffer_len:%d", __func__, buffer_len);
        fwrite(buffer_addr, buffer_len, 1, m_debug.outfile);
    }
    return 0;
}

int venc_dev::venc_extradata_log_buffers(char *buffer_addr, int index, bool input)
{
    int fd;

    if (input)
        fd = input_extradata_info.ion[index].data_fd;
    else
        fd = output_extradata_info.ion[index].data_fd;

    sync_start_read(fd);
    if (!m_debug.extradatafile && m_debug.extradata_log) {
        int size = 0;

        if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_H264 ||
            m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC ||
            m_sVenc_cfg.codectype == V4L2_PIX_FMT_VP8) {
            size = snprintf(m_debug.extradatafile_name, PROPERTY_VALUE_MAX, "%s/extradata_enc_%lu_%lu_%p.bin",
                            m_debug.log_loc, m_sVenc_cfg.input_width, m_sVenc_cfg.input_height, this);
        }
        if ((size > PROPERTY_VALUE_MAX) && (size < 0)) {
             DEBUG_PRINT_ERROR("Failed to open extradata file: %s for logging size:%d",
                                m_debug.extradatafile_name, size);
        }

        m_debug.extradatafile = fopen(m_debug.extradatafile_name, "ab");
        if (!m_debug.extradatafile) {
            DEBUG_PRINT_ERROR("Failed to open extradata file: %s for logging errno:%d",
                               m_debug.extradatafile_name, errno);
            m_debug.extradatafile_name[0] = '\0';
            sync_end_read(fd);
            return -1;
        }
    }

    if (m_debug.extradatafile && buffer_addr) {
        OMX_OTHER_EXTRADATATYPE *p_extra = NULL;
        do {
            p_extra = (OMX_OTHER_EXTRADATATYPE *)(!p_extra ? buffer_addr :
                    ((char *)p_extra) + p_extra->nSize);
            fwrite(p_extra, p_extra->nSize, 1, m_debug.extradatafile);
        } while (p_extra->eType != OMX_ExtraDataNone);
    }
    sync_end_read(fd);
    return 0;
}

int venc_dev::venc_input_log_buffers(OMX_BUFFERHEADERTYPE *pbuffer, int fd, int plane_offset,
        unsigned long inputformat, bool interlaced) {
    int status = 0;
    if (venc_handle->is_secure_session()) {
        DEBUG_PRINT_ERROR("logging secure input buffers is not allowed!");
        return -1;
    }

    sync_start_read(fd);
    if (!m_debug.infile) {
        int size = snprintf(m_debug.infile_name, PROPERTY_VALUE_MAX, "%s/input_enc_%lu_%lu_%p.yuv",
                            m_debug.log_loc, m_sVenc_cfg.input_width, m_sVenc_cfg.input_height, this);
        if ((size > PROPERTY_VALUE_MAX) && (size < 0)) {
             DEBUG_PRINT_ERROR("Failed to open output file: %s for logging size:%d",
                                m_debug.infile_name, size);
        }
        m_debug.infile = fopen (m_debug.infile_name, "ab");
        if (!m_debug.infile) {
            DEBUG_PRINT_HIGH("Failed to open input file: %s for logging", m_debug.infile_name);
            m_debug.infile_name[0] = '\0';
            status = -1;
            goto bailout;
        }
    }

    if (m_debug.infile && pbuffer && pbuffer->nFilledLen) {
        unsigned long stride, scanlines;
        unsigned long color_format;
        unsigned long i, msize;
        unsigned char *pvirt = NULL, *ptemp = NULL;
        unsigned char *temp = (unsigned char *)pbuffer->pBuffer;

        color_format = get_media_colorformat(inputformat);

        msize = VENUS_BUFFER_SIZE_USED(color_format, m_sVenc_cfg.input_width, m_sVenc_cfg.input_height,interlaced);
        const unsigned int extra_size = VENUS_EXTRADATA_SIZE(m_sVenc_cfg.input_width, m_sVenc_cfg.input_height);

        if (metadatamode == 1) {
            pvirt= (unsigned char *)mmap(NULL, msize, PROT_READ|PROT_WRITE,MAP_SHARED, fd, plane_offset);
            if (pvirt == MAP_FAILED) {
                DEBUG_PRINT_ERROR("%s mmap failed", __func__);
                status = -1;
                goto bailout;
            }
            ptemp = pvirt;
        } else {
            ptemp = temp;
        }

        if (color_format == COLOR_FMT_NV12) {
            stride = VENUS_Y_STRIDE(color_format, m_sVenc_cfg.input_width);
            scanlines = VENUS_Y_SCANLINES(color_format, m_sVenc_cfg.input_height);

            for (i = 0; i < m_sVenc_cfg.input_height; i++) {
                fwrite(ptemp, m_sVenc_cfg.input_width, 1, m_debug.infile);
                ptemp += stride;
            }
            if (metadatamode == 1) {
                ptemp = pvirt + (stride * scanlines);
            } else {
                ptemp = (unsigned char *)pbuffer->pBuffer + (stride * scanlines);
            }
            for (i = 0; i < m_sVenc_cfg.input_height/2; i++) {
                fwrite(ptemp, m_sVenc_cfg.input_width, 1, m_debug.infile);
                ptemp += stride;
            }
        } else if (color_format == COLOR_FMT_NV12_512) {
            stride = VENUS_Y_STRIDE(color_format, m_sVenc_cfg.input_width);
            scanlines = VENUS_Y_SCANLINES(color_format, m_sVenc_cfg.input_height);

            for (i = 0; i < scanlines; i++) {
                fwrite(ptemp, stride, 1, m_debug.infile);
                ptemp += stride;
            }
            if (metadatamode == 1) {
                ptemp = pvirt + (stride * scanlines);
            } else {
                ptemp = (unsigned char *)pbuffer->pBuffer + (stride * scanlines);
            }
            for (i = 0; i < scanlines/2; i++) {
                fwrite(ptemp, stride, 1, m_debug.infile);
                ptemp += stride;
            }
        } else if (color_format == COLOR_FMT_RGBA8888) {
            stride = VENUS_RGB_STRIDE(color_format, m_sVenc_cfg.input_width);
            scanlines = VENUS_RGB_SCANLINES(color_format, m_sVenc_cfg.input_height);

            for (i = 0; i < m_sVenc_cfg.input_height; i++) {
                fwrite(ptemp, m_sVenc_cfg.input_width * 4, 1, m_debug.infile);
                ptemp += stride;
            }
        } else if (color_format == COLOR_FMT_NV12_UBWC || color_format == COLOR_FMT_NV12_BPP10_UBWC || color_format == COLOR_FMT_RGBA8888_UBWC) {
            fwrite(ptemp, msize, 1, m_debug.infile);
        } else if(color_format == COLOR_FMT_P010) {
            stride = VENUS_Y_STRIDE(color_format, m_sVenc_cfg.input_width);
            scanlines = VENUS_Y_SCANLINES(color_format, m_sVenc_cfg.input_height);

            for (i = 0; i < m_sVenc_cfg.input_height; i++) {
                fwrite(ptemp, m_sVenc_cfg.input_width*2, 1, m_debug.infile);
                ptemp += stride;
            }
            if (metadatamode == 1) {
                ptemp = pvirt + (stride * scanlines);
            } else {
                ptemp = (unsigned char *)pbuffer->pBuffer + (stride * scanlines);
            }
            for (i = 0; i < m_sVenc_cfg.input_height/2; i++) {
                fwrite(ptemp, m_sVenc_cfg.input_width*2, 1, m_debug.infile);
                ptemp += stride;
            }
        }

        if (metadatamode == 1 && pvirt) {
            munmap(pvirt, msize);
        }
    }
bailout:
    sync_end_read(fd);
    return status;
}

bool venc_dev::venc_open(OMX_U32 codec)
{
    int r, minqp = 0, maxqp = 127;
    unsigned int alignment = 0,buffer_size = 0, temp =0;
    struct v4l2_control control;
    OMX_STRING device_name = (OMX_STRING)"/dev/video33";
    char property_value[PROPERTY_VALUE_MAX] = {0};
    FILE *soc_file = NULL;
    char buffer[10];

    m_nDriver_fd = open (device_name, O_RDWR);
    if ((int)m_nDriver_fd < 0) {
        DEBUG_PRINT_ERROR("ERROR: Omx_venc::Comp Init Returning failure");
        return false;
    }
    m_poll_efd = eventfd(0, 0);
    if (m_poll_efd < 0) {
        DEBUG_PRINT_ERROR("Failed to open event fd(%s)", strerror(errno));
        return false;
    }
    DEBUG_PRINT_LOW("m_nDriver_fd = %u", (unsigned int)m_nDriver_fd);

    // set the basic configuration of the video encoder driver
    m_sVenc_cfg.input_width = OMX_CORE_QCIF_WIDTH;
    m_sVenc_cfg.input_height= OMX_CORE_QCIF_HEIGHT;
    m_sVenc_cfg.dvs_width = OMX_CORE_QCIF_WIDTH;
    m_sVenc_cfg.dvs_height = OMX_CORE_QCIF_HEIGHT;
    m_sVenc_cfg.fps_num = 30;
    m_sVenc_cfg.fps_den = 1;
    m_sVenc_cfg.targetbitrate = 64000;
    m_sVenc_cfg.inputformat= V4L2_DEFAULT_OUTPUT_COLOR_FMT;
    m_rotation.rotation = 0;
    m_codec = codec;
    downscalar_enabled = OMX_FALSE;

    if (codec == OMX_VIDEO_CodingAVC) {
        m_sVenc_cfg.codectype = V4L2_PIX_FMT_H264;
        codec_profile.profile = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE;
        profile_level.level = V4L2_MPEG_VIDEO_H264_LEVEL_1_0;
        idrperiod.idrperiod = 1;
        minqp = 0;
        maxqp = 51;
    } else if (codec == OMX_VIDEO_CodingVP8) {
        m_sVenc_cfg.codectype = V4L2_PIX_FMT_VP8;
        codec_profile.profile = V4L2_MPEG_VIDC_VIDEO_VP8_UNUSED;
        profile_level.level = V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_0;
        minqp = 0;
        maxqp = 127;
    } else if (codec == OMX_VIDEO_CodingHEVC || codec == OMX_VIDEO_CodingImageHEIC) {
        m_sVenc_cfg.codectype = V4L2_PIX_FMT_HEVC;
        idrperiod.idrperiod = 1;
        minqp = 0;
        maxqp = 51;
        if (codec == OMX_VIDEO_CodingImageHEIC) {
            m_sVenc_cfg.input_width = DEFAULT_TILE_DIMENSION;
            m_sVenc_cfg.input_height= DEFAULT_TILE_DIMENSION;
            m_sVenc_cfg.dvs_width = DEFAULT_TILE_DIMENSION;
            m_sVenc_cfg.dvs_height = DEFAULT_TILE_DIMENSION;
            codec_profile.profile = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE;
        } else
            codec_profile.profile = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN;
        profile_level.level = V4L2_MPEG_VIDEO_HEVC_LEVEL_1;
    }

    session_ipb_qp_values.min_qp_packed = minqp | (minqp << 8) | (minqp << 16);
    session_ipb_qp_values.max_qp_packed = maxqp | (maxqp << 8) | (maxqp << 16);

    int ret;
    ret = subscribe_to_events(m_nDriver_fd);

    if (ret) {
        DEBUG_PRINT_ERROR("Subscribe Event Failed");
        return false;
    }

    struct v4l2_fmtdesc fdesc;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers bufreq;
    struct v4l2_capability cap;

    ret = ioctl(m_nDriver_fd, VIDIOC_QUERYCAP, &cap);

    if (ret) {
        DEBUG_PRINT_ERROR("Failed to query capabilities");
    } else {
        DEBUG_PRINT_LOW("Capabilities: driver_name = %s, card = %s, bus_info = %s,"
                " version = %d, capabilities = %x", cap.driver, cap.card,
                cap.bus_info, cap.version, cap.capabilities);
    }

    ret=0;
    fdesc.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fdesc.index=0;

    while (ioctl(m_nDriver_fd, VIDIOC_ENUM_FMT, &fdesc) == 0) {
        DEBUG_PRINT_LOW("fmt: description: %s, fmt: %x, flags = %x", fdesc.description,
                fdesc.pixelformat, fdesc.flags);
        fdesc.index++;
    }

    fdesc.type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    fdesc.index=0;

    while (ioctl(m_nDriver_fd, VIDIOC_ENUM_FMT, &fdesc) == 0) {
        DEBUG_PRINT_LOW("fmt: description: %s, fmt: %x, flags = %x", fdesc.description,
                fdesc.pixelformat, fdesc.flags);
        fdesc.index++;
    }

    if(venc_handle->is_secure_session()) {
        control.id = V4L2_CID_MPEG_VIDC_VIDEO_SECURE;
        control.value = 1;
        DEBUG_PRINT_HIGH("ioctl: open secure device");
        ret=ioctl(m_nDriver_fd, VIDIOC_S_CTRL,&control);
        if (ret) {
            DEBUG_PRINT_ERROR("ioctl: open secure dev fail, rc %d", ret);
            return false;
        }
    }

    if (venc_handle->is_secure_session()) {
        m_sOutput_buff_property.alignment = SZ_1M;
        m_sInput_buff_property.alignment  = SZ_1M;
    } else {
        m_sOutput_buff_property.alignment = SZ_4K;
        m_sInput_buff_property.alignment  = SZ_4K;
    }

    if (m_codec == OMX_VIDEO_CodingImageHEIC) {
        if (!venc_set_grid_enable()) {
            DEBUG_PRINT_ERROR("Failed to enable grid");
            return false;
        }
    }

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.height = m_sVenc_cfg.dvs_height;
    fmt.fmt.pix_mp.width = m_sVenc_cfg.dvs_width;
    fmt.fmt.pix_mp.pixelformat = m_sVenc_cfg.codectype;

    /*TODO: Return values not handled properly in this function anywhere.
     * Need to handle those.*/
    ret = ioctl(m_nDriver_fd, VIDIOC_S_FMT, &fmt);

    if (ret) {
        DEBUG_PRINT_ERROR("Failed to set format on capture port");
        return false;
    }

    m_sOutput_buff_property.datasize=fmt.fmt.pix_mp.plane_fmt[0].sizeimage;

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    fmt.fmt.pix_mp.height = m_sVenc_cfg.input_height;
    fmt.fmt.pix_mp.width = m_sVenc_cfg.input_width;
    fmt.fmt.pix_mp.pixelformat = V4L2_DEFAULT_OUTPUT_COLOR_FMT;
    fmt.fmt.pix_mp.colorspace = V4L2_COLORSPACE_470_SYSTEM_BG;

    ret = ioctl(m_nDriver_fd, VIDIOC_S_FMT, &fmt);
    m_sInput_buff_property.datasize=fmt.fmt.pix_mp.plane_fmt[0].sizeimage;

    bufreq.memory = V4L2_MEMORY_USERPTR;
    bufreq.count = 2;

    bufreq.type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    ret = ioctl(m_nDriver_fd,VIDIOC_REQBUFS, &bufreq);
    m_sInput_buff_property.mincount = m_sInput_buff_property.actualcount = bufreq.count;

    bufreq.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    bufreq.count = 2;
    ret = ioctl(m_nDriver_fd,VIDIOC_REQBUFS, &bufreq);
    m_sOutput_buff_property.mincount = m_sOutput_buff_property.actualcount = bufreq.count;

    resume_in_stopped = 0;
    metadatamode = 0;

    struct v4l2_frmsizeenum frmsize;

    //Get the hardware capabilities
    memset((void *)&frmsize,0,sizeof(frmsize));
    frmsize.index = 0;
    frmsize.pixel_format = m_sVenc_cfg.codectype;
    ret = ioctl(m_nDriver_fd, VIDIOC_ENUM_FRAMESIZES, &frmsize);

    if (ret || frmsize.type != V4L2_FRMSIZE_TYPE_STEPWISE) {
        DEBUG_PRINT_ERROR("Failed to get framesizes");
        return false;
    }

    if (frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
        capability.min_width = frmsize.stepwise.min_width;
        capability.max_width = frmsize.stepwise.max_width;
        capability.min_height = frmsize.stepwise.min_height;
        capability.max_height = frmsize.stepwise.max_height;
    }

    //Initialize non-default parameters
    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_VP8) {
        control.id = V4L2_CID_MPEG_VIDEO_GOP_SIZE;
        control.value = 0x7fffffff;
        if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control))
            DEBUG_PRINT_ERROR("Failed to set V4L2_CID_MPEG_VIDC_VIDEO_NUM_P_FRAME\n");
    }

    //Disable auto blur by default
    if (is_auto_blur_disabled) {
        control.id = V4L2_CID_MPEG_VIDC_VIDEO_BLUR_DIMENSIONS;
        control.value = 0x2;
        if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control))
           DEBUG_PRINT_ERROR("Failed to set V4L2_CID_MPEG_VIDC_VIDEO_BLUR_DIMENSIONS\n");
    }


    /* Enable Low power mode by default for better power */

    input_extradata_info.port_index = OUTPUT_PORT;
    output_extradata_info.port_index = CAPTURE_PORT;

    return true;
}

static OMX_ERRORTYPE unsubscribe_to_events(int fd)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    struct v4l2_event_subscription sub;
    int array_sz = sizeof(event_type)/sizeof(int);
    int i,rc;

    if (fd < 0) {
       DEBUG_PRINT_ERROR("Invalid input: %d", fd);
        return OMX_ErrorBadParameter;
    }

    for (i = 0; i < array_sz; ++i) {
        memset(&sub, 0, sizeof(sub));
        sub.type = event_type[i];
        rc = ioctl(fd, VIDIOC_UNSUBSCRIBE_EVENT, &sub);

        if (rc) {
           DEBUG_PRINT_ERROR("Failed to unsubscribe event: 0x%x", sub.type);
            break;
        }
    }

    return eRet;
}

void venc_dev::venc_close()
{
    DEBUG_PRINT_LOW("venc_close: fd = %u", (unsigned int)m_nDriver_fd);

    if ((int)m_nDriver_fd >= 0) {
        DEBUG_PRINT_HIGH("venc_close E");

        if(eventfd_write(m_poll_efd, 1)) {
            DEBUG_PRINT_ERROR("eventfd_write failed for fd: %d, errno = %d, force stop async_thread", m_poll_efd, errno);
            async_thread_force_stop = true;
        }

        if (async_thread_created)
            pthread_join(m_tid,NULL);

        if (venc_handle->msg_thread_created) {
            venc_handle->msg_thread_created = false;
            venc_handle->msg_thread_stop = true;
            post_message(venc_handle, omx_video::OMX_COMPONENT_CLOSE_MSG);
            DEBUG_PRINT_HIGH("omx_video: Waiting on Msg Thread exit");
            pthread_join(venc_handle->msg_thread_id, NULL);
        }
        DEBUG_PRINT_HIGH("venc_close X");
        unsubscribe_to_events(m_nDriver_fd);
        close(m_poll_efd);
        close(m_nDriver_fd);
        m_nDriver_fd = -1;
    }

    if (m_debug.infile) {
        fclose(m_debug.infile);
        m_debug.infile = NULL;
    }

    if (m_debug.outfile) {
        fclose(m_debug.outfile);
        m_debug.outfile = NULL;
    }

    if (m_debug.extradatafile) {
        fclose(m_debug.extradatafile);
        m_debug.extradatafile = NULL;
    }

    if (m_debug.cvpfile) {
        fclose(m_debug.cvpfile);
        m_debug.cvpfile = NULL;
    }
}

bool venc_dev::venc_set_buf_req(OMX_U32 *min_buff_count,
        OMX_U32 *actual_buff_count,
        OMX_U32 *buff_size,
        OMX_U32 port)
{
    (void)min_buff_count, (void)buff_size;
    unsigned long temp_count = 0;

    if (port == 0) {
        if (*actual_buff_count > m_sInput_buff_property.mincount) {
            temp_count = m_sInput_buff_property.actualcount;
            m_sInput_buff_property.actualcount = *actual_buff_count;
            DEBUG_PRINT_LOW("I/P Count set to %u", (unsigned int)*actual_buff_count);
        }
    } else {
        if (*actual_buff_count > m_sOutput_buff_property.mincount) {
            temp_count = m_sOutput_buff_property.actualcount;
            m_sOutput_buff_property.actualcount = *actual_buff_count;
            DEBUG_PRINT_LOW("O/P Count set to %u", (unsigned int)*actual_buff_count);
        }
    }

    return true;

}

bool venc_dev::venc_loaded_start()
{
    return true;
}

bool venc_dev::venc_loaded_stop()
{
    return true;
}

bool venc_dev::venc_loaded_start_done()
{
    return true;
}

bool venc_dev::venc_loaded_stop_done()
{
    return true;
}

bool venc_dev::venc_get_seq_hdr(void *buffer,
        unsigned buffer_size, unsigned *header_len)
{
    (void) buffer, (void) buffer_size, (void) header_len;
    return true;
}

bool venc_dev::venc_get_dimensions(OMX_U32 portIndex, OMX_U32 *w, OMX_U32 *h) {
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = portIndex == PORT_INDEX_OUT ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE :
            V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;

    if (ioctl(m_nDriver_fd, VIDIOC_G_FMT, &fmt)) {
        DEBUG_PRINT_ERROR("Failed to get format on %s port",
                portIndex == PORT_INDEX_OUT ? "capture" : "output");
        return false;
    }
    *w = fmt.fmt.pix_mp.width;
    *h = fmt.fmt.pix_mp.height;
    return true;
}

bool venc_dev::venc_get_buf_req(OMX_U32 *min_buff_count,
        OMX_U32 *actual_buff_count,
        OMX_U32 *buff_size,
        OMX_U32 port)
{
    struct v4l2_format fmt;
    unsigned int buf_size = 0, extra_data_size = 0, client_extra_data_size = 0;
    int ret;
    int extra_idx = 0;
    struct v4l2_control control;
    unsigned int minCount = 0;

    memset(&control, 0, sizeof(control));
    memset(&fmt, 0, sizeof(fmt));

    if (port == 0) {
        fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        fmt.fmt.pix_mp.height = m_sVenc_cfg.input_height;
        fmt.fmt.pix_mp.width = m_sVenc_cfg.input_width;
        fmt.fmt.pix_mp.pixelformat = m_sVenc_cfg.inputformat;
        fmt.fmt.pix_mp.colorspace = V4L2_COLORSPACE_470_SYSTEM_BG;
        ret = ioctl(m_nDriver_fd, VIDIOC_G_FMT, &fmt);
        if (ret) {
            DEBUG_PRINT_ERROR("set format failed, type %d, wxh %dx%d, format %#x, colorspace %d\n",
                fmt.type, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
                fmt.fmt.pix_mp.pixelformat, fmt.fmt.pix_mp.colorspace);
            return false;
        }
        m_sInput_buff_property.datasize=fmt.fmt.pix_mp.plane_fmt[0].sizeimage;

        control.id = V4L2_CID_MIN_BUFFERS_FOR_OUTPUT;
        ret = ioctl(m_nDriver_fd,  VIDIOC_G_CTRL, &control);
        if (ret || (unsigned int)control.value > MAX_V4L2_BUFS) {
            DEBUG_PRINT_ERROR("Driver returned invalid data, port = %d ret = %d Count = %d",
                port, ret, (unsigned int)control.value);
            return false;
        }

        // Request MAX_V4L2_BUFS from V4L2 in batch mode.
        // Keep the original count for the client
        if (metadatamode && mBatchSize) {
            minCount = MAX_V4L2_BUFS;
            DEBUG_PRINT_LOW("Set buffer count = %d as metadata mode and batchmode enabled", minCount);
        }

        minCount = MAX((unsigned int)control.value, minCount);
        m_sInput_buff_property.mincount = minCount;
        m_sInput_buff_property.actualcount = m_sInput_buff_property.mincount;

        fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        fmt.fmt.pix_mp.height = m_sVenc_cfg.input_height;
        fmt.fmt.pix_mp.width = m_sVenc_cfg.input_width;
        fmt.fmt.pix_mp.pixelformat = m_sVenc_cfg.inputformat;
        ret = ioctl(m_nDriver_fd, VIDIOC_G_FMT, &fmt);
        if (ret) {
            DEBUG_PRINT_ERROR("get format failed, type %d, wxh %dx%d, format %#x\n",
                fmt.type, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
                fmt.fmt.pix_mp.pixelformat);
            return false;
        }
        m_sInput_buff_property.datasize=fmt.fmt.pix_mp.plane_fmt[0].sizeimage;

        *min_buff_count = m_sInput_buff_property.mincount;
        *actual_buff_count = m_sInput_buff_property.actualcount;
#ifdef USE_ION
        // For ION memory allocations of the allocated buffer size
        // must be 4k aligned, hence aligning the input buffer
        // size to 4k.
        m_sInput_buff_property.datasize = ALIGN(m_sInput_buff_property.datasize, SZ_4K);
#endif
        *buff_size = m_sInput_buff_property.datasize;
        num_input_planes = fmt.fmt.pix_mp.num_planes;
        extra_idx = EXTRADATA_IDX(num_input_planes);

        if (extra_idx && (extra_idx < VIDEO_MAX_PLANES)) {
            extra_data_size =  fmt.fmt.pix_mp.plane_fmt[extra_idx].sizeimage;
        } else if (extra_idx >= VIDEO_MAX_PLANES) {
            DEBUG_PRINT_ERROR("Extradata index is more than allowed: %d\n", extra_idx);
            return false;
        }
        input_extradata_info.buffer_size =  ALIGN(extra_data_size, SZ_4K);
        input_extradata_info.count = m_sInput_buff_property.actualcount;
        venc_handle->m_client_in_extradata_info.set_extradata_info(input_extradata_info.buffer_size,m_sInput_buff_property.actualcount);
    } else {
        unsigned int extra_idx = 0;
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.fmt.pix_mp.height = m_sVenc_cfg.dvs_height;
        fmt.fmt.pix_mp.width = m_sVenc_cfg.dvs_width;
        fmt.fmt.pix_mp.pixelformat = m_sVenc_cfg.codectype;

        ret = ioctl(m_nDriver_fd, VIDIOC_S_FMT, &fmt);
        if (ret) {
            DEBUG_PRINT_ERROR("set format failed, type %d, wxh %dx%d, format %#x\n",
                fmt.type, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
                fmt.fmt.pix_mp.pixelformat);
            return false;
        }

        m_sOutput_buff_property.datasize=fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.fmt.pix_mp.height = m_sVenc_cfg.dvs_height;
        fmt.fmt.pix_mp.width = m_sVenc_cfg.dvs_width;
        fmt.fmt.pix_mp.pixelformat = m_sVenc_cfg.codectype;

        ret = ioctl(m_nDriver_fd, VIDIOC_G_FMT, &fmt);
        if (ret) {
            DEBUG_PRINT_ERROR("get format failed, type %d, wxh %dx%d, format %#x\n",
                fmt.type, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
                fmt.fmt.pix_mp.pixelformat);
            return false;
        }
        m_sOutput_buff_property.datasize=fmt.fmt.pix_mp.plane_fmt[0].sizeimage;

        control.id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE;

        ret = ioctl(m_nDriver_fd,  VIDIOC_G_CTRL, &control);
        if (ret || (unsigned int)control.value > MAX_V4L2_BUFS) {
            DEBUG_PRINT_ERROR("Driver returned invalid data port = %d ret = %d Count = %d",
                port, ret, (unsigned int)control.value);
            return false;
        }
        minCount = control.value;

        if (mBatchSize) {
            // If we're in batch mode, we'd like to end up in a situation where
            // driver is able to own mBatchSize buffers and we'd also own atleast
            // mBatchSize buffers
            minCount = MAX((unsigned int)control.value, mBatchSize) + mBatchSize;
            DEBUG_PRINT_LOW("set min count %d as mBatchSize %d", minCount, mBatchSize);
        }

        m_sOutput_buff_property.mincount = minCount;
        m_sOutput_buff_property.actualcount = m_sOutput_buff_property.mincount;

        *min_buff_count = m_sOutput_buff_property.mincount;
        *actual_buff_count = m_sOutput_buff_property.actualcount;
        *buff_size = m_sOutput_buff_property.datasize;
        num_output_planes = fmt.fmt.pix_mp.num_planes;
        extra_idx = EXTRADATA_IDX(num_output_planes);

        if (extra_idx && (extra_idx < VIDEO_MAX_PLANES)) {
            extra_data_size =  fmt.fmt.pix_mp.plane_fmt[extra_idx].sizeimage;
        } else if (extra_idx >= VIDEO_MAX_PLANES) {
            DEBUG_PRINT_ERROR("Extradata index is more than allowed: %d", extra_idx);
            return false;
        }

        output_extradata_info.buffer_size = ALIGN(extra_data_size, SZ_4K);
        output_extradata_info.count = m_sOutput_buff_property.actualcount;
        venc_handle->m_client_out_extradata_info.set_extradata_info(output_extradata_info.buffer_size,output_extradata_info.count);
    }

    DEBUG_PRINT_HIGH("venc_get_buf_req: port %d, wxh %dx%d, format %#x, driver min count %d, "
        "updated min count %d, act count %d, size %d, num planes %d",
        port, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height, fmt.fmt.pix_mp.pixelformat,
        control.value, *min_buff_count, *actual_buff_count, *buff_size, fmt.fmt.pix_mp.num_planes);

    return true;
}

bool venc_dev::venc_handle_empty_eos_buffer( void)
{
    struct v4l2_encoder_cmd enc;
    int rc = 0;

    if (!streaming[OUTPUT_PORT]) {
        enum v4l2_buf_type buf_type;
        struct v4l2_control control;
        int ret = 0;

        buf_type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;

        DEBUG_PRINT_HIGH("Calling streamon before issuing stop command for EOS");
        ret = ioctl(m_nDriver_fd, VIDIOC_STREAMON, &buf_type);
        if (ret) {
            DEBUG_PRINT_ERROR("Failed to call streamon");
            if (errno == EBUSY) {
                hw_overload = true;
            }
            return false;
        } else {
            streaming[OUTPUT_PORT] = true;
        }
    }

    memset(&enc, 0, sizeof(enc));
    enc.cmd = V4L2_ENC_CMD_STOP;
    DEBUG_PRINT_LOW("Sending : Encoder STOP comamnd");
    rc = ioctl(m_nDriver_fd, VIDIOC_ENCODER_CMD, &enc);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed : Encoder STOP comamnd");
        return false;
    }
    return true;
}

unsigned venc_dev::venc_stop( void)
{
    struct venc_msg venc_msg;
    struct v4l2_requestbuffers bufreq;
    int rc = 0, ret = 0;

    if (!stopped) {
        enum v4l2_buf_type cap_type;

        if (streaming[OUTPUT_PORT]) {
            cap_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
            rc = ioctl(m_nDriver_fd, VIDIOC_STREAMOFF, &cap_type);

            if (rc) {
                DEBUG_PRINT_ERROR("Failed to call streamoff on driver: capability: %d, %d",
                        cap_type, rc);
            } else
                streaming[OUTPUT_PORT] = false;

            DEBUG_PRINT_LOW("Releasing registered buffers from driver on o/p port");
            bufreq.memory = V4L2_MEMORY_USERPTR;
            bufreq.count = 0;
            bufreq.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
            ret = ioctl(m_nDriver_fd, VIDIOC_REQBUFS, &bufreq);

            if (ret) {
                DEBUG_PRINT_ERROR("ERROR: VIDIOC_REQBUFS OUTPUT MPLANE Failed");
                return false;
            }
        }

        if (!rc && streaming[CAPTURE_PORT]) {
            cap_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            rc = ioctl(m_nDriver_fd, VIDIOC_STREAMOFF, &cap_type);

            if (rc) {
                DEBUG_PRINT_ERROR("Failed to call streamoff on driver: capability: %d, %d",
                        cap_type, rc);
            } else
                streaming[CAPTURE_PORT] = false;

            DEBUG_PRINT_LOW("Releasing registered buffers from driver on capture port");
            bufreq.memory = V4L2_MEMORY_USERPTR;
            bufreq.count = 0;
            bufreq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            ret = ioctl(m_nDriver_fd, VIDIOC_REQBUFS, &bufreq);

            if (ret) {
                DEBUG_PRINT_ERROR("ERROR: VIDIOC_REQBUFS CAPTURE MPLANE Failed");
                return false;
            }
        }

        if (!rc && !ret) {
            venc_stop_done();
            stopped = 1;
            /*set flag to re-configure when started again*/
            resume_in_stopped = 1;
        }
    }

    return rc;
}

bool venc_dev::is_streamon_done(OMX_U32 port)
{
    return streaming[port];
}

unsigned venc_dev::venc_pause(void)
{
    pthread_mutex_lock(&pause_resume_mlock);
    paused = true;
    pthread_mutex_unlock(&pause_resume_mlock);
    return 0;
}

unsigned venc_dev::venc_resume(void)
{
    pthread_mutex_lock(&pause_resume_mlock);
    paused = false;
    pthread_mutex_unlock(&pause_resume_mlock);

    return pthread_cond_signal(&pause_resume_cond);
}

unsigned venc_dev::venc_start_done(void)
{
    struct venc_msg venc_msg;
    venc_msg.msgcode = VEN_MSG_START;
    venc_msg.statuscode = VEN_S_SUCCESS;
    venc_handle->async_message_process(venc_handle,&venc_msg);
    return 0;
}

unsigned venc_dev::venc_stop_done(void)
{
    struct venc_msg venc_msg;
    free_extradata_all();
    venc_msg.msgcode=VEN_MSG_STOP;
    venc_msg.statuscode=VEN_S_SUCCESS;
    venc_handle->async_message_process(venc_handle,&venc_msg);
    return 0;
}

unsigned venc_dev::venc_set_message_thread_id(pthread_t tid)
{
    async_thread_created = true;
    m_tid=tid;
    return 0;
}

bool venc_dev::venc_set_extradata_hdr10metadata(OMX_U32 omx_profile)
{
    /* HDR10 Metadata is enabled by default for HEVC Main10PLUS profile. */
    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC &&
        omx_profile == OMX_VIDEO_HEVCProfileMain10HDR10Plus) {
        DEBUG_PRINT_HIGH("venc_set_extradata:: HDR10PLUS_METADATA");
        struct v4l2_control control;
        control.id = V4L2_CID_MPEG_VIDC_VIDEO_EXTRADATA;
        control.value = EXTRADATA_ENC_INPUT_HDR10PLUS;

        if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control)) {
            DEBUG_PRINT_ERROR("ERROR: Set extradata HDR10PLUS_METADATA failed %d", errno);
            return false;
        }

        m_hdr10meta_enabled = true;

        //Get upated buffer requirement as enable extradata leads to two buffer planes
        venc_get_buf_req (&venc_handle->m_sInPortDef.nBufferCountMin,
                          &venc_handle->m_sInPortDef.nBufferCountActual,
                          &venc_handle->m_sInPortDef.nBufferSize,
                          venc_handle->m_sInPortDef.nPortIndex);
    } else {
        m_hdr10meta_enabled = false;
    }

    return true;
}

unsigned venc_dev::venc_start(void)
{
    enum v4l2_buf_type buf_type;
    int ret, r;
    struct v4l2_control control;

    memset(&control, 0, sizeof(control));

    venc_config_print();

    /* set buffercount before start */
    venc_reconfig_reqbufs();
    resume_in_stopped = 0;

    buf_type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    DEBUG_PRINT_LOW("send_command_proxy(): Idle-->Executing");
    ret=ioctl(m_nDriver_fd, VIDIOC_STREAMON,&buf_type);

    if (ret)
        return 1;

    streaming[CAPTURE_PORT] = true;

    stopped = 0;
    return 0;
}

inline const char* hiermode_string(int val)
{
    switch(val)
    {
    case HIER_NONE:
        return "No Hier";
    case HIER_P:
        return "Hier-P";
    case HIER_B:
        return "Hier-B";
    default:
        return "No hier";
    }
}

inline const char* bitrate_type_string(int val)
{
    switch(val)
    {
    case V4L2_MPEG_MSM_VIDC_DISABLE:
        return "CUMULATIVE";
    case V4L2_MPEG_MSM_VIDC_ENABLE:
        return "LAYER WISE";
    default:
        return "Unknown Bitrate Type";
    }
}

static const char *codec_as_string(unsigned long codec) {
    switch (codec) {
    case V4L2_PIX_FMT_H264:
        return "H264";
    case V4L2_PIX_FMT_HEVC:
        return "HEVC";
    case V4L2_PIX_FMT_VP8:
        return "VP8";
    default:
        return "UNKOWN";
    }
}

void venc_dev::venc_config_print()
{

    DEBUG_PRINT_HIGH("ENC_CONFIG: Codec: %s, Profile %ld, level : %ld",
            codec_as_string(m_sVenc_cfg.codectype), codec_profile.profile, profile_level.level);

    DEBUG_PRINT_HIGH("ENC_CONFIG: Input Width: %ld, Height:%ld, Fps: %ld",
            m_sVenc_cfg.input_width, m_sVenc_cfg.input_height,
            m_sVenc_cfg.fps_num/m_sVenc_cfg.fps_den);

    DEBUG_PRINT_HIGH("ENC_CONFIG: Output Width: %ld, Height:%ld, Fps: %ld",
            m_sVenc_cfg.dvs_width, m_sVenc_cfg.dvs_height,
            m_sVenc_cfg.fps_num/m_sVenc_cfg.fps_den);

    DEBUG_PRINT_HIGH("ENC_CONFIG: Color Space: Primaries = %u, Range = %u, Transfer Chars = %u, Matrix Coeffs = %u",
            color_space.primaries, color_space.range, color_space.transfer_chars, color_space.matrix_coeffs);

    DEBUG_PRINT_HIGH("ENC_CONFIG: Bitrate: %ld, RC: %ld, P - Frames : %ld, B - Frames = %ld",
            bitrate.target_bitrate, rate_ctrl.rcmode, intra_period.num_pframes, intra_period.num_bframes);

    DEBUG_PRINT_HIGH("ENC_CONFIG: qpI: %ld, qpP: %ld, qpb: %ld enableqp : %ld",
            session_qp.iframeqp, session_qp.pframeqp, session_qp.bframeqp, session_qp.enableqp);

    DEBUG_PRINT_HIGH("ENC_CONFIG: minQP: %lu, maxQP: %lu",
                     session_ipb_qp_values.min_qp_packed, session_ipb_qp_values.max_qp_packed);

    DEBUG_PRINT_HIGH("ENC_CONFIG: VOP_Resolution: %ld, Slice-Mode: %ld, Slize_Size: %ld",
            voptimecfg.voptime_resolution, multislice.mslice_mode,
            multislice.mslice_size);

    DEBUG_PRINT_HIGH("ENC_CONFIG: EntropyMode: %d, CabacModel: %ld",
            entropy.longentropysel, entropy.cabacmodel);

    DEBUG_PRINT_HIGH("ENC_CONFIG: DB-Mode: %ld, alpha: %ld, Beta: %ld",
            dbkfilter.db_mode, dbkfilter.slicealpha_offset,
            dbkfilter.slicebeta_offset);

    DEBUG_PRINT_HIGH("ENC_CONFIG: HEC: %ld, IDR Period: %ld",
            hec.header_extension, idrperiod.idrperiod);

    if (temporal_layers_config.nPLayers) {
        DEBUG_PRINT_HIGH("ENC_CONFIG: Temporal layers: P-layers: %u, B-layers: %u, Adjusted I-frame-interval: %lu",
                temporal_layers_config.nPLayers, temporal_layers_config.nBLayers,
                intra_period.num_pframes + intra_period.num_bframes + 1);
    }

    DEBUG_PRINT_HIGH("ENC_CONFIG: VUI timing info enabled: %d", vui_timing_info.enabled);

    DEBUG_PRINT_HIGH("ENC_CONFIG: Peak bitrate: %d", peak_bitrate.peakbitrate);

    DEBUG_PRINT_HIGH("ENC_CONFIG: Session Priority: %s", sess_priority.priority ? "NonRealTime" : "RealTime");

    DEBUG_PRINT_HIGH("ENC_CONFIG: ROI : %u", m_roi_enabled);

    DEBUG_PRINT_HIGH("ENC_CONFIG: Operating Rate: %u", operating_rate);
}

bool venc_dev::venc_reconfig_reqbufs()
{
    struct v4l2_requestbuffers bufreq;

    DEBUG_PRINT_HIGH("venc_reconfig_reqbufs: output_mplane %lu, capture_mplane %lu",
        m_sInput_buff_property.actualcount, m_sOutput_buff_property.actualcount);

    bufreq.memory = V4L2_MEMORY_USERPTR;
    bufreq.count = m_sInput_buff_property.actualcount;
    bufreq.type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    if(ioctl(m_nDriver_fd,VIDIOC_REQBUFS, &bufreq)) {
        DEBUG_PRINT_ERROR("VIDIOC_REQBUFS: OUTPUT_MPLANE (count %d) failed", bufreq.count);
        return false;
    }

    bufreq.memory = V4L2_MEMORY_USERPTR;
    bufreq.count = m_sOutput_buff_property.actualcount;
    bufreq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if(ioctl(m_nDriver_fd,VIDIOC_REQBUFS, &bufreq)) {
        DEBUG_PRINT_ERROR("VIDIOC_REQBUFS: CAPTURE_MPLANE (count %d) failed", bufreq.count);
        return false;
    }
    return true;
}

unsigned venc_dev::venc_flush( unsigned port)
{
    struct v4l2_encoder_cmd enc;
    DEBUG_PRINT_LOW("in %s", __func__);

    enc.cmd = V4L2_CMD_FLUSH;
    enc.flags = V4L2_CMD_FLUSH_OUTPUT | V4L2_CMD_FLUSH_CAPTURE;

    if (ioctl(m_nDriver_fd, VIDIOC_ENCODER_CMD, &enc)) {
        DEBUG_PRINT_ERROR("Flush Port (%d) Failed ", port);
        return -1;
    }

    return 0;
}

//allocating I/P memory from pmem and register with the device
bool venc_dev::allocate_extradata(unsigned port)
{
    int rc = 0;
    unsigned int extra_idx = 0;

    // PORT_INDEX_IN = 0
    // PORT_INDEX_OUT = 1
    struct port_info_s {
        int num_planes;
        struct extradata_buffer_info *extradata_info;
        int flag;
    }port_info[2] = {
        {
            .num_planes = num_input_planes,
            .extradata_info = &input_extradata_info,
            .flag = 0
        },
        {
            .num_planes = num_output_planes,
            .extradata_info = &output_extradata_info,
            .flag = 0
        }
    };

    if (port != PORT_INDEX_IN && port != PORT_INDEX_OUT) {
        DEBUG_PRINT_ERROR("ERROR: venc_use_buf:Invalid Port Index ");
        return false;
    }

    extra_idx = EXTRADATA_IDX(port_info[port].num_planes);
    if ((port_info[port].num_planes > 1) && (extra_idx)) {
        rc = allocate_extradata(port_info[port].extradata_info,
                                port_info[port].flag);
        if (rc) {
            DEBUG_PRINT_ERROR("Failed to allocate extradata: %d\n", rc);
            return false;
        }
    }

    return true;
}

bool venc_dev::venc_free_buf(void *buf_addr, unsigned port)
{
    struct pmem *pmem_tmp;
    struct venc_bufferpayload dev_buffer;

    memset(&dev_buffer, 0, sizeof(dev_buffer));
    pmem_tmp = (struct pmem *)buf_addr;

    if (port == PORT_INDEX_IN) {
        dev_buffer.pbuffer = (OMX_U8 *)pmem_tmp->buffer;
        dev_buffer.fd  = pmem_tmp->fd;
        dev_buffer.maped_size = pmem_tmp->size;
        dev_buffer.sz = pmem_tmp->size;
        dev_buffer.offset = pmem_tmp->offset;
        DEBUG_PRINT_LOW("venc_free_buf:pbuffer = %p,fd = %x, offset = %d, maped_size = %d", \
                dev_buffer.pbuffer, \
                dev_buffer.fd, \
                dev_buffer.offset, \
                dev_buffer.maped_size);

    } else if (port == PORT_INDEX_OUT) {
        dev_buffer.pbuffer = (OMX_U8 *)pmem_tmp->buffer;
        dev_buffer.fd  = pmem_tmp->fd;
        dev_buffer.sz = pmem_tmp->size;
        dev_buffer.maped_size = pmem_tmp->size;
        dev_buffer.offset = pmem_tmp->offset;

        DEBUG_PRINT_LOW("venc_free_buf:pbuffer = %p,fd = %x, offset = %d, maped_size = %d", \
                dev_buffer.pbuffer, \
                dev_buffer.fd, \
                dev_buffer.offset, \
                dev_buffer.maped_size);
    } else {
        DEBUG_PRINT_ERROR("ERROR: venc_free_buf:Invalid Port Index ");
        return false;
    }

    return true;
}

bool venc_dev::venc_color_align(OMX_BUFFERHEADERTYPE *buffer,
        OMX_U32 width, OMX_U32 height)
{
    OMX_U32 y_stride = VENUS_Y_STRIDE(COLOR_FMT_NV12, width),
            y_scanlines = VENUS_Y_SCANLINES(COLOR_FMT_NV12, height),
            uv_stride = VENUS_UV_STRIDE(COLOR_FMT_NV12, width),
            uv_scanlines = VENUS_UV_SCANLINES(COLOR_FMT_NV12, height),
            src_chroma_offset = width * height;

    if (buffer->nAllocLen >= VENUS_BUFFER_SIZE(COLOR_FMT_NV12, width, height)) {
        OMX_U8* src_buf = buffer->pBuffer, *dst_buf = buffer->pBuffer;
        //Do chroma first, so that we can convert it in-place
        src_buf += width * height;
        dst_buf += y_stride * y_scanlines;
        for (int line = height / 2 - 1; line >= 0; --line) {
            /* Align the length to 16 for better memove performance. */
            memmove(dst_buf + line * uv_stride,
                    src_buf + line * width,
                    ALIGN(width, 16));
        }

        dst_buf = src_buf = buffer->pBuffer;
        //Copy the Y next
        for (int line = height - 1; line > 0; --line) {
            /* Align the length to 16 for better memove performance. */
            memmove(dst_buf + line * y_stride,
                    src_buf + line * width,
                    ALIGN(width, 16));
        }
        /* Inform driver to do cache flush on total buffer */
        buffer->nFilledLen = buffer->nAllocLen;
    } else {
        DEBUG_PRINT_ERROR("Failed to align Chroma. from %u to %u : \
                Insufficient bufferLen=%u v/s Required=%u",
                (unsigned int)(width*height), (unsigned int)src_chroma_offset, (unsigned int)buffer->nAllocLen,
                VENUS_BUFFER_SIZE(COLOR_FMT_NV12, width, height));
        return false;
    }

    return true;
}

bool venc_dev::venc_get_vui_timing_info(OMX_U32 *enabled)
{
    if (!enabled) {
        DEBUG_PRINT_ERROR("Null pointer error");
        return false;
    } else {
        *enabled = vui_timing_info.enabled;
        return true;
    }
}

bool venc_dev::venc_get_peak_bitrate(OMX_U32 *peakbitrate)
{
    if (!peakbitrate) {
        DEBUG_PRINT_ERROR("Null pointer error");
        return false;
    } else {
        *peakbitrate = peak_bitrate.peakbitrate;
        return true;
    }
}

bool venc_dev::venc_get_batch_size(OMX_U32 *size)
{
    if (!size) {
        DEBUG_PRINT_ERROR("Null pointer error");
        return false;
    } else {
        *size = mBatchSize;
        return true;
    }
}

bool venc_dev::venc_get_buffer_mode()
{
    return metadatamode;
}

bool venc_dev::venc_is_avtimer_needed()
{
    return mUseAVTimerTimestamps;
}

bool venc_dev::venc_empty_buf(void *buffer, void *pmem_data_buf, unsigned index, unsigned fd)
{
    struct pmem *temp_buffer;
    struct v4l2_buffer buf;
    struct v4l2_requestbuffers bufreq;
    struct v4l2_plane plane[VIDEO_MAX_PLANES];
    int rc = 0, extra_idx, c2d_enabled = 0;
    bool interlace_flag = false;
    struct OMX_BUFFERHEADERTYPE *bufhdr;
    LEGACY_CAM_METADATA_TYPE * meta_buf = NULL;
    temp_buffer = (struct pmem *)buffer;

    memset (&buf, 0, sizeof(buf));
    memset (&plane, 0, sizeof(plane));

    if (buffer == NULL) {
        DEBUG_PRINT_ERROR("ERROR: venc_etb: buffer is NULL");
        return false;
    }

    bufhdr = (OMX_BUFFERHEADERTYPE *)buffer;
    bufreq.memory = V4L2_MEMORY_USERPTR;
    bufreq.count = m_sInput_buff_property.actualcount;
    bufreq.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;

    if (pmem_data_buf) {
        DEBUG_PRINT_LOW("\n Internal PMEM addr for i/p Heap UseBuf: %p", pmem_data_buf);
        plane[0].m.userptr = (unsigned long)pmem_data_buf;
        plane[0].data_offset = bufhdr->nOffset;
        plane[0].length = bufhdr->nAllocLen;
        plane[0].bytesused = bufhdr->nFilledLen;
    } else {
        // --------------------------------------------------------------------------------------
        // [Usage]             [metadatamode] [Type]        [color_format] [Where is buffer info]
        // ---------------------------------------------------------------------------------------
        // Camera-2              1            CameraSource   0              meta-handle
        // Camera-3              1            GrallocSource  0              gralloc-private-handle
        // surface encode (RBG)  1            GrallocSource  1              bufhdr (color-converted)
        // CPU (Eg: MediaCodec)  0            --             0              bufhdr
        // ---------------------------------------------------------------------------------------
        if (metadatamode) {
            plane[0].m.userptr = index;
            meta_buf = (LEGACY_CAM_METADATA_TYPE *)bufhdr->pBuffer;

            if (!meta_buf) {
                if (!bufhdr->nFilledLen) {
                    if (bufhdr->nFlags & OMX_BUFFERFLAG_EOS) {
                        DEBUG_PRINT_ERROR("venc_empty_buf: Zero length EOS buffers are not valid");
                        DEBUG_PRINT_ERROR("Use this function instead : venc_handle_empty_eos_buffer");
                        return false;
                    }
                    DEBUG_PRINT_ERROR("venc_empty_buf: Zero length buffers are not valid");
                    return false;
                }
            } else if (!color_format) { // Metadata mode

                if (meta_buf->buffer_type == LEGACY_CAM_SOURCE) {
                    native_handle_t *hnd = (native_handle_t*)meta_buf->meta_handle;
                    if (!hnd) {
                        DEBUG_PRINT_ERROR("ERROR: venc_etb: handle is NULL");
                        return false;
                    }
                    int usage = 0;
                    usage = MetaBufferUtil::getIntAt(hnd, 0, MetaBufferUtil::INT_USAGE);
                    usage = usage > 0 ? usage : 0;

                    if (!streaming[OUTPUT_PORT] && !(m_sVenc_cfg.inputformat == V4L2_PIX_FMT_RGB32 ||
                        m_sVenc_cfg.inputformat == V4L2_PIX_FMT_RGBA8888_UBWC)) {

                        unsigned int is_csc_enabled = 0;
                        struct v4l2_format fmt;
                        OMX_COLOR_FORMATTYPE color_format = (OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m;

                        color_format = (OMX_COLOR_FORMATTYPE)MetaBufferUtil::getIntAt(hnd, 0, MetaBufferUtil::INT_COLORFORMAT);

                        memset(&fmt, 0, sizeof(fmt));
                        if (usage & private_handle_t::PRIV_FLAGS_ITU_R_709 ||
                                usage & private_handle_t::PRIV_FLAGS_ITU_R_601) {
                            DEBUG_PRINT_ERROR("Camera buffer color format is not 601_FR.");
                            DEBUG_PRINT_ERROR(" This leads to unknown color space");
                        }
                        if (usage & private_handle_t::PRIV_FLAGS_ITU_R_601_FR) {
                            if (is_csc_enabled) {
                                struct v4l2_control control;
                                control.id = V4L2_CID_MPEG_VIDC_VIDEO_VPE_CSC;
                                control.value = V4L2_MPEG_MSM_VIDC_ENABLE;
                                if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control)) {
                                    DEBUG_PRINT_ERROR("venc_empty_buf: Failed to set VPE CSC for 601_to_709");
                                } else {
                                    DEBUG_PRINT_INFO("venc_empty_buf: Will convert 601-FR to 709");
                                    fmt.fmt.pix_mp.colorspace = V4L2_COLORSPACE_REC709;
                                    venc_set_colorspace(MSM_VIDC_BT709_5, 0,
                                            MSM_VIDC_TRANSFER_BT709_5, MSM_VIDC_MATRIX_BT_709_5);
                                }
                            } else {
                                venc_set_colorspace(MSM_VIDC_BT601_6_525, 1,
                                        MSM_VIDC_TRANSFER_601_6_525, MSM_VIDC_MATRIX_601_6_525);
                                fmt.fmt.pix_mp.colorspace = V4L2_COLORSPACE_470_SYSTEM_BG;
                            }
                        }
                        fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
                        m_sVenc_cfg.inputformat = V4L2_PIX_FMT_NV12;
                        fmt.fmt.pix_mp.height = m_sVenc_cfg.input_height;
                        fmt.fmt.pix_mp.width = m_sVenc_cfg.input_width;
                        if (usage & private_handle_t::PRIV_FLAGS_UBWC_ALIGNED ||
                            usage & private_handle_t::PRIV_FLAGS_UBWC_ALIGNED_PI) {
                            m_sVenc_cfg.inputformat = V4L2_PIX_FMT_NV12_UBWC;
                        }

                        if (color_format > 0 && !venc_set_color_format(color_format)) {
                            DEBUG_PRINT_ERROR("Failed setting color format in Camerasource %lx", m_sVenc_cfg.inputformat);
                            return false;
                        }

                        if(ioctl(m_nDriver_fd,VIDIOC_REQBUFS, &bufreq)) {
                            DEBUG_PRINT_ERROR("VIDIOC_REQBUFS OUTPUT_MPLANE Failed");
                            return false;
                        }
                    }

                    // Setting batch mode is sticky. We do not expect camera to change
                    // between batch and normal modes at runtime.
                    if (mBatchSize) {
                        if ((unsigned int)MetaBufferUtil::getBatchSize(hnd) != mBatchSize) {
                            DEBUG_PRINT_ERROR("Don't support dynamic batch sizes (changed from %d->%d)",
                                    mBatchSize, MetaBufferUtil::getBatchSize(hnd));
                            return false;
                        }

                        return venc_empty_batch ((OMX_BUFFERHEADERTYPE*)buffer, index);
                    }

                    int offset = MetaBufferUtil::getIntAt(hnd, 0, MetaBufferUtil::INT_OFFSET);
                    int length = MetaBufferUtil::getIntAt(hnd, 0, MetaBufferUtil::INT_SIZE);
                    if (offset < 0 || length < 0) {
                        DEBUG_PRINT_ERROR("Invalid meta buffer handle!");
                        return false;
                    }
                    plane[0].data_offset = offset;
                    plane[0].length = length;
                    plane[0].bytesused = length;
                    DEBUG_PRINT_LOW("venc_empty_buf: camera buf: fd = %d filled %d of %d flag 0x%x format 0x%lx",
                            fd, plane[0].bytesused, plane[0].length, buf.flags, m_sVenc_cfg.inputformat);
                } else if (meta_buf->buffer_type == kMetadataBufferTypeGrallocSource) {
                    VideoGrallocMetadata *meta_buf = (VideoGrallocMetadata *)bufhdr->pBuffer;
                    private_handle_t *handle = (private_handle_t *)meta_buf->pHandle;

                    if (!handle) {
                        DEBUG_PRINT_ERROR("%s : handle is null!", __FUNCTION__);
                        return false;
                    }
                    interlace_flag = is_ubwc_interlaced(handle);

                    if (mUseAVTimerTimestamps) {
                        uint64_t avTimerTimestampNs = bufhdr->nTimeStamp * 1000;
                        if (getMetaData(handle, GET_VT_TIMESTAMP, &avTimerTimestampNs) == 0
                                && avTimerTimestampNs > 0) {
                            bufhdr->nTimeStamp = avTimerTimestampNs / 1000;
                            DEBUG_PRINT_LOW("AVTimer TS : %llu us", (unsigned long long)bufhdr->nTimeStamp);
                        }
                    }

                    if (!streaming[OUTPUT_PORT]) {
                        // Moment of truth... actual colorspace is known here..
                        if (getMetaData(handle, GET_COLOR_METADATA, &colorData) == 0) {
                            DEBUG_PRINT_INFO("ENC_CONFIG: gralloc Color MetaData colorPrimaries=%d colorRange=%d "
                                             "transfer=%d matrixcoefficients=%d"
                                             "dynamicMetaDataValid %u dynamicMetaDataLen %u",
                                             colorData.colorPrimaries, colorData.range,
                                             colorData.transfer, colorData.matrixCoefficients,
                                             colorData.dynamicMetaDataValid, colorData.dynamicMetaDataLen);
                        }

                        if (!venc_cvp_enable(handle)) {
                            DEBUG_PRINT_ERROR("ERROR: Setting CVP enable");
                            return false;
                        }

                        struct v4l2_format fmt;
                        memset(&fmt, 0, sizeof(fmt));
                        fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;

                        bool isUBWC = ((handle->flags & private_handle_t::PRIV_FLAGS_UBWC_ALIGNED ||
                                        handle->flags & private_handle_t::PRIV_FLAGS_UBWC_ALIGNED_PI) &&
                                       is_gralloc_source_ubwc);

                        char grallocFormatStr[200];
                        get_gralloc_format_as_string(grallocFormatStr, sizeof(grallocFormatStr), handle->format);
                        DEBUG_PRINT_LOW("gralloc format 0x%x (%s) (%s)",
                            handle->format, grallocFormatStr, isUBWC ? "UBWC" : "Linear");

                        if (m_codec == OMX_VIDEO_CodingHEVC && (handle->format == HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC ||
                            handle->format == HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS) &&
                            codec_profile.profile != V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10)
                            {
                                if (!venc_set_profile (OMX_VIDEO_HEVCProfileMain10)) {
                                    DEBUG_PRINT_ERROR("ERROR: Unsuccessful in updating Profile OMX_VIDEO_HEVCProfileMain10");
                                    return false;
                                }
                            }
                        if (handle->format == HAL_PIXEL_FORMAT_NV12_ENCODEABLE) {
                            m_sVenc_cfg.inputformat = isUBWC ? V4L2_PIX_FMT_NV12_UBWC : V4L2_PIX_FMT_NV12;
                            DEBUG_PRINT_INFO("ENC_CONFIG: Input Color = NV12 %s", isUBWC ? "UBWC" : "Linear");
                        } else if (handle->format == HAL_PIXEL_FORMAT_NV12_HEIF) {
                            m_sVenc_cfg.inputformat = V4L2_PIX_FMT_NV12_512;
                            DEBUG_PRINT_INFO("ENC_CONFIG: Input Color = NV12_512");
                        } else if (handle->format == HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC) {
                            m_sVenc_cfg.inputformat = V4L2_PIX_FMT_NV12_UBWC;
                            DEBUG_PRINT_INFO("ENC_CONFIG: Input Color = NV12_UBWC");
                        } else if (handle->format == VIDC_HAL_PIXEL_FORMAT_NV12_LINEAR_FLEX) {
                             m_sVenc_cfg.inputformat = V4L2_PIX_FMT_NV12;
                             DEBUG_PRINT_INFO("ENC_CONFIG: Input Color = NV12 FLEX");
                        } else if (handle->format == VIDC_HAL_PIXEL_FORMAT_NV12_UBWC_FLEX) {
                            m_sVenc_cfg.inputformat = V4L2_PIX_FMT_NV12_UBWC;
                            DEBUG_PRINT_INFO("ENC_CONFIG: Input Color = NV12 UBWC FLEX");
                        } else if (handle->format == HAL_PIXEL_FORMAT_RGBA_8888) {
                            // In case of RGB, conversion to YUV is handled within encoder.
                            // Disregard the Colorspace in gralloc-handle in case of RGB and use
                            //   [a] 601 for non-UBWC case : C2D output is (apparently) 601-LR
                            //   [b] 601 for UBWC case     : Venus can convert to 601-LR or FR. use LR for now.
                            //set colormetadata corresponding to ITU_R_601;
                            colorData.colorPrimaries =  ColorPrimaries_BT601_6_525;
                            colorData.range = Range_Limited;
                            colorData.transfer = Transfer_SMPTE_170M;
                            colorData.matrixCoefficients = MatrixCoEff_BT601_6_525;
                            m_sVenc_cfg.inputformat = isUBWC ? V4L2_PIX_FMT_RGBA8888_UBWC : V4L2_PIX_FMT_RGB32;
                            DEBUG_PRINT_INFO("ENC_CONFIG: Input Color = RGBA8888 %s", isUBWC ? "UBWC" : "Linear");
                        } else if (handle->format == QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m) {
                            m_sVenc_cfg.inputformat = V4L2_PIX_FMT_NV12;
                            DEBUG_PRINT_INFO("ENC_CONFIG: Input Color = NV12 Linear");
                        } else if (handle->format == HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC ||
                                   handle->format == HAL_PIXEL_FORMAT_YCbCr_420_P010_VENUS) {
                            if (((m_codec == OMX_VIDEO_CodingHEVC) &&
                                 (codec_profile.profile == V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10)) ||
                                     (csc_enable == true)) {
                                m_sVenc_cfg.inputformat =
                                    (handle->format == HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC)?
                                             V4L2_PIX_FMT_NV12_TP10_UBWC :
                                             V4L2_PIX_FMT_SDE_Y_CBCR_H2V2_P010_VENUS;
                                DEBUG_PRINT_INFO("ENC_CONFIG: Input Color = 10bit");
                            } else {
                                DEBUG_PRINT_ERROR("ENC_CONFIG: 10bit colorformat not supported for this codec and profile");
                                return false;
                            }

                            if(colorData.masteringDisplayInfo.colorVolumeSEIEnabled ||
                               colorData.contentLightLevel.lightLevelSEIEnabled) {
                                if (!venc_set_hdr_info(colorData.masteringDisplayInfo, colorData.contentLightLevel)) {
                                    DEBUG_PRINT_ERROR("HDR10-PQ Info Setting failed");
                                    return false;
                                } else {
                                    DEBUG_PRINT_INFO("Encoding in HDR10-PQ mode");
                                }
                            } else {
                                DEBUG_PRINT_INFO("Encoding in HLG mode");
                            }
                        } else if (handle->format == QOMX_COLOR_FormatYVU420SemiPlanar) {
                           m_sVenc_cfg.inputformat = V4L2_PIX_FMT_NV21;
                           DEBUG_PRINT_INFO("ENC_CONFIG: Input Color = NV21 Linear");
                        } else {
                            DEBUG_PRINT_ERROR("Color format is not recoganized. Format 0x%X", handle->format);
                            return false;
                        }

                        DEBUG_PRINT_INFO("color_space.primaries %d colorData.colorPrimaries %d, is_csc_custom_matrix_enabled=%d",
                                         color_space.primaries, colorData.colorPrimaries, is_csc_custom_matrix_enabled);

                        if (csc_enable) {
                            struct v4l2_control control;
                            control.id = V4L2_CID_MPEG_VIDC_VIDEO_VPE_CSC;
                            control.value = V4L2_MPEG_MSM_VIDC_ENABLE;
                            if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control)) {
                                DEBUG_PRINT_ERROR("venc_empty_buf: Failed to set VPE CSC");
                            }
                            else {
                                if (is_csc_custom_matrix_enabled) {
                                    control.id = V4L2_CID_MPEG_VIDC_VIDEO_VPE_CSC_CUSTOM_MATRIX;
                                    control.value = 1;
                                    if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control)) {
                                        DEBUG_PRINT_ERROR("venc_empty_buf: Failed to enable VPE CSC custom matrix");
                                    } else {
                                        DEBUG_PRINT_INFO("venc_empty_buf: Enabled VPE CSC custom matrix");
                                    }
                                }
                            }
                        }

                        /* Enum values from gralloc ColorMetaData matches with the driver values
                           as it is standard compliant */
                        venc_set_colorspace(colorData.colorPrimaries, colorData.range,
                                            colorData.transfer, colorData.matrixCoefficients);
                        if ((handle->format == VIDC_HAL_PIXEL_FORMAT_NV12_LINEAR_FLEX) ||
                                (handle->format == VIDC_HAL_PIXEL_FORMAT_NV12_UBWC_FLEX)) {
                            if (!venc_superframe_enable(handle)) {
                                DEBUG_PRINT_ERROR("ERROR: Enabling Superframe");
                                return false;
                            }
                        }

                        fmt.fmt.pix_mp.pixelformat = m_sVenc_cfg.inputformat;
                        fmt.fmt.pix_mp.height = m_sVenc_cfg.input_height;
                        fmt.fmt.pix_mp.width = m_sVenc_cfg.input_width;
                        if (ioctl(m_nDriver_fd, VIDIOC_S_FMT, &fmt)) {
                            DEBUG_PRINT_ERROR("Failed setting color format in Grallocsource %lx", m_sVenc_cfg.inputformat);
                            return false;
                        }
                        if(ioctl(m_nDriver_fd,VIDIOC_REQBUFS, &bufreq)) {
                            DEBUG_PRINT_ERROR("VIDIOC_REQBUFS OUTPUT_MPLANE Failed");
                            return false;
                        }
                    } else {
                        if (m_hdr10meta_enabled) {
                            if (getMetaData(handle, GET_COLOR_METADATA, &colorData) == 0) {
                                DEBUG_PRINT_INFO("ENC_CONFIG: gralloc Color MetaData dynamicMetaDataValid=%u dynamicMetaDataLen=%u",
                                                 colorData.dynamicMetaDataValid, colorData.dynamicMetaDataLen);
                            }
                        }
                    } // Check OUTPUT Streaming

                    if (!venc_get_cvp_metadata(handle, &buf))
                        return false;

                    struct UBWCStats cam_ubwc_stats[2];
                    unsigned long long int compression_ratio = 1 << 16;

                    if (getMetaData(handle, GET_UBWC_CR_STATS_INFO, (void *)cam_ubwc_stats) == 0) {
                        if (cam_ubwc_stats[0].bDataValid) {
                            switch (cam_ubwc_stats[0].version) {
                            case UBWC_2_0:
                            case UBWC_3_0:
                            case UBWC_4_0:
                                {
                                    unsigned long long int sum = 0, weighted_sum = 0;

                                    DEBUG_PRINT_HIGH("Field 0 : 32 Tile = %d 64 Tile = %d 96 Tile = %d "
                                       "128 Tile = %d 160 Tile = %d 192 Tile = %d 256 Tile = %d\n",
                                       cam_ubwc_stats[0].ubwc_stats.nCRStatsTile32,
                                       cam_ubwc_stats[0].ubwc_stats.nCRStatsTile64,
                                       cam_ubwc_stats[0].ubwc_stats.nCRStatsTile96,
                                       cam_ubwc_stats[0].ubwc_stats.nCRStatsTile128,
                                       cam_ubwc_stats[0].ubwc_stats.nCRStatsTile160,
                                       cam_ubwc_stats[0].ubwc_stats.nCRStatsTile192,
                                       cam_ubwc_stats[0].ubwc_stats.nCRStatsTile256);

                                    weighted_sum =
                                        32  * cam_ubwc_stats[0].ubwc_stats.nCRStatsTile32 +
                                        64  * cam_ubwc_stats[0].ubwc_stats.nCRStatsTile64 +
                                        96  * cam_ubwc_stats[0].ubwc_stats.nCRStatsTile96 +
                                        128 * cam_ubwc_stats[0].ubwc_stats.nCRStatsTile128 +
                                        160 * cam_ubwc_stats[0].ubwc_stats.nCRStatsTile160 +
                                        192 * cam_ubwc_stats[0].ubwc_stats.nCRStatsTile192 +
                                        256 * cam_ubwc_stats[0].ubwc_stats.nCRStatsTile256;

                                    sum =
                                        cam_ubwc_stats[0].ubwc_stats.nCRStatsTile32 +
                                        cam_ubwc_stats[0].ubwc_stats.nCRStatsTile64 +
                                        cam_ubwc_stats[0].ubwc_stats.nCRStatsTile96 +
                                        cam_ubwc_stats[0].ubwc_stats.nCRStatsTile128 +
                                        cam_ubwc_stats[0].ubwc_stats.nCRStatsTile160 +
                                        cam_ubwc_stats[0].ubwc_stats.nCRStatsTile192 +
                                        cam_ubwc_stats[0].ubwc_stats.nCRStatsTile256;

                                    compression_ratio = (weighted_sum && sum) ?
                                        ((256 * sum) << 16) / weighted_sum : compression_ratio;
                                }
                                break;
                            default:
                                break;
                            }
                        }
                    }

                    uint32_t encodePerfMode = 0;
                    if (getMetaData(handle, GET_VIDEO_PERF_MODE, &encodePerfMode) == 0) {
                        if (encodePerfMode == OMX_TRUE)
                            buf.flags |= V4L2_BUF_FLAG_PERF_MODE;
                        // Clear SET_VIDEO_PERF_MODE in buffer handle
                        setMetaData(handle, SET_VIDEO_PERF_MODE, 0);
                    }
                    fd = handle->fd;
                    plane[0].data_offset = 0;
                    plane[0].length = handle->size;
                    plane[0].bytesused = handle->size;
                    plane[0].reserved[MSM_VIDC_COMP_RATIO] = (unsigned long int)compression_ratio;
                    char v4l2ColorFormatStr[200];
                    get_v4l2_color_format_as_string(v4l2ColorFormatStr, sizeof(v4l2ColorFormatStr), m_sVenc_cfg.inputformat);
                    DEBUG_PRINT_LOW("venc_empty_buf: Opaque camera buf: fd = %d "
                                ": filled %d of %d format 0x%lx (%s) CR %d", fd, plane[0].bytesused,
                                plane[0].length, m_sVenc_cfg.inputformat, v4l2ColorFormatStr, plane[0].reserved[MSM_VIDC_COMP_RATIO]);
                }
            } else {
                // Metadata mode
                // color_format == 1 ==> RGBA to YUV Color-converted buffer
                // Buffers color-converted via C2D have 601 color
                if (!streaming[OUTPUT_PORT]) {
                    c2d_enabled = 1;
                    DEBUG_PRINT_HIGH("Setting colorspace 601 for Color-converted buffer");
                    venc_set_colorspace(MSM_VIDC_BT601_6_625, color_space.range,
                            MSM_VIDC_TRANSFER_601_6_525, MSM_VIDC_MATRIX_601_6_525);
                }
                plane[0].m.userptr = (unsigned long) bufhdr->pBuffer;
                plane[0].data_offset = bufhdr->nOffset;
                plane[0].length = bufhdr->nAllocLen;
                plane[0].bytesused = bufhdr->nFilledLen;
                DEBUG_PRINT_LOW("venc_empty_buf: Opaque non-camera buf: fd = %d filled %d of %d",
                        fd, plane[0].bytesused, plane[0].length);
            }
        } else { // Not Metadata mode
            plane[0].m.userptr = (unsigned long) bufhdr->pBuffer;
            plane[0].data_offset = bufhdr->nOffset;
            plane[0].length = bufhdr->nAllocLen;
            plane[0].bytesused = bufhdr->nFilledLen;
            DEBUG_PRINT_LOW("venc_empty_buf: non-camera buf: fd = %d filled %d of %d",
                    fd, plane[0].bytesused, plane[0].length);
        }
    }

    if (!handle_dynamic_config(bufhdr)) {
        DEBUG_PRINT_ERROR("%s Failed to set dynamic configs", __func__);
        return false;
    }

    if (!streaming[OUTPUT_PORT]) {
        enum v4l2_buf_type buf_type;
        buf_type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        int ret;

        // Some 3rd APPs use NativeRecorder to implement their applications
        // like screenrecorder, implicitly enable B-frame may cause issues.
        // So disallow implicit B-frame when input format is non-UBWC or RGBA(c2d enabled).
        if ((m_sVenc_cfg.inputformat != V4L2_PIX_FMT_NV12_TP10_UBWC &&
             m_sVenc_cfg.inputformat != V4L2_PIX_FMT_NV12_UBWC) || c2d_enabled) {
            DEBUG_PRINT_HIGH("Disallow implicitly enable B-frames");
            if (!set_native_recoder(OMX_FALSE)) {
                DEBUG_PRINT_ERROR("Failed to set Native Recorder");
                return false;
            }
        }

        venc_set_quality_boost((OMX_BOOL)c2d_enabled);

        if (!downscalar_enabled) {
            OMX_U32 inp_width = 0, inp_height = 0, out_width = 0, out_height = 0;

            if (!venc_get_dimensions(PORT_INDEX_IN, &inp_width, &inp_height)) {
                return false;
            }

            if (!venc_get_dimensions(PORT_INDEX_OUT, &out_width, &out_height)) {
                return false;
            }

            // Tiling in HEIC requires output WxH to be Tile size; difference is permitted
            if (!(m_codec == OMX_VIDEO_CodingImageHEIC) &&
                inp_width * inp_height != out_width * out_height) {
                DEBUG_PRINT_ERROR("Downscalar is disabled and input/output dimenstions don't match");
                DEBUG_PRINT_ERROR("Input WxH : %dx%d Output WxH : %dx%d",inp_width, inp_height, out_width, out_height);
                return false;
            }
        }

        ret = ioctl(m_nDriver_fd, VIDIOC_STREAMON, &buf_type);

        if (ret) {
            DEBUG_PRINT_ERROR("Failed to call streamon");
            if (errno == EBUSY) {
                hw_overload = true;
            }
            return false;
        } else {
            streaming[OUTPUT_PORT] = true;
        }
    }

    extra_idx = EXTRADATA_IDX(num_input_planes);

    if (extra_idx && (extra_idx < VIDEO_MAX_PLANES)) {
        plane[extra_idx].bytesused = input_extradata_info.buffer_size;
        plane[extra_idx].length = input_extradata_info.buffer_size;
        plane[extra_idx].m.userptr = (unsigned long)input_extradata_info.ion[index].uaddr;
#ifdef USE_ION
        plane[extra_idx].reserved[MSM_VIDC_BUFFER_FD] = input_extradata_info.ion[index].data_fd;
#endif
        plane[extra_idx].reserved[MSM_VIDC_DATA_OFFSET] = 0;
        plane[extra_idx].data_offset = 0;
    } else if (extra_idx >= VIDEO_MAX_PLANES) {
        DEBUG_PRINT_ERROR("Extradata index higher than expected: %d\n", extra_idx);
        return false;
    }

    buf.index = index;
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    buf.memory = V4L2_MEMORY_USERPTR;
    plane[0].reserved[MSM_VIDC_BUFFER_FD] = fd;
    plane[0].reserved[MSM_VIDC_DATA_OFFSET] = 0;
    buf.m.planes = plane;
    buf.length = num_input_planes;
    buf.timestamp.tv_sec = bufhdr->nTimeStamp / 1000000;
    buf.timestamp.tv_usec = (bufhdr->nTimeStamp % 1000000);

    if (!handle_input_extradata(buf)) {
        DEBUG_PRINT_ERROR("%s Failed to handle input extradata", __func__);
        return false;
    }

    VIDC_TRACE_INT_LOW("ETB-TS", bufhdr->nTimeStamp / 1000);

    if (bufhdr->nFlags & OMX_BUFFERFLAG_EOS)
        buf.flags |= V4L2_BUF_FLAG_EOS;

    if (!plane[0].bytesused) {
        if (buf.flags & V4L2_BUF_FLAG_EOS) {
            DEBUG_PRINT_ERROR("venc_empty_buf: Zero length EOS buffers are not valid");
            DEBUG_PRINT_ERROR("Use this function instead : venc_handle_empty_eos_buffer");
            return false;
        }
        DEBUG_PRINT_ERROR("venc_empty_buf: Zero length buffers are not valid");
        return false;
    }

    if (m_debug.in_buffer_log) {
        venc_input_log_buffers(bufhdr, fd, plane[0].data_offset, m_sVenc_cfg.inputformat, interlace_flag);
    }
    if (m_debug.extradata_log && extra_idx && (extra_idx < VIDEO_MAX_PLANES)) {
        DEBUG_PRINT_ERROR("Extradata Addr 0x%llx, Buffer Addr = 0x%x",
            (OMX_U64)input_extradata_info.ion[index].uaddr, (unsigned int)plane[index].m.userptr);
        venc_extradata_log_buffers((char *)plane[extra_idx].m.userptr, index, true);
    }
    print_v4l2_buffer("QBUF-ETB", &buf);
    rc = ioctl(m_nDriver_fd, VIDIOC_QBUF, &buf);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to qbuf (etb) to driver");
        return false;
    }

    etb++;

    return true;
}

unsigned long venc_dev::get_media_colorformat(unsigned long inputformat)
{
    unsigned long color_format;
    switch (inputformat) {
        case V4L2_PIX_FMT_NV12:
            color_format = COLOR_FMT_NV12;
            break;
        case V4L2_PIX_FMT_NV12_512:
            color_format = COLOR_FMT_NV12_512;
            break;
        case V4L2_PIX_FMT_NV12_UBWC:
            color_format = COLOR_FMT_NV12_UBWC;
            break;
        case V4L2_PIX_FMT_RGB32:
            color_format = COLOR_FMT_RGBA8888;
            break;
        case V4L2_PIX_FMT_RGBA8888_UBWC:
            color_format = COLOR_FMT_RGBA8888_UBWC;
            break;
        case V4L2_PIX_FMT_SDE_Y_CBCR_H2V2_P010_VENUS:
            color_format = COLOR_FMT_P010;
            break;
        case V4L2_PIX_FMT_NV12_TP10_UBWC:
            color_format = COLOR_FMT_NV12_BPP10_UBWC;
            break;
        default:
            color_format = COLOR_FMT_NV12_UBWC;
            DEBUG_PRINT_ERROR("Unknown format %lx,default to NV12_UBWC", inputformat);
            break;
    }
    return color_format;
}

bool venc_dev::venc_empty_batch(OMX_BUFFERHEADERTYPE *bufhdr, unsigned index)
{
    struct v4l2_buffer buf;
    struct v4l2_plane plane[VIDEO_MAX_PLANES];
    int rc = 0, extra_idx, numBufs;
    struct v4l2_control control;
    LEGACY_CAM_METADATA_TYPE * meta_buf = NULL;
    native_handle_t *hnd = NULL;

    if (bufhdr == NULL) {
        DEBUG_PRINT_ERROR("ERROR: %s: buffer is NULL", __func__);
        return false;
    }

    bool status = true;
    if (metadatamode) {
        plane[0].m.userptr = index;
        meta_buf = (LEGACY_CAM_METADATA_TYPE *)bufhdr->pBuffer;

        if (!color_format) {
            if (meta_buf->buffer_type == LEGACY_CAM_SOURCE) {
                hnd = (native_handle_t*)meta_buf->meta_handle;
                if (!hnd) {
                    DEBUG_PRINT_ERROR("venc_empty_batch: invalid handle !");
                    return false;
                } else if (MetaBufferUtil::getBatchSize(hnd) > kMaxBuffersInBatch) {
                    DEBUG_PRINT_ERROR("venc_empty_batch: Too many buffers (%d) in batch. "
                            "Max = %d", MetaBufferUtil::getBatchSize(hnd), kMaxBuffersInBatch);
                    status = false;
                }
                DEBUG_PRINT_LOW("venc_empty_batch: Batch of %d bufs", MetaBufferUtil::getBatchSize(hnd));
            } else {
                DEBUG_PRINT_ERROR("Batch supported for CameraSource buffers only !");
                status = false;
            }
        } else {
            DEBUG_PRINT_ERROR("Batch supported for Camera buffers only !");
            status = false;
        }
    } else {
        DEBUG_PRINT_ERROR("Batch supported for metabuffer mode only !");
        status = false;
    }

    if (status) {
        OMX_TICKS bufTimeStamp = 0ll;
        int numBufs = MetaBufferUtil::getBatchSize(hnd);
        int v4l2Ids[kMaxBuffersInBatch] = {-1};
        for (int i = 0; i < numBufs; ++i) {
            v4l2Ids[i] = mBatchInfo.registerBuffer(index);
            if (v4l2Ids[i] < 0) {
                DEBUG_PRINT_ERROR("Failed to register buffer");
                // TODO: cleanup the map and purge all slots of current index
                status = false;
                break;
            }
        }
        for (int i = 0; i < numBufs; ++i) {
            int v4l2Id = v4l2Ids[i];
            int usage = 0;

            memset(&buf, 0, sizeof(buf));
            memset(&plane, 0, sizeof(plane));

            DEBUG_PRINT_LOW("Batch: registering %d as %d", index, v4l2Id);
            buf.index = (unsigned)v4l2Id;
            buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
            buf.memory = V4L2_MEMORY_USERPTR;
            plane[0].reserved[MSM_VIDC_BUFFER_FD] = MetaBufferUtil::getFdAt(hnd, i);
            plane[0].reserved[MSM_VIDC_DATA_OFFSET] = 0;
            plane[0].data_offset = MetaBufferUtil::getIntAt(hnd, i, MetaBufferUtil::INT_OFFSET);
            plane[0].m.userptr = (unsigned long)meta_buf;
            plane[0].length = plane[0].bytesused = MetaBufferUtil::getIntAt(hnd, i, MetaBufferUtil::INT_SIZE);
            buf.m.planes = plane;
            buf.length = num_input_planes;

            usage = MetaBufferUtil::getIntAt(hnd, i, MetaBufferUtil::INT_USAGE);
            usage = usage > 0 ? usage : 0;

            extra_idx = EXTRADATA_IDX(num_input_planes);

            if (extra_idx && (extra_idx < VIDEO_MAX_PLANES)) {
                plane[extra_idx].bytesused = input_extradata_info.buffer_size;
                plane[extra_idx].length = input_extradata_info.buffer_size;
                plane[extra_idx].m.userptr = (unsigned long)input_extradata_info.ion[v4l2Id].uaddr;
                plane[extra_idx].reserved[MSM_VIDC_BUFFER_FD] = input_extradata_info.ion[v4l2Id].data_fd;
                plane[extra_idx].reserved[MSM_VIDC_DATA_OFFSET] = 0;
                plane[extra_idx].data_offset = 0;
            } else if (extra_idx >= VIDEO_MAX_PLANES) {
                DEBUG_PRINT_ERROR("Extradata index higher than expected: %d\n", extra_idx);
                return false;
            }

            if (bufhdr->nFlags & OMX_BUFFERFLAG_EOS)
                buf.flags |= V4L2_BUF_FLAG_EOS;

            // timestamp differences from camera are in nano-seconds
            bufTimeStamp = bufhdr->nTimeStamp + MetaBufferUtil::getIntAt(hnd, i, MetaBufferUtil::INT_TIMESTAMP) / 1000;

            DEBUG_PRINT_LOW(" Q Batch [%d of %d] : buf=%p fd=%d len=%d TS=%lld",
                i, numBufs, bufhdr, plane[0].reserved[MSM_VIDC_BUFFER_FD], plane[0].length, bufTimeStamp);
            buf.timestamp.tv_sec = bufTimeStamp / 1000000;
            buf.timestamp.tv_usec = (bufTimeStamp % 1000000);

            if (!handle_input_extradata(buf)) {
                DEBUG_PRINT_ERROR("%s Failed to handle input extradata", __func__);
                return false;
            }

            if (!handle_dynamic_config(bufhdr)) {
                DEBUG_PRINT_ERROR("%s Failed to set dynamic configs", __func__);
                return false;
            }

            VIDC_TRACE_INT_LOW("ETB-TS", bufTimeStamp / 1000);

            print_v4l2_buffer("QBUF-ETB", &buf);
            rc = ioctl(m_nDriver_fd, VIDIOC_QBUF, &buf);
            if (rc) {
                DEBUG_PRINT_ERROR("%s: Failed to qbuf (etb) to driver", __func__);
                return false;
            }

            etb++;
        }
    }

    if (status && !streaming[OUTPUT_PORT]) {
        enum v4l2_buf_type buf_type;
        buf_type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        int ret;

        if (!downscalar_enabled) {
            OMX_U32 inp_width = 0, inp_height = 0, out_width = 0, out_height = 0;

            if (!venc_get_dimensions(PORT_INDEX_IN, &inp_width, &inp_height)) {
                return false;
            }

            if (!venc_get_dimensions(PORT_INDEX_OUT, &out_width, &out_height)) {
                return false;
            }

            if (inp_width * inp_height != out_width * out_height) {
                DEBUG_PRINT_ERROR("Downscalar is disabled and input/output dimenstions don't match");
                DEBUG_PRINT_ERROR("Input WxH : %dx%d Output WxH : %dx%d",inp_width, inp_height, out_width, out_height);
                return false;
            }
        }

        ret = ioctl(m_nDriver_fd, VIDIOC_STREAMON, &buf_type);
        if (ret) {
            DEBUG_PRINT_ERROR("Failed to call streamon");
            if (errno == EBUSY) {
                hw_overload = true;
            }
            status = false;
        } else {
            streaming[OUTPUT_PORT] = true;
        }
    }

    return status;
}

bool venc_dev::venc_fill_buf(void *buffer, void *pmem_data_buf,unsigned index,unsigned fd)
{
    struct pmem *temp_buffer = NULL;
    struct venc_buffer  frameinfo;
    struct v4l2_buffer buf;
    struct v4l2_plane plane[VIDEO_MAX_PLANES];
    int rc = 0;
    unsigned int extra_idx;
    struct OMX_BUFFERHEADERTYPE *bufhdr;

    if (buffer == NULL)
        return false;

    bufhdr = (OMX_BUFFERHEADERTYPE *)buffer;

    if (pmem_data_buf) {
        DEBUG_PRINT_LOW("Internal PMEM addr for o/p Heap UseBuf: %p", pmem_data_buf);
        plane[0].m.userptr = (unsigned long)pmem_data_buf;
    } else {
        DEBUG_PRINT_LOW("Shared PMEM addr for o/p PMEM UseBuf/AllocateBuf: %p", bufhdr->pBuffer);
        plane[0].m.userptr = (unsigned long)bufhdr->pBuffer;
    }

    memset(&buf, 0, sizeof(buf));
    memset(&plane, 0, sizeof(plane));

    buf.index = index;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_USERPTR;
    plane[0].length = bufhdr->nAllocLen;
    plane[0].bytesused = bufhdr->nFilledLen;
    plane[0].reserved[MSM_VIDC_BUFFER_FD] = fd;
    plane[0].reserved[MSM_VIDC_DATA_OFFSET] = 0;
    plane[0].data_offset = bufhdr->nOffset;
    buf.m.planes = plane;
    buf.length = num_output_planes;
    buf.flags = 0;

    if (venc_handle->is_secure_session()) {
        if (venc_handle->allocate_native_handle) {
            native_handle_t *handle_t = (native_handle_t *)(bufhdr->pBuffer);
            plane[0].length = handle_t->data[3];
        } else {
            output_metabuffer *meta_buf = (output_metabuffer *)(bufhdr->pBuffer);
            native_handle_t *handle_t = meta_buf->nh;
            plane[0].length = handle_t->data[3];
        }
    }

    extra_idx = EXTRADATA_IDX(num_output_planes);

    if (extra_idx && (extra_idx < VIDEO_MAX_PLANES)) {
        plane[extra_idx].bytesused = output_extradata_info.buffer_size;
        plane[extra_idx].length = output_extradata_info.buffer_size;
        plane[extra_idx].m.userptr = (unsigned long)output_extradata_info.ion[index].uaddr;
#ifdef USE_ION
        plane[extra_idx].reserved[MSM_VIDC_BUFFER_FD] = output_extradata_info.ion[index].data_fd;
#endif
        plane[extra_idx].reserved[MSM_VIDC_DATA_OFFSET] = 0;
        plane[extra_idx].data_offset = 0;
    } else if (extra_idx >= VIDEO_MAX_PLANES) {
        DEBUG_PRINT_ERROR("Extradata index higher than expected: %d", extra_idx);
        return false;
    }

    print_v4l2_buffer("QBUF-FTB", &buf);
    rc = ioctl(m_nDriver_fd, VIDIOC_QBUF, &buf);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to qbuf (ftb) to driver");
        return false;
    }

    ftb++;
    return true;
}

bool venc_dev::venc_set_colorspace(OMX_U32 primaries, OMX_U32 range,
    OMX_U32 transfer_chars, OMX_U32 matrix_coeffs)
{
    int rc;
    struct v4l2_control control;

    DEBUG_PRINT_LOW("Setting color space : Primaries = %d, Range = %d, Trans = %d, Matrix = %d",
        primaries, range, transfer_chars, matrix_coeffs);

    control.id = V4L2_CID_MPEG_VIDC_VIDEO_COLOR_SPACE;
    control.value = primaries;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control : V4L2_CID_MPEG_VIDC_VIDEO_COLOR_SPACE");
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    color_space.primaries = control.value;

    control.id = V4L2_CID_MPEG_VIDC_VIDEO_FULL_RANGE;
    control.value = range;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control : V4L2_CID_MPEG_VIDC_VIDEO_FULL_RANGE");
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    color_space.range = control.value;

    control.id = V4L2_CID_MPEG_VIDC_VIDEO_TRANSFER_CHARS;
    control.value = transfer_chars;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control : V4L2_CID_MPEG_VIDC_VIDEO_TRANSFER_CHARS");
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    color_space.transfer_chars = control.value;

    control.id = V4L2_CID_MPEG_VIDC_VIDEO_MATRIX_COEFFS;
    control.value = matrix_coeffs;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control : V4L2_CID_MPEG_VIDC_VIDEO_MATRIX_COEFFS");
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    color_space.matrix_coeffs = control.value;

    return true;
}

bool venc_dev::venc_set_qp(OMX_U32 i_frame_qp, OMX_U32 p_frame_qp,OMX_U32 b_frame_qp, OMX_U32 enable)
{
    int rc;
    struct v4l2_control control;

    OMX_U32 ids[3] = {
        V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP,
        V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP,
        V4L2_CID_MPEG_VIDEO_HEVC_B_FRAME_QP
    };

    OMX_U32 qp_values[3] = {
        i_frame_qp,
        p_frame_qp,
        b_frame_qp
    };
    unsigned long *session_qp_values[3] = {
        &session_qp.iframeqp,
        &session_qp.pframeqp,
        &session_qp.bframeqp
    };
    OMX_U32 qp_mask[3] = {
        ENABLE_I_QP,
        ENABLE_P_QP,
        ENABLE_B_QP
    };

    for(int i=0; i<3; i++) {
        if (enable & qp_mask[i]) {
            control.id = ids[i];
            control.value = qp_values[i];

            DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
            rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

            if (rc) {
                DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
                return false;
            }
            DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);
            *session_qp_values[i] = control.value;
        }
    }

    return true;
}

bool venc_dev::set_nP_frames(unsigned long nPframes)
{
    struct v4l2_control control;

    control.id = V4L2_CID_MPEG_VIDEO_GOP_SIZE;
    control.value = (nPframes > VENC_INFINITE_GOP) ? VENC_INFINITE_GOP : nPframes;
    int rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }
    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    return true;
}

bool venc_dev::set_nB_frames(unsigned long nBframes)
{
    struct v4l2_control control;

    control.id = V4L2_CID_MPEG_VIDEO_B_FRAMES;
    control.value = (nBframes > VENC_INFINITE_GOP) ? VENC_INFINITE_GOP : nBframes;
    int rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }
    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    return true;
}

bool venc_dev::set_native_recoder(bool enable)
{
    struct v4l2_control control;

    control.id = V4L2_CID_MPEG_VIDC_VENC_NATIVE_RECORDER;
    control.value = enable;
    int rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }
    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    return true;
}

bool venc_dev::venc_set_intra_refresh()
{
    int rc;
    struct v4l2_control control_mode;

    // Default is RANDOM mode
    control_mode.id = V4L2_CID_MPEG_VIDC_VIDEO_INTRA_REFRESH_RANDOM;
    // Set intra refresh period (frame count) for Random mode
    control_mode.value  = intra_refresh.framecount;

    if (intra_refresh.irmode == OMX_VIDEO_IntraRefreshCyclic) {
        control_mode.id = V4L2_CID_MPEG_VIDEO_CYCLIC_INTRA_REFRESH_MB;
        control_mode.value  = intra_refresh.mbcount;
    }

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%u, val=%d",
                    control_mode.id, control_mode.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control_mode);

    if (rc) {
        DEBUG_PRINT_HIGH("Failed to set control, id %#x, value %d",
                         control_mode.id, control_mode.value);
        // This key is ignored if the video encoder does not support the intra refresh feature.
        // From android developer reference documentation.
    }

    return true;
}

bool venc_dev::venc_set_target_bitrate(OMX_U32 nTargetBitrate)
{
    DEBUG_PRINT_LOW("venc_set_target_bitrate: bitrate = %u",
            (unsigned int)nTargetBitrate);
    struct v4l2_control control;
    int rc = 0;
    OMX_U32 ids[2] = {
        V4L2_CID_MPEG_VIDC_COMPRESSION_QUALITY,
        V4L2_CID_MPEG_VIDEO_BITRATE
    };

    control.id =  ids[!!(rate_ctrl.rcmode ^ V4L2_MPEG_VIDEO_BITRATE_MODE_CQ)];
    control.value = nTargetBitrate;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }

    bitrate.target_bitrate = control.value;
    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    return true;
}

bool venc_dev::venc_set_encode_framerate(OMX_U32 encode_framerate)
{
    int rc = 0;
    struct venc_framerate frame_rate_cfg;
    struct v4l2_control control;

    control.id =  V4L2_CID_MPEG_VIDC_VIDEO_FRAME_RATE;
    control.value = encode_framerate;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }

    Q16ToFraction(encode_framerate,frame_rate_cfg.fps_numerator,frame_rate_cfg.fps_denominator);
    m_sVenc_cfg.fps_den = frame_rate_cfg.fps_denominator;
    m_sVenc_cfg.fps_num = frame_rate_cfg.fps_numerator;

    return true;
}

unsigned long venc_dev::venc_get_color_format(OMX_COLOR_FORMATTYPE eColorFormat)
{
    unsigned long format = V4L2_DEFAULT_OUTPUT_COLOR_FMT;

    switch ((int)eColorFormat) {
    case OMX_COLOR_FormatYUV420SemiPlanar:
    case QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m:
        format = V4L2_PIX_FMT_NV12;
        break;
    case QOMX_COLOR_FormatYVU420SemiPlanar:
        format = V4L2_PIX_FMT_NV21;
        break;
    case QOMX_COLOR_FORMATYUV420PackedSemiPlanar32mCompressed:
        format = V4L2_PIX_FMT_NV12_UBWC;
        break;
    case QOMX_COLOR_Format32bitRGBA8888:
        format = V4L2_PIX_FMT_RGB32;
        break;
    case QOMX_COLOR_Format32bitRGBA8888Compressed:
        format = V4L2_PIX_FMT_RGBA8888_UBWC;
        break;
    case QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m10bitCompressed:
        format = V4L2_PIX_FMT_NV12_TP10_UBWC;
        break;
    case QOMX_COLOR_FORMATYUV420SemiPlanarP010Venus:
        format = V4L2_PIX_FMT_SDE_Y_CBCR_H2V2_P010_VENUS;
        break;
    default:
        DEBUG_PRINT_INFO("WARN: Unsupported eColorFormat %#x", eColorFormat);
        format = V4L2_DEFAULT_OUTPUT_COLOR_FMT;
        break;
    }

    if (m_codec == OMX_VIDEO_CodingImageHEIC)
        format = V4L2_PIX_FMT_NV12_512;

    return format;
}

bool venc_dev::venc_set_color_format(OMX_COLOR_FORMATTYPE color_format)
{
    struct v4l2_format fmt;
    int color_space = 0;
    DEBUG_PRINT_LOW("venc_set_color_format: color_format = %u ", color_format);

    switch ((int)color_format) {
        case OMX_COLOR_FormatYUV420SemiPlanar:
        case QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m:
            m_sVenc_cfg.inputformat = V4L2_PIX_FMT_NV12;
            color_space = V4L2_COLORSPACE_470_SYSTEM_BG;
            break;
        case QOMX_COLOR_FormatYVU420SemiPlanar:
            m_sVenc_cfg.inputformat = V4L2_PIX_FMT_NV21;
            color_space = V4L2_COLORSPACE_470_SYSTEM_BG;
            break;
        case QOMX_COLOR_FORMATYUV420PackedSemiPlanar32mCompressed:
            m_sVenc_cfg.inputformat = V4L2_PIX_FMT_NV12_UBWC;
            color_space = V4L2_COLORSPACE_470_SYSTEM_BG;
            break;
        case QOMX_COLOR_Format32bitRGBA8888:
            m_sVenc_cfg.inputformat = V4L2_PIX_FMT_RGB32;
            break;
        case QOMX_COLOR_Format32bitRGBA8888Compressed:
            m_sVenc_cfg.inputformat = V4L2_PIX_FMT_RGBA8888_UBWC;
            break;
        case QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m10bitCompressed:
            m_sVenc_cfg.inputformat = V4L2_PIX_FMT_NV12_TP10_UBWC;
            break;
        case QOMX_COLOR_FORMATYUV420SemiPlanarP010Venus:
            m_sVenc_cfg.inputformat = V4L2_PIX_FMT_SDE_Y_CBCR_H2V2_P010_VENUS;
            break;
        default:
            DEBUG_PRINT_HIGH("WARNING: Unsupported Color format [%d]", color_format);
            m_sVenc_cfg.inputformat = V4L2_DEFAULT_OUTPUT_COLOR_FMT;
            color_space = V4L2_COLORSPACE_470_SYSTEM_BG;
            DEBUG_PRINT_HIGH("Default color format NV12 UBWC is set");
            break;
    }

    if (m_codec == OMX_VIDEO_CodingImageHEIC)
        m_sVenc_cfg.inputformat = V4L2_PIX_FMT_NV12_512;

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    fmt.fmt.pix_mp.pixelformat = m_sVenc_cfg.inputformat;
    fmt.fmt.pix_mp.colorspace = color_space;
    fmt.fmt.pix_mp.height = m_sVenc_cfg.input_height;
    fmt.fmt.pix_mp.width = m_sVenc_cfg.input_width;

    if (ioctl(m_nDriver_fd, VIDIOC_S_FMT, &fmt)) {
        DEBUG_PRINT_ERROR("Failed setting color format %x", color_format);
        return false;
    }

    return true;
}

bool venc_dev::venc_set_intra_vop_refresh(OMX_BOOL intra_vop_refresh)
{
    DEBUG_PRINT_LOW("venc_set_intra_vop_refresh: intra_vop = %uc", intra_vop_refresh);

    if (intra_vop_refresh == OMX_TRUE) {
        struct v4l2_control control;
        int rc;
        control.id = V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME;
        control.value = 1;

        rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);
        if (rc) {
            DEBUG_PRINT_ERROR("Failed to set Intra Frame Request control");
            return false;
        }
        DEBUG_PRINT_HIGH("Success IOCTL set control for id=%x, value=%d", control.id, control.value);
    } else {
        DEBUG_PRINT_ERROR("ERROR: VOP Refresh is False, no effect");
    }

    return true;
}

OMX_ERRORTYPE venc_dev::venc_set_max_hierp_layer()
{
    DEBUG_PRINT_LOW("venc_set_max_hierp_layer");
    struct v4l2_control control;

    DEBUG_PRINT_LOW("Setting hierp max layer: %u",
                    temporal_layers_config.nMaxLayers);

    control.id = V4L2_CID_MPEG_VIDC_VIDEO_HEVC_MAX_HIER_CODING_LAYER;
    control.value = temporal_layers_config.nMaxLayers;

    if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control)) {
        DEBUG_PRINT_ERROR("Failed to set max HP layers: %u", control.value);
        return OMX_ErrorUnsupportedSetting;
    }
    return OMX_ErrorNone;
}

OMX_ERRORTYPE venc_dev::venc_set_hierp_layer()
{
    DEBUG_PRINT_LOW("venc_set_hierp_layer");
    struct v4l2_control control;

    DEBUG_PRINT_LOW("Setting hierp layer: %u", temporal_layers_config.nPLayers);

    control.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_LAYER;
    control.value = temporal_layers_config.nPLayers;

    if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control)) {
        DEBUG_PRINT_ERROR("Failed to set HP layers: %u", control.value);
        return OMX_ErrorUnsupportedSetting;
    }
    return OMX_ErrorNone;
}

bool venc_dev::venc_set_ltrcount(OMX_U32 count)
{
    DEBUG_PRINT_LOW("venc_set_ltrcount: count = %u", (unsigned int)count);
    struct v4l2_control control;

    control.id = V4L2_CID_MPEG_VIDC_VIDEO_LTRCOUNT;
    control.value = count;

    if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control)) {
        DEBUG_PRINT_ERROR("Failed to set LTR count: %u", control.value);
        return OMX_ErrorUnsupportedSetting;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%x, val=%d",
                    control.id, control.value);
    return true;
}

bool venc_dev::venc_set_useltr(OMX_U32 frameIdx)
{
    DEBUG_PRINT_LOW("venc_use_goldenframe");
    int rc = true;
    struct v4l2_control control;

    control.id = V4L2_CID_MPEG_VIDC_VIDEO_USELTRFRAME;
    control.value = frameIdx;

    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set use_ltr %d", rc);
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%x, val=%d",
                    control.id, control.value);
    return true;
}

bool venc_dev::venc_set_markltr(OMX_U32 frameIdx)
{
    DEBUG_PRINT_LOW("venc_set_goldenframe");
    int rc = true;
    struct v4l2_control control;

    control.id = V4L2_CID_MPEG_VIDC_VIDEO_MARKLTRFRAME;
    control.value = frameIdx;

    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set markltr %d", rc);
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%x, val=%d",
                    control.id, control.value);
    return true;
}

bool venc_dev::venc_set_mirror(OMX_MIRRORTYPE mirror)
{
    DEBUG_PRINT_LOW("venc_set_mirror");
    int rc = true;
    struct v4l2_control control[2];

    control[0].id = V4L2_CID_VFLIP;
    control[0].value = V4L2_MPEG_MSM_VIDC_DISABLE;
    control[1].id = V4L2_CID_HFLIP;
    control[1].value = V4L2_MPEG_MSM_VIDC_DISABLE;

    if (mirror == OMX_MirrorVertical || mirror == OMX_MirrorBoth) {
        control[0].value = V4L2_MPEG_MSM_VIDC_ENABLE;
    }
    if (mirror == OMX_MirrorHorizontal || mirror == OMX_MirrorBoth) {
        control[1].value = V4L2_MPEG_MSM_VIDC_ENABLE;
    }

    for(int i=0; i<2; i++) {
        rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control[i]);
        if (rc) {
            DEBUG_PRINT_ERROR("Failed to set mirror %d", rc);
            return false;
        }
        DEBUG_PRINT_LOW("Success IOCTL set control for id=%x, val=%d",
                        control[i].id, control[i].value);
    }

    return true;
}

bool venc_dev::venc_set_vpe_rotation(OMX_S32 rotation_angle)
{
    DEBUG_PRINT_LOW("venc_set_vpe_rotation: rotation angle = %d", (int)rotation_angle);
    struct v4l2_control control;
    int rc;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers bufreq;

    if ((OMX_S32)m_rotation.rotation == rotation_angle) {
        DEBUG_PRINT_HIGH("venc_set_vpe_rotation: rotation (%d) not changed", rotation_angle);
        return true;
    }

    control.id = V4L2_CID_ROTATE;
    control.value = rotation_angle;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%x, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);
    if (rc) {
        DEBUG_PRINT_HIGH("Failed to set VPE Rotation control");
        return false;
    }
    DEBUG_PRINT_LOW("Success IOCTL set control for id=%x, value=%d", control.id, control.value);

    /* successfully set rotation_angle, save it */
    m_rotation.rotation = rotation_angle;

    return true;
}

bool venc_dev::venc_prepare_c2d_rotation(OMX_S32 rotation_angle)
{
   int rc;
   struct v4l2_format fmt;
   struct v4l2_requestbuffers bufreq;
   OMX_PARAM_PORTDEFINITIONTYPE *portDefn;

    DEBUG_PRINT_HIGH("venc_prepare_c2d_rotation angle = %d", (int)rotation_angle);
    if ((OMX_S32)m_rotation.rotation == rotation_angle) {
        DEBUG_PRINT_HIGH("venc_prepare_c2d_rotation: rotation (%d) not changed", rotation_angle);
        return true;
    }

    if (rotation_angle == 90 || rotation_angle == 270) {
        m_bDimensionsNeedFlip = true;
        portDefn = &venc_handle->m_sInPortDef;
        m_sVenc_cfg.input_height = portDefn->format.video.nFrameWidth;
        m_sVenc_cfg.input_width =  portDefn->format.video.nFrameHeight;

        memset(&fmt, 0, sizeof(fmt));

        fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        if (ioctl(m_nDriver_fd, VIDIOC_G_FMT, &fmt)) {
            DEBUG_PRINT_ERROR("Failed to get format on OUTPUT_MPLANE");
            return false;
        }

        fmt.fmt.pix_mp.height = m_sVenc_cfg.input_height;
        fmt.fmt.pix_mp.width = m_sVenc_cfg.input_width;
        if (ioctl(m_nDriver_fd, VIDIOC_S_FMT, &fmt)) {
            DEBUG_PRINT_ERROR("set format failed, type %d, wxh %dx%d",
                fmt.type, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height);
                hw_overload = errno == EBUSY;
                return false;
        }
        m_sInput_buff_property.datasize=fmt.fmt.pix_mp.plane_fmt[0].sizeimage;

        portDefn = &venc_handle->m_sOutPortDef;
        m_sVenc_cfg.dvs_width  = portDefn->format.video.nFrameHeight;
        m_sVenc_cfg.dvs_height = portDefn->format.video.nFrameWidth;

        memset(&fmt, 0, sizeof(fmt));

        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        if (ioctl(m_nDriver_fd, VIDIOC_G_FMT, &fmt)) {
            DEBUG_PRINT_ERROR("Failed to get format on CAPTURE_MPLANE");
            return false;
        }

        fmt.fmt.pix_mp.height = m_sVenc_cfg.dvs_height;
        fmt.fmt.pix_mp.width = m_sVenc_cfg.dvs_width;
        if (ioctl(m_nDriver_fd, VIDIOC_S_FMT, &fmt)) {
            DEBUG_PRINT_ERROR("VIDIOC_S_FMT CAPTURE_MPLANE Failed");
            hw_overload = errno == EBUSY;
            return false;
        }

        m_sOutput_buff_property.datasize = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
    }

    /* successfully set rotation_angle, save it */
    m_rotation.rotation = rotation_angle;

    return true;
}

bool venc_dev::venc_set_baselayerid(OMX_U32 baseid)
{
    struct v4l2_control control;
    if (temporal_layers_config.hier_mode == HIER_P) {
        control.id = V4L2_CID_MPEG_VIDC_VIDEO_BASELAYER_ID;
        control.value = baseid;
        DEBUG_PRINT_LOW("Going to set V4L2_CID_MPEG_VIDC_VIDEO_BASELAYER_ID");
        if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control)) {
            DEBUG_PRINT_ERROR("Failed to set V4L2_CID_MPEG_VIDC_VIDEO_BASELAYER_ID");
            return false;
        }
        return true;
    } else {
        DEBUG_PRINT_ERROR("Invalid mode set for V4L2_CID_MPEG_VIDC_VIDEO_BASELAYER_ID: %d",
                temporal_layers_config.hier_mode);
        return false;
    }
}

bool venc_dev::venc_set_priority(OMX_U32 priority) {
    struct v4l2_control control;

    DEBUG_PRINT_LOW("venc_set_priority: %s", priority ? "NonRealTime" : "RealTime");
    control.id = V4L2_CID_MPEG_VIDC_VIDEO_PRIORITY;
    if (priority == 0)
        control.value = V4L2_MPEG_MSM_VIDC_ENABLE;
    else
        control.value = V4L2_MPEG_MSM_VIDC_DISABLE;

    if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control)) {
        DEBUG_PRINT_ERROR("Failed to set V4L2_MPEG_VIDC_VIDEO_PRIORITY_REALTIME_%s",
                priority == 0 ? "ENABLE" : "DISABLE");
        return false;
    }
    return true;
}

bool venc_dev::reconfigure_avc_param(OMX_VIDEO_PARAM_AVCTYPE *param) {
    param->eProfile = (OMX_VIDEO_AVCPROFILETYPE)QOMX_VIDEO_AVCProfileMain;

    DEBUG_PRINT_LOW("reconfigure_avc_param");

    if (!venc_set_profile (param->eProfile)) {
        DEBUG_PRINT_ERROR("ERROR: Unsuccessful in updating Profile %d",
            param->eProfile);
        return false;
    }
    if (set_nP_frames(param->nPFrames) == false ||
        (param->nBFrames && set_nB_frames(param->nBFrames) == false)) {
            DEBUG_PRINT_ERROR("ERROR: Request for setting intra period failed");
            return false;
    }
    if (!venc_set_entropy_config (param->bEntropyCodingCABAC, param->nCabacInitIdc)) {
        DEBUG_PRINT_ERROR("ERROR: Request for setting Entropy failed");
        return false;
    }
    if (!venc_set_inloop_filter (param->eLoopFilterMode)) {
        DEBUG_PRINT_ERROR("ERROR: Request for setting Inloop filter failed");
        return false;
    }
    if (!venc_set_multislice_cfg(V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_MB, param->nSliceHeaderSpacing)) {
        DEBUG_PRINT_ERROR("WARNING: Unsuccessful in updating slice_config");
        return false;
    }
    if (!venc_h264_transform_8x8(param->bDirect8x8Inference)) {
        DEBUG_PRINT_ERROR("WARNING: Request for setting Transform8x8 failed");
        return false;
    }

    return true;
}

bool venc_dev::venc_set_operatingrate(OMX_U32 rate) {
    struct v4l2_control control;

    control.id = V4L2_CID_MPEG_VIDC_VIDEO_OPERATING_RATE;
    control.value = rate;

    if (rate > INT_MAX)
        control.value = INT_MAX;

    DEBUG_PRINT_LOW("venc_set_operating_rate: %u fps", rate >> 16);
    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%u", control.id, control.value);

    if (!strncmp(venc_handle->m_platform, "bengal", 6) &&
        (rate >> 16) > 30 && m_sVenc_cfg.codectype == V4L2_PIX_FMT_H264 &&
        venc_handle->m_sParamAVC.eProfile ==
            (OMX_VIDEO_AVCPROFILETYPE)QOMX_VIDEO_AVCProfileHigh &&
        (m_sVenc_cfg.input_width * m_sVenc_cfg.input_height >= 1920 * 1080)) {
        if (!reconfigure_avc_param(&venc_handle->m_sParamAVC)) {
            DEBUG_PRINT_ERROR("reconfigure avc param fails");
        }
    }

    if(ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control)) {
        hw_overload = errno == EBUSY;
        DEBUG_PRINT_ERROR("Failed to set operating rate %d fps (%s)",
                rate >> 16, hw_overload ? "HW overload" : strerror(errno));
        return false;
    }

    operating_rate = rate >> 16;
    DEBUG_PRINT_LOW("Operating Rate Set = %d fps",  rate >> 16);

    return true;
}

bool venc_dev::venc_set_roi_qp_info(OMX_QTI_VIDEO_CONFIG_ROIINFO *roiInfo)
{
    struct roidata roi;

    if (!m_roi_enabled) {
        DEBUG_PRINT_ERROR("ROI info not enabled");
        return false;
    }

    if (!roiInfo) {
        DEBUG_PRINT_ERROR("No ROI info present");
        return false;
    }
    if (m_sVenc_cfg.codectype != V4L2_PIX_FMT_H264 &&
    m_sVenc_cfg.codectype != V4L2_PIX_FMT_HEVC) {
        DEBUG_PRINT_ERROR("OMX_QTIIndexConfigVideoRoiInfo is not supported for %d codec", (OMX_U32) m_sVenc_cfg.codectype);
        return false;
    }

    memset(&roi, 0, sizeof(struct roidata));

    roi.info.nRoiMBInfoCount = roiInfo->nRoiMBInfoCount;
    roi.info.nTimeStamp = roiInfo->nTimeStamp;
    memcpy(roi.info.pRoiMBInfo, roiInfo->pRoiMBInfo, roiInfo->nRoiMBInfoCount);

    roi.dirty = true;

    pthread_mutex_lock(&m_roilock);
    DEBUG_PRINT_LOW("list add roidata with timestamp %lld us.", roi.info.nTimeStamp);
    m_roilist.push_back(roi);
    pthread_mutex_unlock(&m_roilock);

    return true;
}

bool venc_dev::venc_set_blur_resolution(OMX_QTI_VIDEO_CONFIG_BLURINFO *blurInfo)
{
    struct v4l2_control ctrl;

    ctrl.id = V4L2_CID_MPEG_VIDC_VIDEO_BLUR_DIMENSIONS;
    ctrl.value = blurInfo->nBlurInfo;

    if(ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &ctrl)) {
        DEBUG_PRINT_ERROR("Failed to set blur resoltion");
        return false;
    }

    return true;
}

bool venc_dev::venc_h264_transform_8x8(OMX_BOOL enable)
{
    struct v4l2_control control;

    control.id = V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM;
    if (enable)
        control.value = V4L2_MPEG_MSM_VIDC_ENABLE;
    else
        control.value = V4L2_MPEG_MSM_VIDC_DISABLE;

    DEBUG_PRINT_LOW("Set h264_transform_8x8 mode: %d", control.value);
    if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control)) {
        DEBUG_PRINT_ERROR("set control: H264 transform 8x8 failed");
        return false;
    }

    return true;
}

bool venc_dev::venc_get_temporal_layer_caps(OMX_U32 *nMaxLayers,
        OMX_U32 *nMaxBLayers, OMX_VIDEO_ANDROID_TEMPORALLAYERINGPATTERNTYPE *eSupportedPattern) {

    (void) nMaxBLayers;
    struct v4l2_queryctrl query_ctrl;

    if(m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC || m_sVenc_cfg.codectype == V4L2_PIX_FMT_H264) {
        *eSupportedPattern = OMX_VIDEO_AndroidTemporalLayeringPatternAndroid;
    } else if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_VP8) {
        *eSupportedPattern = OMX_VIDEO_AndroidTemporalLayeringPatternWebRTC;
    } else {
        *eSupportedPattern = OMX_VIDEO_AndroidTemporalLayeringPatternNone;
    }

    query_ctrl.id = V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_LAYER;

    DEBUG_PRINT_LOW("TemporalLayer: Querying P layer caps");
    if (ioctl(m_nDriver_fd, VIDIOC_QUERYCTRL, &query_ctrl)) {
        DEBUG_PRINT_ERROR("TemporalLayer: Query control P layer caps failed");
        return false;
    }

    //Return +1 as driver works on num max enhancement layers and OMX on num layers
    *nMaxLayers = query_ctrl.maximum + 1;

    return true;
}

OMX_ERRORTYPE venc_dev::venc_set_bitrate_ratios()
{
    struct v4l2_control ctrl;
    int rc = 0;
    OMX_U32 ids[] = {
        V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L0_BR,
        V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L1_BR,
        V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L2_BR,
        V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L3_BR,
        V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L4_BR,
        V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_L5_BR,
    };

    DEBUG_PRINT_LOW("TemporalLayer: layerwise bitrate ratio");

    // Set all bitrate ratios to kernel. If client hasn't set bitrate ratio
    // for a layer, 0 is passed on to kernel.
    for (OMX_U32 i = 0; i < (OMX_U32)(sizeof(ids)/sizeof(ids[0])); ++i) {
        ctrl.id = ids[i];
        ctrl.value = temporal_layers_config.nTemporalLayerBitrateRatio[i];

        rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &ctrl);
        if (rc) {
            DEBUG_PRINT_ERROR("Failed to set layerwise bitrate ratio. Id= %u, Value= %u, error %d",
                              ctrl.id, ctrl.value, rc);
            return OMX_ErrorUnsupportedSetting;
        }
        DEBUG_PRINT_LOW("Layerwise bitrate configured successfully for layer: %u, bitrate: %u",
                        i, temporal_layers_config.nTemporalLayerBitrateRatio[i]);
    }
	return OMX_ErrorNone;
}

bool venc_dev::venc_get_hevc_profile(OMX_U32* profile)
{
    if (profile == nullptr) return false;

    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC) {
        if(profile_level_converter::convert_v4l2_profile_to_omx(V4L2_PIX_FMT_HEVC, codec_profile.profile, (int*)profile)) {
            return true;
        } else return false;
    } else return false;
}

bool venc_dev::venc_get_profile_level(OMX_U32 *eProfile,OMX_U32 *eLevel)
{
    bool status = true;

    if (eProfile == NULL || eLevel == NULL) {
        return false;
    }

    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_H264) {
        switch (codec_profile.profile) {
            case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE:
                *eProfile = OMX_VIDEO_AVCProfileBaseline;
                break;
            case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE:
                *eProfile = QOMX_VIDEO_AVCProfileConstrainedBaseline;
                break;
            case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_HIGH:
                *eProfile = QOMX_VIDEO_AVCProfileConstrainedHigh;
                break;
            case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN:
                *eProfile = OMX_VIDEO_AVCProfileMain;
                break;
            case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH:
                *eProfile = OMX_VIDEO_AVCProfileHigh;
                break;
            case V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED:
                *eProfile = OMX_VIDEO_AVCProfileExtended;
                break;
            case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10:
                *eProfile = OMX_VIDEO_AVCProfileHigh10;
                break;
            case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422:
                *eProfile = OMX_VIDEO_AVCProfileHigh422;
                break;
            case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE:
                *eProfile = OMX_VIDEO_AVCProfileHigh444;
                break;
            default:
                *eProfile = OMX_VIDEO_AVCProfileMax;
                status = false;
                break;
        }

        if (!status) {
            return status;
        }

        switch (profile_level.level) {
            case V4L2_MPEG_VIDEO_H264_LEVEL_1_0:
                *eLevel = OMX_VIDEO_AVCLevel1;
                break;
            case V4L2_MPEG_VIDEO_H264_LEVEL_1B:
                *eLevel = OMX_VIDEO_AVCLevel1b;
                break;
            case V4L2_MPEG_VIDEO_H264_LEVEL_1_1:
                *eLevel = OMX_VIDEO_AVCLevel11;
                break;
            case V4L2_MPEG_VIDEO_H264_LEVEL_1_2:
                *eLevel = OMX_VIDEO_AVCLevel12;
                break;
            case V4L2_MPEG_VIDEO_H264_LEVEL_1_3:
                *eLevel = OMX_VIDEO_AVCLevel13;
                break;
            case V4L2_MPEG_VIDEO_H264_LEVEL_2_0:
                *eLevel = OMX_VIDEO_AVCLevel2;
                break;
            case V4L2_MPEG_VIDEO_H264_LEVEL_2_1:
                *eLevel = OMX_VIDEO_AVCLevel21;
                break;
            case V4L2_MPEG_VIDEO_H264_LEVEL_2_2:
                *eLevel = OMX_VIDEO_AVCLevel22;
                break;
            case V4L2_MPEG_VIDEO_H264_LEVEL_3_0:
                *eLevel = OMX_VIDEO_AVCLevel3;
                break;
            case V4L2_MPEG_VIDEO_H264_LEVEL_3_1:
                *eLevel = OMX_VIDEO_AVCLevel31;
                break;
            case V4L2_MPEG_VIDEO_H264_LEVEL_3_2:
                *eLevel = OMX_VIDEO_AVCLevel32;
                break;
            case V4L2_MPEG_VIDEO_H264_LEVEL_4_0:
                *eLevel = OMX_VIDEO_AVCLevel4;
                break;
            case V4L2_MPEG_VIDEO_H264_LEVEL_4_1:
                *eLevel = OMX_VIDEO_AVCLevel41;
                break;
            case V4L2_MPEG_VIDEO_H264_LEVEL_4_2:
                *eLevel = OMX_VIDEO_AVCLevel42;
                break;
            case V4L2_MPEG_VIDEO_H264_LEVEL_5_0:
                *eLevel = OMX_VIDEO_AVCLevel5;
                break;
            case V4L2_MPEG_VIDEO_H264_LEVEL_5_1:
                *eLevel = OMX_VIDEO_AVCLevel51;
                break;
            case V4L2_MPEG_VIDEO_H264_LEVEL_5_2:
                *eLevel = OMX_VIDEO_AVCLevel52;
                break;
            default :
                *eLevel = OMX_VIDEO_AVCLevelMax;
                status = false;
                break;
        }
    } else if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_VP8) {
        switch (codec_profile.profile) {
            case V4L2_MPEG_VIDC_VIDEO_VP8_UNUSED:
                *eProfile = OMX_VIDEO_VP8ProfileMain;
                break;
            default:
                *eProfile = OMX_VIDEO_VP8ProfileMax;
                status = false;
                break;
        }
        if (!status) {
            return status;
        }

        switch (profile_level.level) {
            case V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_0:
                *eLevel = OMX_VIDEO_VP8Level_Version0;
                break;
            case V4L2_MPEG_VIDC_VIDEO_VP8_VERSION_1:
                *eLevel = OMX_VIDEO_VP8Level_Version1;
                break;
            default:
                *eLevel = OMX_VIDEO_VP8LevelMax;
                status = false;
                break;
        }
    } else if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC) {
        switch (codec_profile.profile) {
            case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN:
                *eProfile = OMX_VIDEO_HEVCProfileMain;
                break;
            case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10:
                *eProfile = OMX_VIDEO_HEVCProfileMain10HDR10;
                break;
            case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE:
                *eProfile = OMX_VIDEO_HEVCProfileMainStill;
                break;
            default:
                *eProfile = OMX_VIDEO_HEVCProfileMax;
                status = false;
                break;
        }
        if (!status) {
            return status;
        }

        switch (profile_level.level) {
            case V4L2_MPEG_VIDEO_HEVC_LEVEL_1:
                *eLevel = (profile_level.tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN)?
                                     OMX_VIDEO_HEVCMainTierLevel1:OMX_VIDEO_HEVCHighTierLevel1;
                break;
            case V4L2_MPEG_VIDEO_HEVC_LEVEL_2:
                *eLevel = (profile_level.tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN)?
                                     OMX_VIDEO_HEVCMainTierLevel2:OMX_VIDEO_HEVCHighTierLevel2;
                break;
            case V4L2_MPEG_VIDEO_HEVC_LEVEL_2_1:
                *eLevel = (profile_level.tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN)?
                                     OMX_VIDEO_HEVCMainTierLevel21:OMX_VIDEO_HEVCHighTierLevel21;
                break;
            case V4L2_MPEG_VIDEO_HEVC_LEVEL_3:
                *eLevel = (profile_level.tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN)?
                                     OMX_VIDEO_HEVCMainTierLevel3:OMX_VIDEO_HEVCHighTierLevel3;
                break;
            case V4L2_MPEG_VIDEO_HEVC_LEVEL_3_1:
                *eLevel = (profile_level.tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN)?
                                     OMX_VIDEO_HEVCMainTierLevel31:OMX_VIDEO_HEVCHighTierLevel31;
                break;
            case V4L2_MPEG_VIDEO_HEVC_LEVEL_4:
                *eLevel = (profile_level.tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN)?
                                     OMX_VIDEO_HEVCMainTierLevel4:OMX_VIDEO_HEVCHighTierLevel4;
                break;
            case V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1:
                *eLevel = (profile_level.tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN)?
                                     OMX_VIDEO_HEVCMainTierLevel41:OMX_VIDEO_HEVCHighTierLevel41;
                break;
            case V4L2_MPEG_VIDEO_HEVC_LEVEL_5:
                *eLevel = (profile_level.tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN)?
                                     OMX_VIDEO_HEVCMainTierLevel5:OMX_VIDEO_HEVCHighTierLevel5;
                break;
            case V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1:
                *eLevel = (profile_level.tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN)?
                                     OMX_VIDEO_HEVCMainTierLevel51:OMX_VIDEO_HEVCHighTierLevel51;
                break;
            case V4L2_MPEG_VIDEO_HEVC_LEVEL_5_2:
                *eLevel = (profile_level.tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN)?
                                     OMX_VIDEO_HEVCMainTierLevel52:OMX_VIDEO_HEVCHighTierLevel52;
                break;
            case V4L2_MPEG_VIDEO_HEVC_LEVEL_6:
                *eLevel = (profile_level.tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN)?
                                     OMX_VIDEO_HEVCMainTierLevel6:OMX_VIDEO_HEVCHighTierLevel6;
                break;
            case V4L2_MPEG_VIDEO_HEVC_LEVEL_6_1:
                *eLevel = (profile_level.tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN)?
                                     OMX_VIDEO_HEVCMainTierLevel61:OMX_VIDEO_HEVCHighTierLevel61;
                break;
            case V4L2_MPEG_VIDEO_HEVC_LEVEL_6_2:
                *eLevel = (profile_level.tier == V4L2_MPEG_VIDEO_HEVC_TIER_MAIN)?
                                     OMX_VIDEO_HEVCMainTierLevel62:OMX_VIDEO_HEVCHighTierLevel62;
                break;
            default:
                *eLevel = OMX_VIDEO_HEVCHighTiermax;
                status = false;
                break;
        }
    }

    return status;
}

bool venc_dev::venc_set_nal_size (OMX_VIDEO_CONFIG_NALSIZE *nalSizeInfo) {
    struct v4l2_control sControl;

    DEBUG_PRINT_HIGH("set video stream format - nal size - %u", nalSizeInfo->nNaluBytes);

    sControl.id = V4L2_CID_MPEG_VIDEO_HEVC_SIZE_OF_LENGTH_FIELD;
    switch (nalSizeInfo->nNaluBytes) {
        case 0:
            sControl.value = V4L2_MPEG_VIDEO_HEVC_SIZE_0;
            break;
        case 4:
            sControl.value = V4L2_MPEG_VIDEO_HEVC_SIZE_4;
            break;
        default:
            return false;
    }

    if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &sControl)) {
        DEBUG_PRINT_ERROR("set control: video stream format failed - %u",
                (unsigned int)nalSizeInfo->nNaluBytes);
        return false;
    }
    return true;
}

bool venc_dev::venc_superframe_enable(private_handle_t *handle)
{
    struct v4l2_control ctrl;
    OMX_U32 frame_size;
    unsigned long color_format;

    ctrl.id = V4L2_CID_MPEG_VIDC_SUPERFRAME;
    color_format = get_media_colorformat(m_sVenc_cfg.inputformat);
    frame_size = VENUS_BUFFER_SIZE(color_format, m_sVenc_cfg.input_width, m_sVenc_cfg.input_height);

    /*
    * disable superframe if handle->size is not multiple of
    * frame_size or if it is a single frame.
    */
    if (handle->size % frame_size || handle->size == frame_size) {
        DEBUG_PRINT_ERROR("Invalid superframe handle size %d for frame size %d",
            handle->size, frame_size);
        return false;
    }
    ctrl.value = handle->size / frame_size;
    DEBUG_PRINT_HIGH("venc_superframe_enable: %d", ctrl.value);
    if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &ctrl)) {
        DEBUG_PRINT_ERROR("Failed to enable superframe, errno %d", errno);
        return false;
    }
    return true;
}

bool venc_dev::venc_cvp_enable(private_handle_t *handle)
{
    cvpMetadata.size = 0;
    if (temporal_layers_config.nMaxLayers > 1) {
        DEBUG_PRINT_INFO("venc_cvp_enable: disabling CVP as max layers %u", temporal_layers_config.nMaxLayers);
        return true;
    }
    if (getMetaData(handle, GET_CVP_METADATA, &cvpMetadata) == 0) {
        if (cvpMetadata.size == CVP_METADATA_SIZE) {
            struct v4l2_control control;
            control.id = V4L2_CID_MPEG_VIDC_VIDEO_EXTRADATA;
            control.value = EXTRADATA_ENC_INPUT_CVP;
            if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control)) {
                DEBUG_PRINT_ERROR("ERROR: Setting CVP metadata extradata control failed");
                return false;
            }
            m_cvp_meta_enabled = true;
            m_cvp_first_metadata = true;
            DEBUG_PRINT_HIGH("CVP metadata enabled");
            if (!venc_set_cvp_skipratio_controls())
                return false;
        } else {
            DEBUG_PRINT_ERROR("ERROR: External CVP mode disabled for this session and continue!");
            setMetaData(handle, SET_CVP_METADATA, 0);
        }
    } else {
        DEBUG_PRINT_INFO("venc_cvp_enable: cvp metadata not available");
    }
    return true;
}

bool venc_dev::venc_set_cvp_skipratio_controls()
{
    struct v4l2_control ctrl;

    if (!cvpMetadata.cvp_frame_rate || !cvpMetadata.capture_frame_rate) {
        DEBUG_PRINT_ERROR("ERROR: Invalid cvp frame rate received");
        return true;
    }
    DEBUG_PRINT_HIGH("cvpMetadata: frame_rate %u capture rate %u", cvpMetadata.cvp_frame_rate, cvpMetadata.capture_frame_rate);

    ctrl.id = V4L2_CID_MPEG_VIDC_CAPTURE_FRAME_RATE;
    ctrl.value = cvpMetadata.capture_frame_rate;
    if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &ctrl)) {
        DEBUG_PRINT_ERROR("ERROR: Setting capture frame rate control failed");
        return false;
    }

    ctrl.id = V4L2_CID_MPEG_VIDC_CVP_FRAME_RATE;
    ctrl.value = cvpMetadata.cvp_frame_rate;
    if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &ctrl)) {
        DEBUG_PRINT_ERROR("ERROR: Setting cvp frame rate control failed");
        return false;
    }
    return true;
}

bool venc_dev::venc_get_cvp_metadata(private_handle_t *handle, struct v4l2_buffer *buf)
{
    if (!m_cvp_meta_enabled)
        return true;

    unsigned int capture_rate = cvpMetadata.capture_frame_rate;
    unsigned int cvp_rate = cvpMetadata.cvp_frame_rate;

    buf->flags &= ~V4L2_BUF_FLAG_CVPMETADATA_SKIP;
    cvpMetadata.size = 0;
    if (getMetaData(handle, GET_CVP_METADATA, &cvpMetadata) == 0) {
        setMetaData(handle, SET_CVP_METADATA, 0);
        if (cvpMetadata.size != CVP_METADATA_SIZE) {
            DEBUG_PRINT_ERROR("ERROR: Invalid CVP metadata size %d",
                cvpMetadata.size);
            cvpMetadata.size = 0;
            /* If camera sends metadata of size not matching to CVP_METADATA_SIZE,
               it is considered as an error case. So, do not add skip flag */
            return false;
        }
        DEBUG_PRINT_LOW("CVP metadata size %d", cvpMetadata.size);
        if (m_debug.cvp_log) {
            venc_cvp_log_buffers("CVP", cvpMetadata.size, cvpMetadata.payload);
        }
    } else {
        DEBUG_PRINT_ERROR("ERROR: CVP metadata not available");
        return false;
    }

    if (m_cvp_first_metadata) {
        m_cvp_first_metadata = false;
    } else if (cvpMetadata.flags & CVP_METADATA_FLAG_REPEAT) {
        buf->flags |= V4L2_BUF_FLAG_CVPMETADATA_SKIP;
        DEBUG_PRINT_LOW("venc_empty_buf: V4L2_BUF_FLAG_CVPMETADATA_SKIP is set");
    }

    if ((cvpMetadata.capture_frame_rate != capture_rate) ||
        (cvpMetadata.cvp_frame_rate != cvp_rate)) {
        if(!venc_set_cvp_skipratio_controls())
            return false;
    }
    return true;
}

bool venc_dev::venc_config_bitrate(OMX_VIDEO_CONFIG_BITRATETYPE *bit_rate)
{
    OMX_U32 bitrate = bit_rate->nEncodeBitrate;
    if (bit_rate->nPortIndex == (OMX_U32)PORT_INDEX_OUT) {
        // If quality boost is eligible, also increase bitrate by 15% in dynamic change case
        if (mQualityBoostEligible && bitrate < VENC_QUALITY_BOOST_BITRATE_THRESHOLD) {
            bitrate += bitrate * 15 / 100;
        }
        if (venc_set_target_bitrate(bitrate) == false) {
            DEBUG_PRINT_ERROR("ERROR: Setting Target Bit rate failed");
            return false;
        }
    } else {
        DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexConfigVideoBitrate");
        return false;
    }
    return true;
}

bool venc_dev::venc_config_framerate(OMX_CONFIG_FRAMERATETYPE *frame_rate)
{
    if (frame_rate->nPortIndex == (OMX_U32)PORT_INDEX_OUT) {
        if (venc_set_encode_framerate(frame_rate->xEncodeFramerate) == false) {
            DEBUG_PRINT_ERROR("ERROR: Setting Encode Framerate failed");
            return false;
        }
    } else {
        DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexConfigVideoFramerate");
        return false;
    }
    return true;
}

bool venc_dev::venc_config_intravoprefresh(OMX_CONFIG_INTRAREFRESHVOPTYPE *intra_vop_refresh)
{
    if (intra_vop_refresh->nPortIndex == (OMX_U32)PORT_INDEX_OUT) {
        if (venc_set_intra_vop_refresh(intra_vop_refresh->IntraRefreshVOP) == false) {
            DEBUG_PRINT_ERROR("ERROR: Setting Encode Framerate failed");
            return false;
        }
    } else {
        DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexConfigVideoFramerate");
        return false;
    }

    return true;
}

bool venc_dev::venc_config_markLTR(OMX_QCOM_VIDEO_CONFIG_LTRMARK_TYPE *markltr)
{
    if (markltr->nPortIndex == (OMX_U32)PORT_INDEX_IN) {
        if(venc_set_markltr(markltr->nID) == false) {
            DEBUG_PRINT_ERROR("ERROR: Mark LTR failed");
            return false;
        }
    }  else {
        DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_QcomIndexConfigVideoLTRMark");
        return false;
    }
    return true;
}

bool venc_dev::venc_config_useLTR(OMX_QCOM_VIDEO_CONFIG_LTRUSE_TYPE *useltr)
{
    if (useltr->nPortIndex == (OMX_U32)PORT_INDEX_IN) {
        if(venc_set_useltr(useltr->nID) == false) {
            DEBUG_PRINT_ERROR("ERROR: Use LTR failed");
            return false;
        }
    } else {
        DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_QcomIndexConfigVideoLTRUse");
        return false;
    }
    return true;
}

bool venc_dev::venc_config_qp(OMX_SKYPE_VIDEO_CONFIG_QP *configqp)
{
    if (venc_set_qp(configqp->nQP,
                    configqp->nQP,
                    configqp->nQP,
                    ENABLE_I_QP | ENABLE_P_QP | ENABLE_B_QP ) == false) {
        DEBUG_PRINT_ERROR("Failed to set OMX_QcomIndexConfigQp failed");
        return false;
    }
    return true;
}

bool venc_dev::venc_config_vp8refframe(OMX_VIDEO_VP8REFERENCEFRAMETYPE *vp8refframe)
{
    if ((vp8refframe->nPortIndex == (OMX_U32)PORT_INDEX_IN) &&
            (vp8refframe->bUseGoldenFrame)) {
        if(venc_set_useltr(0x1) == false) {
            DEBUG_PRINT_ERROR("ERROR: use goldenframe failed");
            return false;
        }
    } else if((vp8refframe->nPortIndex == (OMX_U32)PORT_INDEX_IN) &&
            (vp8refframe->bGoldenFrameRefresh)) {
        if(venc_set_markltr(0x1) == false) {
            DEBUG_PRINT_ERROR("ERROR: Setting goldenframe failed");
            return false;
        }
    } else {
        DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexConfigVideoVp8ReferenceFrame");
        return false;
    }
    return true;
}

#ifdef _ANDROID_ICS_
bool venc_dev::venc_set_meta_mode(bool mode)
{
    metadatamode = mode;
    return true;
}
#endif

bool venc_dev::venc_is_video_session_supported(unsigned long width,
        unsigned long height)
{
    if ((width * height < capability.min_width *  capability.min_height) ||
            (width * height > capability.max_width *  capability.max_height)) {
        DEBUG_PRINT_ERROR(
                "Unsupported video resolution WxH = (%lu)x(%lu) supported range = min (%d)x(%d) - max (%d)x(%d)",
                width, height, capability.min_width, capability.min_height,
                capability.max_width, capability.max_height);
        return false;
    }

    DEBUG_PRINT_LOW("video session supported");
    return true;
}

venc_dev::BatchInfo::BatchInfo()
    : mNumPending(0) {
    pthread_mutex_init(&mLock, NULL);
    for (int i = 0; i < kMaxBufs; ++i) {
        mBufMap[i] = kBufIDFree;
    }
}

int venc_dev::BatchInfo::registerBuffer(int bufferId) {
    pthread_mutex_lock(&mLock);
    int availId = 0;
    for( ; availId < kMaxBufs && mBufMap[availId] != kBufIDFree; ++availId);
    if (availId >= kMaxBufs) {
        DEBUG_PRINT_ERROR("Failed to find free entry !");
        pthread_mutex_unlock(&mLock);
        return -1;
    }
    mBufMap[availId] = bufferId;
    mNumPending++;
    pthread_mutex_unlock(&mLock);
    return availId;
}

int venc_dev::BatchInfo::retrieveBufferAt(int v4l2Id) {
    pthread_mutex_lock(&mLock);
    if (v4l2Id >= kMaxBufs || v4l2Id < 0) {
        DEBUG_PRINT_ERROR("Batch: invalid index %d", v4l2Id);
        pthread_mutex_unlock(&mLock);
        return -1;
    }
    if (mBufMap[v4l2Id] == kBufIDFree) {
        DEBUG_PRINT_ERROR("Batch: buffer @ %d was not registered !", v4l2Id);
        pthread_mutex_unlock(&mLock);
        return -1;
    }
    int bufferId = mBufMap[v4l2Id];
    mBufMap[v4l2Id] = kBufIDFree;
    mNumPending--;
    pthread_mutex_unlock(&mLock);
    return bufferId;
}

bool venc_dev::BatchInfo::isPending(int bufferId) {
    pthread_mutex_lock(&mLock);
    int existsId = 0;
    for(; existsId < kMaxBufs && mBufMap[existsId] != bufferId; ++existsId);
    pthread_mutex_unlock(&mLock);
    return existsId < kMaxBufs;
}

bool venc_dev::venc_set_hdr_info(const MasteringDisplay& mastering_disp_info,
                            const ContentLightLevel& content_light_level_info)
{
    struct v4l2_control ctrl = {0, 0};
    const unsigned int hdr_info[] = {
        MSM_VIDC_RGB_PRIMARY_00,
        MSM_VIDC_RGB_PRIMARY_01,
        MSM_VIDC_RGB_PRIMARY_10,
        MSM_VIDC_RGB_PRIMARY_11,
        MSM_VIDC_RGB_PRIMARY_20,
        MSM_VIDC_RGB_PRIMARY_21,
        MSM_VIDC_WHITEPOINT_X,
        MSM_VIDC_WHITEPOINT_Y,
        MSM_VIDC_MAX_DISP_LUM,
        MSM_VIDC_MIN_DISP_LUM,
        MSM_VIDC_RGB_MAX_CLL,
        MSM_VIDC_RGB_MAX_FLL
    };

    unsigned int values[] = {
        mastering_disp_info.primaries.rgbPrimaries[0][0],
        mastering_disp_info.primaries.rgbPrimaries[0][1],
        mastering_disp_info.primaries.rgbPrimaries[1][0],
        mastering_disp_info.primaries.rgbPrimaries[1][1],
        mastering_disp_info.primaries.rgbPrimaries[2][0],
        mastering_disp_info.primaries.rgbPrimaries[2][1],
        mastering_disp_info.primaries.whitePoint[0],
        mastering_disp_info.primaries.whitePoint[1],
        // maxDisplayLuminance is in cd/m^2 scale. But the standard requires this field
        // to be in 0.0001 cd/m^2 scale. So, multiply with LUMINANCE_MULTIPLICATION_FACTOR
        // and give to be driver
        mastering_disp_info.maxDisplayLuminance * LUMINANCE_MULTIPLICATION_FACTOR,
        mastering_disp_info.minDisplayLuminance,
        content_light_level_info.maxContentLightLevel,
        content_light_level_info.minPicAverageLightLevel
    };

    ctrl.id = V4L2_CID_MPEG_VIDC_VENC_HDR_INFO;

    for (unsigned int i = 0; i < (sizeof(hdr_info)/sizeof(hdr_info[0])); i++) {
        ctrl.value = (unsigned int) ((values[i] & 0xFFFFFFF ) | (hdr_info[i] << 28));
        if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &ctrl)) {
            DEBUG_PRINT_ERROR("VIDIOC_S_CTRL failed for HDR Info : (%u) value %#X : %#X",
                              i, hdr_info[i], values[i]);
            return false;
        }
    }

    return true;
}

/*=================================================================================
 * Function:   venc_set_roi_region_qp_info
 * @brief      set the config of OMX_QTI_VIDEO_CONFIG_ROI_RECT_REGION_INFO and store
 *             the info in the list
 * Parameters:
 * @param      OMX_QTI_VIDEO_CONFIG_ROI_RECT_REGION_INFO *roiRegionInfo:
 *             the config to be set
 * Return value:
 *             bool: return true if the config is set successfully
*==================================================================================*/
bool venc_dev::venc_set_roi_region_qp_info(OMX_QTI_VIDEO_CONFIG_ROI_RECT_REGION_INFO *roiRegionInfo)
{
    if (!m_roi_enabled || m_roi_type == ROI_NONE) {
        DEBUG_PRINT_ERROR("ROI-Region: roi info not enabled (%d) or unknown roi type (%u)",
            m_roi_enabled, m_roi_type);
        return false;
    }
    if (!roiRegionInfo) {
        DEBUG_PRINT_ERROR("ROI-Region: no region info present");
        return false;
    }
    if (m_sVenc_cfg.codectype != V4L2_PIX_FMT_H264 &&
            m_sVenc_cfg.codectype != V4L2_PIX_FMT_HEVC) {
        DEBUG_PRINT_ERROR("ROI-Region: is not supported for %d codec",
                (OMX_U32) m_sVenc_cfg.codectype);
        return false;
    }

    pthread_mutex_lock(&m_roilock);
    DEBUG_PRINT_LOW("ROI-Region: add region with timestamp %lld us.", roiRegionInfo->nTimeStamp);
    mRoiRegionList.push_back(*roiRegionInfo);
    pthread_mutex_unlock(&m_roilock);
    return true;
}

/*=================================================================================
 * Function:   append_extradata_roi_region_qp_info
 * @brief      fill the roi info in the extradata of input Buffer
 * Parameters:
 * @param      OMX_OTHER_EXTRADATATYPE *data: the address of the extradata buffer
 *             OMX_TICKS timestamp:  the timestamp of the input Buffer
 *             OMX_U32: the available size of the extradata buffer
 * Return value:
 *             OMX_U32: the filled size
*==================================================================================*/
OMX_U32 venc_dev::append_extradata_roi_region_qp_info(OMX_OTHER_EXTRADATATYPE *data,
        OMX_TICKS timestamp, OMX_U32 freeSize)
{
    bool found = false;
    pthread_mutex_lock(&m_roilock);
    if (mRoiRegionList.size() == 0) {
        pthread_mutex_unlock(&m_roilock);
        return 0;
    }
    std::list<OMX_QTI_VIDEO_CONFIG_ROI_RECT_REGION_INFO>::iterator it =
            mRoiRegionList.begin();
    while (it != mRoiRegionList.end()) {
        if (it->nTimeStamp < timestamp) {
            it = mRoiRegionList.erase(it);
            continue;
        } else if (it->nTimeStamp == timestamp) {
            found = true;
            break;
        }
        it++;
    }
    pthread_mutex_unlock(&m_roilock);
    if (!found) {
        DEBUG_PRINT_LOW("ROI-Region: no region roi data was found");
        return 0;
    }
    OMX_QTI_VIDEO_CONFIG_ROI_RECT_REGION_INFO regionInfo = *it;
    bool isHevc = m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC ? true:false;
    OMX_U32 height = m_sVenc_cfg.dvs_height;
    OMX_U32 width = m_sVenc_cfg.dvs_width;
    OMX_U32 mbAlign = isHevc ? 32 : 16;
    OMX_U8 mbBit = isHevc ? 5 : 4;
    OMX_U32 mbRow = ALIGN(width, mbAlign) / mbAlign;
    OMX_U32 mbCol = ALIGN(height, mbAlign) / mbAlign;
    OMX_U32 numBytes, mbLeft, mbTop, mbRight, mbBottom = 0;
    OMX_S8 deltaQP = 0;
    DEBUG_PRINT_LOW("ROI-Region: clip(%ux%u: %s), mb(%ux%u), region(num:%u, ts:%lld)",
            width, height, isHevc ? "hevc" : "avc", mbRow, mbCol, regionInfo.nRegionNum,
            regionInfo.nTimeStamp);

    if (m_roi_type == ROI_2BYTE) {
        OMX_U32 mbRowAligned = ALIGN(mbRow, 8);
        numBytes = mbRowAligned * mbCol * 2;
        OMX_U32 numBytesAligned = ALIGN(numBytes, 4);

        data->nDataSize = ALIGN(sizeof(struct msm_vidc_roi_deltaqp_payload), 256)
                            + numBytesAligned;
        data->nSize = ALIGN(sizeof(OMX_OTHER_EXTRADATATYPE) + data->nDataSize, 4);
        if (data->nSize > freeSize) {
            DEBUG_PRINT_ERROR("ROI-Region: Buffer size(%u) is less than ROI extradata size(%u)",
                     freeSize, data->nSize);
            data->nDataSize = 0;
            data->nSize = 0;
            return 0;
        }

        data->nVersion.nVersion = OMX_SPEC_VERSION;
        data->nPortIndex = 0;
        data->eType = (OMX_EXTRADATATYPE)MSM_VIDC_EXTRADATA_ROI_QP;
        struct msm_vidc_roi_deltaqp_payload *roiData =
                (struct msm_vidc_roi_deltaqp_payload *)(data->data);
        roiData->b_roi_info = true;
        roiData->mbi_info_size = numBytesAligned;
        roiData->data[0] = (unsigned int)(ALIGN(&roiData->data[1], 256)
                - (unsigned long)roiData->data);
        OMX_U16* exDataBuf = (OMX_U16*)((OMX_U8*)roiData->data + roiData->data[0]);
        OMX_U32 mb = 0;
        OMX_U16 *pData = NULL;

        for (OMX_U8 i = 0; i < regionInfo.nRegionNum; i++) {
            mbLeft = regionInfo.nRegions[i].nLeft >> mbBit;
            mbTop = regionInfo.nRegions[i].nTop >> mbBit;
            mbRight = regionInfo.nRegions[i].nRight >> mbBit;
            mbBottom = regionInfo.nRegions[i].nBottom >> mbBit;
            deltaQP = regionInfo.nRegions[i].nDeltaQP;
            if (mbLeft >= mbRow || mbRight >= mbRow
                    || mbTop >= mbCol || mbBottom >= mbCol) {
                continue;
            }
            for (OMX_U32 row = mbTop; row <= mbBottom; row++) {
                for (OMX_U32 col = mbLeft; col <= mbRight; col++) {
                    mb = row * mbRowAligned + col;
                    pData = exDataBuf + mb;
                   *pData = (1 << 11) | ((deltaQP & 0x3F) << 4);
                }
            }
        }
        DEBUG_PRINT_LOW("ROI-Region(2Byte): set roi: raw size: %u", numBytesAligned);
    } else if (m_roi_type == ROI_2BIT) {
        numBytes = (mbRow * mbCol * 2 + 7) >> 3;
        data->nDataSize = sizeof(struct msm_vidc_roi_qp_payload) + numBytes;
        data->nSize = ALIGN(sizeof(OMX_OTHER_EXTRADATATYPE) + data->nDataSize, 4);

        if (data->nSize > freeSize) {
            DEBUG_PRINT_ERROR("ROI-Region: Buffer size(%u) is less than ROI extradata size(%u)",
                    freeSize, data->nSize);
            data->nDataSize = 0;
            data->nSize = 0;
            return 0;
        }

        data->nVersion.nVersion = OMX_SPEC_VERSION;
        data->nPortIndex = 0;
        data->eType = (OMX_EXTRADATATYPE)MSM_VIDC_EXTRADATA_ROI_QP;
        struct msm_vidc_roi_qp_payload *roiData =
                (struct msm_vidc_roi_qp_payload *)(data->data);
        roiData->b_roi_info = true;
        roiData->mbi_info_size = numBytes;
        roiData->lower_qp_offset = 0;
        roiData->upper_qp_offset = 0;
        OMX_U8 flag = 0x1;
        OMX_U32 mb, mb_byte = 0;
        OMX_U8 mb_bit = 0;
        OMX_U8 *pData = NULL;

        for (OMX_U8 i = 0; i < regionInfo.nRegionNum; i++) {
            mbLeft = regionInfo.nRegions[i].nLeft >> mbBit;
            mbTop = regionInfo.nRegions[i].nTop >> mbBit;
            mbRight = regionInfo.nRegions[i].nRight >> mbBit;
            mbBottom = regionInfo.nRegions[i].nBottom >> mbBit;
            deltaQP = regionInfo.nRegions[i].nDeltaQP;
            if (mbLeft >= mbRow || mbRight >= mbRow
                    || mbTop >= mbCol || mbBottom >= mbCol
                    || deltaQP == 0) {
                continue;
            }
            // choose the minimum absolute value for lower and upper offset
            if (deltaQP < 0) {
                if (roiData->lower_qp_offset == 0) {
                    roiData->lower_qp_offset = deltaQP;
                } else if (roiData->lower_qp_offset < deltaQP) {
                    roiData->lower_qp_offset = deltaQP;
                }
                flag = 0x1;
            } else {
                if (roiData->upper_qp_offset == 0) {
                    roiData->upper_qp_offset = deltaQP;
                } else if (roiData->upper_qp_offset > deltaQP) {
                    roiData->upper_qp_offset = deltaQP;
                }
                flag = 0x2;
            }
            for (OMX_U32 row = mbTop; row <= mbBottom; row++) {
                for (OMX_U32 col = mbLeft; col <= mbRight; col++) {
                    mb = row * mbRow + col;
                    mb_byte = mb >> 2;
                    mb_bit = (3 - (mb & 0x3)) << 1;
                    pData = (OMX_U8 *)roiData->data + mb_byte;
                    *pData |= (flag << mb_bit);
                }
            }
        }
        DEBUG_PRINT_LOW("ROI-Region(2Bit):set roi low:%d,up:%d", roiData->lower_qp_offset, roiData->upper_qp_offset);
    } else {
        DEBUG_PRINT_ERROR("Invalied roi type : %u", m_roi_type);
        return 0;
    }
    return data->nSize;
}

void venc_dev::venc_set_quality_boost(OMX_BOOL c2d_enable)
{
    OMX_U32 initial_qp;
    OMX_QCOM_VIDEO_PARAM_IPB_QPRANGETYPE qp_range;
    OMX_QTI_VIDEO_CONFIG_BLURINFO blurinfo;

    // Conditions to enable encoder quality boost,
    // 1. Codec is AVC
    // 2. RCMode is VBR
    // 3. Input is RGBA/RGBA_UBWC (C2D enabled)
    // 4. width <= 960 and height <= 960
    // 5. width >= 400 and height >= 400
    // 6. FPS <= 30
    // 7. bitrate < 2Mbps

    if (mQualityBoostRequested && c2d_enable &&
        m_sVenc_cfg.codectype == V4L2_PIX_FMT_H264 &&
        rate_ctrl.rcmode == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR &&
        m_sVenc_cfg.dvs_width <= 960 && m_sVenc_cfg.dvs_height <= 960 &&
        m_sVenc_cfg.dvs_width >= 400 && m_sVenc_cfg.dvs_height >= 400 &&
        (m_sVenc_cfg.fps_num / m_sVenc_cfg.fps_den) <= 30 &&
        bitrate.target_bitrate < VENC_QUALITY_BOOST_BITRATE_THRESHOLD) {
        mQualityBoostEligible = true;
        DEBUG_PRINT_HIGH("Quality boost eligible encoder session");
    } else {
        return;
    }

    if (bitrate.target_bitrate <= 64000)
        venc_set_level(OMX_VIDEO_AVCLevel1);

    // Set below configurations to boost quality
    // 1. Increase bitrate by 15%
    bitrate.target_bitrate += bitrate.target_bitrate * 15 / 100;
    venc_set_target_bitrate(bitrate.target_bitrate);

    // 2. Set initial QP=30
    initial_qp = 30;
    venc_set_qp(initial_qp, initial_qp, initial_qp, 7);

    // 3. Set QP range [10,40]
    qp_range.minIQP = qp_range.minPQP = qp_range.minBQP = 10;
    qp_range.maxIQP = qp_range.maxPQP = qp_range.maxBQP = 40;
    venc_set_session_qp_range(&qp_range);

    // 4. Disable blur (both external and internal)
    blurinfo.nBlurInfo = 2;
    venc_set_blur_resolution(&blurinfo);

    // 5. Disable bitrate savings (CAC)
    venc_set_bitrate_savings_mode(0);
}
