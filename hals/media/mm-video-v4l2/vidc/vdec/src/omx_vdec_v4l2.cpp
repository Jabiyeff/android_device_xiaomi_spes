/*--------------------------------------------------------------------------
Copyright (c) 2010 - 2021, The Linux Foundation. All rights reserved.

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

/*============================================================================
                            O p e n M A X   w r a p p e r s
                             O p e n  M A X   C o r e

  This module contains the implementation of the OpenMAX core & component.

*//*========================================================================*/

//////////////////////////////////////////////////////////////////////////////
//                             Include Files
//////////////////////////////////////////////////////////////////////////////

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include <string.h>
#include <pthread.h>
#include <sys/prctl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include "omx_vdec.h"
#include "vidc_common.h"
#include <fcntl.h>
#include <limits.h>
#include <stdlib.h>
#include <media/hardware/HardwareAPI.h>
#include <sys/eventfd.h>
#include "PlatformConfig.h"
#include <linux/dma-buf.h>
#include <linux/videodev2.h>

#if !defined(_ANDROID_) || defined(SYS_IOCTL)
#include <sys/ioctl.h>
#include <sys/mman.h>
#endif

#ifdef _ANDROID_
#include <cutils/properties.h>

#ifdef _QUERY_DISP_RES_
#include "display_config.h"
#endif
#endif

#ifdef _USE_GLIB_
#include <glib.h>
#define strlcpy g_strlcpy
#endif

#include <qdMetaData.h>
#include <gralloc_priv.h>

#ifdef ANDROID_JELLYBEAN_MR2
#include "QComOMXMetadata.h"
#endif

#define BUFFER_LOG_LOC "/data/vendor/media"

#ifdef OUTPUT_EXTRADATA_LOG
FILE *outputExtradataFile;
char output_extradata_filename [] = "/data/vendor/media/extradata";
#endif

#define POLL_TIMEOUT 0x7fffffff

#ifdef _ANDROID_
extern "C" {
#include<utils/Log.h>
}
#endif//_ANDROID_

#define PREFETCH_PIXEL_BUFFER_COUNT 16
#define PREFETCH_NON_PIXEL_BUFFER_COUNT 1

#ifndef ION_FLAG_CP_BITSTREAM
#define ION_FLAG_CP_BITSTREAM 0
#endif

#ifndef ION_FLAG_CP_PIXEL
#define ION_FLAG_CP_PIXEL 0
#endif

#ifdef SLAVE_SIDE_CP
#define MEM_HEAP_ID ION_CP_MM_HEAP_ID
#define SECURE_ALIGN SZ_1M
#define SECURE_FLAGS_INPUT_BUFFER ION_FLAG_SECURE
#define SECURE_FLAGS_OUTPUT_BUFFER ION_FLAG_SECURE
#else //MASTER_SIDE_CP
#define MEM_HEAP_ID ION_SECURE_HEAP_ID
#define SECURE_ALIGN SZ_4K
#define SECURE_FLAGS_INPUT_BUFFER (ION_FLAG_SECURE | ION_FLAG_CP_BITSTREAM)
#define SECURE_FLAGS_OUTPUT_BUFFER (ION_FLAG_SECURE | ION_FLAG_CP_PIXEL)
#endif

#define LUMINANCE_DIV_FACTOR 10000.0

/* defined in mp-ctl.h */
#define MPCTLV3_VIDEO_DECODE_PB_HINT 0x41C04000

#define MIN(x,y) (((x) < (y)) ? (x) : (y))
#define MAX(x,y) (((x) > (y)) ? (x) : (y))

/*
To enable sending vp9/hevc hdr10plus metadata via private gralloc
handle to display, define HDR10PLUS_SETMETADATA_ENABLE as 1.This
disables sending metadata via framework. To enable sending vp9/hevc
hdr10plus metadata via framework, define HDR10PLUS_SETMETADATA_ENABLE
as 0. This disables sending metadata via gralloc handle.
*/
#define HDR10_SETMETADATA_ENABLE 0
#define DEC_HDR_DISABLE_FLAG 0x1

#define THUMBNAIL_YUV420P_8BIT 0x01
#define THUMBNAIL_YUV420P_10BIT 0x02

using namespace android;

void* async_message_thread (void *input)
{
    OMX_BUFFERHEADERTYPE *buffer;
    struct v4l2_plane plane[VIDEO_MAX_PLANES];
    struct pollfd pfds[2];
    struct v4l2_buffer v4l2_buf;
    memset((void *)&v4l2_buf,0,sizeof(v4l2_buf));
    struct v4l2_event dqevent;
    omx_vdec *omx = reinterpret_cast<omx_vdec*>(input);
    pfds[0].events = POLLIN | POLLRDNORM | POLLOUT | POLLWRNORM | POLLRDBAND | POLLPRI;
    pfds[1].events = POLLIN | POLLERR;
    pfds[0].fd = omx->drv_ctx.video_driver_fd;
    pfds[1].fd = omx->m_poll_efd;
    int error_code = 0,rc=0,bytes_read = 0,bytes_written = 0;
    DEBUG_PRINT_HIGH("omx_vdec: Async thread start");
    prctl(PR_SET_NAME, (unsigned long)"VideoDecCallBackThread", 0, 0, 0);
    while (!omx->async_thread_force_stop) {
        rc = poll(pfds, 2, POLL_TIMEOUT);
        if (!rc) {
            DEBUG_PRINT_ERROR("Poll timedout");
            break;
        } else if (rc < 0 && errno != EINTR && errno != EAGAIN) {
            DEBUG_PRINT_ERROR("Error while polling: %d, errno = %d", rc, errno);
            break;
        }
        if ((pfds[1].revents & POLLIN) || (pfds[1].revents & POLLERR)) {
            DEBUG_PRINT_HIGH("async_message_thread interrupted to be exited");
            break;
        }
        if ((pfds[0].revents & POLLIN) || (pfds[0].revents & POLLRDNORM)) {
            struct vdec_msginfo vdec_msg;
            memset(&vdec_msg, 0, sizeof(vdec_msg));
            v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            v4l2_buf.memory = V4L2_MEMORY_USERPTR;
            v4l2_buf.length = omx->drv_ctx.num_planes;
            v4l2_buf.m.planes = plane;
            while (!ioctl(pfds[0].fd, VIDIOC_DQBUF, &v4l2_buf)) {
                vdec_msg.msgcode=VDEC_MSG_RESP_OUTPUT_BUFFER_DONE;
                vdec_msg.status_code=VDEC_S_SUCCESS;
                vdec_msg.msgdata.output_frame.client_data=(void*)&v4l2_buf;
                vdec_msg.msgdata.output_frame.len=plane[0].bytesused;
                vdec_msg.msgdata.output_frame.bufferaddr=(void*)plane[0].m.userptr;
                vdec_msg.msgdata.output_frame.time_stamp= ((int64_t)v4l2_buf.timestamp.tv_sec * (int64_t)1000000) +
                    (int64_t)v4l2_buf.timestamp.tv_usec;

                if (omx->async_message_process(input,&vdec_msg) < 0) {
                    DEBUG_PRINT_HIGH("async_message_thread Exited");
                    break;
                }
            }
        }
        if ((pfds[0].revents & POLLOUT) || (pfds[0].revents & POLLWRNORM)) {
            struct vdec_msginfo vdec_msg;
            v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
            v4l2_buf.memory = V4L2_MEMORY_USERPTR;
            v4l2_buf.length = 1;
            v4l2_buf.m.planes = plane;
            while (!ioctl(pfds[0].fd, VIDIOC_DQBUF, &v4l2_buf)) {
                vdec_msg.msgcode=VDEC_MSG_RESP_INPUT_BUFFER_DONE;
                vdec_msg.status_code=VDEC_S_SUCCESS;
                vdec_msg.msgdata.input_frame_clientdata=(void*)&v4l2_buf;
                if (omx->async_message_process(input,&vdec_msg) < 0) {
                    DEBUG_PRINT_HIGH("async_message_thread Exited");
                    break;
                }
            }
        }
        if (pfds[0].revents & POLLPRI) {
            rc = ioctl(pfds[0].fd, VIDIOC_DQEVENT, &dqevent);
            if (dqevent.type == V4L2_EVENT_MSM_VIDC_PORT_SETTINGS_CHANGED_INSUFFICIENT ) {
                struct vdec_msginfo vdec_msg;
                unsigned int *ptr = (unsigned int *)(void *)dqevent.u.data;

                vdec_msg.msgcode=VDEC_MSG_EVT_CONFIG_CHANGED;
                vdec_msg.status_code=VDEC_S_SUCCESS;
                vdec_msg.msgdata.output_frame.picsize.frame_height = ptr[MSM_VIDC_HEIGHT];
                vdec_msg.msgdata.output_frame.picsize.frame_width = ptr[MSM_VIDC_WIDTH];
                vdec_msg.msgdata.output_frame.flags = true; // INSUFFICIENT event
                DEBUG_PRINT_HIGH("VIDC Port Reconfig received insufficient");
                omx->dpb_bit_depth = ptr[MSM_VIDC_BIT_DEPTH];
                DEBUG_PRINT_HIGH("VIDC Port Reconfig Bitdepth - %d", ptr[MSM_VIDC_BIT_DEPTH]);
                omx->m_progressive = ptr[MSM_VIDC_PIC_STRUCT];
                DEBUG_PRINT_HIGH("VIDC Port Reconfig PicStruct - %d", ptr[MSM_VIDC_PIC_STRUCT]);
                omx->m_color_space = (ptr[MSM_VIDC_COLOR_SPACE] == MSM_VIDC_BT2020 ? (omx_vdec::BT2020):
                                      (omx_vdec:: EXCEPT_BT2020));
                DEBUG_PRINT_HIGH("VIDC Port Reconfig ColorSpace - %d", omx->m_color_space);
                if (omx->async_message_process(input,&vdec_msg) < 0) {
                    DEBUG_PRINT_HIGH("async_message_thread Exited");
                    break;
                }
            } else if (dqevent.type == V4L2_EVENT_MSM_VIDC_FLUSH_DONE) {
                struct vdec_msginfo vdec_msg;
                uint32_t flush_type = *(uint32_t *)dqevent.u.data;
                // Old driver doesn't send flushType information.
                // To make this backward compatible fallback to old approach
                // if the flush_type is not present.
                vdec_msg.status_code=VDEC_S_SUCCESS;
                if (!flush_type || (flush_type & V4L2_CMD_FLUSH_OUTPUT)) {
                    vdec_msg.msgcode=VDEC_MSG_RESP_FLUSH_INPUT_DONE;
                    DEBUG_PRINT_HIGH("VIDC Input Flush Done Recieved");
                    if (omx->async_message_process(input,&vdec_msg) < 0) {
                        DEBUG_PRINT_HIGH("async_message_thread Exited");
                        break;
                    }
                }

                if (!flush_type || (flush_type & V4L2_CMD_FLUSH_CAPTURE)) {
                    vdec_msg.msgcode=VDEC_MSG_RESP_FLUSH_OUTPUT_DONE;
                    DEBUG_PRINT_HIGH("VIDC Output Flush Done Recieved");
                    if (omx->async_message_process(input,&vdec_msg) < 0) {
                        DEBUG_PRINT_HIGH("async_message_thread Exited");
                        break;
                    }
                }
            } else if (dqevent.type == V4L2_EVENT_MSM_VIDC_HW_OVERLOAD) {
                struct vdec_msginfo vdec_msg;
                vdec_msg.msgcode=VDEC_MSG_EVT_HW_OVERLOAD;
                vdec_msg.status_code=VDEC_S_SUCCESS;
                DEBUG_PRINT_ERROR("HW Overload received");
                if (omx->async_message_process(input,&vdec_msg) < 0) {
                    DEBUG_PRINT_HIGH("async_message_thread Exited");
                    break;
                }
            } else if (dqevent.type == V4L2_EVENT_MSM_VIDC_HW_UNSUPPORTED) {
                struct vdec_msginfo vdec_msg;
                vdec_msg.msgcode=VDEC_MSG_EVT_HW_UNSUPPORTED;
                vdec_msg.status_code=VDEC_S_SUCCESS;
                DEBUG_PRINT_ERROR("HW Unsupported received");
                if (omx->async_message_process(input,&vdec_msg) < 0) {
                    DEBUG_PRINT_HIGH("async_message_thread Exited");
                    break;
                }
            } else if (dqevent.type == V4L2_EVENT_MSM_VIDC_SYS_ERROR) {
                struct vdec_msginfo vdec_msg;
                vdec_msg.msgcode = VDEC_MSG_EVT_HW_ERROR;
                vdec_msg.status_code = VDEC_S_SUCCESS;
                DEBUG_PRINT_HIGH("SYS Error Recieved");
                if (omx->async_message_process(input,&vdec_msg) < 0) {
                    DEBUG_PRINT_HIGH("async_message_thread Exited");
                    break;
                }
            } else if (dqevent.type == V4L2_EVENT_MSM_VIDC_RELEASE_BUFFER_REFERENCE) {
                unsigned int *ptr = (unsigned int *)(void *)dqevent.u.data;

                DEBUG_PRINT_LOW("REFERENCE RELEASE EVENT RECVD fd = %d offset = %d", ptr[0], ptr[1]);
            } else if (dqevent.type == V4L2_EVENT_MSM_VIDC_RELEASE_UNQUEUED_BUFFER) {
                unsigned int *ptr = (unsigned int *)(void *)dqevent.u.data;
                struct vdec_msginfo vdec_msg;

                DEBUG_PRINT_LOW("Release unqueued buffer event recvd fd = %d offset = %d", ptr[0], ptr[1]);

                v4l2_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                v4l2_buf.memory = V4L2_MEMORY_USERPTR;
                v4l2_buf.length = omx->drv_ctx.num_planes;
                v4l2_buf.m.planes = plane;
                v4l2_buf.index = ptr[5];
                v4l2_buf.flags = 0;

                vdec_msg.msgcode = VDEC_MSG_RESP_OUTPUT_BUFFER_DONE;
                vdec_msg.status_code = VDEC_S_SUCCESS;
                vdec_msg.msgdata.output_frame.client_data = (void*)&v4l2_buf;
                vdec_msg.msgdata.output_frame.len = 0;
                vdec_msg.msgdata.output_frame.bufferaddr = (void*)(intptr_t)ptr[2];
                vdec_msg.msgdata.output_frame.time_stamp = ((uint64_t)ptr[3] * (uint64_t)1000000) +
                    (uint64_t)ptr[4];
                if (omx->async_message_process(input,&vdec_msg) < 0) {
                    DEBUG_PRINT_HIGH("async_message_thread Exitedn");
                    break;
                }
            } else {
                DEBUG_PRINT_HIGH("VIDC Some Event recieved");
                continue;
            }
        }
    }
    DEBUG_PRINT_HIGH("omx_vdec: Async thread stop");
    return NULL;
}

void* message_thread_dec(void *input)
{
    omx_vdec* omx = reinterpret_cast<omx_vdec*>(input);
    int res = 0;

    DEBUG_PRINT_HIGH("omx_vdec: message thread start");
    prctl(PR_SET_NAME, (unsigned long)"VideoDecMsgThread", 0, 0, 0);
    while (!omx->message_thread_stop) {
        res = omx->signal.wait(2 * 1000000000);
        if (res == ETIMEDOUT || omx->message_thread_stop) {
            continue;
        } else if (res) {
            DEBUG_PRINT_ERROR("omx_vdec: message_thread_dec wait on condition failed, exiting");
            break;
        }
        omx->process_event_cb(omx);
    }
    DEBUG_PRINT_HIGH("omx_vdec: message thread stop");
    return 0;
}

void post_message(omx_vdec *omx, unsigned char id)
{
    (void)id;
    omx->signal.signal();
}

// omx_cmd_queue destructor
omx_vdec::omx_cmd_queue::~omx_cmd_queue()
{
    // Nothing to do
}

// omx cmd queue constructor
omx_vdec::omx_cmd_queue::omx_cmd_queue(): m_read(0),m_write(0),m_size(0)
{
    memset(m_q,0,sizeof(omx_event)*OMX_CORE_CONTROL_CMDQ_SIZE);
}

// omx cmd queue insert
bool omx_vdec::omx_cmd_queue::insert_entry(unsigned long p1, unsigned long p2, unsigned long id)
{
    bool ret = true;
    if (m_size < OMX_CORE_CONTROL_CMDQ_SIZE) {
        m_q[m_write].id       = id;
        m_q[m_write].param1   = p1;
        m_q[m_write].param2   = p2;
        m_write++;
        m_size ++;
        if (m_write >= OMX_CORE_CONTROL_CMDQ_SIZE) {
            m_write = 0;
        }
    } else {
        ret = false;
        DEBUG_PRINT_ERROR("ERROR: %s()::Command Queue Full", __func__);
    }
    return ret;
}

// omx cmd queue pop
bool omx_vdec::omx_cmd_queue::pop_entry(unsigned long *p1, unsigned long *p2, unsigned long *id)
{
    bool ret = true;
    if (m_size > 0) {
        *id = m_q[m_read].id;
        *p1 = m_q[m_read].param1;
        *p2 = m_q[m_read].param2;
        // Move the read pointer ahead
        ++m_read;
        --m_size;
        if (m_read >= OMX_CORE_CONTROL_CMDQ_SIZE) {
            m_read = 0;
        }
    } else {
        ret = false;
    }
    return ret;
}

// Retrieve the first mesg type in the queue
unsigned omx_vdec::omx_cmd_queue::get_q_msg_type()
{
    return m_q[m_read].id;
}

#ifdef _ANDROID_
omx_vdec::ts_arr_list::ts_arr_list()
{
    //initialize timestamps array
    memset(m_ts_arr_list, 0, ( sizeof(ts_entry) * MAX_NUM_INPUT_OUTPUT_BUFFERS) );
}
omx_vdec::ts_arr_list::~ts_arr_list()
{
    //free m_ts_arr_list?
}

bool omx_vdec::ts_arr_list::insert_ts(OMX_TICKS ts)
{
    bool ret = true;
    bool duplicate_ts = false;
    int idx = 0;

    //insert at the first available empty location
    for ( ; idx < MAX_NUM_INPUT_OUTPUT_BUFFERS; idx++) {
        if (!m_ts_arr_list[idx].valid) {
            //found invalid or empty entry, save timestamp
            m_ts_arr_list[idx].valid = true;
            m_ts_arr_list[idx].timestamp = ts;
            DEBUG_PRINT_LOW("Insert_ts(): Inserting TIMESTAMP (%lld) at idx (%d)",
                    ts, idx);
            break;
        }
    }

    if (idx == MAX_NUM_INPUT_OUTPUT_BUFFERS) {
        DEBUG_PRINT_LOW("Timestamp array list is FULL. Unsuccessful insert");
        ret = false;
    }
    return ret;
}

bool omx_vdec::ts_arr_list::pop_min_ts(OMX_TICKS &ts)
{
    bool ret = true;
    int min_idx = -1;
    OMX_TICKS min_ts = 0;
    int idx = 0;

    for ( ; idx < MAX_NUM_INPUT_OUTPUT_BUFFERS; idx++) {

        if (m_ts_arr_list[idx].valid) {
            //found valid entry, save index
            if (min_idx < 0) {
                //first valid entry
                min_ts = m_ts_arr_list[idx].timestamp;
                min_idx = idx;
            } else if (m_ts_arr_list[idx].timestamp < min_ts) {
                min_ts = m_ts_arr_list[idx].timestamp;
                min_idx = idx;
            }
        }

    }

    if (min_idx < 0) {
        //no valid entries found
        DEBUG_PRINT_LOW("Timestamp array list is empty. Unsuccessful pop");
        ts = 0;
        ret = false;
    } else {
        ts = m_ts_arr_list[min_idx].timestamp;
        m_ts_arr_list[min_idx].valid = false;
        DEBUG_PRINT_LOW("Pop_min_ts:Timestamp (%lld), index(%d)",
                ts, min_idx);
    }

    return ret;

}


bool omx_vdec::ts_arr_list::reset_ts_list()
{
    bool ret = true;
    int idx = 0;

    DEBUG_PRINT_LOW("reset_ts_list(): Resetting timestamp array list");
    for ( ; idx < MAX_NUM_INPUT_OUTPUT_BUFFERS; idx++) {
        m_ts_arr_list[idx].valid = false;
    }
    return ret;
}
#endif

// factory function executed by the core to create instances
void *get_omx_component_factory_fn(void)
{
    return (new omx_vdec);
}

bool is_platform_tp10capture_supported()
{
    DEBUG_PRINT_HIGH("TP10 on capture port is supported");
    return true;
}

inline int omx_vdec::get_session_codec_type()
{
    return output_capability;
}
/* ======================================================================
   FUNCTION
   omx_vdec::omx_vdec

   DESCRIPTION
   Constructor

   PARAMETERS
   None

   RETURN VALUE
   None.
   ========================================================================== */
omx_vdec::omx_vdec(): m_error_propogated(false),
    m_state(OMX_StateInvalid),
    m_app_data(NULL),
    m_inp_mem_ptr(NULL),
    m_out_mem_ptr(NULL),
    m_intermediate_out_mem_ptr(NULL),
    m_client_output_extradata_mem_ptr(NULL),
    input_flush_progress (false),
    output_flush_progress (false),
    input_use_buffer (false),
    output_use_buffer (false),
    ouput_egl_buffers(false),
    m_use_output_pmem(OMX_FALSE),
    pending_input_buffers(0),
    pending_output_buffers(0),
    m_out_bm_count(0),
    m_inp_bm_count(0),
    m_out_extradata_bm_count(0),
    m_inp_bPopulated(OMX_FALSE),
    m_out_bPopulated(OMX_FALSE),
    m_flags(0),
    m_inp_bEnabled(OMX_TRUE),
    m_out_bEnabled(OMX_TRUE),
    m_in_alloc_cnt(0),
    m_platform_list(NULL),
    m_platform_entry(NULL),
    m_pmem_info(NULL),
    psource_frame (NULL),
    pdest_frame (NULL),
    m_inp_heap_ptr (NULL),
    m_phdr_pmem_ptr(NULL),
    m_heap_inp_bm_count (0),
    first_frame_meta (true),
    frame_count (0),
    nal_count (0),
    nal_length(0),
    first_frame(0),
    first_buffer(NULL),
    first_frame_size (0),
    m_device_file_ptr(NULL),
    m_disp_hor_size(0),
    m_disp_vert_size(0),
    prev_ts(LLONG_MAX),
    frm_int(0),
    m_fps_received(0),
    in_reconfig(false),
    c2d_enable_pending(false),
    m_display_id(NULL),
#ifdef _ANDROID_
    m_enable_android_native_buffers(OMX_FALSE),
    m_use_android_native_buffers(OMX_FALSE),
#endif
    m_disable_dynamic_buf_mode(0),
    m_desc_buffer_ptr(NULL),
    secure_mode(false),
    allocate_native_handle(false),
    client_set_fps(false),
    stereo_output_mode(HAL_NO_3D),
    m_prev_timestampUs(0),
    m_prev_frame_rendered(false),
    m_dec_hfr_fps(0),
    m_dec_secure_prefetch_size_internal(0),
    m_dec_secure_prefetch_size_output(0),
    m_queued_codec_config_count(0),
    m_thumbnail_yuv_output(0),
    m_prefetch_done(0),
    m_buffer_error(false)
{
    m_poll_efd = -1;
    memset(&drv_ctx, 0, sizeof(drv_ctx));
    drv_ctx.video_driver_fd = -1;
    for (int i = 0; i < VIDEO_MAX_FRAME; i++) {
        drv_ctx.extradata_info.ion[i].data_fd = -1;
        drv_ctx.extradata_info.ion[i].dev_fd = -1;
    }
    /* Assumption is that , to begin with , we have all the frames with decoder */
    DEBUG_PRINT_HIGH("In %u bit OMX vdec Constructor", (unsigned int)sizeof(long) * 8);
    memset(&m_debug,0,sizeof(m_debug));
#ifdef _ANDROID_

    char property_value[PROPERTY_VALUE_MAX] = {0};
    property_get("vendor.vidc.debug.level", property_value, "1");
    debug_level = strtoul(property_value, NULL, 16);
    property_value[0] = '\0';

    DEBUG_PRINT_HIGH("In OMX vdec Constructor");

    // TODO: Support in XML
    perf_flag = 0;
    if (perf_flag) {
        DEBUG_PRINT_HIGH("perf flag is %d", perf_flag);
        dec_time.start();
    }
    proc_frms = latency = 0;
    prev_n_filled_len = 0;

    Platform::Config::getInt32(Platform::vidc_dec_log_in,
            (int32_t *)&m_debug.in_buffer_log, 0);
    Platform::Config::getInt32(Platform::vidc_dec_log_out,
            (int32_t *)&m_debug.out_buffer_log, 0);

    Platform::Config::getInt32(Platform::vidc_dec_sec_prefetch_size_internal,
            (int32_t *)&m_dec_secure_prefetch_size_internal, 0);
    Platform::Config::getInt32(Platform::vidc_dec_sec_prefetch_size_output,
            (int32_t *)&m_dec_secure_prefetch_size_output, 0);

    DEBUG_PRINT_HIGH("Prefetch size internal = %d, output = %d",
            m_dec_secure_prefetch_size_internal, m_dec_secure_prefetch_size_output);

    Platform::Config::getInt32(Platform::vidc_perf_control_enable,
            (int32_t *)&m_perf_control.m_perf_control_enable, 0);
    if (m_perf_control.m_perf_control_enable) {
        DEBUG_PRINT_HIGH("perf cotrol enabled");
        m_perf_control.load_perf_library();
    }

    Platform::Config::getInt32(Platform::vidc_dec_thumbnail_yuv_output,
            (int32_t *)&m_thumbnail_yuv_output, 0);

    Platform::Config::getInt32(Platform::vidc_disable_hdr,
            (int32_t *)&m_disable_hdr, 0);

    Platform::Config::getInt32(Platform::vidc_dec_output_rate,
            (int32_t *)&m_dec_output_rate, 60);

    property_value[0] = '\0';
    property_get("vendor.vidc.dec.log.in", property_value, "0");
    m_debug.in_buffer_log |= atoi(property_value);

    DEBUG_PRINT_HIGH("vendor.vidc.dec.log.in value is %d", m_debug.in_buffer_log);

    property_value[0] = '\0';
    property_get("vendor.vidc.dec.log.out", property_value, "0");
    m_debug.out_buffer_log |= atoi(property_value);

    DEBUG_PRINT_HIGH("vendor.vidc.dec.log.out value is %d", m_debug.out_buffer_log);

    property_value[0] = '\0';
    property_get("vendor.vidc.dec.log.cc.out", property_value, "0");
    m_debug.out_cc_buffer_log |= atoi(property_value);

    DEBUG_PRINT_HIGH("vendor.vidc.dec.log.cc.out value is %d", m_debug.out_buffer_log);

    property_value[0] = '\0';
    property_get("vendor.vidc.dec.meta.log.out", property_value, "0");
    m_debug.out_meta_buffer_log = atoi(property_value);

    property_value[0] = '\0';
    property_get("vendor.vidc.log.loc", property_value, BUFFER_LOG_LOC);
    if (*property_value)
        strlcpy(m_debug.log_loc, property_value, PROPERTY_VALUE_MAX);

    struct timeval te;
    gettimeofday(&te, NULL);
    m_debug.session_id = te.tv_sec*1000LL + te.tv_usec/1000;
    m_debug.seq_count = 0;

#ifdef _UBWC_
    property_value[0] = '\0';
    property_get("vendor.gralloc.disable_ubwc", property_value, "0");
    m_disable_ubwc_mode = atoi(property_value);
    DEBUG_PRINT_HIGH("UBWC mode is %s", m_disable_ubwc_mode ? "disabled" : "enabled");
#else
    m_disable_ubwc_mode = true;
#endif
#endif
    memset(&m_cmp,0,sizeof(m_cmp));
    memset(&m_cb,0,sizeof(m_cb));
    memset (m_hwdevice_name,0,sizeof(m_hwdevice_name));
    memset(m_demux_offsets, 0, ( sizeof(OMX_U32) * 8192) );
    memset(&m_custom_buffersize, 0, sizeof(m_custom_buffersize));
    memset(&m_client_color_space, 0, sizeof(DescribeColorAspectsParams));
    memset(&m_internal_color_space, 0, sizeof(DescribeColorAspectsParams));
    memset(&m_client_hdr_info, 0, sizeof(DescribeHDRStaticInfoParams));
    memset(&m_internal_hdr_info, 0, sizeof(DescribeHDRStaticInfoParams));
    m_demux_entries = 0;
    msg_thread_id = 0;
    async_thread_id = 0;
    msg_thread_created = false;
    async_thread_created = false;
    async_thread_force_stop = false;
    message_thread_stop = false;
#ifdef _ANDROID_ICS_
    memset(&native_buffer, 0 ,(sizeof(struct nativebuffer) * MAX_NUM_INPUT_OUTPUT_BUFFERS));
#endif

    drv_ctx.timestamp_adjust = false;
    m_vendor_config.pData = NULL;
    pthread_mutex_init(&m_lock, NULL);
    pthread_mutex_init(&c_lock, NULL);
    pthread_mutex_init(&buf_lock, NULL);
    pthread_mutex_init(&m_hdr10pluslock, NULL);
    sem_init(&m_cmd_lock,0,0);
    sem_init(&m_safe_flush, 0, 0);
    streaming[CAPTURE_PORT] =
        streaming[OUTPUT_PORT] = false;
#ifdef _ANDROID_
    // TODO: Support in XML
#endif
    m_client_extradata = EXTRADATA_NONE;
    m_fill_output_msg = OMX_COMPONENT_GENERATE_FTB;
    client_buffers.set_vdec_client(this);
    dynamic_buf_mode = false;
    m_reconfig_height = 0;
    m_reconfig_width = 0;
    m_decode_order_mode = false;
    m_perf_control.perf_lock_acquire();
    m_input_pass_buffer_fd = false;
    memset(&m_extradata_misr, 0, sizeof(m_extradata_misr));
    m_client_color_space.nPortIndex = (OMX_U32)OMX_CORE_INPUT_PORT_INDEX;
    m_client_color_space.sAspects.mRange =  ColorAspects::RangeUnspecified;
    m_client_color_space.sAspects.mPrimaries = ColorAspects::PrimariesUnspecified;
    m_client_color_space.sAspects.mMatrixCoeffs = ColorAspects::MatrixUnspecified;
    m_client_color_space.sAspects.mTransfer = ColorAspects::TransferUnspecified;

    m_internal_color_space.nPortIndex = (OMX_U32)OMX_CORE_OUTPUT_PORT_INDEX;
    m_internal_color_space.sAspects.mRange =  ColorAspects::RangeUnspecified;
    m_internal_color_space.sAspects.mPrimaries = ColorAspects::PrimariesUnspecified;
    m_internal_color_space.sAspects.mMatrixCoeffs = ColorAspects::MatrixUnspecified;
    m_internal_color_space.sAspects.mTransfer = ColorAspects::TransferUnspecified;
    m_internal_color_space.nSize = sizeof(DescribeColorAspectsParams);

    m_client_hdr_info.nPortIndex = (OMX_U32)OMX_CORE_INPUT_PORT_INDEX;
    m_internal_hdr_info.nPortIndex = (OMX_U32)OMX_CORE_OUTPUT_PORT_INDEX;

    m_etb_count = 0;
    m_etb_timestamp = 0;

    m_color_space = EXCEPT_BT2020;

    init_color_aspects_map();
    m_hist_metadata.stat_len = 0;

    profile_level_converter::init();
    clientSet_profile_level.eProfile = 0;
    clientSet_profile_level.eLevel = 0;
#ifdef USE_GBM
     drv_ctx.gbm_device_fd = -1;
#endif
}

static const int event_type[] = {
    V4L2_EVENT_MSM_VIDC_FLUSH_DONE,
    V4L2_EVENT_MSM_VIDC_PORT_SETTINGS_CHANGED_INSUFFICIENT,
    V4L2_EVENT_MSM_VIDC_RELEASE_BUFFER_REFERENCE,
    V4L2_EVENT_MSM_VIDC_RELEASE_UNQUEUED_BUFFER,
    V4L2_EVENT_MSM_VIDC_SYS_ERROR,
    V4L2_EVENT_MSM_VIDC_HW_OVERLOAD,
    V4L2_EVENT_MSM_VIDC_HW_UNSUPPORTED
};

static OMX_ERRORTYPE subscribe_to_events(int fd)
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

/* ======================================================================
   FUNCTION
   omx_vdec::~omx_vdec

   DESCRIPTION
   Destructor

   PARAMETERS
   None

   RETURN VALUE
   None.
   ========================================================================== */
omx_vdec::~omx_vdec()
{
    m_pmem_info = NULL;
    DEBUG_PRINT_HIGH("In OMX vdec Destructor");
    if (msg_thread_created) {
        DEBUG_PRINT_HIGH("Signalling close to OMX Msg Thread");
        message_thread_stop = true;
        post_message(this, OMX_COMPONENT_CLOSE_MSG);
        DEBUG_PRINT_HIGH("Waiting on OMX Msg Thread exit");
        pthread_join(msg_thread_id,NULL);
    }
    DEBUG_PRINT_HIGH("Waiting on OMX Async Thread exit");
    if(eventfd_write(m_poll_efd, 1)) {
         DEBUG_PRINT_ERROR("eventfd_write failed for fd: %d, errno = %d, force stop async_thread", m_poll_efd, errno);
         async_thread_force_stop = true;
    }
    if (async_thread_created)
        pthread_join(async_thread_id,NULL);

    if (m_prefetch_done & 0x1)
        prefetch_buffers(PREFETCH_PIXEL_BUFFER_COUNT, m_dec_secure_prefetch_size_output, ION_IOC_DRAIN, ION_FLAG_CP_PIXEL);
    if (m_prefetch_done & 0x2)
        prefetch_buffers(PREFETCH_NON_PIXEL_BUFFER_COUNT, m_dec_secure_prefetch_size_internal, ION_IOC_DRAIN, ION_FLAG_CP_NON_PIXEL);

    unsubscribe_to_events(drv_ctx.video_driver_fd);
    close(m_poll_efd);
    close(drv_ctx.video_driver_fd);
    clear_hdr10plusinfo();
    pthread_mutex_destroy(&m_lock);
    pthread_mutex_destroy(&c_lock);
    pthread_mutex_destroy(&buf_lock);
    pthread_mutex_destroy(&m_hdr10pluslock);
    sem_destroy(&m_cmd_lock);
    if (perf_flag) {
        DEBUG_PRINT_HIGH("--> TOTAL PROCESSING TIME");
        dec_time.end();
    }
    DEBUG_PRINT_INFO("Exit OMX vdec Destructor: fd=%d",drv_ctx.video_driver_fd);
    m_perf_control.perf_lock_release();
}

bool omx_vdec::check_supported_flexible_formats(OMX_COLOR_FORMATTYPE required_format)
{
    if(required_format == (OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m ||
         required_format == (OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420SemiPlanarP010Venus) {
         //for now, the flexible formats should be NV12 by default for 8bit cases
         //it will change to P010 after 10bit port-reconfig accordingly
       return OMX_TRUE;
    }
    else {
       return OMX_FALSE;
    }
}

/* ======================================================================
   FUNCTION
   omx_vdec::OMXCntrlProcessMsgCb

   DESCRIPTION
   IL Client callbacks are generated through this routine. The decoder
   provides the thread context for this routine.

   PARAMETERS
   ctxt -- Context information related to the self.
   id   -- Event identifier. This could be any of the following:
   1. Command completion event
   2. Buffer done callback event
   3. Frame done callback event

   RETURN VALUE
   None.

   ========================================================================== */
void omx_vdec::process_event_cb(void *ctxt)
{
    unsigned long p1; // Parameter - 1
    unsigned long p2; // Parameter - 2
    unsigned long ident;
    unsigned qsize=0; // qsize
    omx_vdec *pThis = (omx_vdec *) ctxt;

    if (!pThis) {
        DEBUG_PRINT_ERROR("ERROR: %s()::Context is incorrect, bailing out",
                __func__);
        return;
    }

    // Protect the shared queue data structure
    do {
        /*Read the message id's from the queue*/
        pthread_mutex_lock(&pThis->m_lock);
        qsize = pThis->m_cmd_q.m_size;
        if (qsize) {
            pThis->m_cmd_q.pop_entry(&p1, &p2, &ident);
        }

        if (qsize == 0 && pThis->m_state != OMX_StatePause) {
            qsize = pThis->m_ftb_q.m_size;
            if (qsize) {
                pThis->m_ftb_q.pop_entry(&p1, &p2, &ident);
            }
        }

        if (qsize == 0 && pThis->m_state != OMX_StatePause) {
            qsize = pThis->m_etb_q.m_size;
            if (qsize) {
                pThis->m_etb_q.pop_entry(&p1, &p2, &ident);
            }
        }
        pthread_mutex_unlock(&pThis->m_lock);

        /*process message if we have one*/
        if (qsize > 0) {
            switch (ident) {
                case OMX_COMPONENT_GENERATE_EVENT:
                    if (pThis->m_cb.EventHandler) {
                        switch (p1) {
                            case OMX_CommandStateSet:
                                pThis->m_state = (OMX_STATETYPE) p2;
                                DEBUG_PRINT_HIGH("OMX_CommandStateSet complete, m_state = %d",
                                        pThis->m_state);
                                pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data,
                                        OMX_EventCmdComplete, p1, p2, NULL);
                                break;

                            case OMX_EventError:
                                if (p2 == OMX_StateInvalid) {
                                    DEBUG_PRINT_ERROR("OMX_EventError: p2 is OMX_StateInvalid");
                                    pThis->m_state = (OMX_STATETYPE) p2;
                                    pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data,
                                            OMX_EventError, OMX_ErrorInvalidState, p2, NULL);
                                } else if (p2 == (unsigned long)OMX_ErrorHardware) {
                                    pThis->omx_report_error();
                                } else {
                                    pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data,
                                            OMX_EventError, p2, (OMX_U32)NULL, NULL );
                                }
                                break;

                            case OMX_CommandPortDisable:
                                DEBUG_PRINT_HIGH("OMX_CommandPortDisable complete for port [%lu]", p2);
                                if (BITMASK_PRESENT(&pThis->m_flags,
                                            OMX_COMPONENT_OUTPUT_FLUSH_IN_DISABLE_PENDING)) {
                                    BITMASK_SET(&pThis->m_flags, OMX_COMPONENT_DISABLE_OUTPUT_DEFERRED);
                                    break;
                                }
                                if (p2 == OMX_CORE_OUTPUT_PORT_INDEX) {
                                    OMX_ERRORTYPE eRet = OMX_ErrorNone;
                                    pThis->stream_off(OMX_CORE_OUTPUT_PORT_INDEX);
                                    OMX_ERRORTYPE eRet1 = pThis->get_buffer_req(&pThis->drv_ctx.op_buf);
                                    pThis->in_reconfig = false;
                                    pThis->client_buffers.enable_color_conversion(pThis->c2d_enable_pending);
                                    if (eRet !=  OMX_ErrorNone) {
                                        DEBUG_PRINT_ERROR("set_buffer_req failed eRet = %d",eRet);
                                        pThis->omx_report_error();
                                        break;
                                    }
                                }
                                pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data,
                                        OMX_EventCmdComplete, p1, p2, NULL );
                                break;
                            case OMX_CommandPortEnable:
                                DEBUG_PRINT_HIGH("OMX_CommandPortEnable complete for port [%lu]", p2);
                                pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data,\
                                        OMX_EventCmdComplete, p1, p2, NULL );
                                break;

                            default:
                                pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data,
                                        OMX_EventCmdComplete, p1, p2, NULL );
                                break;

                        }
                    } else {
                        DEBUG_PRINT_ERROR("ERROR: %s()::EventHandler is NULL", __func__);
                    }
                    break;
                case OMX_COMPONENT_GENERATE_ETB: {
                        OMX_ERRORTYPE iret;
                        iret = pThis->empty_this_buffer_proxy((OMX_HANDLETYPE)p1, (OMX_BUFFERHEADERTYPE *)p2);
                        if (iret == OMX_ErrorInsufficientResources) {
                            DEBUG_PRINT_ERROR("empty_this_buffer_proxy failure due to HW overload");
                            pThis->omx_report_hw_overload ();
                        } else if (iret != OMX_ErrorNone) {
                            DEBUG_PRINT_ERROR("empty_this_buffer_proxy failure");
                            pThis->omx_report_error ();
                        }
                    }
                    break;

                case OMX_COMPONENT_GENERATE_FTB:
                    if ( pThis->fill_this_buffer_proxy((OMX_HANDLETYPE)(intptr_t)p1,\
                                (OMX_BUFFERHEADERTYPE *)(intptr_t)p2) != OMX_ErrorNone) {
                        DEBUG_PRINT_ERROR("fill_this_buffer_proxy failure");
                        pThis->omx_report_error ();
                    }
                    break;

                case OMX_COMPONENT_GENERATE_COMMAND:
                    pThis->send_command_proxy(&pThis->m_cmp,(OMX_COMMANDTYPE)p1,\
                            (OMX_U32)p2,(OMX_PTR)NULL);
                    break;

                case OMX_COMPONENT_GENERATE_EBD:
                    if (p2 != VDEC_S_SUCCESS && p2 != VDEC_S_INPUT_BITSTREAM_ERR) {
                        DEBUG_PRINT_ERROR("OMX_COMPONENT_GENERATE_EBD failure");
                        pThis->omx_report_error ();
                    } else {
                        if (p2 == VDEC_S_INPUT_BITSTREAM_ERR && p1) {
                            OMX_BUFFERHEADERTYPE* buffer =
                                    ((OMX_BUFFERHEADERTYPE *)(intptr_t)p1);
                            if (!buffer->pMarkData && !buffer->hMarkTargetComponent) {
                                pThis->time_stamp_dts.remove_time_stamp(
                                        buffer->nTimeStamp,
                                        (pThis->drv_ctx.interlace != VDEC_InterlaceFrameProgressive)
                                        ?true:false);
                            }
                        }

                        if ( pThis->empty_buffer_done(&pThis->m_cmp,
                                    (OMX_BUFFERHEADERTYPE *)(intptr_t)p1) != OMX_ErrorNone) {
                            DEBUG_PRINT_ERROR("empty_buffer_done failure");
                            pThis->omx_report_error ();
                        }
                    }
                    break;
                case OMX_COMPONENT_GENERATE_FBD:
                    if (p2 != VDEC_S_SUCCESS) {
                        DEBUG_PRINT_ERROR("OMX_COMPONENT_GENERATE_FBD failure");
                        pThis->omx_report_error ();
                        break;
                    }
                    if (pThis->m_cb.EventHandler) {
                        OMX_BUFFERHEADERTYPE * buffer = (OMX_BUFFERHEADERTYPE *)(intptr_t)p1;
                        if (buffer->nFilledLen && (pThis->output_capability == V4L2_PIX_FMT_HEVC ||
                            (pThis->output_capability == V4L2_PIX_FMT_VP9 && buffer->pMarkData))) {
                           bool is_list_empty;
                           is_list_empty = false;
                           pthread_mutex_lock(&pThis->m_hdr10pluslock);
                           is_list_empty = pThis->m_hdr10pluslist.empty();
                           pthread_mutex_unlock(&pThis->m_hdr10pluslock);
                           if (!is_list_empty) {
                               DEBUG_PRINT_LOW("fill_buffer_done: event config update");
                               pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data,
                                       OMX_EventConfigUpdate, OMX_CORE_OUTPUT_PORT_INDEX,
                                       OMX_QTIIndexConfigDescribeHDR10PlusInfo, NULL);
                           }
                        }
                    }
                    if (pThis->fill_buffer_done(&pThis->m_cmp,
                               (OMX_BUFFERHEADERTYPE *)(intptr_t)p1) != OMX_ErrorNone ) {
                        DEBUG_PRINT_ERROR("fill_buffer_done failure");
                        pThis->omx_report_error ();
                    }
                    break;

                case OMX_COMPONENT_GENERATE_EVENT_INPUT_FLUSH:
                                        DEBUG_PRINT_HIGH("Driver flush i/p Port complete, flags %#llx",
                                                (unsigned long long)pThis->m_flags);
                                        if (!pThis->input_flush_progress) {
                                            DEBUG_PRINT_HIGH("WARNING: Unexpected flush from driver");
                                        } else {
                                            pThis->execute_input_flush();
                                            if (pThis->m_cb.EventHandler) {
                                                if (p2 != VDEC_S_SUCCESS) {
                                                    DEBUG_PRINT_ERROR("OMX_COMPONENT_GENERATE_EVENT_INPUT_FLUSH failure");
                                                    pThis->omx_report_error ();
                                                } else {
                                                    /*Check if we need generate event for Flush done*/
                                                    pThis->notify_flush_done(ctxt);

                                                    if (BITMASK_PRESENT(&pThis->m_flags,
                                                                OMX_COMPONENT_IDLE_PENDING)) {
                                                        if (pThis->stream_off(OMX_CORE_INPUT_PORT_INDEX)) {
                                                            DEBUG_PRINT_ERROR("Failed to call streamoff on OUTPUT Port");
                                                            pThis->omx_report_error ();
                                                        } else {
                                                            pThis->streaming[OUTPUT_PORT] = false;
                                                        }
                                                        if (!pThis->output_flush_progress) {
                                                            DEBUG_PRINT_LOW("Input flush done hence issue stop");
                                                            pThis->post_event ((unsigned int)NULL, VDEC_S_SUCCESS,\
                                                                    OMX_COMPONENT_GENERATE_STOP_DONE);
                                                        }
                                                    }
                                                }
                                            } else {
                                                DEBUG_PRINT_ERROR("ERROR: %s()::EventHandler is NULL", __func__);
                                            }
                                        }
                                        break;

                case OMX_COMPONENT_GENERATE_EVENT_OUTPUT_FLUSH:
                                        DEBUG_PRINT_HIGH("Driver flush o/p Port complete, flags %#llx",
                                                (unsigned long long)pThis->m_flags);
                                        if (!pThis->output_flush_progress) {
                                            DEBUG_PRINT_HIGH("WARNING: Unexpected flush from driver");
                                        } else {
                                            pThis->execute_output_flush();
                                            if (pThis->m_cb.EventHandler) {
                                                if (p2 != VDEC_S_SUCCESS) {
                                                    DEBUG_PRINT_ERROR("OMX_COMPONENT_GENERATE_EVENT_OUTPUT_FLUSH failed");
                                                    pThis->omx_report_error ();
                                                } else {
                                                    /*Check if we need generate event for Flush done*/
                                                    pThis->notify_flush_done(ctxt);

                                                    if (BITMASK_PRESENT(&pThis->m_flags,
                                                                OMX_COMPONENT_OUTPUT_FLUSH_IN_DISABLE_PENDING)) {
                                                        DEBUG_PRINT_LOW("Internal flush complete");
                                                        BITMASK_CLEAR (&pThis->m_flags,
                                                                OMX_COMPONENT_OUTPUT_FLUSH_IN_DISABLE_PENDING);
                                                        if (BITMASK_PRESENT(&pThis->m_flags,
                                                                    OMX_COMPONENT_DISABLE_OUTPUT_DEFERRED)) {
                                                            pThis->post_event(OMX_CommandPortDisable,
                                                                    OMX_CORE_OUTPUT_PORT_INDEX,
                                                                    OMX_COMPONENT_GENERATE_EVENT);
                                                            BITMASK_CLEAR (&pThis->m_flags,
                                                                    OMX_COMPONENT_DISABLE_OUTPUT_DEFERRED);
                                                            BITMASK_CLEAR (&pThis->m_flags,
                                                                    OMX_COMPONENT_OUTPUT_DISABLE_PENDING);

                                                        }
                                                    }

                                                    if (BITMASK_PRESENT(&pThis->m_flags ,OMX_COMPONENT_IDLE_PENDING)) {
                                                        if (pThis->stream_off(OMX_CORE_OUTPUT_PORT_INDEX)) {
                                                            DEBUG_PRINT_ERROR("Failed to call streamoff on CAPTURE Port");
                                                            pThis->omx_report_error ();
                                                            break;
                                                        }
                                                        pThis->streaming[CAPTURE_PORT] = false;
                                                        if (!pThis->input_flush_progress) {
                                                            DEBUG_PRINT_LOW("Output flush done hence issue stop");
                                                            pThis->post_event ((unsigned int)NULL, VDEC_S_SUCCESS,\
                                                                    OMX_COMPONENT_GENERATE_STOP_DONE);
                                                        }
                                                    }
                                                }
                                            } else {
                                                DEBUG_PRINT_ERROR("ERROR: %s()::EventHandler is NULL", __func__);
                                            }
                                        }
                                        break;

                case OMX_COMPONENT_GENERATE_START_DONE:
                                        DEBUG_PRINT_HIGH("Rxd OMX_COMPONENT_GENERATE_START_DONE, flags %#llx",
                                                (unsigned long long)pThis->m_flags);
                                        if (pThis->m_cb.EventHandler) {
                                            if (p2 != VDEC_S_SUCCESS) {
                                                DEBUG_PRINT_ERROR("OMX_COMPONENT_GENERATE_START_DONE Failure");
                                                pThis->omx_report_error ();
                                            } else {
                                                DEBUG_PRINT_LOW("OMX_COMPONENT_GENERATE_START_DONE Success");
                                                if (BITMASK_PRESENT(&pThis->m_flags,OMX_COMPONENT_EXECUTE_PENDING)) {
                                                    DEBUG_PRINT_LOW("Move to executing");
                                                    // Send the callback now
                                                    BITMASK_CLEAR((&pThis->m_flags),OMX_COMPONENT_EXECUTE_PENDING);
                                                    pThis->m_state = OMX_StateExecuting;
                                                    pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data,
                                                            OMX_EventCmdComplete,OMX_CommandStateSet,
                                                            OMX_StateExecuting, NULL);
                                                } else if (BITMASK_PRESENT(&pThis->m_flags,
                                                            OMX_COMPONENT_PAUSE_PENDING)) {
                                                    if (/*ioctl (pThis->drv_ctx.video_driver_fd,
                                                          VDEC_IOCTL_CMD_PAUSE,NULL ) < */0) {
                                                        DEBUG_PRINT_ERROR("VDEC_IOCTL_CMD_PAUSE failed");
                                                        pThis->omx_report_error ();
                                                    }
                                                }
                                            }
                                        } else {
                                            DEBUG_PRINT_LOW("Event Handler callback is NULL");
                                        }
                                        break;

                case OMX_COMPONENT_GENERATE_PAUSE_DONE:
                                        DEBUG_PRINT_HIGH("Rxd OMX_COMPONENT_GENERATE_PAUSE_DONE");
                                        if (pThis->m_cb.EventHandler) {
                                            if (p2 != VDEC_S_SUCCESS) {
                                                DEBUG_PRINT_ERROR("OMX_COMPONENT_GENERATE_PAUSE_DONE ret failed");
                                                pThis->omx_report_error ();
                                            } else {
                                                pThis->complete_pending_buffer_done_cbs();
                                                if (BITMASK_PRESENT(&pThis->m_flags,OMX_COMPONENT_PAUSE_PENDING)) {
                                                    DEBUG_PRINT_LOW("OMX_COMPONENT_GENERATE_PAUSE_DONE nofity");
                                                    //Send the callback now
                                                    BITMASK_CLEAR((&pThis->m_flags),OMX_COMPONENT_PAUSE_PENDING);
                                                    pThis->m_state = OMX_StatePause;
                                                    pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data,
                                                            OMX_EventCmdComplete,OMX_CommandStateSet,
                                                            OMX_StatePause, NULL);
                                                }
                                            }
                                        } else {
                                            DEBUG_PRINT_ERROR("ERROR: %s()::EventHandler is NULL", __func__);
                                        }

                                        break;

                case OMX_COMPONENT_GENERATE_RESUME_DONE:
                                        DEBUG_PRINT_HIGH("Rxd OMX_COMPONENT_GENERATE_RESUME_DONE");
                                        if (pThis->m_cb.EventHandler) {
                                            if (p2 != VDEC_S_SUCCESS) {
                                                DEBUG_PRINT_ERROR("OMX_COMPONENT_GENERATE_RESUME_DONE failed");
                                                pThis->omx_report_error ();
                                            } else {
                                                if (BITMASK_PRESENT(&pThis->m_flags,OMX_COMPONENT_EXECUTE_PENDING)) {
                                                    DEBUG_PRINT_LOW("Moving the decoder to execute state");
                                                    // Send the callback now
                                                    BITMASK_CLEAR((&pThis->m_flags),OMX_COMPONENT_EXECUTE_PENDING);
                                                    pThis->m_state = OMX_StateExecuting;
                                                    pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data,
                                                            OMX_EventCmdComplete,OMX_CommandStateSet,
                                                            OMX_StateExecuting,NULL);
                                                }
                                            }
                                        } else {
                                            DEBUG_PRINT_ERROR("ERROR: %s()::EventHandler is NULL", __func__);
                                        }

                                        break;

                case OMX_COMPONENT_GENERATE_STOP_DONE:
                                        DEBUG_PRINT_HIGH("Rxd OMX_COMPONENT_GENERATE_STOP_DONE");
                                        if (pThis->m_cb.EventHandler) {
                                            if (p2 != VDEC_S_SUCCESS) {
                                                DEBUG_PRINT_ERROR("OMX_COMPONENT_GENERATE_STOP_DONE ret failed");
                                                pThis->omx_report_error ();
                                            } else {
                                                pThis->complete_pending_buffer_done_cbs();
                                                if (BITMASK_PRESENT(&pThis->m_flags,OMX_COMPONENT_IDLE_PENDING)) {
                                                    DEBUG_PRINT_LOW("OMX_COMPONENT_GENERATE_STOP_DONE Success");
                                                    // Send the callback now
                                                    BITMASK_CLEAR((&pThis->m_flags),OMX_COMPONENT_IDLE_PENDING);
                                                    pThis->m_state = OMX_StateIdle;
                                                    DEBUG_PRINT_LOW("Move to Idle State");
                                                    pThis->m_cb.EventHandler(&pThis->m_cmp,pThis->m_app_data,
                                                            OMX_EventCmdComplete,OMX_CommandStateSet,
                                                            OMX_StateIdle,NULL);
                                                }
                                            }
                                        } else {
                                            DEBUG_PRINT_ERROR("ERROR: %s()::EventHandler is NULL", __func__);
                                        }

                                        break;

                case OMX_COMPONENT_GENERATE_PORT_RECONFIG:
                                        if (p2 == OMX_IndexParamPortDefinition) {
                                            DEBUG_PRINT_HIGH("Rxd PORT_RECONFIG: OMX_IndexParamPortDefinition");
                                            pThis->in_reconfig = true;
                                            pThis->prev_n_filled_len = 0;
                                        }  else if (p2 == OMX_IndexConfigCommonOutputCrop) {
                                            DEBUG_PRINT_HIGH("Rxd PORT_RECONFIG: OMX_IndexConfigCommonOutputCrop");
                                            /* Update new crop information */
                                            pThis->rectangle.nLeft = pThis->drv_ctx.frame_size.left;
                                            pThis->rectangle.nTop = pThis->drv_ctx.frame_size.top;
                                            pThis->rectangle.nWidth = pThis->drv_ctx.frame_size.right;
                                            pThis->rectangle.nHeight = pThis->drv_ctx.frame_size.bottom;

                                            /* Validate the new crop information */
                                            if (pThis->rectangle.nLeft + pThis->rectangle.nWidth >
                                                pThis->drv_ctx.video_resolution.frame_width) {

                                                DEBUG_PRINT_HIGH("Crop L[%u] + R[%u] > W[%u]",
                                                        pThis->rectangle.nLeft, pThis->rectangle.nWidth,
                                                        pThis->drv_ctx.video_resolution.frame_width);
                                                pThis->rectangle.nLeft = 0;

                                                if (pThis->rectangle.nWidth >
                                                    pThis->drv_ctx.video_resolution.frame_width) {

                                                    DEBUG_PRINT_HIGH("Crop R[%u] > W[%u]",
                                                            pThis->rectangle.nWidth,
                                                            pThis->drv_ctx.video_resolution.frame_width);
                                                    pThis->rectangle.nWidth =
                                                        pThis->drv_ctx.video_resolution.frame_width;
                                                }
                                            }
                                            if (pThis->rectangle.nTop + pThis->rectangle.nHeight >
                                                pThis->drv_ctx.video_resolution.frame_height) {

                                                DEBUG_PRINT_HIGH("Crop T[%u] + B[%u] > H[%u]",
                                                    pThis->rectangle.nTop, pThis->rectangle.nHeight,
                                                    pThis->drv_ctx.video_resolution.frame_height);
                                                pThis->rectangle.nTop = 0;

                                                if (pThis->rectangle.nHeight >
                                                    pThis->drv_ctx.video_resolution.frame_height) {

                                                    DEBUG_PRINT_HIGH("Crop B[%u] > H[%u]",
                                                        pThis->rectangle.nHeight,
                                                        pThis->drv_ctx.video_resolution.frame_height);
                                                    pThis->rectangle.nHeight =
                                                        pThis->drv_ctx.video_resolution.frame_height;
                                                }
                                            }
                                            DEBUG_PRINT_HIGH("Updated Crop Info: L: %u, T: %u, R: %u, B: %u",
                                                    pThis->rectangle.nLeft, pThis->rectangle.nTop,
                                                    pThis->rectangle.nWidth, pThis->rectangle.nHeight);
                                        } else if (p2 == OMX_QTIIndexConfigDescribeColorAspects) {
                                            DEBUG_PRINT_HIGH("Rxd PORT_RECONFIG: OMX_QTIIndexConfigDescribeColorAspects");
                                        } else if (p2 == OMX_QTIIndexConfigDescribeHDRColorInfo) {
                                            DEBUG_PRINT_HIGH("Rxd PORT_RECONFIG: OMX_QTIIndexConfigDescribeHDRcolorinfo");
                                        } else {
                                            DEBUG_PRINT_ERROR("Rxd Invalid PORT_RECONFIG event (%lu)", p2);
                                            break;
                                        }
                                        if (pThis->m_debug.outfile) {
                                            fclose(pThis->m_debug.outfile);
                                            pThis->m_debug.outfile = NULL;
                                        }
                                        if (pThis->m_debug.ccoutfile) {
                                            fclose(pThis->m_debug.ccoutfile);
                                            pThis->m_debug.ccoutfile = NULL;
                                        }
                                        if (pThis->m_debug.out_ymeta_file) {
                                            fclose(pThis->m_debug.out_ymeta_file);
                                            pThis->m_debug.out_ymeta_file = NULL;
                                        }
                                        if (pThis->m_debug.out_uvmeta_file) {
                                            fclose(pThis->m_debug.out_uvmeta_file);
                                            pThis->m_debug.out_uvmeta_file = NULL;
                                        }
                                        pThis->m_debug.seq_count++;

                                        if (pThis->m_cb.EventHandler) {
                                            void *frame_data = NULL;
                                            reconfig_client_data port_data;
                                            reconfig_client_crop_data crop_data;
                                            if (p2 == OMX_IndexConfigCommonOutputCrop) {
                                                crop_data.width = pThis->rectangle.nWidth;
                                                crop_data.height = pThis->rectangle.nHeight;
                                                crop_data.left = pThis->rectangle.nLeft;
                                                crop_data.top = pThis->rectangle.nTop;
                                                crop_data.isPortReconfigInsufficient = pThis->isPortReconfigInsufficient;
                                                frame_data = (void*)&crop_data;
                                            } else if (p2 == OMX_IndexParamPortDefinition){
                                                port_data.width = pThis->m_reconfig_width;
                                                port_data.height = pThis->m_reconfig_height;
                                                port_data.dpb_bit_depth = pThis->dpb_bit_depth;
                                                port_data.m_progressive = pThis->m_progressive;
                                                port_data.isPortReconfigInsufficient = pThis->isPortReconfigInsufficient;
                                                frame_data = (void*)&port_data;
                                            }
                                            pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data,
                                                     OMX_EventPortSettingsChanged, p1, p2, (void*)frame_data);
                                        } else {
                                            DEBUG_PRINT_ERROR("ERROR: %s()::EventHandler is NULL", __func__);
                                        }
                                        break;

                case OMX_COMPONENT_GENERATE_EOS_DONE:
                                        DEBUG_PRINT_HIGH("Rxd OMX_COMPONENT_GENERATE_EOS_DONE");
                                        if (pThis->m_cb.EventHandler) {
                                            pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data, OMX_EventBufferFlag,
                                                    OMX_CORE_OUTPUT_PORT_INDEX, OMX_BUFFERFLAG_EOS, NULL );
                                        } else {
                                            DEBUG_PRINT_ERROR("ERROR: %s()::EventHandler is NULL", __func__);
                                        }
                                        pThis->prev_ts = LLONG_MAX;
                                        break;

                case OMX_COMPONENT_GENERATE_HARDWARE_ERROR:
                                        DEBUG_PRINT_ERROR("OMX_COMPONENT_GENERATE_HARDWARE_ERROR");
                                        pThis->omx_report_error();
                                        break;

                case OMX_COMPONENT_GENERATE_UNSUPPORTED_SETTING:
                                        DEBUG_PRINT_ERROR("OMX_COMPONENT_GENERATE_UNSUPPORTED_SETTING");
                                        pThis->omx_report_unsupported_setting();
                                        break;

                case OMX_COMPONENT_GENERATE_HARDWARE_OVERLOAD:
                                        DEBUG_PRINT_ERROR("OMX_COMPONENT_GENERATE_HARDWARE_OVERLOAD");
                                        pThis->omx_report_hw_overload();
                                        break;

                case OMX_COMPONENT_GENERATE_ION_PREFETCH_PIXEL:
                                        DEBUG_PRINT_HIGH("OMX_COMPONENT_GENERATE_ION_PREFETCH_PIXEL");
                                        pThis->m_prefetch_done |= pThis->prefetch_buffers(p1, p2, ION_IOC_PREFETCH, ION_FLAG_CP_PIXEL);
                                        break;

                case OMX_COMPONENT_GENERATE_ION_PREFETCH_NON_PIXEL:
                                        DEBUG_PRINT_HIGH("OMX_COMPONENT_GENERATE_ION_PREFETCH_NON_PIXEL");
                                        pThis->m_prefetch_done |= pThis->prefetch_buffers(p1, p2, ION_IOC_PREFETCH, ION_FLAG_CP_NON_PIXEL) << 1;
                                        break;

                default:
                                        break;
            }
        }
        pthread_mutex_lock(&pThis->m_lock);
        qsize = pThis->m_cmd_q.m_size;
        if (pThis->m_state != OMX_StatePause)
            qsize += (pThis->m_ftb_q.m_size + pThis->m_etb_q.m_size);
        pthread_mutex_unlock(&pThis->m_lock);
    } while (qsize>0);

}

int omx_vdec::update_resolution(int width, int height, int stride, int scan_lines)
{
    int format_changed = 0;
    if ((height != (int)drv_ctx.video_resolution.frame_height) ||
            (width != (int)drv_ctx.video_resolution.frame_width)) {
        DEBUG_PRINT_HIGH("NOTE_CIF: W/H %d (%d), %d (%d)",
                width, drv_ctx.video_resolution.frame_width,
                height,drv_ctx.video_resolution.frame_height);
        format_changed = 1;
    }
    drv_ctx.video_resolution.frame_height = height;
    drv_ctx.video_resolution.frame_width = width;
    drv_ctx.video_resolution.scan_lines = scan_lines;
    drv_ctx.video_resolution.stride = stride;

    rectangle.nLeft = m_extradata_misr.output_crop_rect.nLeft;
    rectangle.nTop = m_extradata_misr.output_crop_rect.nTop;
    rectangle.nWidth = m_extradata_misr.output_crop_rect.nWidth;
    rectangle.nHeight = m_extradata_misr.output_crop_rect.nHeight;

    return format_changed;
}

int omx_vdec::log_input_buffers(const char *buffer_addr, int buffer_len, uint64_t timeStamp, int fd)
{
    if (!m_debug.in_buffer_log)
        return 0;

    sync_start_read(fd);
    if (m_debug.in_buffer_log && !m_debug.infile) {
        if(!strncmp(drv_ctx.kind,"OMX.qcom.video.decoder.mpeg2", OMX_MAX_STRINGNAME_SIZE)) {
                snprintf(m_debug.infile_name, OMX_MAX_STRINGNAME_SIZE, "%s/input_dec_%d_%d_%p_%" PRId64 ".mpg", m_debug.log_loc,
                        drv_ctx.video_resolution.frame_width, drv_ctx.video_resolution.frame_height, this, m_debug.session_id);
        } else if(!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.avc", OMX_MAX_STRINGNAME_SIZE)) {
                snprintf(m_debug.infile_name, OMX_MAX_STRINGNAME_SIZE, "%s/input_dec_%d_%d_%p.264",
                        m_debug.log_loc, drv_ctx.video_resolution.frame_width, drv_ctx.video_resolution.frame_height, this);
        } else if(!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.hevc", OMX_MAX_STRINGNAME_SIZE)) {
                snprintf(m_debug.infile_name, OMX_MAX_STRINGNAME_SIZE, "%s/input_dec_%d_%d_%p.265",
                        m_debug.log_loc, drv_ctx.video_resolution.frame_width, drv_ctx.video_resolution.frame_height, this);
        } else if(!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.vp8", OMX_MAX_STRINGNAME_SIZE)) {
                snprintf(m_debug.infile_name, OMX_MAX_STRINGNAME_SIZE, "%s/input_dec_%d_%d_%p.ivf",
                        m_debug.log_loc, drv_ctx.video_resolution.frame_width, drv_ctx.video_resolution.frame_height, this);
        } else if(!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.vp9", OMX_MAX_STRINGNAME_SIZE)) {
                snprintf(m_debug.infile_name, OMX_MAX_STRINGNAME_SIZE, "%s/input_dec_%d_%d_%p.ivf",
                        m_debug.log_loc, drv_ctx.video_resolution.frame_width, drv_ctx.video_resolution.frame_height, this);
        } else {
               snprintf(m_debug.infile_name, OMX_MAX_STRINGNAME_SIZE, "%s/input_dec_%d_%d_%p.bin",
                        m_debug.log_loc, drv_ctx.video_resolution.frame_width, drv_ctx.video_resolution.frame_height, this);
        }
        m_debug.infile = fopen (m_debug.infile_name, "ab");
        if (!m_debug.infile) {
            DEBUG_PRINT_HIGH("Failed to open input file: %s for logging (%d:%s)",
                             m_debug.infile_name, errno, strerror(errno));
            m_debug.infile_name[0] = '\0';
            sync_end_read(fd);
            return -1;
        }
        if (!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.vp8", OMX_MAX_STRINGNAME_SIZE) ||
                !strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.vp9", OMX_MAX_STRINGNAME_SIZE)) {
            bool isVp9 = drv_ctx.decoder_format == VDEC_CODECTYPE_VP9;
            int width = drv_ctx.video_resolution.frame_width;
            int height = drv_ctx.video_resolution.frame_height;
            int fps = drv_ctx.frame_rate.fps_numerator;
            IvfFileHeader ivfHeader(isVp9, width, height, 1, fps, 0);
            fwrite((const char *)&ivfHeader,
                    sizeof(ivfHeader),1,m_debug.infile);
         }
    }
    if (m_debug.infile && buffer_addr && buffer_len) {
        if (!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.vp8", OMX_MAX_STRINGNAME_SIZE) ||
                !strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.vp9", OMX_MAX_STRINGNAME_SIZE)) {
            IvfFrameHeader ivfFrameHeader(buffer_len, timeStamp);
            fwrite(&ivfFrameHeader, sizeof(ivfFrameHeader), 1, m_debug.infile);
        }
        fwrite(buffer_addr, buffer_len, 1, m_debug.infile);
    }
    sync_end_read(fd);
    return 0;
}

int omx_vdec::log_cc_output_buffers(OMX_BUFFERHEADERTYPE *buffer)
{
    vdec_bufferpayload *vdec_buf = NULL;
    int index = 0;
    if (client_buffers.client_buffers_invalid() ||
        !m_debug.out_cc_buffer_log || !buffer || !buffer->nFilledLen)
        return 0;

    index = buffer - m_out_mem_ptr;
    vdec_buf = &drv_ctx.ptr_outputbuffer[index];
    sync_start_read(vdec_buf->pmem_fd);
    if (m_debug.out_cc_buffer_log && !m_debug.ccoutfile) {
        snprintf(m_debug.ccoutfile_name, OMX_MAX_STRINGNAME_SIZE, "%s/output_cc_%d_%d_%p_%" PRId64 "_%d.yuv",
                m_debug.log_loc, drv_ctx.video_resolution.frame_width, drv_ctx.video_resolution.frame_height, this,
                m_debug.session_id, m_debug.seq_count);
        m_debug.ccoutfile = fopen (m_debug.ccoutfile_name, "ab");
        if (!m_debug.ccoutfile) {
            DEBUG_PRINT_HIGH("Failed to open output file: %s for logging", m_debug.log_loc);
            m_debug.ccoutfile_name[0] = '\0';
            return -1;
        }
        DEBUG_PRINT_HIGH("Opened CC output file: %s for logging", m_debug.ccoutfile_name);
    }

    fwrite(buffer->pBuffer, buffer->nFilledLen, 1, m_debug.ccoutfile);
    sync_end_read(vdec_buf->pmem_fd);
    return 0;
}

int omx_vdec::log_output_buffers(OMX_BUFFERHEADERTYPE *buffer)
{
    int buf_index = 0;
    char *temp = NULL;
    char *bufaddr = NULL;

    if (!(m_debug.out_buffer_log || m_debug.out_meta_buffer_log) || !buffer || !buffer->nFilledLen)
        return 0;

    if (m_debug.out_buffer_log && !m_debug.outfile) {
        snprintf(m_debug.outfile_name, OMX_MAX_STRINGNAME_SIZE, "%s/output_%d_%d_%p_%" PRId64 "_%d.yuv",
                m_debug.log_loc, drv_ctx.video_resolution.frame_width, drv_ctx.video_resolution.frame_height, this,
                m_debug.session_id, m_debug.seq_count);
        m_debug.outfile = fopen (m_debug.outfile_name, "ab");
        if (!m_debug.outfile) {
            DEBUG_PRINT_HIGH("Failed to open output file: %s for logging", m_debug.log_loc);
            m_debug.outfile_name[0] = '\0';
            return -1;
        }
        DEBUG_PRINT_HIGH("Opened output file: %s for logging", m_debug.outfile_name);
    }

    if (m_debug.out_meta_buffer_log && !m_debug.out_ymeta_file && !m_debug.out_uvmeta_file) {
        snprintf(m_debug.out_ymetafile_name, OMX_MAX_STRINGNAME_SIZE, "%s/output_%d_%d_%p.ymeta",
                m_debug.log_loc, drv_ctx.video_resolution.frame_width, drv_ctx.video_resolution.frame_height, this);
        snprintf(m_debug.out_uvmetafile_name, OMX_MAX_STRINGNAME_SIZE, "%s/output_%d_%d_%p.uvmeta",
                m_debug.log_loc, drv_ctx.video_resolution.frame_width, drv_ctx.video_resolution.frame_height, this);
        m_debug.out_ymeta_file = fopen (m_debug.out_ymetafile_name, "ab");
        m_debug.out_uvmeta_file = fopen (m_debug.out_uvmetafile_name, "ab");
        if (!m_debug.out_ymeta_file || !m_debug.out_uvmeta_file) {
            DEBUG_PRINT_HIGH("Failed to open output y/uv meta file: %s for logging", m_debug.log_loc);
            m_debug.out_ymetafile_name[0] = '\0';
            m_debug.out_uvmetafile_name[0] = '\0';
            return -1;
        }
    }

    vdec_bufferpayload *vdec_buf = NULL;
    if (client_buffers.is_color_conversion_enabled()) {
        buf_index = buffer - m_intermediate_out_mem_ptr;
        vdec_buf = &drv_ctx.ptr_intermediate_outputbuffer[buf_index];
    } else {
        buf_index = buffer - m_out_mem_ptr;
        vdec_buf = &drv_ctx.ptr_outputbuffer[buf_index];
    }
    bufaddr = (char *)vdec_buf->bufferaddr;
    if (dynamic_buf_mode && !secure_mode) {
        bufaddr = ion_map(vdec_buf->pmem_fd,
                          vdec_buf->buffer_len);
        //mmap returns (void *)-1 on failure and sets error code in errno.
        if (bufaddr == MAP_FAILED) {
            DEBUG_PRINT_ERROR("mmap failed - errno: %d", errno);
            return -1;
        }
    }
    temp = bufaddr;

    if (drv_ctx.output_format == VDEC_YUV_FORMAT_NV12_UBWC ||
            drv_ctx.output_format == VDEC_YUV_FORMAT_NV12_TP10_UBWC) {
        DEBUG_PRINT_HIGH("Logging UBWC yuv width/height(%u/%u)",
            drv_ctx.video_resolution.frame_width,
            drv_ctx.video_resolution.frame_height);

        if (m_debug.outfile)
            fwrite(temp, buffer->nFilledLen, 1, m_debug.outfile);

        if (m_debug.out_ymeta_file && m_debug.out_uvmeta_file) {
            unsigned int width = 0, height = 0;
            unsigned int y_plane, y_meta_plane;
            int y_stride = 0, y_sclines = 0;
            int y_meta_stride = 0, y_meta_scanlines = 0, uv_meta_stride = 0, uv_meta_scanlines = 0;
            int color_fmt = (drv_ctx.output_format== VDEC_YUV_FORMAT_NV12_UBWC)? COLOR_FMT_NV12_UBWC: COLOR_FMT_NV12_BPP10_UBWC;
            int i;
            int bytes_written = 0;

            width = drv_ctx.video_resolution.frame_width;
            height = drv_ctx.video_resolution.frame_height;
            y_meta_stride = VENUS_Y_META_STRIDE(color_fmt, width);
            y_meta_scanlines = VENUS_Y_META_SCANLINES(color_fmt, height);
            y_stride = VENUS_Y_STRIDE(color_fmt, width);
            y_sclines = VENUS_Y_SCANLINES(color_fmt, height);
            uv_meta_stride = VENUS_UV_META_STRIDE(color_fmt, width);
            uv_meta_scanlines = VENUS_UV_META_SCANLINES(color_fmt, height);

            y_meta_plane = MSM_MEDIA_ALIGN(y_meta_stride * y_meta_scanlines, 4096);
            y_plane = MSM_MEDIA_ALIGN(y_stride * y_sclines, 4096);

            for (i = 0; i < y_meta_scanlines; i++) {
                 bytes_written = fwrite(temp, y_meta_stride, 1, m_debug.out_ymeta_file);
                 temp += y_meta_stride;
            }

            temp = bufaddr + y_meta_plane + y_plane;
            for(i = 0; i < uv_meta_scanlines; i++) {
                bytes_written += fwrite(temp, uv_meta_stride, 1, m_debug.out_uvmeta_file);
                temp += uv_meta_stride;
            }
        }
    } else if (m_debug.outfile && drv_ctx.output_format == VDEC_YUV_FORMAT_NV12) {
        int stride = drv_ctx.video_resolution.stride;
        int scanlines = drv_ctx.video_resolution.scan_lines;
        unsigned i;
        DEBUG_PRINT_HIGH("Logging width/height(%u/%u) stride/scanlines(%u/%u)",
            drv_ctx.video_resolution.frame_width,
            drv_ctx.video_resolution.frame_height, stride, scanlines);
        int bytes_written = 0;
        for (i = 0; i < drv_ctx.video_resolution.frame_height; i++) {
             bytes_written = fwrite(temp, drv_ctx.video_resolution.frame_width, 1, m_debug.outfile);
             temp += stride;
        }
        temp = bufaddr + stride * scanlines;
        int stride_c = stride;
        for(i = 0; i < drv_ctx.video_resolution.frame_height/2; i++) {
            bytes_written += fwrite(temp, drv_ctx.video_resolution.frame_width, 1, m_debug.outfile);
            temp += stride_c;
        }
    } else if (m_debug.outfile && drv_ctx.output_format == VDEC_YUV_FORMAT_P010_VENUS) {
        int stride = drv_ctx.video_resolution.stride;
        int scanlines = drv_ctx.video_resolution.scan_lines;
        unsigned i;
        DEBUG_PRINT_HIGH("Logging width/height(%u/%u) stride/scanlines(%u/%u)",
            drv_ctx.video_resolution.frame_width,
            drv_ctx.video_resolution.frame_height, stride, scanlines);
        int bytes_written = 0;
        for (i = 0; i < drv_ctx.video_resolution.frame_height; i++) {
             bytes_written = fwrite(temp, drv_ctx.video_resolution.frame_width, 2, m_debug.outfile);
             temp += stride;
        }
        temp = bufaddr + stride * scanlines;
        int stride_c = stride;
        for(i = 0; i < drv_ctx.video_resolution.frame_height/2; i++) {
            bytes_written += fwrite(temp, drv_ctx.video_resolution.frame_width, 2, m_debug.outfile);
            temp += stride_c;
        }
    }

    if (dynamic_buf_mode && !secure_mode) {
        ion_unmap(vdec_buf->pmem_fd, bufaddr,
                  vdec_buf->buffer_len);
    }
    return 0;
}

void omx_vdec::init_color_aspects_map()
{
    mPrimariesMap.insert({
            {ColorAspects::PrimariesUnspecified, (ColorPrimaries)(2)},
            {ColorAspects::PrimariesBT709_5, ColorPrimaries_BT709_5},
            {ColorAspects::PrimariesBT470_6M, ColorPrimaries_BT470_6M},
            {ColorAspects::PrimariesBT601_6_625, ColorPrimaries_BT601_6_625},
            {ColorAspects::PrimariesBT601_6_525, ColorPrimaries_BT601_6_525},
            {ColorAspects::PrimariesGenericFilm, ColorPrimaries_GenericFilm},
            {ColorAspects::PrimariesBT2020, ColorPrimaries_BT2020},
        });
    mTransferMap.insert({
            {ColorAspects::TransferUnspecified, (GammaTransfer)(2)},
            {ColorAspects::TransferLinear, Transfer_Linear},
            {ColorAspects::TransferSRGB, Transfer_sRGB},
            {ColorAspects::TransferSMPTE170M, Transfer_SMPTE_170M},
            {ColorAspects::TransferGamma22, Transfer_Gamma2_2},
            {ColorAspects::TransferGamma28, Transfer_Gamma2_8},
            {ColorAspects::TransferST2084, Transfer_SMPTE_ST2084},
            {ColorAspects::TransferHLG, Transfer_HLG},
            {ColorAspects::TransferSMPTE240M, Transfer_SMPTE_240M},
            {ColorAspects::TransferXvYCC, Transfer_XvYCC},
            {ColorAspects::TransferBT1361, Transfer_BT1361},
            {ColorAspects::TransferST428, Transfer_ST_428},
        });
    mMatrixCoeffMap.insert({
            {ColorAspects::MatrixUnspecified, (MatrixCoEfficients)(2)},
            {ColorAspects::MatrixBT709_5, MatrixCoEff_BT709_5},
            {ColorAspects::MatrixBT470_6M, MatrixCoeff_FCC_73_682},
            {ColorAspects::MatrixBT601_6, MatrixCoEff_BT601_6_625},
            {ColorAspects::MatrixSMPTE240M, MatrixCoEff_SMPTE240M},
            {ColorAspects::MatrixBT2020, MatrixCoEff_BT2020},
            {ColorAspects::MatrixBT2020Constant, MatrixCoEff_BT2020Constant},
        });
    mColorRangeMap.insert({
            {ColorAspects::RangeUnspecified, (ColorRange)(2)},
            {ColorAspects::RangeFull, Range_Full},
            {ColorAspects::RangeLimited, Range_Limited},
        });
}
/* ======================================================================
   FUNCTION
   omx_vdec::ComponentInit

   DESCRIPTION
   Initialize the component.

   PARAMETERS
   ctxt -- Context information related to the self.
   id   -- Event identifier. This could be any of the following:
   1. Command completion event
   2. Buffer done callback event
   3. Frame done callback event

   RETURN VALUE
   None.

   ========================================================================== */
OMX_ERRORTYPE omx_vdec::component_init(OMX_STRING role)
{

    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    struct v4l2_fmtdesc fdesc;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers bufreq;
    struct v4l2_control control;
    struct v4l2_frmsizeenum frmsize;
    struct v4l2_queryctrl query;
    unsigned int   alignment = 0,buffer_size = 0;
    int fds[2];
    int r,ret=0;
    bool codec_ambiguous = false;
    OMX_STRING device_name = (OMX_STRING)"/dev/video32";
    char property_value[PROPERTY_VALUE_MAX] = {0};
    FILE *soc_file = NULL;
    char buffer[10];
    struct v4l2_control ctrl[2];
    int conceal_color_8bit = 0, conceal_color_10bit = 0;

    property_get("ro.board.platform", m_platform_name, "0");
#ifdef _ANDROID_
    if (!strncmp(m_platform_name, "msm8610", 7)) {
        device_name = (OMX_STRING)"/dev/video/q6_dec";
    }
#endif

    if (!strncmp(role, "OMX.qcom.video.decoder.avc.secure",
                OMX_MAX_STRINGNAME_SIZE)) {
        secure_mode = true;
        role = (OMX_STRING)"OMX.qcom.video.decoder.avc";
    } else if (!strncmp(role, "OMX.qcom.video.decoder.mpeg2.secure",
                OMX_MAX_STRINGNAME_SIZE)) {
        secure_mode = true;
        role = (OMX_STRING)"OMX.qcom.video.decoder.mpeg2";
    } else if (!strncmp(role, "OMX.qcom.video.decoder.hevc.secure",
                OMX_MAX_STRINGNAME_SIZE)) {
        secure_mode = true;
        role = (OMX_STRING)"OMX.qcom.video.decoder.hevc";
    } else if (!strncmp(role, "OMX.qcom.video.decoder.vp9.secure",
                OMX_MAX_STRINGNAME_SIZE)) {
        secure_mode = true;
        role = (OMX_STRING)"OMX.qcom.video.decoder.vp9";
    }

    drv_ctx.video_driver_fd = open(device_name, O_RDWR);

    DEBUG_PRINT_INFO("component_init: %s : fd=%d", role, drv_ctx.video_driver_fd);

    if (drv_ctx.video_driver_fd < 0) {
        DEBUG_PRINT_ERROR("Omx_vdec::Comp Init Returning failure, errno %d", errno);
        return OMX_ErrorInsufficientResources;
    }
    drv_ctx.frame_rate.fps_numerator = DEFAULT_FPS;
    drv_ctx.frame_rate.fps_denominator = 1;
    m_poll_efd = eventfd(0, 0);
    if (m_poll_efd < 0) {
        DEBUG_PRINT_ERROR("Failed to create event fd(%s)", strerror(errno));
        return OMX_ErrorInsufficientResources;
    }
    ret = subscribe_to_events(drv_ctx.video_driver_fd);
    if (!ret) {
        async_thread_created = true;
        ret = pthread_create(&async_thread_id,0,async_message_thread,this);
    }
    if (ret) {
        DEBUG_PRINT_ERROR("Failed to create async_message_thread");
        async_thread_created = false;
        return OMX_ErrorInsufficientResources;
    }

#ifdef OUTPUT_EXTRADATA_LOG
    outputExtradataFile = fopen (output_extradata_filename, "ab");
#endif

    // Copy the role information which provides the decoder kind
    strlcpy(drv_ctx.kind,role,128);


    if (!strncmp(drv_ctx.kind,"OMX.qcom.video.decoder.mpeg2",\
                OMX_MAX_STRINGNAME_SIZE)) {
        strlcpy((char *)m_cRole, "video_decoder.mpeg2",\
                OMX_MAX_STRINGNAME_SIZE);
        drv_ctx.decoder_format = VDEC_CODECTYPE_MPEG2;
        output_capability = V4L2_PIX_FMT_MPEG2;
        eCompressionFormat = OMX_VIDEO_CodingMPEG2;
    } else if (!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.avc",\
                OMX_MAX_STRINGNAME_SIZE)) {
        strlcpy((char *)m_cRole, "video_decoder.avc",OMX_MAX_STRINGNAME_SIZE);
        drv_ctx.decoder_format = VDEC_CODECTYPE_H264;
        output_capability=V4L2_PIX_FMT_H264;
        eCompressionFormat = OMX_VIDEO_CodingAVC;
    } else if (!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.hevc",\
                OMX_MAX_STRINGNAME_SIZE)) {
        strlcpy((char *)m_cRole, "video_decoder.hevc",OMX_MAX_STRINGNAME_SIZE);
        drv_ctx.decoder_format = VDEC_CODECTYPE_HEVC;
        output_capability = V4L2_PIX_FMT_HEVC;
        eCompressionFormat = (OMX_VIDEO_CODINGTYPE)QOMX_VIDEO_CodingHevc;
    } else if (!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.vp8",    \
                OMX_MAX_STRINGNAME_SIZE)) {
        char version[PROP_VALUE_MAX] = {0};
        if (!strncmp(m_platform_name, "lito", 4))
            if (property_get("vendor.media.target.version", version, "0") &&
                    ((atoi(version) == 2) || (atoi(version) == 3))) {
                //sku version, VP8 is disabled on lagoon
                DEBUG_PRINT_ERROR("VP8 unsupported on lagoon");
                eRet = OMX_ErrorInvalidComponentName;
                return eRet;
            }
        strlcpy((char *)m_cRole, "video_decoder.vp8",OMX_MAX_STRINGNAME_SIZE);
        drv_ctx.decoder_format = VDEC_CODECTYPE_VP8;
        output_capability = V4L2_PIX_FMT_VP8;
        eCompressionFormat = OMX_VIDEO_CodingVP8;
    } else if (!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.vp9",    \
                OMX_MAX_STRINGNAME_SIZE)) {
        strlcpy((char *)m_cRole, "video_decoder.vp9",OMX_MAX_STRINGNAME_SIZE);
        drv_ctx.decoder_format = VDEC_CODECTYPE_VP9;
        output_capability = V4L2_PIX_FMT_VP9;
        eCompressionFormat = OMX_VIDEO_CodingVP9;
    } else {
        DEBUG_PRINT_ERROR("ERROR:Unknown Component");
        eRet = OMX_ErrorInvalidComponentName;
    }

    m_progressive = MSM_VIDC_PIC_STRUCT_PROGRESSIVE;

    if (eRet == OMX_ErrorNone) {
        OMX_COLOR_FORMATTYPE dest_color_format;
        if (m_disable_ubwc_mode) {
            drv_ctx.output_format = VDEC_YUV_FORMAT_NV12;
        } else {
            drv_ctx.output_format = VDEC_YUV_FORMAT_NV12_UBWC;
        }
        if (eCompressionFormat == (OMX_VIDEO_CODINGTYPE)QOMX_VIDEO_CodingMVC)
            dest_color_format = (OMX_COLOR_FORMATTYPE)
                QOMX_COLOR_FORMATYUV420PackedSemiPlanar32mMultiView;
        else
            dest_color_format = (OMX_COLOR_FORMATTYPE)
                QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m;
        if (!client_buffers.set_color_format(dest_color_format)) {
            DEBUG_PRINT_ERROR("Setting color format failed");
            eRet = OMX_ErrorInsufficientResources;
        }

        dpb_bit_depth = MSM_VIDC_BIT_DEPTH_8;
        is_flexible_format = OMX_FALSE;
        is_mbaff = OMX_FALSE;

        if (m_disable_ubwc_mode) {
            capture_capability = V4L2_PIX_FMT_NV12;
        } else {
            capture_capability = V4L2_PIX_FMT_NV12_UBWC;
        }

        struct v4l2_capability cap;
        ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_QUERYCAP, &cap);
        if (ret) {
            DEBUG_PRINT_ERROR("Failed to query capabilities");
            /*TODO: How to handle this case */
        } else {
            DEBUG_PRINT_LOW("Capabilities: driver_name = %s, card = %s, bus_info = %s,"
                " version = %d, capabilities = %x", cap.driver, cap.card,
                cap.bus_info, cap.version, cap.capabilities);
        }
        ret=0;
        fdesc.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fdesc.index=0;
        while (ioctl(drv_ctx.video_driver_fd, VIDIOC_ENUM_FMT, &fdesc) == 0) {
            DEBUG_PRINT_HIGH("fmt: description: %s, fmt: %x, flags = %x", fdesc.description,
                    fdesc.pixelformat, fdesc.flags);
            fdesc.index++;
        }
        fdesc.type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        fdesc.index=0;
        while (ioctl(drv_ctx.video_driver_fd, VIDIOC_ENUM_FMT, &fdesc) == 0) {

            DEBUG_PRINT_HIGH("fmt: description: %s, fmt: %x, flags = %x", fdesc.description,
                    fdesc.pixelformat, fdesc.flags);
            fdesc.index++;
        }
        m_extradata_misr.output_crop_rect.nLeft = 0;
        m_extradata_misr.output_crop_rect.nTop = 0;
        m_extradata_misr.output_crop_rect.nWidth = 320;
        m_extradata_misr.output_crop_rect.nHeight = 240;
        update_resolution(320, 240, 320, 240);

        fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        fmt.fmt.pix_mp.height = drv_ctx.video_resolution.frame_height;
        fmt.fmt.pix_mp.width = drv_ctx.video_resolution.frame_width;
        fmt.fmt.pix_mp.pixelformat = output_capability;
        ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_S_FMT, &fmt);
        if (ret) {
            /*TODO: How to handle this case */
            DEBUG_PRINT_ERROR("Failed to set format on output port");
            return OMX_ErrorInsufficientResources;
        }
        DEBUG_PRINT_HIGH("Set Format was successful");

        /*
         * refer macro DEFAULT_CONCEAL_COLOR to set conceal color values
         */
        Platform::Config::getInt32(Platform::vidc_dec_conceal_color_8bit, &conceal_color_8bit, DEFAULT_VIDEO_CONCEAL_COLOR_BLACK);
        Platform::Config::getInt32(Platform::vidc_dec_conceal_color_10bit, &conceal_color_10bit, DEFAULT_VIDEO_CONCEAL_COLOR_BLACK);
        memset(ctrl, 0, sizeof(ctrl));
        ctrl[0].id = V4L2_CID_MPEG_VIDC_VIDEO_CONCEAL_COLOR_8BIT;
        ctrl[0].value = conceal_color_8bit;
        ctrl[1].id = V4L2_CID_MPEG_VIDC_VIDEO_CONCEAL_COLOR_10BIT;
        ctrl[1].value = conceal_color_10bit;

        for(int i=0; i<2; i++) {
            ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_S_CTRL, &ctrl[i]);
            if (ret) {
                DEBUG_PRINT_ERROR("Failed to set conceal color %s, error %d\n",
                                  i==0?"8Bit":"10Bit", ret);
            }
        }
        //Get the hardware capabilities
        memset((void *)&frmsize,0,sizeof(frmsize));
        frmsize.index = 0;
        frmsize.pixel_format = output_capability;
        ret = ioctl(drv_ctx.video_driver_fd,
                VIDIOC_ENUM_FRAMESIZES, &frmsize);
        if (ret || frmsize.type != V4L2_FRMSIZE_TYPE_STEPWISE) {
            DEBUG_PRINT_ERROR("Failed to get framesizes");
            return OMX_ErrorHardware;
        }

        if (frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
            m_decoder_capability.min_width = frmsize.stepwise.min_width;
            m_decoder_capability.max_width = frmsize.stepwise.max_width;
            m_decoder_capability.min_height = frmsize.stepwise.min_height;
            m_decoder_capability.max_height = frmsize.stepwise.max_height;
        }

        memset(&fmt, 0x0, sizeof(struct v4l2_format));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.fmt.pix_mp.height = drv_ctx.video_resolution.frame_height;
        fmt.fmt.pix_mp.width = drv_ctx.video_resolution.frame_width;
        fmt.fmt.pix_mp.pixelformat = capture_capability;
        ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_S_FMT, &fmt);
        if (ret) {
            /*TODO: How to handle this case */
            DEBUG_PRINT_ERROR("Failed to set format on capture port");
        }
        memset(&framesize, 0, sizeof(OMX_FRAMESIZETYPE));
        framesize.nWidth = drv_ctx.video_resolution.frame_width;
        framesize.nHeight = drv_ctx.video_resolution.frame_height;

        memset(&rectangle, 0, sizeof(OMX_CONFIG_RECTTYPE));
        rectangle.nWidth = drv_ctx.video_resolution.frame_width;
        rectangle.nHeight = drv_ctx.video_resolution.frame_height;

        DEBUG_PRINT_HIGH("Set Format was successful");
        if (secure_mode) {
            control.id = V4L2_CID_MPEG_VIDC_VIDEO_SECURE;
            control.value = 1;
            DEBUG_PRINT_LOW("Omx_vdec:: calling to open secure device %d", ret);
            ret=ioctl(drv_ctx.video_driver_fd, VIDIOC_S_CTRL,&control);
            if (ret) {
                DEBUG_PRINT_ERROR("Omx_vdec:: Unable to open secure device %d", ret);
                return OMX_ErrorInsufficientResources;
            }
        }

        /*Get the Buffer requirements for input and output ports*/
        drv_ctx.ip_buf.buffer_type = VDEC_BUFFER_TYPE_INPUT;
        drv_ctx.op_buf.buffer_type = VDEC_BUFFER_TYPE_OUTPUT;

        if (secure_mode) {
            drv_ctx.op_buf.alignment = SECURE_ALIGN;
            drv_ctx.ip_buf.alignment = SECURE_ALIGN;
        } else {
            drv_ctx.op_buf.alignment = SZ_4K;
            drv_ctx.ip_buf.alignment = SZ_4K;
        }

        drv_ctx.interlace = VDEC_InterlaceFrameProgressive;
        drv_ctx.picture_order = VDEC_ORDER_DISPLAY;

        control.id = V4L2_CID_MPEG_VIDC_VIDEO_DECODE_ORDER;
        control.value = V4L2_MPEG_MSM_VIDC_DISABLE;

        ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_S_CTRL, &control);
        drv_ctx.idr_only_decoding = 0;

        m_state = OMX_StateLoaded;

        eRet = get_buffer_req(&drv_ctx.ip_buf);
        DEBUG_PRINT_HIGH("Input Buffer Size =%u",(unsigned int)drv_ctx.ip_buf.buffer_size);
        get_buffer_req(&drv_ctx.op_buf);
        msg_thread_created = true;
        r = pthread_create(&msg_thread_id,0,message_thread_dec,this);

        if (r < 0) {
            DEBUG_PRINT_ERROR("component_init(): message_thread_dec creation failed");
            msg_thread_created = false;
            eRet = OMX_ErrorInsufficientResources;
        } else if (secure_mode) {
            this->post_event(PREFETCH_PIXEL_BUFFER_COUNT, m_dec_secure_prefetch_size_output, OMX_COMPONENT_GENERATE_ION_PREFETCH_PIXEL);
            this->post_event(PREFETCH_NON_PIXEL_BUFFER_COUNT, m_dec_secure_prefetch_size_internal, OMX_COMPONENT_GENERATE_ION_PREFETCH_NON_PIXEL);
        }
    }

    {
        VendorExtensionStore *extStore = const_cast<VendorExtensionStore *>(&mVendorExtensionStore);
        init_vendor_extensions(*extStore);
        mVendorExtensionStore.dumpExtensions((const char *)role);
    }

    if (eRet != OMX_ErrorNone) {
        DEBUG_PRINT_ERROR("Component Init Failed");
    } else {
        DEBUG_PRINT_INFO("omx_vdec::component_init() success : fd=%d",
                drv_ctx.video_driver_fd);
    }

    OMX_INIT_STRUCT(&m_sParamLowLatency, QOMX_EXTNINDEX_VIDEO_LOW_LATENCY_MODE);
    m_sParamLowLatency.nNumFrames = 0;
    m_sParamLowLatency.bEnableLowLatencyMode = OMX_FALSE;

    return eRet;
}

/* ======================================================================
   FUNCTION
   omx_vdec::GetComponentVersion

   DESCRIPTION
   Returns the component version.

   PARAMETERS
   TBD.

   RETURN VALUE
   OMX_ErrorNone.

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::get_component_version
(
 OMX_IN OMX_HANDLETYPE hComp,
 OMX_OUT OMX_STRING componentName,
 OMX_OUT OMX_VERSIONTYPE* componentVersion,
 OMX_OUT OMX_VERSIONTYPE* specVersion,
 OMX_OUT OMX_UUIDTYPE* componentUUID
 )
{
    (void) hComp;
    (void) componentName;
    (void) componentVersion;
    (void) componentUUID;
    if (m_state == OMX_StateInvalid) {
        DEBUG_PRINT_ERROR("Get Comp Version in Invalid State");
        return OMX_ErrorInvalidState;
    }
    /* TBD -- Return the proper version */
    if (specVersion) {
        specVersion->nVersion = OMX_SPEC_VERSION;
    }
    return OMX_ErrorNone;
}
/* ======================================================================
   FUNCTION
   omx_vdec::SendCommand

   DESCRIPTION
   Returns zero if all the buffers released..

   PARAMETERS
   None.

   RETURN VALUE
   true/false

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::send_command(OMX_IN OMX_HANDLETYPE hComp,
        OMX_IN OMX_COMMANDTYPE cmd,
        OMX_IN OMX_U32 param1,
        OMX_IN OMX_PTR cmdData
        )
{
    (void) hComp;
    (void) cmdData;
    DEBUG_PRINT_LOW("send_command: Recieved a Command from Client");
    if (m_state == OMX_StateInvalid) {
        DEBUG_PRINT_ERROR("ERROR: Send Command in Invalid State");
        return OMX_ErrorInvalidState;
    }
    if (cmd == OMX_CommandFlush && param1 != OMX_CORE_INPUT_PORT_INDEX
            && param1 != OMX_CORE_OUTPUT_PORT_INDEX && param1 != OMX_ALL) {
        DEBUG_PRINT_ERROR("send_command(): ERROR OMX_CommandFlush "
                "to invalid port: %u", (unsigned int)param1);
        return OMX_ErrorBadPortIndex;
    }

    post_event((unsigned)cmd,(unsigned)param1,OMX_COMPONENT_GENERATE_COMMAND);
    sem_wait(&m_cmd_lock);
    DEBUG_PRINT_LOW("send_command: Command Processed");
    return OMX_ErrorNone;
}

/* ======================================================================
   FUNCTION
   omx_vdec::SendCommand

   DESCRIPTION
   Returns zero if all the buffers released..

   PARAMETERS
   None.

   RETURN VALUE
   true/false

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::send_command_proxy(OMX_IN OMX_HANDLETYPE hComp,
        OMX_IN OMX_COMMANDTYPE cmd,
        OMX_IN OMX_U32 param1,
        OMX_IN OMX_PTR cmdData
        )
{
    (void) hComp;
    (void) cmdData;
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    OMX_STATETYPE eState = (OMX_STATETYPE) param1;
    int bFlag = 1,sem_posted = 0,ret=0;

    DEBUG_PRINT_LOW("send_command_proxy(): cmd = %d", cmd);
    DEBUG_PRINT_HIGH("send_command_proxy(): Current State %d, Expected State %d",
            m_state, eState);

    if (cmd == OMX_CommandStateSet) {
        DEBUG_PRINT_HIGH("send_command_proxy(): OMX_CommandStateSet issued");
        DEBUG_PRINT_HIGH("Current State %d, Expected State %d", m_state, eState);
        /***************************/
        /* Current State is Loaded */
        /***************************/
        if (m_state == OMX_StateLoaded) {
            if (eState == OMX_StateIdle) {
                //if all buffers are allocated or all ports disabled
                if (allocate_done() ||
                        (m_inp_bEnabled == OMX_FALSE && m_out_bEnabled == OMX_FALSE)) {
                    DEBUG_PRINT_LOW("send_command_proxy(): Loaded-->Idle");
                } else {
                    DEBUG_PRINT_LOW("send_command_proxy(): Loaded-->Idle-Pending");
                    BITMASK_SET(&m_flags, OMX_COMPONENT_IDLE_PENDING);
                    // Skip the event notification
                    bFlag = 0;
                }
            }
            /* Requesting transition from Loaded to Loaded */
            else if (eState == OMX_StateLoaded) {
                DEBUG_PRINT_ERROR("ERROR::send_command_proxy(): Loaded-->Loaded");
                post_event(OMX_EventError,OMX_ErrorSameState,\
                        OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorSameState;
            }
            /* Requesting transition from Loaded to WaitForResources */
            else if (eState == OMX_StateWaitForResources) {
                /* Since error is None , we will post an event
                   at the end of this function definition */
                DEBUG_PRINT_LOW("send_command_proxy(): Loaded-->WaitForResources");
            }
            /* Requesting transition from Loaded to Executing */
            else if (eState == OMX_StateExecuting) {
                DEBUG_PRINT_ERROR("ERROR::send_command_proxy(): Loaded-->Executing");
                post_event(OMX_EventError,OMX_ErrorIncorrectStateTransition,\
                        OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorIncorrectStateTransition;
            }
            /* Requesting transition from Loaded to Pause */
            else if (eState == OMX_StatePause) {
                DEBUG_PRINT_ERROR("ERROR::send_command_proxy(): Loaded-->Pause");
                post_event(OMX_EventError,OMX_ErrorIncorrectStateTransition,\
                        OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorIncorrectStateTransition;
            }
            /* Requesting transition from Loaded to Invalid */
            else if (eState == OMX_StateInvalid) {
                DEBUG_PRINT_ERROR("ERROR::send_command_proxy(): Loaded-->Invalid");
                post_event(OMX_EventError,eState,OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorInvalidState;
            } else {
                DEBUG_PRINT_ERROR("ERROR::send_command_proxy(): Loaded-->Invalid(%d Not Handled)",\
                        eState);
                eRet = OMX_ErrorBadParameter;
            }
        }

        /***************************/
        /* Current State is IDLE */
        /***************************/
        else if (m_state == OMX_StateIdle) {
            if (eState == OMX_StateLoaded) {
                if (release_done()) {
                    /*
                     * Since error is None , we will post an event at the end
                     * of this function definition
                     * Reset buffer requirements here to ensure setting buffer requirement
                     * when component move to executing state from loaded state via Idle.
                     */
                    drv_ctx.op_buf.buffer_size = 0;
                    drv_ctx.op_buf.actualcount = 0;
                    DEBUG_PRINT_LOW("send_command_proxy(): Idle-->Loaded");
                } else {
                    DEBUG_PRINT_LOW("send_command_proxy(): Idle-->Loaded-Pending");
                    BITMASK_SET(&m_flags, OMX_COMPONENT_LOADING_PENDING);
                    // Skip the event notification
                    bFlag = 0;
                }
            }
            /* Requesting transition from Idle to Executing */
            else if (eState == OMX_StateExecuting) {
                bFlag = 1;
                DEBUG_PRINT_LOW("send_command_proxy(): Idle-->Executing");
                m_state=OMX_StateExecuting;
            }
            /* Requesting transition from Idle to Idle */
            else if (eState == OMX_StateIdle) {
                DEBUG_PRINT_ERROR("ERROR::send_command_proxy(): Idle-->Idle");
                post_event(OMX_EventError,OMX_ErrorSameState,\
                        OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorSameState;
            }
            /* Requesting transition from Idle to WaitForResources */
            else if (eState == OMX_StateWaitForResources) {
                DEBUG_PRINT_ERROR("ERROR::send_command_proxy(): Idle-->WaitForResources");
                post_event(OMX_EventError,OMX_ErrorIncorrectStateTransition,\
                        OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorIncorrectStateTransition;
            }
            /* Requesting transition from Idle to Pause */
            else if (eState == OMX_StatePause) {
                /*To pause the Video core we need to start the driver*/
                if (/*ioctl (drv_ctx.video_driver_fd,VDEC_IOCTL_CMD_START,
                      NULL) < */0) {
                    DEBUG_PRINT_ERROR("VDEC_IOCTL_CMD_START FAILED");
                    omx_report_error ();
                    eRet = OMX_ErrorHardware;
                } else {
                    BITMASK_SET(&m_flags,OMX_COMPONENT_PAUSE_PENDING);
                    DEBUG_PRINT_LOW("send_command_proxy(): Idle-->Pause");
                    bFlag = 0;
                }
            }
            /* Requesting transition from Idle to Invalid */
            else if (eState == OMX_StateInvalid) {
                DEBUG_PRINT_ERROR("ERROR::send_command_proxy(): Idle-->Invalid");
                post_event(OMX_EventError,eState,OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorInvalidState;
            } else {
                DEBUG_PRINT_ERROR("ERROR::send_command_proxy(): Idle --> %d Not Handled",eState);
                eRet = OMX_ErrorBadParameter;
            }
        }

        /******************************/
        /* Current State is Executing */
        /******************************/
        else if (m_state == OMX_StateExecuting) {
            DEBUG_PRINT_LOW("Command Recieved in OMX_StateExecuting");
            /* Requesting transition from Executing to Idle */
            if (eState == OMX_StateIdle) {
                /* Since error is None , we will post an event
                   at the end of this function definition
                 */
                DEBUG_PRINT_LOW("send_command_proxy(): Executing --> Idle");
                BITMASK_SET(&m_flags,OMX_COMPONENT_IDLE_PENDING);
                if (!sem_posted) {
                    sem_posted = 1;
                    sem_post (&m_cmd_lock);
                    execute_omx_flush(OMX_ALL);
                }
                bFlag = 0;
            }
            /* Requesting transition from Executing to Paused */
            else if (eState == OMX_StatePause) {
                DEBUG_PRINT_LOW("PAUSE Command Issued");
                m_state = OMX_StatePause;
                bFlag = 1;
            }
            /* Requesting transition from Executing to Loaded */
            else if (eState == OMX_StateLoaded) {
                DEBUG_PRINT_ERROR("send_command_proxy(): Executing --> Loaded");
                post_event(OMX_EventError,OMX_ErrorIncorrectStateTransition,\
                        OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorIncorrectStateTransition;
            }
            /* Requesting transition from Executing to WaitForResources */
            else if (eState == OMX_StateWaitForResources) {
                DEBUG_PRINT_ERROR("send_command_proxy(): Executing --> WaitForResources");
                post_event(OMX_EventError,OMX_ErrorIncorrectStateTransition,\
                        OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorIncorrectStateTransition;
            }
            /* Requesting transition from Executing to Executing */
            else if (eState == OMX_StateExecuting) {
                DEBUG_PRINT_ERROR("send_command_proxy(): Executing --> Executing");
                post_event(OMX_EventError,OMX_ErrorSameState,\
                        OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorSameState;
            }
            /* Requesting transition from Executing to Invalid */
            else if (eState == OMX_StateInvalid) {
                DEBUG_PRINT_ERROR("send_command_proxy(): Executing --> Invalid");
                post_event(OMX_EventError,eState,OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorInvalidState;
            } else {
                DEBUG_PRINT_ERROR("ERROR::send_command_proxy(): Executing --> %d Not Handled",eState);
                eRet = OMX_ErrorBadParameter;
            }
        }
        /***************************/
        /* Current State is Pause  */
        /***************************/
        else if (m_state == OMX_StatePause) {
            /* Requesting transition from Pause to Executing */
            if (eState == OMX_StateExecuting) {
                DEBUG_PRINT_LOW("Pause --> Executing");
                m_state = OMX_StateExecuting;
                bFlag = 1;
            }
            /* Requesting transition from Pause to Idle */
            else if (eState == OMX_StateIdle) {
                /* Since error is None , we will post an event
                   at the end of this function definition */
                DEBUG_PRINT_LOW("Pause --> Idle");
                BITMASK_SET(&m_flags,OMX_COMPONENT_IDLE_PENDING);
                if (!sem_posted) {
                    sem_posted = 1;
                    sem_post (&m_cmd_lock);
                    execute_omx_flush(OMX_ALL);
                }
                bFlag = 0;
            }
            /* Requesting transition from Pause to loaded */
            else if (eState == OMX_StateLoaded) {
                DEBUG_PRINT_ERROR("Pause --> loaded");
                post_event(OMX_EventError,OMX_ErrorIncorrectStateTransition,\
                        OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorIncorrectStateTransition;
            }
            /* Requesting transition from Pause to WaitForResources */
            else if (eState == OMX_StateWaitForResources) {
                DEBUG_PRINT_ERROR("Pause --> WaitForResources");
                post_event(OMX_EventError,OMX_ErrorIncorrectStateTransition,\
                        OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorIncorrectStateTransition;
            }
            /* Requesting transition from Pause to Pause */
            else if (eState == OMX_StatePause) {
                DEBUG_PRINT_ERROR("Pause --> Pause");
                post_event(OMX_EventError,OMX_ErrorSameState,\
                        OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorSameState;
            }
            /* Requesting transition from Pause to Invalid */
            else if (eState == OMX_StateInvalid) {
                DEBUG_PRINT_ERROR("Pause --> Invalid");
                post_event(OMX_EventError,eState,OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorInvalidState;
            } else {
                DEBUG_PRINT_ERROR("ERROR::send_command_proxy(): Paused --> %d Not Handled",eState);
                eRet = OMX_ErrorBadParameter;
            }
        }
        /***************************/
        /* Current State is WaitForResources  */
        /***************************/
        else if (m_state == OMX_StateWaitForResources) {
            /* Requesting transition from WaitForResources to Loaded */
            if (eState == OMX_StateLoaded) {
                /* Since error is None , we will post an event
                   at the end of this function definition */
                DEBUG_PRINT_LOW("send_command_proxy(): WaitForResources-->Loaded");
            }
            /* Requesting transition from WaitForResources to WaitForResources */
            else if (eState == OMX_StateWaitForResources) {
                DEBUG_PRINT_ERROR("ERROR::send_command_proxy(): WaitForResources-->WaitForResources");
                post_event(OMX_EventError,OMX_ErrorSameState,
                        OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorSameState;
            }
            /* Requesting transition from WaitForResources to Executing */
            else if (eState == OMX_StateExecuting) {
                DEBUG_PRINT_ERROR("ERROR::send_command_proxy(): WaitForResources-->Executing");
                post_event(OMX_EventError,OMX_ErrorIncorrectStateTransition,\
                        OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorIncorrectStateTransition;
            }
            /* Requesting transition from WaitForResources to Pause */
            else if (eState == OMX_StatePause) {
                DEBUG_PRINT_ERROR("ERROR::send_command_proxy(): WaitForResources-->Pause");
                post_event(OMX_EventError,OMX_ErrorIncorrectStateTransition,\
                        OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorIncorrectStateTransition;
            }
            /* Requesting transition from WaitForResources to Invalid */
            else if (eState == OMX_StateInvalid) {
                DEBUG_PRINT_ERROR("ERROR::send_command_proxy(): WaitForResources-->Invalid");
                post_event(OMX_EventError,eState,OMX_COMPONENT_GENERATE_EVENT);
                eRet = OMX_ErrorInvalidState;
            }
            /* Requesting transition from WaitForResources to Loaded -
               is NOT tested by Khronos TS */

        } else {
            DEBUG_PRINT_ERROR("ERROR::send_command_proxy(): %d --> %d(Not Handled)",m_state,eState);
            eRet = OMX_ErrorBadParameter;
        }
    }
    /********************************/
    /* Current State is Invalid */
    /*******************************/
    else if (m_state == OMX_StateInvalid) {
        /* State Transition from Inavlid to any state */
        if ((eState == OMX_StateLoaded) || (eState == OMX_StateWaitForResources) ||
                (eState == OMX_StateIdle) || (eState == OMX_StateExecuting) ||
                (eState == OMX_StatePause) || (eState == OMX_StateInvalid)) {
            DEBUG_PRINT_ERROR("ERROR::send_command_proxy(): Invalid -->Loaded");
            post_event(OMX_EventError,OMX_ErrorInvalidState,\
                    OMX_COMPONENT_GENERATE_EVENT);
            eRet = OMX_ErrorInvalidState;
        }
    } else if (cmd == OMX_CommandFlush) {
        DEBUG_PRINT_HIGH("send_command_proxy(): OMX_CommandFlush issued"
                "with param1: %u", (unsigned int)param1);
        send_codec_config();
        if (cmd == OMX_CommandFlush && (param1 == OMX_CORE_INPUT_PORT_INDEX ||
                    param1 == OMX_ALL)) {
            if (android_atomic_add(0, &m_queued_codec_config_count) > 0) {
                struct timespec ts;
                int rc = 0;
#ifdef __BIONIC__
                clock_gettime(CLOCK_MONOTONIC, &ts);
#elif __GLIBC__
                clock_gettime(CLOCK_REALTIME, &ts);
#endif
                ts.tv_sec += 1;
                DEBUG_PRINT_LOW("waiting for %d EBDs of CODEC CONFIG buffers ",
                       m_queued_codec_config_count);
                BITMASK_SET(&m_flags, OMX_COMPONENT_FLUSH_DEFERRED);
#ifdef __BIONIC__
                rc = sem_timedwait_monotonic_np(&m_safe_flush, &ts);
#elif __GLIBC__
                rc = sem_timedwait(&m_safe_flush, &ts);
#endif
                if (rc) {
                    DEBUG_PRINT_ERROR("Failed to wait for EBDs of CODEC CONFIG buffers");
                }
               BITMASK_CLEAR (&m_flags,OMX_COMPONENT_FLUSH_DEFERRED);
            }
        }

        if (OMX_CORE_INPUT_PORT_INDEX == param1 || OMX_ALL == param1) {
            BITMASK_SET(&m_flags, OMX_COMPONENT_INPUT_FLUSH_PENDING);
        }
        if (OMX_CORE_OUTPUT_PORT_INDEX == param1 || OMX_ALL == param1) {
            BITMASK_SET(&m_flags, OMX_COMPONENT_OUTPUT_FLUSH_PENDING);
        }
        if (!sem_posted) {
            sem_posted = 1;
            DEBUG_PRINT_LOW("Set the Semaphore");
            sem_post (&m_cmd_lock);
            execute_omx_flush(param1);
        }
        bFlag = 0;
    } else if ( cmd == OMX_CommandPortEnable) {
        DEBUG_PRINT_HIGH("send_command_proxy(): OMX_CommandPortEnable issued"
                "with param1: %u", (unsigned int)param1);
        if (param1 == OMX_CORE_INPUT_PORT_INDEX || param1 == OMX_ALL) {
            m_inp_bEnabled = OMX_TRUE;

            if ( (m_state == OMX_StateLoaded &&
                        !BITMASK_PRESENT(&m_flags,OMX_COMPONENT_IDLE_PENDING))
                    || allocate_input_done()) {
                post_event(OMX_CommandPortEnable,OMX_CORE_INPUT_PORT_INDEX,
                        OMX_COMPONENT_GENERATE_EVENT);
            } else {
                DEBUG_PRINT_LOW("send_command_proxy(): Disabled-->Enabled Pending");
                BITMASK_SET(&m_flags, OMX_COMPONENT_INPUT_ENABLE_PENDING);
                // Skip the event notification
                bFlag = 0;
            }
        }
        if (param1 == OMX_CORE_OUTPUT_PORT_INDEX || param1 == OMX_ALL) {
            DEBUG_PRINT_LOW("Enable output Port command recieved");
            m_out_bEnabled = OMX_TRUE;

            if ( (m_state == OMX_StateLoaded &&
                        !BITMASK_PRESENT(&m_flags,OMX_COMPONENT_IDLE_PENDING))
                    || (allocate_output_done())) {
                post_event(OMX_CommandPortEnable,OMX_CORE_OUTPUT_PORT_INDEX,
                        OMX_COMPONENT_GENERATE_EVENT);

            } else {
                DEBUG_PRINT_LOW("send_command_proxy(): Disabled-->Enabled Pending");
                BITMASK_SET(&m_flags, OMX_COMPONENT_OUTPUT_ENABLE_PENDING);
                // Skip the event notification
                bFlag = 0;
            }
        }
    } else if (cmd == OMX_CommandPortDisable) {
        DEBUG_PRINT_HIGH("send_command_proxy(): OMX_CommandPortDisable issued"
                "with param1: %u", (unsigned int)param1);
        if (param1 == OMX_CORE_INPUT_PORT_INDEX || param1 == OMX_ALL) {
            codec_config_flag = false;
            m_inp_bEnabled = OMX_FALSE;
            if ((m_state == OMX_StateLoaded || m_state == OMX_StateIdle)
                    && release_input_done()) {
                post_event(OMX_CommandPortDisable,OMX_CORE_INPUT_PORT_INDEX,
                        OMX_COMPONENT_GENERATE_EVENT);
            } else {
                DEBUG_PRINT_HIGH("Set input port disable pending");
                BITMASK_SET(&m_flags, OMX_COMPONENT_INPUT_DISABLE_PENDING);
                if (m_state == OMX_StatePause ||m_state == OMX_StateExecuting) {
                    if (!sem_posted) {
                        sem_posted = 1;
                        sem_post (&m_cmd_lock);
                    }
                    execute_omx_flush(OMX_CORE_INPUT_PORT_INDEX);
                }

                // Skip the event notification
                bFlag = 0;
            }
        }
        if (param1 == OMX_CORE_OUTPUT_PORT_INDEX || param1 == OMX_ALL) {
            m_out_bEnabled = OMX_FALSE;
            DEBUG_PRINT_LOW("Disable output Port command recieved");
            if ((m_state == OMX_StateLoaded || m_state == OMX_StateIdle)
                    && release_output_done()) {
                post_event(OMX_CommandPortDisable,OMX_CORE_OUTPUT_PORT_INDEX,\
                        OMX_COMPONENT_GENERATE_EVENT);
            } else {
                DEBUG_PRINT_HIGH("Set output port disable pending");
                BITMASK_SET(&m_flags, OMX_COMPONENT_OUTPUT_DISABLE_PENDING);
                if (m_state == OMX_StatePause ||m_state == OMX_StateExecuting) {
                    if (!sem_posted) {
                        sem_posted = 1;
                        sem_post (&m_cmd_lock);
                    }
                    DEBUG_PRINT_HIGH("Set output port flush in disable pending");
                    BITMASK_SET(&m_flags, OMX_COMPONENT_OUTPUT_FLUSH_IN_DISABLE_PENDING);
                    execute_omx_flush(OMX_CORE_OUTPUT_PORT_INDEX);
                }
                // Skip the event notification
                bFlag = 0;

            }
        }
    } else {
        DEBUG_PRINT_ERROR("Error: Invalid Command other than StateSet (%d)",cmd);
        eRet = OMX_ErrorNotImplemented;
    }
    if (eRet == OMX_ErrorNone && bFlag) {
        post_event(cmd,eState,OMX_COMPONENT_GENERATE_EVENT);
    }
    if (!sem_posted) {
        sem_post(&m_cmd_lock);
    }

    return eRet;
}

/* ======================================================================
   FUNCTION
   omx_vdec::ExecuteOmxFlush

   DESCRIPTION
   Executes the OMX flush.

   PARAMETERS
   flushtype - input flush(1)/output flush(0)/ both.

   RETURN VALUE
   true/false

   ========================================================================== */
bool omx_vdec::execute_omx_flush(OMX_U32 flushType)
{
    bool bRet = false;
    struct v4l2_plane plane;
    struct v4l2_buffer v4l2_buf;
    struct v4l2_decoder_cmd dec;
    DEBUG_PRINT_LOW("in %s, flushing %u", __func__, (unsigned int)flushType);
    memset((void *)&v4l2_buf,0,sizeof(v4l2_buf));
    dec.cmd = V4L2_CMD_FLUSH;

    DEBUG_PRINT_HIGH("in %s: reconfig? %d", __func__, in_reconfig);

    if (in_reconfig && flushType == OMX_CORE_OUTPUT_PORT_INDEX) {
        output_flush_progress = true;
        dec.flags = V4L2_CMD_FLUSH_CAPTURE;
    } else {
        /* XXX: The driver/hardware does not support flushing of individual ports
         * in all states. So we pretty much need to flush both ports internally,
         * but client should only get the FLUSH_(INPUT|OUTPUT)_DONE for the one it
         * requested.  Since OMX_COMPONENT_(OUTPUT|INPUT)_FLUSH_PENDING isn't set,
         * we automatically omit sending the FLUSH done for the "opposite" port. */
        input_flush_progress = true;
        output_flush_progress = true;
        dec.flags = V4L2_CMD_FLUSH_OUTPUT | V4L2_CMD_FLUSH_CAPTURE;
    }

    if (ioctl(drv_ctx.video_driver_fd, VIDIOC_DECODER_CMD, &dec)) {
        DEBUG_PRINT_ERROR("Flush Port (%u) Failed ", (unsigned int)flushType);
        bRet = false;
    }

    return bRet;
}
/*=========================================================================
FUNCTION : execute_output_flush

DESCRIPTION
Executes the OMX flush at OUTPUT PORT.

PARAMETERS
None.

RETURN VALUE
true/false
==========================================================================*/
bool omx_vdec::execute_output_flush()
{
    unsigned long p1 = 0; // Parameter - 1
    unsigned long p2 = 0; // Parameter - 2
    unsigned long ident = 0;
    bool bRet = true;

    /*Generate FBD for all Buffers in the FTBq*/
    pthread_mutex_lock(&m_lock);
    DEBUG_PRINT_LOW("Initiate Output Flush");

    m_prev_timestampUs = 0;

    while (m_ftb_q.m_size) {
        m_ftb_q.pop_entry(&p1,&p2,&ident);
        if (ident == m_fill_output_msg ) {
            print_omx_buffer("Flush FBD", (OMX_BUFFERHEADERTYPE *)p2);
            m_cb.FillBufferDone(&m_cmp, m_app_data, (OMX_BUFFERHEADERTYPE *)(intptr_t)p2);
        } else if (ident == OMX_COMPONENT_GENERATE_FBD) {
            fill_buffer_done(&m_cmp,(OMX_BUFFERHEADERTYPE *)(intptr_t)p1);
        }
    }
    pthread_mutex_unlock(&m_lock);
    output_flush_progress = false;

    DEBUG_PRINT_HIGH("OMX flush o/p Port complete PenBuf(%d)", pending_output_buffers);
    return bRet;
}
/*=========================================================================
FUNCTION : execute_input_flush

DESCRIPTION
Executes the OMX flush at INPUT PORT.

PARAMETERS
None.

RETURN VALUE
true/false
==========================================================================*/
bool omx_vdec::execute_input_flush()
{
    unsigned       i =0;
    unsigned long p1 = 0; // Parameter - 1
    unsigned long p2 = 0; // Parameter - 2
    unsigned long ident = 0;
    bool bRet = true;

    /*Generate EBD for all Buffers in the ETBq*/
    DEBUG_PRINT_LOW("Initiate Input Flush");

    pthread_mutex_lock(&m_lock);
    DEBUG_PRINT_LOW("Check if the Queue is empty");
    while (m_etb_q.m_size) {
        m_etb_q.pop_entry(&p1,&p2,&ident);

        if (ident == OMX_COMPONENT_GENERATE_ETB) {
            pending_input_buffers++;
            VIDC_TRACE_INT_LOW("ETB-pending", pending_input_buffers);
            print_omx_buffer("Flush ETB", (OMX_BUFFERHEADERTYPE *)p2);
            empty_buffer_done(&m_cmp,(OMX_BUFFERHEADERTYPE *)p2);
        } else if (ident == OMX_COMPONENT_GENERATE_EBD) {
            print_omx_buffer("Flush EBD", (OMX_BUFFERHEADERTYPE *)p1);
            empty_buffer_done(&m_cmp,(OMX_BUFFERHEADERTYPE *)p1);
        }
    }
    time_stamp_dts.flush_timestamp();
    /*Check if Heap Buffers are to be flushed*/
    pthread_mutex_unlock(&m_lock);
    input_flush_progress = false;
    prev_ts = LLONG_MAX;

    DEBUG_PRINT_HIGH("OMX flush i/p Port complete PenBuf(%d)", pending_input_buffers);
    return bRet;
}

/*=========================================================================
FUNCTION : notify_flush_done

DESCRIPTION
Notifies flush done to the OMX Client.

PARAMETERS
ctxt -- Context information related to the self..

RETURN VALUE
NONE
==========================================================================*/
void omx_vdec::notify_flush_done(void *ctxt)
{

    omx_vdec *pThis = (omx_vdec *) ctxt;

    if (!pThis->input_flush_progress && !pThis->output_flush_progress) {
        if (BITMASK_PRESENT(&pThis->m_flags,
                OMX_COMPONENT_OUTPUT_FLUSH_PENDING)) {
            DEBUG_PRINT_LOW("Notify Output Flush done");
            BITMASK_CLEAR (&pThis->m_flags,OMX_COMPONENT_OUTPUT_FLUSH_PENDING);
            pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data,
                OMX_EventCmdComplete,OMX_CommandFlush,
                OMX_CORE_OUTPUT_PORT_INDEX,NULL );
        }

        if (BITMASK_PRESENT(&pThis->m_flags,
                OMX_COMPONENT_INPUT_FLUSH_PENDING)) {
            BITMASK_CLEAR (&pThis->m_flags,OMX_COMPONENT_INPUT_FLUSH_PENDING);
            DEBUG_PRINT_LOW("Input Flush completed - Notify Client");
            pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data,
                    OMX_EventCmdComplete,OMX_CommandFlush,
                    OMX_CORE_INPUT_PORT_INDEX,NULL );
            //clear hdr10plusinfo list upon input flush done
            clear_hdr10plusinfo();
        }
    }
}

/* ======================================================================
   FUNCTION
   omx_vdec::SendCommandEvent

   DESCRIPTION
   Send the event to decoder pipe.  This is needed to generate the callbacks
   in decoder thread context.

   PARAMETERS
   None.

   RETURN VALUE
   true/false

   ========================================================================== */
bool omx_vdec::post_event(unsigned long p1,
        unsigned long p2,
        unsigned long id)
{
    bool bRet = false;

    /* Just drop messages typically generated by hardware (w/o client request),
     * if we've reported an error to client. */
    if (m_error_propogated) {
        switch (id) {
            case OMX_COMPONENT_GENERATE_PORT_RECONFIG:
            case OMX_COMPONENT_GENERATE_HARDWARE_ERROR:
                DEBUG_PRINT_ERROR("Dropping message %lx "
                        "since client expected to be in error state", id);
                return false;
            default:
                /* whatever */
                break;
        }
    }

    pthread_mutex_lock(&m_lock);

    if (id == m_fill_output_msg ||
            id == OMX_COMPONENT_GENERATE_FBD ||
            id == OMX_COMPONENT_GENERATE_PORT_RECONFIG ||
            id == OMX_COMPONENT_GENERATE_EVENT_OUTPUT_FLUSH) {
        m_ftb_q.insert_entry(p1,p2,id);
    } else if (id == OMX_COMPONENT_GENERATE_ETB ||
            id == OMX_COMPONENT_GENERATE_EBD ||
            id == OMX_COMPONENT_GENERATE_EVENT_INPUT_FLUSH) {
        m_etb_q.insert_entry(p1,p2,id);
    } else {
        DEBUG_PRINT_HIGH("post_event(%ld, %ld, %ld)", p1, p2, id);
        m_cmd_q.insert_entry(p1,p2,id);
    }

    bRet = true;
    post_message(this, id);

    pthread_mutex_unlock(&m_lock);

    return bRet;
}

bool inline omx_vdec::vdec_query_cap(struct v4l2_queryctrl &cap)
{

    if (ioctl(drv_ctx.video_driver_fd, VIDIOC_QUERYCTRL, &cap)) {
        DEBUG_PRINT_ERROR("Query caps for id = %u failed\n", cap.id);
        return false;
    }
    return true;
}

OMX_ERRORTYPE omx_vdec::get_supported_profile_level(OMX_VIDEO_PARAM_PROFILELEVELTYPE *profileLevelType)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    bool hdr_supported = true;
    struct v4l2_queryctrl profile_cap, level_cap, tier_cap;
    int v4l2_profile;
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
    int mpeg2_profiles[2] = { OMX_VIDEO_MPEG2ProfileSimple,
                              OMX_VIDEO_MPEG2ProfileMain};
    int vp9_profiles[4] = { OMX_VIDEO_VP9Profile0,
                            OMX_VIDEO_VP9Profile2,
                            OMX_VIDEO_VP9Profile2HDR,
                            OMX_VIDEO_VP9Profile2HDR10Plus};

    if (!profileLevelType)
        return OMX_ErrorBadParameter;

    memset(&tier_cap, 0, sizeof(struct v4l2_queryctrl));
    memset(&level_cap, 0, sizeof(struct v4l2_queryctrl));
    memset(&profile_cap, 0, sizeof(struct v4l2_queryctrl));

    if (output_capability == V4L2_PIX_FMT_H264) {
        level_cap.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL;
        profile_cap.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE;
    } else if (output_capability == V4L2_PIX_FMT_VP8) {
        level_cap.id = V4L2_CID_MPEG_VIDC_VIDEO_VP8_PROFILE_LEVEL;
    } else if (output_capability == V4L2_PIX_FMT_VP9) {
        level_cap.id = V4L2_CID_MPEG_VIDC_VIDEO_VP9_LEVEL;
        profile_cap.id = V4L2_CID_MPEG_VIDEO_VP9_PROFILE;
    } else if (output_capability == V4L2_PIX_FMT_HEVC) {
        tier_cap.id = V4L2_CID_MPEG_VIDEO_HEVC_TIER;
        level_cap.id = V4L2_CID_MPEG_VIDEO_HEVC_LEVEL;
        profile_cap.id = V4L2_CID_MPEG_VIDEO_HEVC_PROFILE;
    } else if (output_capability == V4L2_PIX_FMT_MPEG2) {
        level_cap.id = V4L2_CID_MPEG_VIDC_VIDEO_MPEG2_LEVEL;
        profile_cap.id = V4L2_CID_MPEG_VIDC_VIDEO_MPEG2_PROFILE;
    } else {
        DEBUG_PRINT_ERROR("get_parameter: OMX_IndexParamVideoProfileLevelQuerySupported Invalid codec");
        return OMX_ErrorInvalidComponent;
    }

    if (profile_cap.id) {
        if(!vdec_query_cap(profile_cap)) {
            DEBUG_PRINT_ERROR("Getting capabilities for profile failed");
            return OMX_ErrorHardware;
        }
    }

    if (level_cap.id) {
        if(!vdec_query_cap(level_cap)) {
            DEBUG_PRINT_ERROR("Getting capabilities for level failed");
            return OMX_ErrorHardware;
        }
    }

    if (tier_cap.id) {
        if(!vdec_query_cap(tier_cap)) {
            DEBUG_PRINT_ERROR("Getting capabilities for tier failed");
            return OMX_ErrorHardware;
        }
    }

    /* Get the corresponding omx level from v4l2 level */
    if (!profile_level_converter::convert_v4l2_level_to_omx(output_capability, level_cap.maximum, (int *)&profileLevelType->eLevel)) {
        DEBUG_PRINT_ERROR("Invalid level, cannot find corresponding v4l2 level : %d ", level_cap.maximum);
        return OMX_ErrorHardware;
    }
    if (output_capability == V4L2_PIX_FMT_HEVC && tier_cap.maximum == V4L2_MPEG_VIDEO_HEVC_TIER_HIGH) {
        /* handle HEVC high tier */
        profileLevelType->eLevel <<= 1;
    }

    /* For given profile index get corresponding profile that needs to be supported */
    if (profileLevelType->nPortIndex != OMX_CORE_INPUT_PORT_INDEX) {
        DEBUG_PRINT_ERROR("get_parameter: OMX_IndexParamVideoProfileLevelQuerySupported should be queried on Input port only %u", (unsigned int)profileLevelType->nPortIndex);
        return OMX_ErrorBadPortIndex;
    }

    if (output_capability == V4L2_PIX_FMT_H264) {
        if (profileLevelType->nProfileIndex < (sizeof(avc_profiles)/sizeof(int))) {
            profileLevelType->eProfile = avc_profiles[profileLevelType->nProfileIndex];
        } else {
            DEBUG_PRINT_LOW("AVC: get_parameter: OMX_IndexParamVideoProfileLevelQuerySupported nProfileIndex ret NoMore %u",
                                          (unsigned int)profileLevelType->nProfileIndex);
            return OMX_ErrorNoMore;
        }
    } else if (output_capability == V4L2_PIX_FMT_VP8) {
        if (profileLevelType->nProfileIndex == 0) {
            profileLevelType->eProfile = OMX_VIDEO_VP8ProfileMain;
        } else {
            DEBUG_PRINT_LOW("VP8: get_parameter: OMX_IndexParamVideoProfileLevelQuerySupported nProfileIndex ret NoMore %u",
                                          (unsigned int)profileLevelType->nProfileIndex);
            return OMX_ErrorNoMore;
        }
        /* Driver has no notion of VP8 profile. Only one profile is supported.  Return this */
        return OMX_ErrorNone;
    } else if (output_capability == V4L2_PIX_FMT_VP9) {
        if (profileLevelType->nProfileIndex < (sizeof(vp9_profiles)/sizeof(int))) {
            profileLevelType->eProfile = vp9_profiles[profileLevelType->nProfileIndex];
        } else {
            DEBUG_PRINT_LOW("VP9: get_parameter: OMX_IndexParamVideoProfileLevelQuerySupported nProfileIndex ret NoMore %u",
                                          (unsigned int)profileLevelType->nProfileIndex);
            return OMX_ErrorNoMore;
        }
    } else if (output_capability == V4L2_PIX_FMT_HEVC) {
        if (profileLevelType->nProfileIndex < (sizeof(hevc_profiles)/sizeof(int))) {
            profileLevelType->eProfile =  hevc_profiles[profileLevelType->nProfileIndex];
        } else {
            DEBUG_PRINT_LOW("HEVC: get_parameter: OMX_IndexParamVideoProfileLevelQuerySupported nProfileIndex ret NoMore %u",
                                         (unsigned int)profileLevelType->nProfileIndex);
            return OMX_ErrorNoMore;
        }
    } else if (output_capability == V4L2_PIX_FMT_MPEG2) {
        if (profileLevelType->nProfileIndex < (sizeof(mpeg2_profiles)/sizeof(int))) {
            profileLevelType->eProfile = mpeg2_profiles[profileLevelType->nProfileIndex];
        } else {
            DEBUG_PRINT_LOW("get_parameter: OMX_IndexParamVideoProfileLevelQuerySupported nProfileIndex ret NoMore %u",
                                         (unsigned int)profileLevelType->nProfileIndex);
            return OMX_ErrorNoMore;
        }
    }

    /* Check if the profile is supported by driver or not  */
    /* During query caps of profile driver sends a mask of */
    /* of all v4l2 profiles supported(in the flags field)  */
    if (!profile_level_converter::convert_omx_profile_to_v4l2(output_capability, profileLevelType->eProfile, &v4l2_profile)) {
        DEBUG_PRINT_ERROR("Invalid profile, cannot find corresponding omx profile");
        return OMX_ErrorHardware;
    }
    if(!((profile_cap.flags >> v4l2_profile) & 0x1)) {
        DEBUG_PRINT_ERROR("%s: Invalid index corresponding profile not supported : %d ",__FUNCTION__, profileLevelType->eProfile);
        eRet = OMX_ErrorNoMore;
    }
    if (m_disable_hdr & DEC_HDR_DISABLE_FLAG) {
        if (output_capability == V4L2_PIX_FMT_VP9) {
            if (profileLevelType->eProfile == OMX_VIDEO_VP9Profile2HDR || profileLevelType->eProfile == OMX_VIDEO_VP9Profile2HDR10Plus
                || profileLevelType->eProfile == OMX_VIDEO_VP9Profile2)
                hdr_supported = false;
        }
        if (output_capability == V4L2_PIX_FMT_HEVC) {
            if (profileLevelType->eProfile == OMX_VIDEO_HEVCProfileMain10 || profileLevelType->eProfile == OMX_VIDEO_HEVCProfileMain10HDR10
            || profileLevelType->eProfile == OMX_VIDEO_HEVCProfileMain10HDR10Plus) {
                hdr_supported = false;
            }
        }
        if (!hdr_supported) {
            DEBUG_PRINT_LOW("%s: HDR profile unsupported", __FUNCTION__);
            return OMX_ErrorHardware;
        }
    }

    DEBUG_PRINT_LOW("get_parameter: OMX_IndexParamVideoProfileLevelQuerySupported for Input port returned Profile:%u, Level:%u",
            (unsigned int)profileLevelType->eProfile, (unsigned int)profileLevelType->eLevel);
    return eRet;
}

#if defined (_ANDROID_HONEYCOMB_) || defined (_ANDROID_ICS_)
OMX_ERRORTYPE omx_vdec::use_android_native_buffer(OMX_IN OMX_HANDLETYPE hComp, OMX_PTR data)
{
    DEBUG_PRINT_LOW("Inside use_android_native_buffer");
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    UseAndroidNativeBufferParams *params = (UseAndroidNativeBufferParams *)data;

    if ((params == NULL) ||
            (params->nativeBuffer == NULL) ||
            (params->nativeBuffer->handle == NULL) ||
            !m_enable_android_native_buffers)
        return OMX_ErrorBadParameter;
    m_use_android_native_buffers = OMX_TRUE;
    sp<android_native_buffer_t> nBuf = params->nativeBuffer;
    private_handle_t *handle = (private_handle_t *)nBuf->handle;
    if (OMX_CORE_OUTPUT_PORT_INDEX == params->nPortIndex) { //android native buffers can be used only on Output port
        OMX_U8 *buffer = NULL;
        if (!secure_mode) {
            buffer = (OMX_U8*)mmap(0, handle->size,
                    PROT_READ|PROT_WRITE, MAP_SHARED, handle->fd, 0);
            if (buffer == MAP_FAILED) {
                DEBUG_PRINT_ERROR("Failed to mmap pmem with fd = %d, size = %d", handle->fd, handle->size);
                return OMX_ErrorInsufficientResources;
            }
        }
        eRet = use_buffer(hComp,params->bufferHeader,params->nPortIndex,data,handle->size,buffer);
    } else {
        eRet = OMX_ErrorBadParameter;
    }
    return eRet;
}
#endif

/* ======================================================================
   FUNCTION
   omx_vdec::GetConfig

   DESCRIPTION
   OMX Get Config Method implementation.

   PARAMETERS
   <TBD>.

   RETURN VALUE
   OMX Error None if successful.

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::get_config(OMX_IN OMX_HANDLETYPE      hComp,
        OMX_IN OMX_INDEXTYPE configIndex,
        OMX_INOUT OMX_PTR     configData)
{
    (void) hComp;
    OMX_ERRORTYPE eRet = OMX_ErrorNone;

    if (m_state == OMX_StateInvalid) {
        DEBUG_PRINT_ERROR("Get Config in Invalid State");
        return OMX_ErrorInvalidState;
    }

    switch ((unsigned long)configIndex) {
        case OMX_QcomIndexQueryNumberOfVideoDecInstance: {
                                     VALIDATE_OMX_PARAM_DATA(configData, QOMX_VIDEO_QUERY_DECODER_INSTANCES);
                                     QOMX_VIDEO_QUERY_DECODER_INSTANCES *decoderinstances =
                                         (QOMX_VIDEO_QUERY_DECODER_INSTANCES*)configData;
                                     decoderinstances->nNumOfInstances = 16;
                                     /*TODO: How to handle this case */
                                     break;
                                 }
        case OMX_IndexConfigCommonOutputCrop: {
                                  VALIDATE_OMX_PARAM_DATA(configData, OMX_CONFIG_RECTTYPE);
                                  OMX_CONFIG_RECTTYPE *rect = (OMX_CONFIG_RECTTYPE *) configData;
                                  memcpy(rect, &rectangle, sizeof(OMX_CONFIG_RECTTYPE));
                                  DEBUG_PRINT_HIGH("get_config: crop info: L: %u, T: %u, R: %u, B: %u",
                                        rectangle.nLeft, rectangle.nTop,
                                        rectangle.nWidth, rectangle.nHeight);
                                  break;
                              }
        case OMX_QTIIndexConfigDescribeColorAspects:
        {
            VALIDATE_OMX_PARAM_DATA(configData, DescribeColorAspectsParams);
            DescribeColorAspectsParams *params = (DescribeColorAspectsParams *)configData;

            if (params->bRequestingDataSpace) {
                DEBUG_PRINT_LOW("Does not handle dataspace request. Please ignore this Unsupported Setting (0x80001019).");
                return OMX_ErrorUnsupportedSetting;
            }

            print_debug_color_aspects(&(m_client_color_space.sAspects), "GetConfig Client");
            print_debug_color_aspects(&(m_internal_color_space.sAspects), "GetConfig Internal");
            get_preferred_color_aspects(params->sAspects);
            print_debug_color_aspects(&(params->sAspects), "Frameworks path GetConfig Color Aspects");

            break;
        }
        case OMX_QTIIndexConfigDescribeHDRColorInfo:
        {
            VALIDATE_OMX_PARAM_DATA(configData, DescribeHDRStaticInfoParams);
            DescribeHDRStaticInfoParams *params = (DescribeHDRStaticInfoParams *)configData;
            print_debug_hdr_color_info(&(m_client_hdr_info.sInfo), "GetConfig Client HDR");
            print_debug_hdr_color_info(&(m_internal_hdr_info.sInfo), "GetConfig Internal HDR");
            get_preferred_hdr_info(params->sInfo);
            print_debug_hdr_color_info(&(params->sInfo), "Frameworks path GetConfig HDR");

            break;
        }
        case OMX_QTIIndexConfigDescribeHDR10PlusInfo:
        {
            VALIDATE_OMX_PARAM_DATA(configData, DescribeHDR10PlusInfoParams);
            DescribeHDR10PlusInfoParams *params = (DescribeHDR10PlusInfoParams *)configData;
            get_hdr10plusinfo(params);
            print_hdr10plusinfo(params);
            break;
        }
        case OMX_IndexConfigAndroidVendorExtension:
        {
            VALIDATE_OMX_PARAM_DATA(configData, OMX_CONFIG_ANDROID_VENDOR_EXTENSIONTYPE);

            OMX_CONFIG_ANDROID_VENDOR_EXTENSIONTYPE *ext =
                reinterpret_cast<OMX_CONFIG_ANDROID_VENDOR_EXTENSIONTYPE *>(configData);
            VALIDATE_OMX_VENDOR_EXTENSION_PARAM_DATA(ext);
            return get_vendor_extension_config(ext);
        }
        default:
        {
            DEBUG_PRINT_ERROR("get_config: unknown param %d",configIndex);
            eRet = OMX_ErrorBadParameter;
        }

    }

    return eRet;
}

/* ======================================================================
   FUNCTION
   omx_vdec::SetConfig

   DESCRIPTION
   OMX Set Config method implementation

   PARAMETERS
   <TBD>.

   RETURN VALUE
   OMX Error None if successful.
   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::set_config(OMX_IN OMX_HANDLETYPE      hComp,
        OMX_IN OMX_INDEXTYPE configIndex,
        OMX_IN OMX_PTR        configData)
{
    (void) hComp;
    if (m_state == OMX_StateInvalid) {
        DEBUG_PRINT_ERROR("Get Config in Invalid State");
        return OMX_ErrorInvalidState;
    }

    OMX_ERRORTYPE ret = OMX_ErrorNone;
    OMX_VIDEO_CONFIG_NALSIZE *pNal;

    DEBUG_PRINT_LOW("Set Config Called");


    if ((int)configIndex == (int)OMX_IndexVendorVideoFrameRate) {
        OMX_VENDOR_VIDEOFRAMERATE *config = (OMX_VENDOR_VIDEOFRAMERATE *) configData;
        DEBUG_PRINT_HIGH("Index OMX_IndexVendorVideoFrameRate %u", (unsigned int)config->nFps);

        if (config->nPortIndex == OMX_CORE_INPUT_PORT_INDEX) {
            if (config->bEnabled) {
                if ((config->nFps >> 16) > 0 &&
                        (config->nFps >> 16) <= MAX_SUPPORTED_FPS) {
                    m_fps_received = config->nFps;
                    DEBUG_PRINT_HIGH("set_config: frame rate set by omx client : %u",
                            (unsigned int)config->nFps >> 16);
                    Q16ToFraction(config->nFps, drv_ctx.frame_rate.fps_numerator,
                            drv_ctx.frame_rate.fps_denominator);

                    if (!drv_ctx.frame_rate.fps_numerator) {
                        DEBUG_PRINT_ERROR("Numerator is zero setting to 30");
                        drv_ctx.frame_rate.fps_numerator = 30;
                    }

                    if (drv_ctx.frame_rate.fps_denominator) {
                        drv_ctx.frame_rate.fps_numerator = (int)
                            drv_ctx.frame_rate.fps_numerator / drv_ctx.frame_rate.fps_denominator;
                    }

                    drv_ctx.frame_rate.fps_denominator = 1;
                    frm_int = drv_ctx.frame_rate.fps_denominator * 1e6 /
                        drv_ctx.frame_rate.fps_numerator;

                    struct v4l2_control control;

                    control.id =  V4L2_CID_MPEG_VIDC_VIDEO_FRAME_RATE;
                    control.value = drv_ctx.frame_rate.fps_numerator / drv_ctx.frame_rate.fps_denominator;
                    control.value <<= 16;
                    control.value |= (0x0000FFFF & (drv_ctx.frame_rate.fps_numerator % drv_ctx.frame_rate.fps_denominator));
                    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
                    if (ioctl(drv_ctx.video_driver_fd, VIDIOC_S_CTRL, &control)) {
                       DEBUG_PRINT_ERROR("Unable to convey fps info to driver, \
                            performance might be affected");
                       return OMX_ErrorHardware;
                    }
                    client_set_fps = true;
                } else {
                    DEBUG_PRINT_ERROR("Frame rate not supported.");
                    ret = OMX_ErrorUnsupportedSetting;
                }
            } else {
                DEBUG_PRINT_HIGH("set_config: Disabled client's frame rate");
                client_set_fps = false;
            }
        } else {
            DEBUG_PRINT_ERROR(" Set_config: Bad Port idx %d",
                    (int)config->nPortIndex);
            ret = OMX_ErrorBadPortIndex;
        }

        return ret;
    } else if ((int)configIndex == (int)OMX_IndexConfigPriority) {
        OMX_PARAM_U32TYPE *priority = (OMX_PARAM_U32TYPE *)configData;
        DEBUG_PRINT_LOW("Set_config: priority %d",priority->nU32);

        struct v4l2_control control;

        control.id = V4L2_CID_MPEG_VIDC_VIDEO_PRIORITY;
        if (priority->nU32 == 0)
            control.value = V4L2_MPEG_MSM_VIDC_ENABLE;
        else
            control.value = V4L2_MPEG_MSM_VIDC_DISABLE;

        if (ioctl(drv_ctx.video_driver_fd, VIDIOC_S_CTRL, &control)) {
            DEBUG_PRINT_ERROR("Failed to set Priority");
            ret = OMX_ErrorUnsupportedSetting;
        }
        return ret;
    } else if ((int)configIndex == (int)OMX_IndexConfigOperatingRate) {
        OMX_PARAM_U32TYPE *rate = (OMX_PARAM_U32TYPE *)configData;
        DEBUG_PRINT_LOW("Set_config: operating-rate %u fps", rate->nU32 >> 16);

        struct v4l2_control control;

        control.id = V4L2_CID_MPEG_VIDC_VIDEO_OPERATING_RATE;
        control.value = rate->nU32;

        if (rate->nU32 > INT_MAX)
            control.value = INT_MAX;

        if (ioctl(drv_ctx.video_driver_fd, VIDIOC_S_CTRL, &control)) {
            ret = errno == EBUSY ? OMX_ErrorInsufficientResources :
                    OMX_ErrorUnsupportedSetting;
            DEBUG_PRINT_ERROR("Failed to set operating rate %u fps (%s)",
                    rate->nU32 >> 16, errno == -EBUSY ? "HW Overload" : strerror(errno));
        }
        return ret;

    } else if ((int)configIndex == (int)OMX_QTIIndexConfigDescribeColorAspects) {
        VALIDATE_OMX_PARAM_DATA(configData, DescribeColorAspectsParams);
        DescribeColorAspectsParams *params = (DescribeColorAspectsParams *)configData;
        print_debug_color_aspects(&(params->sAspects), "Set Config");
        memcpy(&m_client_color_space, params, sizeof(DescribeColorAspectsParams));
        return ret;
    } else if ((int)configIndex == (int)OMX_QTIIndexConfigDescribeHDRColorInfo) {
        VALIDATE_OMX_PARAM_DATA(configData, DescribeHDRStaticInfoParams);
        DescribeHDRStaticInfoParams *params = (DescribeHDRStaticInfoParams *)configData;
        print_debug_hdr_color_info(&(params->sInfo), "Set Config HDR");
        memcpy(&m_client_hdr_info, params, sizeof(DescribeHDRStaticInfoParams));
        return ret;

    } else if ((int)configIndex == (int)OMX_QTIIndexConfigDescribeHDR10PlusInfo) {
        VALIDATE_OMX_PARAM_DATA(configData, DescribeHDR10PlusInfoParams);
        if (!store_vp9_hdr10plusinfo((DescribeHDR10PlusInfoParams *)configData)) {
            DEBUG_PRINT_ERROR("Failed to set hdr10plus info");
        }
        return ret;

    } else if ((int)configIndex == (int)OMX_IndexConfigLowLatency) {
        OMX_CONFIG_BOOLEANTYPE *lowLatency = (OMX_CONFIG_BOOLEANTYPE *)configData;
        DEBUG_PRINT_LOW("Set_config: low-latency %u",(uint32_t)lowLatency->bEnabled);

        struct v4l2_control control;

        control.id = V4L2_CID_MPEG_VIDC_VIDEO_LOWLATENCY_MODE;
        if (lowLatency->bEnabled) {
            control.value = V4L2_MPEG_MSM_VIDC_ENABLE;
        } else {
            control.value = V4L2_MPEG_MSM_VIDC_DISABLE;
        }

        if (ioctl(drv_ctx.video_driver_fd, VIDIOC_S_CTRL, &control)) {
            DEBUG_PRINT_ERROR("Set low latency failed");
            ret = OMX_ErrorUnsupportedSetting;
        } else {
            m_sParamLowLatency.bEnableLowLatencyMode = lowLatency->bEnabled;
        }
        return ret;

    } else if ((int)configIndex == (int)OMX_IndexConfigAndroidVendorExtension) {
        VALIDATE_OMX_PARAM_DATA(configData, OMX_CONFIG_ANDROID_VENDOR_EXTENSIONTYPE);

        OMX_CONFIG_ANDROID_VENDOR_EXTENSIONTYPE *ext =
                reinterpret_cast<OMX_CONFIG_ANDROID_VENDOR_EXTENSIONTYPE *>(configData);
        VALIDATE_OMX_VENDOR_EXTENSION_PARAM_DATA(ext);

        return set_vendor_extension_config(ext);
    }

    return OMX_ErrorNotImplemented;
}

#define extn_equals(param, extn) (!strcmp(param, extn))

/* ======================================================================
   FUNCTION
   omx_vdec::GetExtensionIndex

   DESCRIPTION
   OMX GetExtensionIndex method implementaion.  <TBD>

   PARAMETERS
   <TBD>.

   RETURN VALUE
   OMX Error None if everything successful.

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::get_extension_index(OMX_IN OMX_HANDLETYPE      hComp,
        OMX_IN OMX_STRING      paramName,
        OMX_OUT OMX_INDEXTYPE* indexType)
{
    (void) hComp;
    if (m_state == OMX_StateInvalid) {
        DEBUG_PRINT_ERROR("Get Extension Index in Invalid State");
        return OMX_ErrorInvalidState;
    } else if (extn_equals(paramName, "OMX.QCOM.index.param.video.SyncFrameDecodingMode")) {
        *indexType = (OMX_INDEXTYPE)OMX_QcomIndexParamVideoSyncFrameDecodingMode;
    } else if (extn_equals(paramName, "OMX.QCOM.index.param.IndexExtraData")) {
        *indexType = (OMX_INDEXTYPE)OMX_QcomIndexParamIndexExtraDataType;
    }
#if defined (_ANDROID_HONEYCOMB_) || defined (_ANDROID_ICS_)
    else if (extn_equals(paramName, "OMX.google.android.index.enableAndroidNativeBuffers")) {
        *indexType = (OMX_INDEXTYPE)OMX_GoogleAndroidIndexEnableAndroidNativeBuffers;
    } else if (extn_equals(paramName, "OMX.google.android.index.useAndroidNativeBuffer2")) {
        *indexType = (OMX_INDEXTYPE)OMX_GoogleAndroidIndexUseAndroidNativeBuffer2;
    } else if (extn_equals(paramName, "OMX.google.android.index.useAndroidNativeBuffer")) {
        DEBUG_PRINT_ERROR("Extension: %s is supported", paramName);
        *indexType = (OMX_INDEXTYPE)OMX_GoogleAndroidIndexUseAndroidNativeBuffer;
    } else if (extn_equals(paramName, "OMX.google.android.index.getAndroidNativeBufferUsage")) {
        *indexType = (OMX_INDEXTYPE)OMX_GoogleAndroidIndexGetAndroidNativeBufferUsage;
    }
#if ALLOCATE_OUTPUT_NATIVEHANDLE
    else if (extn_equals(paramName, "OMX.google.android.index.allocateNativeHandle")) {
        *indexType = (OMX_INDEXTYPE)OMX_GoogleAndroidIndexAllocateNativeHandle;
    }
#endif //ALLOCATE_OUTPUT_NATIVEHANDLE
#endif
    else if (extn_equals(paramName, "OMX.google.android.index.storeMetaDataInBuffers")) {
        *indexType = (OMX_INDEXTYPE)OMX_QcomIndexParamVideoMetaBufferMode;
    }
#ifdef ADAPTIVE_PLAYBACK_SUPPORTED
    else if (extn_equals(paramName, "OMX.google.android.index.prepareForAdaptivePlayback")) {
        *indexType = (OMX_INDEXTYPE)OMX_QcomIndexParamVideoAdaptivePlaybackMode;
    } else if (extn_equals(paramName, OMX_QTI_INDEX_PARAM_VIDEO_PREFER_ADAPTIVE_PLAYBACK)) {
        *indexType = (OMX_INDEXTYPE)OMX_QTIIndexParamVideoPreferAdaptivePlayback;
    }
#endif
#ifdef FLEXYUV_SUPPORTED
    else if (extn_equals(paramName,"OMX.google.android.index.describeColorFormat")) {
        *indexType = (OMX_INDEXTYPE)OMX_QcomIndexFlexibleYUVDescription;
    }
#endif
    else if (extn_equals(paramName, "OMX.QCOM.index.param.video.PassInputBufferFd")) {
        *indexType = (OMX_INDEXTYPE)OMX_QTIIndexParamPassInputBufferFd;
    } else if (extn_equals(paramName, "OMX.QTI.index.param.video.ForceCompressedForDPB")) {
        *indexType = (OMX_INDEXTYPE)OMX_QTIIndexParamForceCompressedForDPB;
    } else if (extn_equals(paramName, "OMX.QTI.index.param.video.ForceUnCompressedForOPB")) {
        *indexType = (OMX_INDEXTYPE)OMX_QTIIndexParamForceUnCompressedForOPB;
    } else if (extn_equals(paramName, "OMX.QTI.index.param.video.LowLatency")) {
        *indexType = (OMX_INDEXTYPE)OMX_QTIIndexParamLowLatencyMode;
    } else if (extn_equals(paramName, OMX_QTI_INDEX_PARAM_VIDEO_CLIENT_EXTRADATA)) {
        *indexType = (OMX_INDEXTYPE)OMX_QTIIndexParamVideoClientExtradata;
    } else if (extn_equals(paramName, "OMX.google.android.index.describeColorAspects")) {
        *indexType = (OMX_INDEXTYPE)OMX_QTIIndexConfigDescribeColorAspects;
    } else if (extn_equals(paramName, "OMX.google.android.index.describeHDRStaticInfo")) {
        *indexType = (OMX_INDEXTYPE)OMX_QTIIndexConfigDescribeHDRColorInfo;
    } else if (extn_equals(paramName, "OMX.google.android.index.describeHDR10PlusInfo")) {
        *indexType = (OMX_INDEXTYPE)OMX_QTIIndexConfigDescribeHDR10PlusInfo;
    }else {
        DEBUG_PRINT_ERROR("Extension: %s not implemented", paramName);
        return OMX_ErrorNotImplemented;
    }
    return OMX_ErrorNone;
}

/* ======================================================================
   FUNCTION
   omx_vdec::GetState

   DESCRIPTION
   Returns the state information back to the caller.<TBD>

   PARAMETERS
   <TBD>.

   RETURN VALUE
   Error None if everything is successful.
   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::get_state(OMX_IN OMX_HANDLETYPE  hComp,
        OMX_OUT OMX_STATETYPE* state)
{
    (void) hComp;
    *state = m_state;
    DEBUG_PRINT_LOW("get_state: Returning the state %d",*state);
    return OMX_ErrorNone;
}

/* ======================================================================
   FUNCTION
   omx_vdec::ComponentTunnelRequest

   DESCRIPTION
   OMX Component Tunnel Request method implementation. <TBD>

   PARAMETERS
   None.

   RETURN VALUE
   OMX Error None if everything successful.

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::component_tunnel_request(OMX_IN OMX_HANDLETYPE  hComp,
        OMX_IN OMX_U32                        port,
        OMX_IN OMX_HANDLETYPE        peerComponent,
        OMX_IN OMX_U32                    peerPort,
        OMX_INOUT OMX_TUNNELSETUPTYPE* tunnelSetup)
{
    (void) hComp;
    (void) port;
    (void) peerComponent;
    (void) peerPort;
    (void) tunnelSetup;
    DEBUG_PRINT_ERROR("Error: component_tunnel_request Not Implemented");
    return OMX_ErrorNotImplemented;
}



/* ======================================================================
   FUNCTION
   omx_vdec::ion_map

   DESCRIPTION
   Map the memory and run the ioctl SYNC operations
   on ION fd

   PARAMETERS
   fd    : ION fd
   len   : Lenth of the memory

   RETURN VALUE
   ERROR: mapped memory pointer

   ========================================================================== */
char *omx_vdec::ion_map(int fd, int len)
{
    char *bufaddr = (char*)mmap(NULL, len, PROT_READ|PROT_WRITE,
                                MAP_SHARED, fd, 0);
    if (bufaddr != MAP_FAILED)
        cache_clean_invalidate(fd);
    return bufaddr;
}

/* ======================================================================
   FUNCTION
   omx_vdec::ion_unmap

   DESCRIPTION
   Unmap the memory

   PARAMETERS
   fd      : ION fd
   bufaddr : buffer address
   len     : Lenth of the memory

   RETURN VALUE
   OMX_Error*

   ========================================================================== */
OMX_ERRORTYPE omx_vdec::ion_unmap(int fd, void *bufaddr, int len)
{
    cache_clean_invalidate(fd);
    if (-1 == munmap(bufaddr, len)) {
        DEBUG_PRINT_ERROR("munmap failed.");
        return OMX_ErrorInsufficientResources;
    }
    return OMX_ErrorNone;
}

/* ======================================================================
   FUNCTION
   omx_vdec::UseOutputBuffer

   DESCRIPTION
   Helper function for Use buffer in the input pin

   PARAMETERS
   None.

   RETURN VALUE
   true/false

   ========================================================================== */
OMX_ERRORTYPE omx_vdec::allocate_extradata()
{
#ifdef USE_ION
    if (drv_ctx.extradata_info.buffer_size) {
        // free previous extradata if any
        free_extradata();

        for (int i = 0; i < drv_ctx.extradata_info.count; i++) {
            drv_ctx.extradata_info.buffer_size = (drv_ctx.extradata_info.buffer_size + 4095) & (~4095);
            // Decoder extradata is always uncached as buffer sizes are very small
            bool status = alloc_map_ion_memory(
                    drv_ctx.extradata_info.buffer_size, &drv_ctx.extradata_info.ion[i], 0);
            if (status == false) {
                DEBUG_PRINT_ERROR("Failed to alloc extradata memory");
                return OMX_ErrorInsufficientResources;
            }
            DEBUG_PRINT_HIGH("Allocated extradata size : %lu fd: %d",
                drv_ctx.extradata_info.buffer_size, drv_ctx.extradata_info.ion[i].data_fd);
            drv_ctx.extradata_info.ion[i].uaddr = ion_map(drv_ctx.extradata_info.ion[i].data_fd,
                                                   drv_ctx.extradata_info.buffer_size);
            if (drv_ctx.extradata_info.ion[i].uaddr == MAP_FAILED) {
                DEBUG_PRINT_ERROR("Failed to map extradata memory");
                free_ion_memory(&drv_ctx.extradata_info.ion[i]);
                return OMX_ErrorInsufficientResources;
            }
        }
    }
#endif
    return OMX_ErrorNone;
}

void omx_vdec::free_extradata()
{
#ifdef USE_ION
    for (int i = 0; i < drv_ctx.extradata_info.count; i++) {
        if (drv_ctx.extradata_info.ion[i].uaddr) {
            ion_unmap(drv_ctx.extradata_info.ion[i].data_fd,
                      (void *)drv_ctx.extradata_info.ion[i].uaddr,
                      drv_ctx.extradata_info.ion[i].alloc_data.len);
            free_ion_memory(&drv_ctx.extradata_info.ion[i]);
            drv_ctx.extradata_info.ion[i].uaddr = NULL;
            drv_ctx.extradata_info.ion[i].data_fd = -1;
        }
    }
#endif
}

OMX_ERRORTYPE  omx_vdec::use_output_buffer(
        OMX_IN OMX_HANDLETYPE            hComp,
        OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
        OMX_IN OMX_U32                   port,
        OMX_IN OMX_PTR                   appData,
        OMX_IN OMX_U32                   bytes,
        OMX_IN OMX_U8*                   buffer)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    OMX_BUFFERHEADERTYPE       *bufHdr= NULL; // buffer header
    unsigned                         i= 0; // Temporary counter
    OMX_PTR privateAppData = NULL;
    private_handle_t *handle = NULL;
    OMX_U8 *buff = buffer;
    bool intermediate = client_buffers.is_color_conversion_enabled();
    (void) hComp;
    (void) port;

    if (!m_out_mem_ptr) {
        DEBUG_PRINT_HIGH("Use_op_buf:Allocating output headers C2D(%d)",
                         client_buffers.is_color_conversion_enabled());
        eRet = allocate_output_headers();
        if (eRet == OMX_ErrorNone)
            eRet = allocate_extradata();
        output_use_buffer = true;
    }

    OMX_BUFFERHEADERTYPE  **omx_base_address =
        intermediate?&m_intermediate_out_mem_ptr:&m_out_mem_ptr;

    if (eRet == OMX_ErrorNone) {
        for (i=0; i< drv_ctx.op_buf.actualcount; i++) {
            if (BITMASK_ABSENT(&m_out_bm_count,i)) {
                break;
            }
        }
    }

    if (i >= drv_ctx.op_buf.actualcount) {
        DEBUG_PRINT_ERROR("Already using %d o/p buffers", drv_ctx.op_buf.actualcount);
        return OMX_ErrorInsufficientResources;
    }

    if (intermediate) {
        DEBUG_PRINT_HIGH("Use_op_buf:Allocating intermediate output. %d", i);
        OMX_BUFFERHEADERTYPE *temp_bufferHdr = NULL;
        eRet = allocate_output_buffer(hComp, &temp_bufferHdr,
                                      port, appData,
                                      drv_ctx.op_buf.buffer_size,
                                      true, i);
    }

    if (eRet == OMX_ErrorNone && dynamic_buf_mode) {
        *bufferHdr = (m_out_mem_ptr + i );
        (*bufferHdr)->pBuffer = NULL;
        if (i == (drv_ctx.op_buf.actualcount - 1) && !streaming[CAPTURE_PORT]) {
            enum v4l2_buf_type buf_type;
            int rr = 0;
            DEBUG_PRINT_LOW("USE intermediate bufferSTREAMON(CAPTURE_MPLANE)");
            set_buffer_req(&drv_ctx.op_buf);
            buf_type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            if (rr = ioctl(drv_ctx.video_driver_fd, VIDIOC_STREAMON, &buf_type)) {
                DEBUG_PRINT_ERROR("STREAMON FAILED : %d", rr);
                return OMX_ErrorInsufficientResources;
            } else {
                streaming[CAPTURE_PORT] = true;
                DEBUG_PRINT_LOW("STREAMON Successful");
            }
        }
        BITMASK_SET(&m_out_bm_count,i);
        (*bufferHdr)->pAppPrivate = appData;
        (*bufferHdr)->pBuffer = buffer;
        (*bufferHdr)->nAllocLen = sizeof(struct VideoDecoderOutputMetaData);
        return eRet;
    }

    if (eRet == OMX_ErrorNone) {
#if defined(_ANDROID_HONEYCOMB_) || defined(_ANDROID_ICS_)
        if (m_enable_android_native_buffers) {
            if (m_use_android_native_buffers) {
                UseAndroidNativeBufferParams *params = (UseAndroidNativeBufferParams *)appData;
                sp<android_native_buffer_t> nBuf = params->nativeBuffer;
                handle = (private_handle_t *)nBuf->handle;
                privateAppData = params->pAppPrivate;
            } else {
                handle = (private_handle_t *)buff;
                privateAppData = appData;
            }
            if (!handle) {
                DEBUG_PRINT_ERROR("handle is invalid");
                return OMX_ErrorBadParameter;
            }

            if ((OMX_U32)handle->size < drv_ctx.op_buf.buffer_size) {
                if (secure_mode) {
                    DEBUG_PRINT_HIGH("Buffer size expected %u, got %u, but it's ok since we will never map it",
                        (unsigned int)drv_ctx.op_buf.buffer_size, (unsigned int)handle->size);
                } else {
                    DEBUG_PRINT_ERROR("Insufficient sized buffer given for playback,"
                            " expected %u, got %u",
                            (unsigned int)drv_ctx.op_buf.buffer_size, (unsigned int)handle->size);
                    return OMX_ErrorBadParameter;
                }
            }

            drv_ctx.op_buf.buffer_size = handle->size;

            if (!m_use_android_native_buffers) {
                if (!secure_mode) {
                    buff = (OMX_U8*)ion_map(handle->fd, handle->size);
                    if (buff == MAP_FAILED) {
                        DEBUG_PRINT_ERROR("Failed to mmap pmem with fd = %d, size = %d", handle->fd, handle->size);
                        return OMX_ErrorInsufficientResources;
                    }
                }
            }
#if defined(_ANDROID_ICS_)
            native_buffer[i].nativehandle = handle;
            native_buffer[i].privatehandle = handle;
#endif
            if (!handle) {
                DEBUG_PRINT_ERROR("Native Buffer handle is NULL");
                return OMX_ErrorBadParameter;
            }
            drv_ctx.ptr_outputbuffer[i].pmem_fd = handle->fd;
            drv_ctx.ptr_outputbuffer[i].offset = 0;
            drv_ctx.ptr_outputbuffer[i].bufferaddr = buff;
            drv_ctx.ptr_outputbuffer[i].buffer_len = drv_ctx.op_buf.buffer_size;
            drv_ctx.ptr_outputbuffer[i].mmaped_size = handle->size;
        } else
#endif
            if (!ouput_egl_buffers && !m_use_output_pmem) {
#ifdef USE_GBM
               bool status = alloc_map_gbm_memory(
                       drv_ctx.video_resolution.frame_width,
                       drv_ctx.video_resolution.frame_height,
                       drv_ctx.gbm_device_fd,
                       &drv_ctx.op_buf_gbm_info[i],
                       secure_mode ? SECURE_FLAGS_OUTPUT_BUFFER : 0);
                if (status == false) {
                    DEBUG_PRINT_ERROR("ION device fd is bad %d",
                                      (int) drv_ctx.op_buf_ion_info[i].data_fd);
                    return OMX_ErrorInsufficientResources;
                }
                drv_ctx.ptr_outputbuffer[i].pmem_fd = \
                                     drv_ctx.op_buf_gbm_info[i].bo_fd;
                if (intermediate)
                   m_pmem_info[i].pmeta_fd = drv_ctx.op_buf_gbm_info[i].meta_fd;
#elif defined USE_ION
                bool status = alloc_map_ion_memory(
                        drv_ctx.op_buf.buffer_size, &drv_ctx.op_buf_ion_info[i],
                        secure_mode ? SECURE_FLAGS_OUTPUT_BUFFER : 0);
                if (status == false) {
                    DEBUG_PRINT_ERROR("ION device fd is bad %d",
                                      (int) drv_ctx.op_buf_ion_info[i].data_fd);
                    return OMX_ErrorInsufficientResources;
                }
                drv_ctx.ptr_outputbuffer[i].pmem_fd = \
                                     drv_ctx.op_buf_ion_info[i].data_fd;
#endif
                if (!secure_mode) {
                    drv_ctx.ptr_outputbuffer[i].bufferaddr =
                        (unsigned char *)ion_map(drv_ctx.ptr_outputbuffer[i].pmem_fd,
                                                 drv_ctx.op_buf.buffer_size);
                    if (drv_ctx.ptr_outputbuffer[i].bufferaddr == MAP_FAILED) {
#ifdef USE_GBM
                        free_gbm_memory(&drv_ctx.op_buf_gbm_info[i]);
#elif defined USE_ION
                        free_ion_memory(&drv_ctx.op_buf_ion_info[i]);
#endif
                        DEBUG_PRINT_ERROR("Unable to mmap output buffer");
                        return OMX_ErrorInsufficientResources;
                    }
                }
                drv_ctx.ptr_outputbuffer[i].offset = 0;
                privateAppData = appData;
            } else {
                DEBUG_PRINT_LOW("Use_op_buf: out_pmem=%d",m_use_output_pmem);
                if (!appData || !bytes ) {
                    if (!secure_mode && !buffer) {
                        DEBUG_PRINT_ERROR("Bad parameters for use buffer");
                        return OMX_ErrorBadParameter;
                    }
                }

                OMX_QCOM_PLATFORM_PRIVATE_LIST *pmem_list;
                OMX_QCOM_PLATFORM_PRIVATE_PMEM_INFO *pmem_info;
                pmem_list = (OMX_QCOM_PLATFORM_PRIVATE_LIST*) appData;
                if (!pmem_list || !pmem_list->entryList || !pmem_list->entryList->entry ||
                        !pmem_list->nEntries ||
                        pmem_list->entryList->type != OMX_QCOM_PLATFORM_PRIVATE_PMEM) {
                    DEBUG_PRINT_ERROR("Pmem info not valid in use buffer");
                    return OMX_ErrorBadParameter;
                }
                pmem_info = (OMX_QCOM_PLATFORM_PRIVATE_PMEM_INFO *)
                    pmem_list->entryList->entry;
                DEBUG_PRINT_LOW("vdec: use buf: pmem_fd=0x%lx",
                        pmem_info->pmem_fd);
                drv_ctx.ptr_outputbuffer[i].pmem_fd = pmem_info->pmem_fd;
#ifdef USE_GBM
                if (intermediate)
                   m_pmem_info[i].pmeta_fd = pmem_info->pmeta_fd;
#endif
                drv_ctx.ptr_outputbuffer[i].offset = pmem_info->offset;
                drv_ctx.ptr_outputbuffer[i].bufferaddr = buff;
                drv_ctx.ptr_outputbuffer[i].mmaped_size =
                    drv_ctx.ptr_outputbuffer[i].buffer_len = drv_ctx.op_buf.buffer_size;
                privateAppData = appData;
            }
        if (intermediate) {
            m_pmem_info[i].offset = drv_ctx.ptr_outputbuffer[i].offset;
            m_pmem_info[i].pmem_fd = drv_ctx.ptr_outputbuffer[i].pmem_fd;
            m_pmem_info[i].size = drv_ctx.ptr_outputbuffer[i].buffer_len;
            m_pmem_info[i].mapped_size = drv_ctx.ptr_outputbuffer[i].mmaped_size;
            m_pmem_info[i].buffer = drv_ctx.ptr_outputbuffer[i].bufferaddr;
        }
        *bufferHdr = (m_out_mem_ptr + i );

        if (secure_mode)
            drv_ctx.ptr_outputbuffer[i].bufferaddr = *bufferHdr;

        if (i == (drv_ctx.op_buf.actualcount -1) && !streaming[CAPTURE_PORT]) {
            enum v4l2_buf_type buf_type;
            buf_type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            if (ioctl(drv_ctx.video_driver_fd, VIDIOC_STREAMON,&buf_type)) {
                return OMX_ErrorInsufficientResources;
            } else {
                streaming[CAPTURE_PORT] = true;
                DEBUG_PRINT_LOW("STREAMON Successful");
            }
        }

        (*bufferHdr)->nAllocLen = drv_ctx.op_buf.buffer_size;
        if (m_enable_android_native_buffers) {
            DEBUG_PRINT_LOW("setting pBuffer to private_handle_t %p", handle);
            (*bufferHdr)->pBuffer = (OMX_U8 *)handle;
        } else {
            (*bufferHdr)->pBuffer = buff;
        }
        (*bufferHdr)->pAppPrivate = privateAppData;
        BITMASK_SET(&m_out_bm_count,i);
    }
    return eRet;
}

OMX_ERRORTYPE omx_vdec::allocate_client_output_extradata_headers()
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    OMX_BUFFERHEADERTYPE *bufHdr = NULL;
    int i = 0;

    if (!m_client_output_extradata_mem_ptr) {
        int nBufferCount       = 0;

        nBufferCount = m_client_out_extradata_info.getBufferCount();
        DEBUG_PRINT_HIGH("allocate_client_output_extradata_headers buffer_count - %d", nBufferCount);

        m_client_output_extradata_mem_ptr = (OMX_BUFFERHEADERTYPE  *)calloc(nBufferCount, sizeof(OMX_BUFFERHEADERTYPE));

        if (m_client_output_extradata_mem_ptr) {
            bufHdr          =  m_client_output_extradata_mem_ptr;
            for (i=0; i < nBufferCount; i++) {
                bufHdr->nSize              = sizeof(OMX_BUFFERHEADERTYPE);
                bufHdr->nVersion.nVersion  = OMX_SPEC_VERSION;
                // Set the values when we determine the right HxW param
                bufHdr->nAllocLen          = 0;
                bufHdr->nFilledLen         = 0;
                bufHdr->pAppPrivate        = NULL;
                bufHdr->nOutputPortIndex   = OMX_CORE_OUTPUT_EXTRADATA_INDEX;
                bufHdr->pBuffer            = NULL;
                bufHdr->pOutputPortPrivate = NULL;
                bufHdr++;
            }
        } else {
             DEBUG_PRINT_ERROR("Extradata header buf mem alloc failed[0x%p]",\
                    m_client_output_extradata_mem_ptr);
              eRet =  OMX_ErrorInsufficientResources;
        }
    }
    return eRet;
}
OMX_ERRORTYPE  omx_vdec::use_client_output_extradata_buffer(
        OMX_IN OMX_HANDLETYPE            hComp,
        OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
        OMX_IN OMX_U32                   port,
        OMX_IN OMX_PTR                   appData,
        OMX_IN OMX_U32                   bytes,
        OMX_IN OMX_U8*                   buffer)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    unsigned i = 0; // Temporary counter
    unsigned buffer_count = m_client_out_extradata_info.getBufferCount();;
    OMX_U32 buffer_size = m_client_out_extradata_info.getSize();
    (void) hComp;

    if (port != OMX_CORE_OUTPUT_EXTRADATA_INDEX ||
            bytes != buffer_size|| bufferHdr == NULL) {
        DEBUG_PRINT_ERROR("Bad Parameters PortIndex is - %d expected is- %d,"
            "bytes = %d expected is %d bufferHdr - %p", port,
            OMX_CORE_OUTPUT_EXTRADATA_INDEX, bytes, buffer_size, bufferHdr);
        eRet = OMX_ErrorBadParameter;
        return eRet;
    }

    if (!m_client_output_extradata_mem_ptr) {
        eRet = allocate_client_output_extradata_headers();
    }

    if (eRet == OMX_ErrorNone) {
        for (i = 0; i < buffer_count; i++) {
            if (BITMASK_ABSENT(&m_out_extradata_bm_count,i)) {
                break;
            }
        }
    }

    if (i >= buffer_count) {
        DEBUG_PRINT_ERROR("Already using %d Extradata o/p buffers", buffer_count);
        eRet = OMX_ErrorInsufficientResources;
    }

    if (eRet == OMX_ErrorNone) {
        BITMASK_SET(&m_out_extradata_bm_count,i);
        *bufferHdr = (m_client_output_extradata_mem_ptr + i );
        (*bufferHdr)->pAppPrivate = appData;
        (*bufferHdr)->pBuffer = buffer;
        (*bufferHdr)->nAllocLen = bytes;
    }

    return eRet;
}
/* ======================================================================
   FUNCTION
   omx_vdec::use_input_heap_buffers

   DESCRIPTION
   OMX Use Buffer Heap allocation method implementation.

   PARAMETERS
   <TBD>.

   RETURN VALUE
   OMX Error None , if everything successful.

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::use_input_heap_buffers(
        OMX_IN OMX_HANDLETYPE            hComp,
        OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
        OMX_IN OMX_U32                   port,
        OMX_IN OMX_PTR                   appData,
        OMX_IN OMX_U32                   bytes,
        OMX_IN OMX_U8*                   buffer)
{
    DEBUG_PRINT_LOW("Inside %s, %p", __FUNCTION__, buffer);
    OMX_ERRORTYPE eRet = OMX_ErrorNone;

    if (secure_mode) {
        DEBUG_PRINT_ERROR("use_input_heap_buffers is not allowed in secure mode");
        return OMX_ErrorUndefined;
    }

    if (!m_inp_heap_ptr)
        m_inp_heap_ptr = (OMX_BUFFERHEADERTYPE*)
            calloc( (sizeof(OMX_BUFFERHEADERTYPE)),
                    drv_ctx.ip_buf.actualcount);
    if (!m_phdr_pmem_ptr)
        m_phdr_pmem_ptr = (OMX_BUFFERHEADERTYPE**)
            calloc( (sizeof(OMX_BUFFERHEADERTYPE*)),
                    drv_ctx.ip_buf.actualcount);
    if (!m_inp_heap_ptr || !m_phdr_pmem_ptr) {
        DEBUG_PRINT_ERROR("Insufficent memory");
        eRet = OMX_ErrorInsufficientResources;
    } else if (m_in_alloc_cnt < drv_ctx.ip_buf.actualcount) {
        input_use_buffer = true;
        memset(&m_inp_heap_ptr[m_in_alloc_cnt], 0, sizeof(OMX_BUFFERHEADERTYPE));
        m_inp_heap_ptr[m_in_alloc_cnt].pBuffer = buffer;
        m_inp_heap_ptr[m_in_alloc_cnt].nAllocLen = bytes;
        m_inp_heap_ptr[m_in_alloc_cnt].pAppPrivate = appData;
        m_inp_heap_ptr[m_in_alloc_cnt].nInputPortIndex = (OMX_U32) OMX_DirInput;
        m_inp_heap_ptr[m_in_alloc_cnt].nOutputPortIndex = (OMX_U32) OMX_DirMax;
        *bufferHdr = &m_inp_heap_ptr[m_in_alloc_cnt];
        eRet = allocate_input_buffer(hComp, &m_phdr_pmem_ptr[m_in_alloc_cnt], port, appData, bytes);
        DEBUG_PRINT_HIGH("Heap buffer(%p) Pmem buffer(%p)", *bufferHdr, m_phdr_pmem_ptr[m_in_alloc_cnt]);
        if (!m_input_free_q.insert_entry((unsigned long)m_phdr_pmem_ptr[m_in_alloc_cnt],
                    (unsigned)NULL, (unsigned)NULL)) {
            DEBUG_PRINT_ERROR("ERROR:Free_q is full");
            return OMX_ErrorInsufficientResources;
        }
        m_in_alloc_cnt++;
    } else {
        DEBUG_PRINT_ERROR("All i/p buffers have been set!");
        eRet = OMX_ErrorInsufficientResources;
    }
    return eRet;
}

/* ======================================================================
   FUNCTION
   omx_vdec::UseBuffer

   DESCRIPTION
   OMX Use Buffer method implementation.

   PARAMETERS
   <TBD>.

   RETURN VALUE
   OMX Error None , if everything successful.

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::use_buffer(
        OMX_IN OMX_HANDLETYPE            hComp,
        OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
        OMX_IN OMX_U32                   port,
        OMX_IN OMX_PTR                   appData,
        OMX_IN OMX_U32                   bytes,
        OMX_IN OMX_U8*                   buffer)
{
    OMX_ERRORTYPE error = OMX_ErrorNone;

    if (bufferHdr == NULL || bytes == 0 || (!secure_mode && buffer == NULL)) {
            DEBUG_PRINT_ERROR("bad param 0x%p %u 0x%p",bufferHdr, (unsigned int)bytes, buffer);
            return OMX_ErrorBadParameter;
    }
    if (m_state == OMX_StateInvalid) {
        DEBUG_PRINT_ERROR("Use Buffer in Invalid State");
        return OMX_ErrorInvalidState;
    }
    if (port == OMX_CORE_INPUT_PORT_INDEX) {
        // If this is not the first allocation (i.e m_inp_mem_ptr is allocated),
        // ensure that use-buffer was called for previous allocation.
        // Mix-and-match of useBuffer and allocateBuffer is not allowed
        if (m_inp_mem_ptr && !input_use_buffer) {
            DEBUG_PRINT_ERROR("'Use' Input buffer called after 'Allocate' Input buffer !");
            return OMX_ErrorUndefined;
        }
        error = use_input_heap_buffers(hComp, bufferHdr, port, appData, bytes, buffer);
    } else if (port == OMX_CORE_OUTPUT_PORT_INDEX) {
        error = use_output_buffer(hComp,bufferHdr,port,appData,bytes,buffer); //not tested
    } else if (port == OMX_CORE_OUTPUT_EXTRADATA_INDEX) {
        error = use_client_output_extradata_buffer(hComp,bufferHdr,port,appData,bytes,buffer);
    } else {
        DEBUG_PRINT_ERROR("Error: Invalid Port Index received %d",(int)port);
        error = OMX_ErrorBadPortIndex;
    }
    DEBUG_PRINT_LOW("Use Buffer: port %u, buffer %p, eRet %d", (unsigned int)port, *bufferHdr, error);
    if (error == OMX_ErrorNone) {
        if (allocate_done() && BITMASK_PRESENT(&m_flags,OMX_COMPONENT_IDLE_PENDING)) {
            // Send the callback now
            BITMASK_CLEAR((&m_flags),OMX_COMPONENT_IDLE_PENDING);
            post_event(OMX_CommandStateSet,OMX_StateIdle,
                    OMX_COMPONENT_GENERATE_EVENT);
        }
        if (port == OMX_CORE_INPUT_PORT_INDEX && m_inp_bPopulated &&
                BITMASK_PRESENT(&m_flags,OMX_COMPONENT_INPUT_ENABLE_PENDING)) {
            BITMASK_CLEAR((&m_flags),OMX_COMPONENT_INPUT_ENABLE_PENDING);
            post_event(OMX_CommandPortEnable,
                    OMX_CORE_INPUT_PORT_INDEX,
                    OMX_COMPONENT_GENERATE_EVENT);
        } else if (port == OMX_CORE_OUTPUT_PORT_INDEX && m_out_bPopulated &&
                BITMASK_PRESENT(&m_flags,OMX_COMPONENT_OUTPUT_ENABLE_PENDING)) {
            BITMASK_CLEAR((&m_flags),OMX_COMPONENT_OUTPUT_ENABLE_PENDING);
            post_event(OMX_CommandPortEnable,
                    OMX_CORE_OUTPUT_PORT_INDEX,
                    OMX_COMPONENT_GENERATE_EVENT);
        }
    }
    return error;
}

OMX_ERRORTYPE omx_vdec::free_input_buffer(unsigned int bufferindex,
        OMX_BUFFERHEADERTYPE *pmem_bufferHdr)
{
    (void)bufferindex;
    if (pmem_bufferHdr)
        free_input_buffer(pmem_bufferHdr);
    return OMX_ErrorNone;
}

OMX_ERRORTYPE omx_vdec::free_input_buffer(OMX_BUFFERHEADERTYPE *bufferHdr)
{
    unsigned int index = 0;
    if (bufferHdr == NULL || m_inp_mem_ptr == NULL) {
        return OMX_ErrorBadParameter;
    }
    print_omx_buffer("free_input_buffer", bufferHdr);

    index = bufferHdr - m_inp_mem_ptr;
    DEBUG_PRINT_LOW("Free Input Buffer index = %d",index);

    bufferHdr->pInputPortPrivate = NULL;

    if (index < drv_ctx.ip_buf.actualcount && drv_ctx.ptr_inputbuffer) {
        if (drv_ctx.ptr_inputbuffer[index].pmem_fd >= 0) {
            if (!secure_mode) {
                ion_unmap(drv_ctx.ptr_inputbuffer[index].pmem_fd,
                          drv_ctx.ptr_inputbuffer[index].bufferaddr,
                          drv_ctx.ptr_inputbuffer[index].mmaped_size);
            }

            if (allocate_native_handle){
                native_handle_t *nh = (native_handle_t *)bufferHdr->pBuffer;
                native_handle_close(nh);
                native_handle_delete(nh);
            } else {
#ifndef USE_ION
                // Close fd for non-secure and secure non-native-handle case
                close(drv_ctx.ptr_inputbuffer[index].pmem_fd);
#endif
            }
            drv_ctx.ptr_inputbuffer[index].pmem_fd = -1;

            if (m_desc_buffer_ptr && m_desc_buffer_ptr[index].buf_addr) {
                free(m_desc_buffer_ptr[index].buf_addr);
                m_desc_buffer_ptr[index].buf_addr = NULL;
                m_desc_buffer_ptr[index].desc_data_size = 0;
            }
#ifdef USE_ION
            free_ion_memory(&drv_ctx.ip_buf_ion_info[index]);
#endif
            if (m_in_alloc_cnt)
                m_in_alloc_cnt--;
        } else {
            DEBUG_PRINT_ERROR("Invalid input buffer fd %d", drv_ctx.ptr_inputbuffer[index].pmem_fd);
        }
    } else {
        DEBUG_PRINT_ERROR("Invalid input buffer index %d, drv_ctx.ptr_inputbuffer %p",
            index, drv_ctx.ptr_inputbuffer);
    }

    return OMX_ErrorNone;
}

OMX_ERRORTYPE omx_vdec::free_output_buffer(OMX_BUFFERHEADERTYPE *bufferHdr,
                                           bool                 intermediate)
{
    unsigned int index = 0;

    OMX_BUFFERHEADERTYPE  *omx_base_address =
             intermediate?m_intermediate_out_mem_ptr:m_out_mem_ptr;
    vdec_bufferpayload *omx_ptr_outputbuffer =
         intermediate?drv_ctx.ptr_intermediate_outputbuffer:drv_ctx.ptr_outputbuffer;
    vdec_ion *omx_op_buf_ion_info =
        intermediate?drv_ctx.op_intermediate_buf_ion_info:drv_ctx.op_buf_ion_info;
#ifdef USE_GBM
    vdec_gbm *omx_op_buf_gbm_info =
        intermediate?drv_ctx.op_intermediate_buf_gbm_info:drv_ctx.op_buf_gbm_info;
#endif
    if (bufferHdr == NULL || omx_base_address == NULL) {
        return OMX_ErrorBadParameter;
    }
    print_omx_buffer("free_output_buffer", bufferHdr);

    index = bufferHdr - omx_base_address;

    if (index < drv_ctx.op_buf.actualcount
        && omx_ptr_outputbuffer) {
        DEBUG_PRINT_LOW("Free ouput Buffer index = %d addr = %p", index,
                        omx_ptr_outputbuffer[index].bufferaddr);

        if (!dynamic_buf_mode) {
            if (streaming[CAPTURE_PORT] &&
                !(in_reconfig || BITMASK_PRESENT(&m_flags,OMX_COMPONENT_OUTPUT_FLUSH_PENDING))) {
                if (stream_off(OMX_CORE_OUTPUT_PORT_INDEX)) {
                    DEBUG_PRINT_ERROR("STREAMOFF(CAPTURE_MPLANE) Failed");
                } else {
                    DEBUG_PRINT_LOW("STREAMOFF(CAPTURE_MPLANE) Successful");
                }
            }
#ifdef _ANDROID_
            if (m_enable_android_native_buffers) {
                if (!secure_mode) {
                    if (omx_ptr_outputbuffer[index].pmem_fd > 0) {
                        ion_unmap(omx_ptr_outputbuffer[index].pmem_fd,
                                  omx_ptr_outputbuffer[index].bufferaddr,
                                  omx_ptr_outputbuffer[index].mmaped_size);
                    }
                }
            } else {
#endif
                if (omx_ptr_outputbuffer[index].pmem_fd > 0 && !ouput_egl_buffers && !m_use_output_pmem) {
                    if (!secure_mode) {
                        ion_unmap(omx_ptr_outputbuffer[index].pmem_fd,
                                  omx_ptr_outputbuffer[index].bufferaddr,
                                  omx_ptr_outputbuffer[index].mmaped_size);
                        omx_ptr_outputbuffer[index].bufferaddr = NULL;
                        omx_ptr_outputbuffer[index].mmaped_size = 0;
                    }
#ifdef USE_GBM
                    free_gbm_memory(&omx_op_buf_gbm_info[index]);
#elif defined USE_ION
                    free_ion_memory(&omx_op_buf_ion_info[index]);
#endif

                    omx_ptr_outputbuffer[index].pmem_fd = -1;
                }
#ifdef _ANDROID_
            }
#endif
        } //!dynamic_buf_mode
        if (intermediate == false) {
            OMX_BUFFERHEADERTYPE *tempBufHdr = m_intermediate_out_mem_ptr + index;
            if (client_buffers.is_color_conversion_enabled() &&
                free_output_buffer(tempBufHdr, true) != OMX_ErrorNone) {
                return OMX_ErrorBadParameter;
            }

            if (release_output_done()) {
            DEBUG_PRINT_HIGH("All output buffers released, free extradata");
            free_extradata();
            }
        }
    }

    return OMX_ErrorNone;
}

/* ======================================================================
   FUNCTION
   omx_vdec::AllocateInputBuffer

   DESCRIPTION
   Helper function for allocate buffer in the input pin

   PARAMETERS
   None.

   RETURN VALUE
   true/false

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::allocate_input_buffer(
        OMX_IN OMX_HANDLETYPE            hComp,
        OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
        OMX_IN OMX_U32                   port,
        OMX_IN OMX_PTR                   appData,
        OMX_IN OMX_U32                   bytes)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    OMX_BUFFERHEADERTYPE *input = NULL;
    unsigned   i = 0;
    unsigned char *buf_addr = NULL;
    int pmem_fd = -1, ret = 0;
    unsigned int align_size = 0;

    (void) hComp;
    (void) port;


    if (bytes != drv_ctx.ip_buf.buffer_size) {
        DEBUG_PRINT_LOW("Requested Size is wrong %u epected is %u",
                (unsigned int)bytes, (unsigned int)drv_ctx.ip_buf.buffer_size);
        return OMX_ErrorBadParameter;
    }

    if (!m_inp_mem_ptr) {
        /* Currently buffer reqs is being set only in set port defn                          */
        /* Client need not do set port definition if he sees enough buffers in get port defn */
        /* In such cases we need to do a set buffer reqs to driver. Doing it here            */
        struct v4l2_requestbuffers bufreq;

        DEBUG_PRINT_HIGH("Calling REQBUFS in %s ",__FUNCTION__);
        bufreq.memory = V4L2_MEMORY_USERPTR;
        bufreq.type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        bufreq.count = drv_ctx.ip_buf.actualcount;
        ret = ioctl(drv_ctx.video_driver_fd,VIDIOC_REQBUFS, &bufreq);
        if (ret) {
            DEBUG_PRINT_ERROR("Setting buffer requirements (reqbufs) failed %d", ret);
            /*TODO: How to handle this case */
            eRet = OMX_ErrorInsufficientResources;
        } else if (bufreq.count != drv_ctx.ip_buf.actualcount) {
            DEBUG_PRINT_ERROR("%s Count(%d) is not expected to change to %d",
                __FUNCTION__, drv_ctx.ip_buf.actualcount, bufreq.count);
            eRet = OMX_ErrorInsufficientResources;
        }

        DEBUG_PRINT_HIGH("Allocate i/p buffer Header: Cnt(%d) Sz(%u)",
                drv_ctx.ip_buf.actualcount,
                (unsigned int)drv_ctx.ip_buf.buffer_size);

        m_inp_mem_ptr = (OMX_BUFFERHEADERTYPE*) \
                calloc( (sizeof(OMX_BUFFERHEADERTYPE)), drv_ctx.ip_buf.actualcount);

        if (m_inp_mem_ptr == NULL) {
            return OMX_ErrorInsufficientResources;
        }

        drv_ctx.ptr_inputbuffer = (struct vdec_bufferpayload *) \
                      calloc ((sizeof (struct vdec_bufferpayload)),drv_ctx.ip_buf.actualcount);

        if (drv_ctx.ptr_inputbuffer == NULL) {
            return OMX_ErrorInsufficientResources;
        }
#ifdef USE_ION
        drv_ctx.ip_buf_ion_info = (struct vdec_ion *) \
                      calloc ((sizeof (struct vdec_ion)),drv_ctx.ip_buf.actualcount);

        if (drv_ctx.ip_buf_ion_info == NULL) {
            return OMX_ErrorInsufficientResources;
        }
#endif

        for (i=0; i < drv_ctx.ip_buf.actualcount; i++) {
            drv_ctx.ptr_inputbuffer [i].pmem_fd = -1;
#ifdef USE_ION
            drv_ctx.ip_buf_ion_info[i].data_fd = -1;
            drv_ctx.ip_buf_ion_info[i].dev_fd = -1;
#endif
        }
    }

    for (i=0; i< drv_ctx.ip_buf.actualcount; i++) {
        if (BITMASK_ABSENT(&m_inp_bm_count,i)) {
            DEBUG_PRINT_LOW("Free Input Buffer Index %d",i);
            break;
        }
    }

    if (i < drv_ctx.ip_buf.actualcount) {
        int rc;
        DEBUG_PRINT_LOW("Allocate input Buffer");
#ifdef USE_ION
        align_size = drv_ctx.ip_buf.buffer_size + 512;
        align_size = (align_size + drv_ctx.ip_buf.alignment - 1)&(~(drv_ctx.ip_buf.alignment - 1));
        // Input buffers are cached to make parsing faster
        bool status = alloc_map_ion_memory(
                align_size, &drv_ctx.ip_buf_ion_info[i],
                secure_mode ? SECURE_FLAGS_INPUT_BUFFER : ION_FLAG_CACHED);
        if (status == false) {
            return OMX_ErrorInsufficientResources;
        }
        pmem_fd = drv_ctx.ip_buf_ion_info[i].data_fd;
#endif
        if (!secure_mode) {
            buf_addr = (unsigned char *)ion_map(pmem_fd, drv_ctx.ip_buf.buffer_size);
            if (buf_addr == MAP_FAILED) {
#ifdef USE_ION
                free_ion_memory(&drv_ctx.ip_buf_ion_info[i]);
#endif
                DEBUG_PRINT_ERROR("Map Failed to allocate input buffer");
                return OMX_ErrorInsufficientResources;
            }
        }
        *bufferHdr = (m_inp_mem_ptr + i);
        if (secure_mode)
            drv_ctx.ptr_inputbuffer [i].bufferaddr = *bufferHdr;
        else
            drv_ctx.ptr_inputbuffer [i].bufferaddr = buf_addr;
        drv_ctx.ptr_inputbuffer [i].pmem_fd = pmem_fd;
        drv_ctx.ptr_inputbuffer [i].buffer_len = drv_ctx.ip_buf.buffer_size;
        drv_ctx.ptr_inputbuffer [i].mmaped_size = drv_ctx.ip_buf.buffer_size;
        drv_ctx.ptr_inputbuffer [i].offset = 0;

        input = *bufferHdr;
        BITMASK_SET(&m_inp_bm_count,i);
        if (allocate_native_handle) {
            native_handle_t *nh = native_handle_create(1 /*numFds*/, 0 /*numInts*/);
            if (!nh) {
                DEBUG_PRINT_ERROR("Native handle create failed");
                return OMX_ErrorInsufficientResources;
            }
            nh->data[0] = drv_ctx.ptr_inputbuffer[i].pmem_fd;
            input->pBuffer = (OMX_U8 *)nh;
        } else if (secure_mode || m_input_pass_buffer_fd) {
            /*Legacy method, pass ion fd stashed directly in pBuffer*/
            input->pBuffer = (OMX_U8 *)(intptr_t)drv_ctx.ptr_inputbuffer[i].pmem_fd;
        } else {
            input->pBuffer           = (OMX_U8 *)buf_addr;
        }
        input->nSize             = sizeof(OMX_BUFFERHEADERTYPE);
        input->nVersion.nVersion = OMX_SPEC_VERSION;
        input->nAllocLen         = drv_ctx.ip_buf.buffer_size;
        input->pAppPrivate       = appData;
        input->nInputPortIndex   = OMX_CORE_INPUT_PORT_INDEX;
        input->pInputPortPrivate = (void *)&drv_ctx.ptr_inputbuffer [i];

        if (drv_ctx.disable_dmx) {
            eRet = allocate_desc_buffer(i);
        }
    } else {
        DEBUG_PRINT_ERROR("ERROR:Input Buffer Index not found");
        eRet = OMX_ErrorInsufficientResources;
    }

    if (eRet == OMX_ErrorNone)
        DEBUG_PRINT_HIGH("Allocate_input_buffer(%d): Header %p buffer %p allocLen %d offset %d fd = %d",
            i, input, input->pBuffer, input->nAllocLen,
            input->nOffset, drv_ctx.ptr_inputbuffer[i].pmem_fd);

    return eRet;
}


/* ======================================================================
   FUNCTION
   omx_vdec::AllocateOutputBuffer

   DESCRIPTION
   Helper fn for AllocateBuffer in the output pin

   PARAMETERS
   <TBD>.

   RETURN VALUE
   OMX Error None if everything went well.

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::allocate_output_buffer(
        OMX_IN OMX_HANDLETYPE            hComp,
        OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
        OMX_IN OMX_U32                   port,
        OMX_IN OMX_PTR                   appData,
        OMX_IN OMX_U32                   bytes,
        OMX_IN bool                      intermediate,
        OMX_IN int                       index)
{
    (void)hComp;
    (void)port;

    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    OMX_BUFFERHEADERTYPE       *bufHdr= NULL; // buffer header
    unsigned                         i= 0; // Temporary counter
#ifdef USE_ION
    struct ion_allocation_data ion_alloc_data;
#endif
    OMX_BUFFERHEADERTYPE  **omx_base_address =
        intermediate?&m_intermediate_out_mem_ptr:&m_out_mem_ptr;

    vdec_bufferpayload **omx_ptr_outputbuffer =
        intermediate?&drv_ctx.ptr_intermediate_outputbuffer:&drv_ctx.ptr_outputbuffer;
    vdec_output_frameinfo **omx_ptr_respbuffer =
        intermediate?&drv_ctx.ptr_intermediate_respbuffer:&drv_ctx.ptr_respbuffer;
    vdec_ion **omx_op_buf_ion_info =
        intermediate?&drv_ctx.op_intermediate_buf_ion_info:&drv_ctx.op_buf_ion_info;
#ifdef USE_GBM
    vdec_gbm **omx_op_buf_gbm_info =
        intermediate?&drv_ctx.op_intermediate_buf_gbm_info:&drv_ctx.op_buf_gbm_info;
#endif

    if (!*omx_base_address) {
        DEBUG_PRINT_HIGH("Allocate o/p buffer Header: Cnt(%d) Sz(%u)",
                drv_ctx.op_buf.actualcount,
                (unsigned int)drv_ctx.op_buf.buffer_size);
        int nBufHdrSize        = 0;
        int nPlatformEntrySize = 0;
        int nPlatformListSize  = 0;
        int nPMEMInfoSize = 0;

        OMX_QCOM_PLATFORM_PRIVATE_LIST      *pPlatformList;
        OMX_QCOM_PLATFORM_PRIVATE_ENTRY     *pPlatformEntry;
        OMX_QCOM_PLATFORM_PRIVATE_PMEM_INFO *pPMEMInfo;

        nBufHdrSize        = drv_ctx.op_buf.actualcount *
            sizeof(OMX_BUFFERHEADERTYPE);
        nPMEMInfoSize      = drv_ctx.op_buf.actualcount *
            sizeof(OMX_QCOM_PLATFORM_PRIVATE_PMEM_INFO);
        nPlatformListSize  = drv_ctx.op_buf.actualcount *
            sizeof(OMX_QCOM_PLATFORM_PRIVATE_LIST);
        nPlatformEntrySize = drv_ctx.op_buf.actualcount *
            sizeof(OMX_QCOM_PLATFORM_PRIVATE_ENTRY);

        *omx_base_address = (OMX_BUFFERHEADERTYPE  *)calloc(nBufHdrSize,1);
        // Alloc mem for platform specific info
        char *pPtr=NULL;
        pPtr = (char*) calloc(nPlatformListSize + nPlatformEntrySize +
                nPMEMInfoSize,1);
        *omx_ptr_outputbuffer = (struct vdec_bufferpayload *)    \
                       calloc (sizeof(struct vdec_bufferpayload),
                               drv_ctx.op_buf.actualcount);
        *omx_ptr_respbuffer = (struct vdec_output_frameinfo  *)\
                     calloc (sizeof (struct vdec_output_frameinfo),
                             drv_ctx.op_buf.actualcount);
        if (!*omx_ptr_outputbuffer || !*omx_ptr_respbuffer) {
            DEBUG_PRINT_ERROR("Failed to alloc outputbuffer or respbuffer ");
            free(pPtr);
            return OMX_ErrorInsufficientResources;
        }

#ifdef USE_GBM
        *omx_op_buf_gbm_info = (struct vdec_gbm *)\
                      calloc (sizeof(struct vdec_gbm),
                      drv_ctx.op_buf.actualcount);
        if (!*omx_op_buf_gbm_info) {
                 DEBUG_PRINT_ERROR("Failed to alloc op_buf_gbm_info");
            return OMX_ErrorInsufficientResources;
        }
        drv_ctx.gbm_device_fd = open("/dev/dri/renderD128", O_RDWR | O_CLOEXEC);
        if (drv_ctx.gbm_device_fd < 0) {
           DEBUG_PRINT_ERROR("opening dri device for gbm failed with fd = %d", drv_ctx.gbm_device_fd);
           return OMX_ErrorInsufficientResources;
        }
#elif defined USE_ION
        *omx_op_buf_ion_info = (struct vdec_ion *)\
                      calloc (sizeof(struct vdec_ion),
                              drv_ctx.op_buf.actualcount);
        if (!*omx_op_buf_ion_info) {
            DEBUG_PRINT_ERROR("Failed to alloc op_buf_ion_info");
            return OMX_ErrorInsufficientResources;
        }
#endif

        if (*omx_base_address && pPtr && *omx_ptr_outputbuffer
            && *omx_ptr_respbuffer) {
            bufHdr          =  *omx_base_address;
            if (m_platform_list) {
                free(m_platform_list);
            }
            m_platform_list = (OMX_QCOM_PLATFORM_PRIVATE_LIST *)(pPtr);
            m_platform_entry= (OMX_QCOM_PLATFORM_PRIVATE_ENTRY *)
                (((char *) m_platform_list)  + nPlatformListSize);
            m_pmem_info     = (OMX_QCOM_PLATFORM_PRIVATE_PMEM_INFO *)
                (((char *) m_platform_entry) + nPlatformEntrySize);
            pPlatformList   = m_platform_list;
            pPlatformEntry  = m_platform_entry;
            pPMEMInfo       = m_pmem_info;

            DEBUG_PRINT_LOW("Memory Allocation Succeeded for OUT port%p", *omx_base_address);

            // Settting the entire storage nicely
            for (i=0; i < drv_ctx.op_buf.actualcount ; i++) {
                bufHdr->nSize              = sizeof(OMX_BUFFERHEADERTYPE);
                bufHdr->nVersion.nVersion  = OMX_SPEC_VERSION;
                // Set the values when we determine the right HxW param
                bufHdr->nAllocLen          = bytes;
                bufHdr->nFilledLen         = 0;
                bufHdr->pAppPrivate        = appData;
                bufHdr->nOutputPortIndex   = OMX_CORE_OUTPUT_PORT_INDEX;
                // Platform specific PMEM Information
                // Initialize the Platform Entry
                //DEBUG_PRINT_LOW("Initializing the Platform Entry for %d",i);
                pPlatformEntry->type       = OMX_QCOM_PLATFORM_PRIVATE_PMEM;
                pPlatformEntry->entry      = pPMEMInfo;
                // Initialize the Platform List
                pPlatformList->nEntries    = 1;
                pPlatformList->entryList   = pPlatformEntry;
                // Keep pBuffer NULL till vdec is opened
                bufHdr->pBuffer            = NULL;
                bufHdr->nOffset            = 0;
                pPMEMInfo->offset = 0;
                pPMEMInfo->pmem_fd = -1;
                bufHdr->pPlatformPrivate = pPlatformList;
                /*Create a mapping between buffers*/
                bufHdr->pOutputPortPrivate = &(*omx_ptr_respbuffer)[i];
                (*omx_ptr_respbuffer)[i].client_data = (void *) \
                    &(*omx_ptr_outputbuffer)[i];
                // Move the buffer and buffer header pointers
                bufHdr++;
                pPMEMInfo++;
                pPlatformEntry++;
                pPlatformList++;
            }
        } else {
            DEBUG_PRINT_ERROR("Output buf mem alloc failed[0x%p][0x%p]",\
                              *omx_base_address, pPtr);
            if (*omx_base_address) {
                free(*omx_base_address);
                *omx_base_address = NULL;
            }
            if (pPtr) {
                free(pPtr);
                pPtr = NULL;
            }
            if (*omx_ptr_outputbuffer) {
                free(*omx_ptr_outputbuffer);
                *omx_ptr_outputbuffer = NULL;
            }
            if (*omx_ptr_respbuffer) {
                free(*omx_ptr_respbuffer);
                *omx_ptr_respbuffer = NULL;
            }
#ifdef USE_GBM
      if(drv_ctx.gbm_device_fd >= 0) {
       DEBUG_PRINT_LOW("Close gbm device");
       close(drv_ctx.gbm_device_fd);
       drv_ctx.gbm_device_fd = -1;
    }
            if (*omx_op_buf_gbm_info) {
                DEBUG_PRINT_LOW("Free o/p gbm context");
                free(*omx_op_buf_gbm_info);
                *omx_op_buf_gbm_info = NULL;
            }
#elif defined USE_ION
            if (*omx_op_buf_ion_info) {
                DEBUG_PRINT_LOW("Free o/p ion context");
                free(*omx_op_buf_ion_info);
                *omx_op_buf_ion_info = NULL;
            }
#endif
            eRet =  OMX_ErrorInsufficientResources;
        }
        if (eRet == OMX_ErrorNone)
            eRet = allocate_extradata();
    }

    if (intermediate == true && index != -1) {
        i = index;
    } else {
        for (i=0; i< drv_ctx.op_buf.actualcount; i++) {
            if (BITMASK_ABSENT(&m_out_bm_count,i)) {
                break;
            }
        }
    }

    if (eRet == OMX_ErrorNone) {
        if (i < drv_ctx.op_buf.actualcount) {
            int rc;
            int pmem_fd = -1;
            int fd = -1;
            unsigned char *pmem_baseaddress = NULL;
#ifdef USE_GBM
            int pmeta_fd = -1;
            // Allocate output buffers as cached to improve performance of software-reading
            // of the YUVs. Output buffers are cache-invalidated in driver.
            // If color-conversion is involved, Only the C2D output buffers are cached, no
            // need to cache the decoder's output buffers
            int cache_flag = client_buffers.is_color_conversion_enabled() ? 0 : ION_FLAG_CACHED;
            bool status = alloc_map_gbm_memory(
                               drv_ctx.video_resolution.frame_width,
                               drv_ctx.video_resolution.frame_height,
                               drv_ctx.gbm_device_fd,
                               &(*omx_op_buf_gbm_info)[i],
                               (secure_mode) ? SECURE_FLAGS_OUTPUT_BUFFER : cache_flag);
            if (status == false) {
                return OMX_ErrorInsufficientResources;
            }
            pmem_fd = (*omx_op_buf_gbm_info)[i].bo_fd;
            pmeta_fd = (*omx_op_buf_gbm_info)[i].meta_fd;
#elif defined USE_ION
            // Allocate output buffers as cached to improve performance of software-reading
            // of the YUVs. Output buffers are cache-invalidated in driver.
            // If color-conversion is involved, Only the C2D output buffers are cached, no
            // need to cache the decoder's output buffers
            int cache_flag = ION_FLAG_CACHED;
            if (intermediate == true && client_buffers.is_color_conversion_enabled()) {
                cache_flag = 0;
            }
            bool status = alloc_map_ion_memory(bytes,
                                               &(*omx_op_buf_ion_info)[i],
                    (secure_mode) ? SECURE_FLAGS_OUTPUT_BUFFER : cache_flag);
            if (status == false) {
                return OMX_ErrorInsufficientResources;
            }
            pmem_fd = (*omx_op_buf_ion_info)[i].data_fd;
#endif
            if (!secure_mode) {
                pmem_baseaddress = (unsigned char *)ion_map(pmem_fd, bytes);
                if (pmem_baseaddress == MAP_FAILED) {
                    DEBUG_PRINT_ERROR("MMAP failed for Size %u",
                            (unsigned int)drv_ctx.op_buf.buffer_size);
#ifdef USE_GBM
                    free_gbm_memory(&(*omx_op_buf_gbm_info)[i]);
#elif defined USE_ION
                    free_ion_memory(&(*omx_op_buf_ion_info)[i]);
#endif
                    return OMX_ErrorInsufficientResources;
                }
            }
            (*omx_ptr_outputbuffer)[i].pmem_fd = pmem_fd;
#ifdef USE_GBM
            m_pmem_info[i].pmeta_fd = pmeta_fd;
#endif
            (*omx_ptr_outputbuffer)[i].offset = 0;
            (*omx_ptr_outputbuffer)[i].bufferaddr = pmem_baseaddress;
            (*omx_ptr_outputbuffer)[i].mmaped_size = bytes;
            (*omx_ptr_outputbuffer)[i].buffer_len = bytes;
            m_pmem_info[i].pmem_fd = pmem_fd;
            m_pmem_info[i].size = (*omx_ptr_outputbuffer)[i].buffer_len;
            m_pmem_info[i].mapped_size = (*omx_ptr_outputbuffer)[i].mmaped_size;
            m_pmem_info[i].buffer = (*omx_ptr_outputbuffer)[i].bufferaddr;
            m_pmem_info[i].offset = (*omx_ptr_outputbuffer)[i].offset;

            *bufferHdr = (*omx_base_address + i );
            if (secure_mode) {
#ifdef USE_GBM
                (*omx_ptr_outputbuffer)[i].bufferaddr =
                    (OMX_U8 *)(intptr_t)(*omx_op_buf_gbm_info)[i].bo_fd;
#elif defined USE_ION
                (*omx_ptr_outputbuffer)[i].bufferaddr =
                    (OMX_U8 *)(intptr_t)(*omx_op_buf_ion_info)[i].data_fd;
#endif
            }
            if (intermediate == false &&
                client_buffers.is_color_conversion_enabled()) {
                OMX_BUFFERHEADERTYPE *temp_bufferHdr = NULL;
                eRet = allocate_output_buffer(hComp, &temp_bufferHdr,
                                              port, appData,
                                              drv_ctx.op_buf.buffer_size,
                                              true, i);
            }
            if (i == (drv_ctx.op_buf.actualcount -1 ) && !streaming[CAPTURE_PORT]) {
                enum v4l2_buf_type buf_type;

                set_buffer_req(&drv_ctx.op_buf);
                buf_type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                if (!client_buffers.is_color_conversion_enabled() ||
                    (client_buffers.is_color_conversion_enabled() && intermediate == true)) {
                    rc=ioctl(drv_ctx.video_driver_fd, VIDIOC_STREAMON,&buf_type);
                    if (rc) {
                        DEBUG_PRINT_ERROR("STREAMON(CAPTURE_MPLANE) Failed");
                        return OMX_ErrorInsufficientResources;
                    } else {
                        streaming[CAPTURE_PORT] = true;
                        DEBUG_PRINT_LOW("STREAMON(CAPTURE_MPLANE) Successful");
                    }
                }
            }

            (*bufferHdr)->pBuffer = (OMX_U8*)(*omx_ptr_outputbuffer)[i].bufferaddr;
            (*bufferHdr)->pAppPrivate = appData;
            BITMASK_SET(&m_out_bm_count,i);
        } else {
            DEBUG_PRINT_ERROR("Faile to allocate output buffer (%d) maxcount %d",
                i, drv_ctx.op_buf.actualcount);
            eRet = OMX_ErrorInsufficientResources;
        }
    }

    if (eRet == OMX_ErrorNone)
        DEBUG_PRINT_HIGH("Allocate_output_buffer(%d): Header %p buffer %p allocLen %d offset %d fd = %d intermediate %d",
                         i, (*bufferHdr), (*bufferHdr)->pBuffer, (*bufferHdr)->nAllocLen,
                         (*bufferHdr)->nOffset, (*omx_ptr_outputbuffer)[i].pmem_fd,
                         intermediate);
    return eRet;
}


// AllocateBuffer  -- API Call
/* ======================================================================
   FUNCTION
   omx_vdec::AllocateBuffer

   DESCRIPTION
   Returns zero if all the buffers released..

   PARAMETERS
   None.

   RETURN VALUE
   true/false

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::allocate_buffer(OMX_IN OMX_HANDLETYPE                hComp,
        OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
        OMX_IN OMX_U32                        port,
        OMX_IN OMX_PTR                     appData,
        OMX_IN OMX_U32                       bytes)
{
    unsigned i = 0;
    OMX_ERRORTYPE eRet = OMX_ErrorNone; // OMX return type

    DEBUG_PRINT_LOW("Allocate buffer on port %d", (int)port);
    if (m_state == OMX_StateInvalid) {
        DEBUG_PRINT_ERROR("Allocate Buf in Invalid State");
        return OMX_ErrorInvalidState;
    }

    if (port == OMX_CORE_INPUT_PORT_INDEX) {
        // If this is not the first allocation (i.e m_inp_mem_ptr is allocated),
        // ensure that use-buffer was never called.
        // Mix-and-match of useBuffer and allocateBuffer is not allowed
        if (m_inp_mem_ptr && input_use_buffer) {
            DEBUG_PRINT_ERROR("'Allocate' Input buffer called after 'Use' Input buffer !");
            return OMX_ErrorUndefined;
        }
        eRet = allocate_input_buffer(hComp,bufferHdr,port,appData,bytes);
    } else if (port == OMX_CORE_OUTPUT_PORT_INDEX) {
        if (output_use_buffer) {
            DEBUG_PRINT_ERROR("Allocate output buffer not allowed after use buffer");
            return OMX_ErrorBadParameter;
        }
        eRet = allocate_output_buffer(hComp, bufferHdr, port, appData, bytes);
    } else {
        DEBUG_PRINT_ERROR("Error: Invalid Port Index received %d",(int)port);
        eRet = OMX_ErrorBadPortIndex;
    }
    if (eRet == OMX_ErrorNone) {
        if (allocate_done()) {
            DEBUG_PRINT_HIGH("Allocated all buffers on port %d", port);
            if (BITMASK_PRESENT(&m_flags,OMX_COMPONENT_IDLE_PENDING)) {
                // Send the callback now
                BITMASK_CLEAR((&m_flags),OMX_COMPONENT_IDLE_PENDING);
                post_event(OMX_CommandStateSet,OMX_StateIdle,
                        OMX_COMPONENT_GENERATE_EVENT);
            }
        }
        if (port == OMX_CORE_INPUT_PORT_INDEX && m_inp_bPopulated) {
            if (BITMASK_PRESENT(&m_flags,OMX_COMPONENT_INPUT_ENABLE_PENDING)) {
                BITMASK_CLEAR((&m_flags),OMX_COMPONENT_INPUT_ENABLE_PENDING);
                post_event(OMX_CommandPortEnable,
                        OMX_CORE_INPUT_PORT_INDEX,
                        OMX_COMPONENT_GENERATE_EVENT);
            }
        }
        if (port == OMX_CORE_OUTPUT_PORT_INDEX && m_out_bPopulated) {
            if (BITMASK_PRESENT(&m_flags,OMX_COMPONENT_OUTPUT_ENABLE_PENDING)) {
                BITMASK_CLEAR((&m_flags),OMX_COMPONENT_OUTPUT_ENABLE_PENDING);
                post_event(OMX_CommandPortEnable,
                        OMX_CORE_OUTPUT_PORT_INDEX,
                        OMX_COMPONENT_GENERATE_EVENT);
            }
        }
    }
    return eRet;
}

// Free Buffer - API call
/* ======================================================================
   FUNCTION
   omx_vdec::FreeBuffer

   DESCRIPTION

   PARAMETERS
   None.

   RETURN VALUE
   true/false

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::free_buffer(OMX_IN OMX_HANDLETYPE         hComp,
        OMX_IN OMX_U32                 port,
        OMX_IN OMX_BUFFERHEADERTYPE* buffer)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    unsigned int nPortIndex;
    (void) hComp;

    auto_lock l(buf_lock);
    if (m_state == OMX_StateIdle &&
            (BITMASK_PRESENT(&m_flags ,OMX_COMPONENT_LOADING_PENDING))) {
        DEBUG_PRINT_LOW(" free buffer while Component in Loading pending");
    } else if (m_state == OMX_StateIdle) {
        DEBUG_PRINT_LOW("Free buffer while Component in Idle");
    } else if ((m_inp_bEnabled == OMX_FALSE && port == OMX_CORE_INPUT_PORT_INDEX)||
            (m_out_bEnabled == OMX_FALSE && port == OMX_CORE_OUTPUT_PORT_INDEX)) {
        DEBUG_PRINT_LOW("Free Buffer while port %u disabled", (unsigned int)port);
    } else if ((port == OMX_CORE_INPUT_PORT_INDEX &&
                BITMASK_PRESENT(&m_flags, OMX_COMPONENT_INPUT_ENABLE_PENDING)) ||
            (port == OMX_CORE_OUTPUT_PORT_INDEX &&
             BITMASK_PRESENT(&m_flags, OMX_COMPONENT_OUTPUT_ENABLE_PENDING))) {
        DEBUG_PRINT_LOW("Free Buffer while port %u enable pending", (unsigned int)port);
    } else if (m_state == OMX_StateExecuting || m_state == OMX_StatePause) {
        DEBUG_PRINT_ERROR("Invalid state to free buffer,ports need to be disabled");
        post_event(OMX_EventError,
                OMX_ErrorPortUnpopulated,
                OMX_COMPONENT_GENERATE_EVENT);
        m_buffer_error = true;
        return OMX_ErrorIncorrectStateOperation;
    } else if (m_state != OMX_StateInvalid) {
        DEBUG_PRINT_ERROR("Invalid state to free buffer,port lost Buffers");
        post_event(OMX_EventError,
                OMX_ErrorPortUnpopulated,
                OMX_COMPONENT_GENERATE_EVENT);
    }

    if (port == OMX_CORE_INPUT_PORT_INDEX) {
        if (!input_use_buffer)
            nPortIndex = buffer - m_inp_mem_ptr;
        else
            nPortIndex = buffer - m_inp_heap_ptr;

        DEBUG_PRINT_LOW("free_buffer on i/p port - Port idx %d", nPortIndex);
        if (nPortIndex < drv_ctx.ip_buf.actualcount &&
                BITMASK_PRESENT(&m_inp_bm_count, nPortIndex)) {
            // Clear the bit associated with it.
            BITMASK_CLEAR(&m_inp_bm_count,nPortIndex);
            BITMASK_CLEAR(&m_heap_inp_bm_count,nPortIndex);
            if (input_use_buffer == true) {

                DEBUG_PRINT_LOW("Free pmem Buffer index %d",nPortIndex);
                if (m_phdr_pmem_ptr)
                    free_input_buffer(m_phdr_pmem_ptr[nPortIndex]);
            } else {
                free_input_buffer(buffer);
            }
            m_inp_bPopulated = OMX_FALSE;
            /*Free the Buffer Header*/
            if (release_input_done()) {
                DEBUG_PRINT_HIGH("ALL input buffers are freed/released");
                free_input_buffer_header();
            }
        } else {
            DEBUG_PRINT_ERROR("Error: free_buffer ,Port Index Invalid");
            eRet = OMX_ErrorBadPortIndex;
        }

        if (BITMASK_PRESENT((&m_flags),OMX_COMPONENT_INPUT_DISABLE_PENDING)
                && release_input_done()) {
            DEBUG_PRINT_LOW("MOVING TO DISABLED STATE");
            BITMASK_CLEAR((&m_flags),OMX_COMPONENT_INPUT_DISABLE_PENDING);
            post_event(OMX_CommandPortDisable,
                    OMX_CORE_INPUT_PORT_INDEX,
                    OMX_COMPONENT_GENERATE_EVENT);
        }
    } else if (port == OMX_CORE_OUTPUT_PORT_INDEX) {
        // check if the buffer is valid
        OMX_BUFFERHEADERTYPE  *omx_base_address =
            client_buffers.is_color_conversion_enabled()?
            m_intermediate_out_mem_ptr:m_out_mem_ptr;
        nPortIndex = buffer - m_out_mem_ptr;
        if (nPortIndex < drv_ctx.op_buf.actualcount &&
                BITMASK_PRESENT(&m_out_bm_count, nPortIndex)) {
            DEBUG_PRINT_LOW("free_buffer on o/p port - Port idx %d", nPortIndex);
            // Clear the bit associated with it.
            BITMASK_CLEAR(&m_out_bm_count,nPortIndex);
            m_out_bPopulated = OMX_FALSE;
            free_output_buffer (buffer);

            if (release_output_done()) {
                DEBUG_PRINT_HIGH("All output buffers released.");
                free_output_buffer_header();
            }
        } else {
            DEBUG_PRINT_ERROR("Error: free_buffer , Port Index Invalid");
            eRet = OMX_ErrorBadPortIndex;
        }
        if (BITMASK_PRESENT((&m_flags),OMX_COMPONENT_OUTPUT_DISABLE_PENDING)
                && release_output_done()) {
            DEBUG_PRINT_LOW("MOVING TO DISABLED STATE");
            BITMASK_CLEAR((&m_flags),OMX_COMPONENT_OUTPUT_DISABLE_PENDING);
#ifdef _ANDROID_ICS_
            if (m_enable_android_native_buffers) {
                DEBUG_PRINT_LOW("FreeBuffer - outport disabled: reset native buffers");
                memset(&native_buffer, 0 ,(sizeof(struct nativebuffer) * MAX_NUM_INPUT_OUTPUT_BUFFERS));
            }
#endif

            post_event(OMX_CommandPortDisable,
                    OMX_CORE_OUTPUT_PORT_INDEX,
                    OMX_COMPONENT_GENERATE_EVENT);
        }
    } else if (port == OMX_CORE_OUTPUT_EXTRADATA_INDEX) {
        nPortIndex = buffer - m_client_output_extradata_mem_ptr;
        DEBUG_PRINT_LOW("free_buffer on extradata output port - Port idx %d", nPortIndex);

        BITMASK_CLEAR(&m_out_extradata_bm_count,nPortIndex);

        if (release_output_extradata_done()) {
            free_output_extradata_buffer_header();
        }
    } else {
        eRet = OMX_ErrorBadPortIndex;
    }
    if ((eRet == OMX_ErrorNone) &&
            (BITMASK_PRESENT(&m_flags ,OMX_COMPONENT_LOADING_PENDING))) {
        if (release_done()) {
            /*
             * Reset buffer requirements here to ensure setting buffer requirement
             * when component move to executing state from loaded state via idle.
             */
            drv_ctx.op_buf.buffer_size = 0;
            drv_ctx.op_buf.actualcount = 0;

            // Send the callback now
            BITMASK_CLEAR((&m_flags),OMX_COMPONENT_LOADING_PENDING);
            post_event(OMX_CommandStateSet, OMX_StateLoaded,
                    OMX_COMPONENT_GENERATE_EVENT);
            m_buffer_error = false;
        }
    }
    return eRet;
}


/* ======================================================================
   FUNCTION
   omx_vdec::EmptyThisBuffer

   DESCRIPTION
   This routine is used to push the encoded video frames to
   the video decoder.

   PARAMETERS
   None.

   RETURN VALUE
   OMX Error None if everything went successful.

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::empty_this_buffer(OMX_IN OMX_HANDLETYPE         hComp,
        OMX_IN OMX_BUFFERHEADERTYPE* buffer)
{
    OMX_ERRORTYPE ret1 = OMX_ErrorNone;
    unsigned int nBufferIndex = drv_ctx.ip_buf.actualcount;

    if (m_state != OMX_StateExecuting &&
            m_state != OMX_StatePause &&
            m_state != OMX_StateIdle) {
        DEBUG_PRINT_ERROR("Empty this buffer in Invalid State");
        return OMX_ErrorInvalidState;
    }

    if (m_error_propogated) {
        DEBUG_PRINT_ERROR("Empty this buffer not allowed after error");
        return OMX_ErrorHardware;
    }

    if (buffer == NULL) {
        DEBUG_PRINT_ERROR("ERROR:ETB Buffer is NULL");
        return OMX_ErrorBadParameter;
    }
    print_omx_buffer("EmptyThisBuffer", buffer);

    if (!m_inp_bEnabled) {
        DEBUG_PRINT_ERROR("ERROR:ETB incorrect state operation, input port is disabled.");
        return OMX_ErrorIncorrectStateOperation;
    }

    if (buffer->nInputPortIndex != OMX_CORE_INPUT_PORT_INDEX) {
        DEBUG_PRINT_ERROR("ERROR:ETB invalid port in header %u", (unsigned int)buffer->nInputPortIndex);
        return OMX_ErrorBadPortIndex;
    }

    if (perf_flag) {
        if (!latency) {
            dec_time.stop();
            latency = dec_time.processing_time_us();
            dec_time.start();
        }
    }

    if (input_use_buffer == true) {
        nBufferIndex = buffer - m_inp_heap_ptr;
        if (nBufferIndex >= drv_ctx.ip_buf.actualcount ) {
            DEBUG_PRINT_ERROR("ERROR: ETB nBufferIndex is invalid in use-buffer mode");
            return OMX_ErrorBadParameter;
        }
        m_inp_mem_ptr[nBufferIndex].nFilledLen = m_inp_heap_ptr[nBufferIndex].nFilledLen;
        m_inp_mem_ptr[nBufferIndex].nTimeStamp = m_inp_heap_ptr[nBufferIndex].nTimeStamp;
        m_inp_mem_ptr[nBufferIndex].nFlags = m_inp_heap_ptr[nBufferIndex].nFlags;
        buffer = &m_inp_mem_ptr[nBufferIndex];
        DEBUG_PRINT_LOW("Non-Arbitrary mode - buffer address is: malloc %p, pmem%p in Index %d, buffer %p of size %u",
                &m_inp_heap_ptr[nBufferIndex], &m_inp_mem_ptr[nBufferIndex],nBufferIndex, buffer, (unsigned int)buffer->nFilledLen);
    } else {
        nBufferIndex = buffer - m_inp_mem_ptr;
    }

    if (nBufferIndex >= drv_ctx.ip_buf.actualcount ) {
        DEBUG_PRINT_ERROR("ERROR:ETB nBufferIndex is invalid");
        return OMX_ErrorBadParameter;
    }

    if (buffer->nFlags & OMX_BUFFERFLAG_CODECCONFIG) {
        codec_config_flag = true;
        DEBUG_PRINT_LOW("%s: codec_config buffer", __FUNCTION__);
    }

    if (m_input_pass_buffer_fd) {
        buffer->pBuffer = (OMX_U8*)drv_ctx.ptr_inputbuffer[nBufferIndex].bufferaddr;
    }

    m_etb_count++;
    //To handle wrap around case. m_etb_count++ to ensure value is non-zero.
    if (!m_etb_count)
        m_etb_count++;
    m_etb_timestamp = buffer->nTimeStamp;
    buffer->pMarkData = (OMX_PTR)(unsigned long)m_etb_count;
    post_event ((unsigned long)hComp,(unsigned long)buffer,OMX_COMPONENT_GENERATE_ETB);

    time_stamp_dts.insert_timestamp(buffer);
    return OMX_ErrorNone;
}

/* ======================================================================
   FUNCTION
   omx_vdec::empty_this_buffer_proxy

   DESCRIPTION
   This routine is used to push the encoded video frames to
   the video decoder.

   PARAMETERS
   None.

   RETURN VALUE
   OMX Error None if everything went successful.

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::empty_this_buffer_proxy(OMX_IN OMX_HANDLETYPE  hComp,
        OMX_IN OMX_BUFFERHEADERTYPE* buffer)
{
    VIDC_TRACE_NAME_HIGH("ETB");
    (void) hComp;
    int push_cnt = 0,i=0;
    unsigned nPortIndex = 0;
    OMX_ERRORTYPE ret = OMX_ErrorNone;
    struct vdec_bufferpayload *temp_buffer;
    bool port_setting_changed = true;

    /*Should we generate a Aync error event*/
    if (buffer == NULL || buffer->pInputPortPrivate == NULL) {
        DEBUG_PRINT_ERROR("ERROR:empty_this_buffer_proxy is invalid");
        return OMX_ErrorBadParameter;
    }

    nPortIndex = buffer-((OMX_BUFFERHEADERTYPE *)m_inp_mem_ptr);

    if (nPortIndex >= drv_ctx.ip_buf.actualcount) {
        DEBUG_PRINT_ERROR("ERROR:empty_this_buffer_proxy invalid nPortIndex[%u]",
                nPortIndex);
        return OMX_ErrorBadParameter;
    }

    pending_input_buffers++;
    VIDC_TRACE_INT_LOW("ETB-pending", pending_input_buffers);

    /* return zero length and not an EOS buffer */
    if ((buffer->nFilledLen == 0) &&
            ((buffer->nFlags & OMX_BUFFERFLAG_EOS) == 0)) {
        DEBUG_PRINT_HIGH("return zero length buffer");
        post_event ((unsigned long)buffer,VDEC_S_SUCCESS,
                OMX_COMPONENT_GENERATE_EBD);
        return OMX_ErrorNone;
    }

    if (input_flush_progress == true) {
        DEBUG_PRINT_LOW("Flush in progress return buffer ");
        post_event ((unsigned long)buffer,VDEC_S_SUCCESS,
                OMX_COMPONENT_GENERATE_EBD);
        return OMX_ErrorNone;
    }

    if (m_error_propogated == true) {
        DEBUG_PRINT_LOW("Return buffer in error state");
        post_event ((unsigned long)buffer,VDEC_S_SUCCESS,
                OMX_COMPONENT_GENERATE_EBD);
        return OMX_ErrorNone;
    }

    auto_lock l(buf_lock);
    temp_buffer = (struct vdec_bufferpayload *)buffer->pInputPortPrivate;

    if (!temp_buffer || (temp_buffer -  drv_ctx.ptr_inputbuffer) > (int)drv_ctx.ip_buf.actualcount) {
        return OMX_ErrorBadParameter;
    }

    if (BITMASK_ABSENT(&m_inp_bm_count, nPortIndex) || m_buffer_error) {
        DEBUG_PRINT_ERROR("ETBProxy: ERROR: invalid buffer, nPortIndex %u", nPortIndex);
        return OMX_ErrorBadParameter;
    }

    VIDC_TRACE_INT_LOW("ETB-TS", buffer->nTimeStamp / 1000);
    VIDC_TRACE_INT_LOW("ETB-size", buffer->nFilledLen);
    /*for use buffer we need to memcpy the data*/
    temp_buffer->buffer_len = buffer->nFilledLen;

    if (input_use_buffer && temp_buffer->bufferaddr && !secure_mode) {
        if (buffer->nFilledLen <= temp_buffer->buffer_len) {
            memcpy (temp_buffer->bufferaddr, (m_inp_heap_ptr[nPortIndex].pBuffer + m_inp_heap_ptr[nPortIndex].nOffset),
                    buffer->nFilledLen);
        } else {
            return OMX_ErrorBadParameter;
        }

    }

    if (drv_ctx.disable_dmx && m_desc_buffer_ptr && m_desc_buffer_ptr[nPortIndex].buf_addr) {
        DEBUG_PRINT_LOW("ETB: dmx enabled");
        if (m_demux_entries == 0) {
            extract_demux_addr_offsets(buffer);
        }

        DEBUG_PRINT_LOW("ETB: handle_demux_data - entries=%u",(unsigned int)m_demux_entries);
        handle_demux_data(buffer);
    }

    log_input_buffers((const char *)temp_buffer->bufferaddr, temp_buffer->buffer_len, buffer->nTimeStamp, temp_buffer->pmem_fd);

    if (buffer->nFlags & QOMX_VIDEO_BUFFERFLAG_EOSEQ) {
        buffer->nFlags &= ~QOMX_VIDEO_BUFFERFLAG_EOSEQ;
    }

    if (temp_buffer->buffer_len == 0 || (buffer->nFlags & OMX_BUFFERFLAG_EOS)) {
        DEBUG_PRINT_HIGH("Rxd i/p EOS, Notify Driver that EOS has been reached");
        nal_count = 0;
        frame_count = 0;
        memset(m_demux_offsets, 0, ( sizeof(OMX_U32) * 8192) );
        m_demux_entries = 0;
    }
    struct v4l2_buffer buf;
    struct v4l2_plane plane;
    memset( (void *)&buf, 0, sizeof(buf));
    memset( (void *)&plane, 0, sizeof(plane));
    int rc;
    unsigned long  print_count;
    if (temp_buffer->buffer_len == 0 && (buffer->nFlags & OMX_BUFFERFLAG_EOS)) {
        struct v4l2_decoder_cmd dec;

        if (!streaming[OUTPUT_PORT]) {
            enum v4l2_buf_type buf_type;
            int ret = 0;

            buf_type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
            DEBUG_PRINT_HIGH("Calling streamon before issuing stop command for EOS");
            ret=ioctl(drv_ctx.video_driver_fd, VIDIOC_STREAMON,&buf_type);
            if (!ret) {
                DEBUG_PRINT_HIGH("Streamon on OUTPUT Plane was successful");
                streaming[OUTPUT_PORT] = true;
            } else {
                DEBUG_PRINT_ERROR("Streamon failed before sending stop command");
                return OMX_ErrorHardware;
            }
        }

        DEBUG_PRINT_HIGH("Input EOS reached. Converted to STOP command") ;
        memset(&dec, 0, sizeof(dec));
        dec.cmd = V4L2_DEC_CMD_STOP;
        rc = ioctl(drv_ctx.video_driver_fd, VIDIOC_DECODER_CMD, &dec);
        post_event ((unsigned long)buffer, VDEC_S_SUCCESS,
            OMX_COMPONENT_GENERATE_EBD);
        if (rc < 0) {
            DEBUG_PRINT_ERROR("Decoder CMD failed");
            return OMX_ErrorHardware;
        }
        return OMX_ErrorNone;
    }

    if (buffer->nFlags & OMX_BUFFERFLAG_EOS) {
        DEBUG_PRINT_HIGH("Input EOS reached") ;
        buf.flags = V4L2_BUF_FLAG_EOS;
    }

    // update hdr10plusinfo list with cookie as pMarkData
    update_hdr10plusinfo_cookie_using_timestamp(buffer->pMarkData, buffer->nTimeStamp);

    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    buf.index = nPortIndex;
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    buf.memory = V4L2_MEMORY_USERPTR;
    plane.bytesused = temp_buffer->buffer_len;
    plane.length = drv_ctx.ip_buf.buffer_size;
    plane.m.userptr = (unsigned long)temp_buffer->bufferaddr -
        (unsigned long)temp_buffer->offset;
    plane.reserved[MSM_VIDC_BUFFER_FD] = temp_buffer->pmem_fd;
    plane.reserved[MSM_VIDC_DATA_OFFSET] = temp_buffer->offset;
    plane.reserved[MSM_VIDC_INPUT_TAG_1] = (unsigned long)buffer->pMarkData;
    plane.reserved[MSM_VIDC_INPUT_TAG_2] = (unsigned long)buffer->hMarkTargetComponent;
    plane.data_offset = 0;
    buf.m.planes = &plane;
    buf.length = 1;
    //assumption is that timestamp is in milliseconds
    buf.timestamp.tv_sec = buffer->nTimeStamp / 1000000;
    buf.timestamp.tv_usec = (buffer->nTimeStamp % 1000000);
    buf.flags |= (buffer->nFlags & OMX_BUFFERFLAG_CODECCONFIG) ? V4L2_BUF_FLAG_CODECCONFIG: 0;

    if (buffer->nFlags & OMX_BUFFERFLAG_CODECCONFIG) {
        DEBUG_PRINT_LOW("Increment codec_config buffer counter");
        android_atomic_inc(&m_queued_codec_config_count);
    }

    print_v4l2_buffer("QBUF-ETB", &buf);
    rc = ioctl(drv_ctx.video_driver_fd, VIDIOC_QBUF, &buf);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed to qbuf Input buffer to driver, send ETB back to client");
        print_v4l2_buffer("QBUF failed", &buf);
        print_omx_buffer("EBD on qbuf failed", buffer);
        m_cb.EmptyBufferDone(hComp, m_app_data, buffer);
        return OMX_ErrorHardware;
    }

    if (codec_config_flag && !(buffer->nFlags & OMX_BUFFERFLAG_CODECCONFIG)) {
        codec_config_flag = false;
    }
    if (!streaming[OUTPUT_PORT]) {
        enum v4l2_buf_type buf_type;
        int ret,r;

        buf_type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        DEBUG_PRINT_LOW("send_command_proxy(): Idle-->Executing");
        ret=ioctl(drv_ctx.video_driver_fd, VIDIOC_STREAMON,&buf_type);
        if (!ret) {
            DEBUG_PRINT_HIGH("Streamon on OUTPUT Plane was successful");
            streaming[OUTPUT_PORT] = true;
        } else if (errno == EBUSY) {
            DEBUG_PRINT_ERROR("Failed to call stream on OUTPUT due to HW_OVERLOAD");
            post_event ((unsigned long)buffer, VDEC_S_SUCCESS,
                    OMX_COMPONENT_GENERATE_EBD);
            return OMX_ErrorInsufficientResources;
        } else {
            DEBUG_PRINT_ERROR("Failed to call streamon on OUTPUT");
            DEBUG_PRINT_LOW("If Stream on failed no buffer should be queued");
            post_event ((unsigned long)buffer, VDEC_S_SUCCESS,
                    OMX_COMPONENT_GENERATE_EBD);
            return OMX_ErrorBadParameter;
        }
    }

    return ret;
}

/* ======================================================================
   FUNCTION
   omx_vdec::FillThisBuffer

   DESCRIPTION
   IL client uses this method to release the frame buffer
   after displaying them.

   PARAMETERS
   None.

   RETURN VALUE
   true/false

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::fill_this_buffer(OMX_IN OMX_HANDLETYPE  hComp,
        OMX_IN OMX_BUFFERHEADERTYPE* buffer)
{
    if (m_state != OMX_StateExecuting &&
            m_state != OMX_StatePause &&
            m_state != OMX_StateIdle) {
        DEBUG_PRINT_ERROR("FTB in Invalid State");
        return OMX_ErrorInvalidState;
    }

    if (buffer == NULL || buffer->nOutputPortIndex != OMX_CORE_OUTPUT_PORT_INDEX) {
        DEBUG_PRINT_ERROR("ERROR:FTB invalid buffer %p or PortIndex - %d",
             buffer, buffer ? (int)buffer->nOutputPortIndex : -1);
        return OMX_ErrorBadPortIndex;
    }
    print_omx_buffer("FillThisBuffer", buffer);

    if (m_error_propogated) {
        DEBUG_PRINT_ERROR("Fill this buffer not allowed after error");
        return OMX_ErrorHardware;
    }

    if (!m_out_bEnabled) {
        DEBUG_PRINT_ERROR("ERROR:FTB incorrect state operation, output port is disabled.");
        return OMX_ErrorIncorrectStateOperation;
    }

    unsigned nPortIndex = buffer - m_out_mem_ptr;
    if (dynamic_buf_mode) {
        private_handle_t *handle = NULL;
        struct VideoDecoderOutputMetaData *meta = NULL;

        if (!buffer || !buffer->pBuffer) {
            DEBUG_PRINT_ERROR("%s: invalid params: %p", __FUNCTION__, buffer);
            return OMX_ErrorBadParameter;
        }

        meta = (struct VideoDecoderOutputMetaData *)buffer->pBuffer;
        handle = (private_handle_t *)meta->pHandle;

        if (!handle) {
            DEBUG_PRINT_ERROR("FTB: Error: IL client passed an invalid buf handle - %p", handle);
            return OMX_ErrorBadParameter;
        }

        //Fill outputbuffer with buffer details, this will be sent to f/w during VIDIOC_QBUF
        if (nPortIndex < drv_ctx.op_buf.actualcount &&
            nPortIndex < MAX_NUM_INPUT_OUTPUT_BUFFERS) {
            drv_ctx.ptr_outputbuffer[nPortIndex].pmem_fd = handle->fd;
            drv_ctx.ptr_outputbuffer[nPortIndex].bufferaddr = (OMX_U8*) buffer;

            //Store private handle from GraphicBuffer
            native_buffer[nPortIndex].privatehandle = handle;
            native_buffer[nPortIndex].nativehandle = handle;
        } else {
            DEBUG_PRINT_ERROR("[FTB]Invalid native_buffer index: %d", nPortIndex);
            return OMX_ErrorBadParameter;
        }

        buffer->nAllocLen = handle->size;

        if (!client_buffers.is_color_conversion_enabled()) {
            drv_ctx.op_buf.buffer_size = handle->size;
        }

    }

    if (client_buffers.is_color_conversion_enabled()) {
        buffer = m_intermediate_out_mem_ptr + nPortIndex;
        buffer->nAllocLen = drv_ctx.op_buf.buffer_size;
    }

    //buffer->nAllocLen will be sizeof(struct VideoDecoderOutputMetaData). Overwrite
    //this with a more sane size so that we don't compensate in rest of code
    //We'll restore this size later on, so that it's transparent to client
    buffer->nFilledLen = 0;

    post_event((unsigned long) hComp, (unsigned long)buffer, m_fill_output_msg);
    return OMX_ErrorNone;
}

/* ======================================================================
   FUNCTION
   omx_vdec::fill_this_buffer_proxy

   DESCRIPTION
   IL client uses this method to release the frame buffer
   after displaying them.

   PARAMETERS
   None.

   RETURN VALUE
   true/false

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::fill_this_buffer_proxy(
        OMX_IN OMX_HANDLETYPE        hComp,
        OMX_IN OMX_BUFFERHEADERTYPE* bufferAdd)
{
    VIDC_TRACE_NAME_HIGH("FTB");
    OMX_ERRORTYPE nRet = OMX_ErrorNone;
    OMX_BUFFERHEADERTYPE *buffer = bufferAdd;
    unsigned bufIndex = 0;
    struct vdec_bufferpayload     *ptr_outputbuffer = NULL;
    struct vdec_output_frameinfo  *ptr_respbuffer = NULL;

    auto_lock l(buf_lock);
    OMX_BUFFERHEADERTYPE  *omx_base_address =
        client_buffers.is_color_conversion_enabled()?
                           m_intermediate_out_mem_ptr:m_out_mem_ptr;
    vdec_bufferpayload *omx_ptr_outputbuffer =
        client_buffers.is_color_conversion_enabled()?
                    drv_ctx.ptr_intermediate_outputbuffer:drv_ctx.ptr_outputbuffer;
    bufIndex = buffer-omx_base_address;

    if (bufferAdd == NULL || bufIndex >= drv_ctx.op_buf.actualcount) {
        DEBUG_PRINT_ERROR("FTBProxy: ERROR: invalid buffer index, bufIndex %u bufCount %u",
            bufIndex, drv_ctx.op_buf.actualcount);
        return OMX_ErrorBadParameter;
    }

    if (BITMASK_ABSENT(&m_out_bm_count, bufIndex) || m_buffer_error) {
        DEBUG_PRINT_ERROR("FTBProxy: ERROR: invalid buffer, bufIndex %u", bufIndex);
        return OMX_ErrorBadParameter;
    }

    /*Return back the output buffer to client*/
    if (m_out_bEnabled != OMX_TRUE || output_flush_progress == true || in_reconfig) {
        DEBUG_PRINT_LOW("Output Buffers return flush/disable condition");
        buffer->nFilledLen = 0;
        print_omx_buffer("FBD in FTBProxy", &m_out_mem_ptr[bufIndex]);
        m_cb.FillBufferDone (hComp,m_app_data,&m_out_mem_ptr[bufIndex]);
        return OMX_ErrorNone;
    }
    if (m_error_propogated == true) {
        DEBUG_PRINT_LOW("Return buffers in error state");
        buffer->nFilledLen = 0;
        print_omx_buffer("FBD in FTBProxy", &m_out_mem_ptr[bufIndex]);
        m_cb.FillBufferDone (hComp,m_app_data,&m_out_mem_ptr[bufIndex]);
        return OMX_ErrorNone;
    }

    if (dynamic_buf_mode) {
        omx_ptr_outputbuffer[bufIndex].offset = 0;
        omx_ptr_outputbuffer[bufIndex].buffer_len = buffer->nAllocLen;
        omx_ptr_outputbuffer[bufIndex].mmaped_size = buffer->nAllocLen;
    }

    pending_output_buffers++;
    VIDC_TRACE_INT_LOW("FTB-pending", pending_output_buffers);
    ptr_respbuffer = (struct vdec_output_frameinfo*)buffer->pOutputPortPrivate;
    if (ptr_respbuffer) {
        ptr_outputbuffer =  (struct vdec_bufferpayload*)ptr_respbuffer->client_data;
    }

    if (ptr_respbuffer == NULL || ptr_outputbuffer == NULL) {
        DEBUG_PRINT_ERROR("Invalid ptr_respbuffer %p, ptr_outputbuffer %p",
            ptr_respbuffer, ptr_outputbuffer);
        buffer->nFilledLen = 0;
        print_omx_buffer("FBD in error", &m_out_mem_ptr[bufIndex]);
        m_cb.FillBufferDone (hComp,m_app_data,&m_out_mem_ptr[bufIndex]);
        pending_output_buffers--;
        VIDC_TRACE_INT_LOW("FTB-pending", pending_output_buffers);
        return OMX_ErrorBadParameter;
    }

    int rc = 0;
    struct v4l2_buffer buf;
    struct v4l2_plane plane[VIDEO_MAX_PLANES];
    memset( (void *)&buf, 0, sizeof(buf));
    memset( (void *)plane, 0, (sizeof(struct v4l2_plane)*VIDEO_MAX_PLANES));
    unsigned int extra_idx = 0;

    buf.index = bufIndex;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_USERPTR;
    plane[0].bytesused = buffer->nFilledLen;
    plane[0].length = buffer->nAllocLen;
    plane[0].m.userptr =
        (unsigned long)omx_ptr_outputbuffer[bufIndex].bufferaddr -
        (unsigned long)omx_ptr_outputbuffer[bufIndex].offset;
    plane[0].reserved[MSM_VIDC_BUFFER_FD] = omx_ptr_outputbuffer[bufIndex].pmem_fd;
    plane[0].reserved[MSM_VIDC_DATA_OFFSET] = omx_ptr_outputbuffer[bufIndex].offset;
    plane[0].data_offset = 0;
    extra_idx = EXTRADATA_IDX(drv_ctx.num_planes);
    if (extra_idx && (extra_idx < VIDEO_MAX_PLANES)) {
        plane[extra_idx].bytesused = drv_ctx.extradata_info.buffer_size;
        plane[extra_idx].length = drv_ctx.extradata_info.buffer_size;
        plane[extra_idx].m.userptr = (long unsigned int)drv_ctx.extradata_info.ion[bufIndex].uaddr;
#ifdef USE_ION
        plane[extra_idx].reserved[MSM_VIDC_BUFFER_FD] = drv_ctx.extradata_info.ion[bufIndex].data_fd;
#endif
        plane[extra_idx].reserved[MSM_VIDC_DATA_OFFSET] = 0;
        plane[extra_idx].data_offset = 0;
    } else if (extra_idx >= VIDEO_MAX_PLANES) {
        DEBUG_PRINT_ERROR("Extradata index higher than expected: %u", extra_idx);
        return OMX_ErrorBadParameter;
    }
    buf.m.planes = plane;
    buf.length = drv_ctx.num_planes;
    print_v4l2_buffer("QBUF-FTB", &buf);
    rc = ioctl(drv_ctx.video_driver_fd, VIDIOC_QBUF, &buf);
    if (rc) {
        buffer->nFilledLen = 0;
        DEBUG_PRINT_ERROR("Failed to qbuf to driver, error %s", strerror(errno));
        print_omx_buffer("FBD in error", &m_out_mem_ptr[bufIndex]);
        m_cb.FillBufferDone(hComp, m_app_data, &m_out_mem_ptr[bufIndex]);
        return OMX_ErrorHardware;
    }

    return OMX_ErrorNone;
}

/* ======================================================================
   FUNCTION
   omx_vdec::SetCallbacks

   DESCRIPTION
   Set the callbacks.

   PARAMETERS
   None.

   RETURN VALUE
   OMX Error None if everything successful.

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::set_callbacks(OMX_IN OMX_HANDLETYPE        hComp,
        OMX_IN OMX_CALLBACKTYPE* callbacks,
        OMX_IN OMX_PTR             appData)
{
    (void) hComp;

    if (!callbacks)
        return OMX_ErrorBadParameter;

    m_cb       = *callbacks;
    DEBUG_PRINT_LOW("Callbacks Set %p %p %p",m_cb.EmptyBufferDone,\
            m_cb.EventHandler,m_cb.FillBufferDone);
    m_app_data =    appData;
    return OMX_ErrorNone;
}

/* ======================================================================
   FUNCTION
   omx_vdec::ComponentDeInit

   DESCRIPTION
   Destroys the component and release memory allocated to the heap.

   PARAMETERS
   <TBD>.

   RETURN VALUE
   OMX Error None if everything successful.

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::component_deinit(OMX_IN OMX_HANDLETYPE hComp)
{
   (void) hComp;
   OMX_ERRORTYPE nRet = OMX_ErrorNone;
   OMX_BUFFERHEADERTYPE *buffer;

    unsigned i = 0;
    if (OMX_StateLoaded != m_state) {
        DEBUG_PRINT_ERROR("WARNING:Rxd DeInit,OMX not in LOADED state %d",\
                m_state);
        DEBUG_PRINT_ERROR("Playback Ended - FAILED");
    } else {
        DEBUG_PRINT_HIGH("Playback Ended - PASSED");
    }

    /*Check if the output buffers have to be cleaned up*/
    buffer = m_out_mem_ptr;
    if (buffer) {
        DEBUG_PRINT_LOW("Freeing the Output Memory");
        for (i = 0; i < drv_ctx.op_buf.actualcount; i++ ) {
            if (BITMASK_PRESENT(&m_out_bm_count, i)) {
                BITMASK_CLEAR(&m_out_bm_count, i);
                    nRet = free_output_buffer (buffer+i);
                    if (OMX_ErrorNone != nRet)
                        break;
            }
            if (release_output_done()) {
                DEBUG_PRINT_HIGH("All output buffers are released");
                break;
            }
        }
#ifdef _ANDROID_ICS_
        memset(&native_buffer, 0, (sizeof(nativebuffer) * MAX_NUM_INPUT_OUTPUT_BUFFERS));
#endif
    }

    /*Check if the input buffers have to be cleaned up*/
    if (m_inp_mem_ptr || m_inp_heap_ptr) {
        DEBUG_PRINT_LOW("Freeing the Input Memory");
        for (i = 0; i<drv_ctx.ip_buf.actualcount; i++ ) {

            if (BITMASK_PRESENT(&m_inp_bm_count, i)) {
                BITMASK_CLEAR(&m_inp_bm_count, i);
                if (m_inp_mem_ptr)
                    free_input_buffer (i,&m_inp_mem_ptr[i]);
                else
                    free_input_buffer (i,NULL);
            }

            if (release_input_done()) {
                DEBUG_PRINT_HIGH("All input buffers released");
                break;
            }
       }
    }
    free_input_buffer_header();
    free_output_buffer_header();

    if (m_platform_list) {
        free(m_platform_list);
        m_platform_list = NULL;
    }
    if (m_vendor_config.pData) {
        free(m_vendor_config.pData);
        m_vendor_config.pData = NULL;
    }

    // Reset counters in mesg queues
    m_ftb_q.m_size=0;
    m_cmd_q.m_size=0;
    m_etb_q.m_size=0;
    m_ftb_q.m_read = m_ftb_q.m_write =0;
    m_cmd_q.m_read = m_cmd_q.m_write =0;
    m_etb_q.m_read = m_etb_q.m_write =0;

    DEBUG_PRINT_LOW("Calling VDEC_IOCTL_STOP_NEXT_MSG");
    //(void)ioctl(drv_ctx.video_driver_fd, VDEC_IOCTL_STOP_NEXT_MSG,
    // NULL);
    DEBUG_PRINT_HIGH("Close the driver instance");

    if (m_debug.infile) {
        fclose(m_debug.infile);
        m_debug.infile = NULL;
    }
    if (m_debug.outfile) {
        fclose(m_debug.outfile);
        m_debug.outfile = NULL;
    }
    if (m_debug.ccoutfile) {
        fclose(m_debug.ccoutfile);
        m_debug.ccoutfile = NULL;
    }
    if (m_debug.out_ymeta_file) {
        fclose(m_debug.out_ymeta_file);
        m_debug.out_ymeta_file = NULL;
    }
    if (m_debug.out_uvmeta_file) {
        fclose(m_debug.out_uvmeta_file);
        m_debug.out_uvmeta_file = NULL;
    }
#ifdef OUTPUT_EXTRADATA_LOG
    if (outputExtradataFile)
        fclose (outputExtradataFile);
#endif
    DEBUG_PRINT_INFO("omx_vdec::component_deinit() complete");
    return OMX_ErrorNone;
}

/* ======================================================================
   FUNCTION
   omx_vdec::UseEGLImage

   DESCRIPTION
   OMX Use EGL Image method implementation <TBD>.

   PARAMETERS
   <TBD>.

   RETURN VALUE
   Not Implemented error.

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::use_EGL_image(OMX_IN OMX_HANDLETYPE     hComp,
        OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
        OMX_IN OMX_U32                        port,
        OMX_IN OMX_PTR                     appData,
        OMX_IN void*                      eglImage)
{
    (void) appData;
    OMX_QCOM_PLATFORM_PRIVATE_LIST pmem_list;
    OMX_QCOM_PLATFORM_PRIVATE_ENTRY pmem_entry;
    OMX_QCOM_PLATFORM_PRIVATE_PMEM_INFO pmem_info;

#ifdef USE_EGL_IMAGE_GPU
    PFNEGLQUERYIMAGEQUALCOMMPROC egl_queryfunc;
    EGLint fd = -1, offset = 0,pmemPtr = 0;
#else
    int fd = -1, offset = 0;
#endif
    DEBUG_PRINT_HIGH("use EGL image support for decoder");
    if (!bufferHdr || !eglImage|| port != OMX_CORE_OUTPUT_PORT_INDEX) {
        DEBUG_PRINT_ERROR("Invalid EGL image");
    }
#ifdef USE_EGL_IMAGE_GPU
    if (m_display_id == NULL) {
        DEBUG_PRINT_ERROR("Display ID is not set by IL client");
        return OMX_ErrorInsufficientResources;
    }
    egl_queryfunc = (PFNEGLQUERYIMAGEQUALCOMMPROC)
        eglGetProcAddress("eglQueryImageKHR");
    egl_queryfunc(m_display_id, eglImage, EGL_BUFFER_HANDLE, &fd);
    egl_queryfunc(m_display_id, eglImage, EGL_BUFFER_OFFSET, &offset);
    egl_queryfunc(m_display_id, eglImage, EGL_BITMAP_POINTER_KHR, &pmemPtr);
#else //with OMX test app
    struct temp_egl {
        int pmem_fd;
        int offset;
    };
    struct temp_egl *temp_egl_id = NULL;
    void * pmemPtr = (void *) eglImage;
    temp_egl_id = (struct temp_egl *)eglImage;
    if (temp_egl_id != NULL) {
        fd = temp_egl_id->pmem_fd;
        offset = temp_egl_id->offset;
    }
#endif
    if (fd < 0) {
        DEBUG_PRINT_ERROR("Improper pmem fd by EGL client %d",fd);
        return OMX_ErrorInsufficientResources;
    }
    pmem_info.pmem_fd = (OMX_U32) fd;
    pmem_info.offset = (OMX_U32) offset;
    pmem_entry.entry = (void *) &pmem_info;
    pmem_entry.type = OMX_QCOM_PLATFORM_PRIVATE_PMEM;
    pmem_list.entryList = &pmem_entry;
    pmem_list.nEntries = 1;
    ouput_egl_buffers = true;
    if (OMX_ErrorNone != use_buffer(hComp,bufferHdr, port,
                (void *)&pmem_list, drv_ctx.op_buf.buffer_size,
                (OMX_U8 *)pmemPtr)) {
        DEBUG_PRINT_ERROR("use buffer call failed for egl image");
        return OMX_ErrorInsufficientResources;
    }
    return OMX_ErrorNone;
}

/* ======================================================================
   FUNCTION
   omx_vdec::ComponentRoleEnum

   DESCRIPTION
   OMX Component Role Enum method implementation.

   PARAMETERS
   <TBD>.

   RETURN VALUE
   OMX Error None if everything is successful.
   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::component_role_enum(OMX_IN OMX_HANDLETYPE hComp,
        OMX_OUT OMX_U8*        role,
        OMX_IN OMX_U32        index)
{
    (void) hComp;
    OMX_ERRORTYPE eRet = OMX_ErrorNone;

    if (!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.mpeg2",OMX_MAX_STRINGNAME_SIZE)) {
        if ((0 == index) && role) {
            strlcpy((char *)role, "video_decoder.mpeg2",OMX_MAX_STRINGNAME_SIZE);
            DEBUG_PRINT_LOW("component_role_enum: role %s",role);
        } else {
            eRet = OMX_ErrorNoMore;
        }
    } else if (!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.avc",OMX_MAX_STRINGNAME_SIZE)) {
        if ((0 == index) && role) {
            strlcpy((char *)role, "video_decoder.avc",OMX_MAX_STRINGNAME_SIZE);
            DEBUG_PRINT_LOW("component_role_enum: role %s",role);
        } else {
            DEBUG_PRINT_LOW("No more roles");
            eRet = OMX_ErrorNoMore;
        }
    } else if (!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.hevc", OMX_MAX_STRINGNAME_SIZE)) {
        if ((0 == index) && role) {
            strlcpy((char *)role, "video_decoder.hevc", OMX_MAX_STRINGNAME_SIZE);
            DEBUG_PRINT_LOW("component_role_enum: role %s", role);
        } else {
            DEBUG_PRINT_LOW("No more roles");
            eRet = OMX_ErrorNoMore;
        }
    } else if (!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.vp8",OMX_MAX_STRINGNAME_SIZE)) {
        if ((0 == index) && role) {
            strlcpy((char *)role, "video_decoder.vp8",OMX_MAX_STRINGNAME_SIZE);
            DEBUG_PRINT_LOW("component_role_enum: role %s",role);
        } else {
            DEBUG_PRINT_LOW("No more roles");
            eRet = OMX_ErrorNoMore;
        }
    } else if (!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.vp9",OMX_MAX_STRINGNAME_SIZE)) {
        if ((0 == index) && role) {
            strlcpy((char *)role, "video_decoder.vp9",OMX_MAX_STRINGNAME_SIZE);
            DEBUG_PRINT_LOW("component_role_enum: role %s",role);
        } else {
            DEBUG_PRINT_LOW("No more roles");
            eRet = OMX_ErrorNoMore;
        }
    } else {
        DEBUG_PRINT_ERROR("ERROR:Querying Role on Unknown Component");
        eRet = OMX_ErrorInvalidComponentName;
    }
    return eRet;
}




/* ======================================================================
   FUNCTION
   omx_vdec::AllocateDone

   DESCRIPTION
   Checks if entire buffer pool is allocated by IL Client or not.
   Need this to move to IDLE state.

   PARAMETERS
   None.

   RETURN VALUE
   true/false.

   ========================================================================== */
bool omx_vdec::allocate_done(void)
{
    bool bRet = false;
    bool bRet_In = false;
    bool bRet_Out = false;
    bool bRet_Out_Extra = false;

    bRet_In = allocate_input_done();
    bRet_Out = allocate_output_done();
    bRet_Out_Extra = allocate_output_extradata_done();

    if (bRet_In && bRet_Out && bRet_Out_Extra) {
        DEBUG_PRINT_HIGH("All ports buffers are allocated");
        bRet = true;
    }

    return bRet;
}
/* ======================================================================
   FUNCTION
   omx_vdec::AllocateInputDone

   DESCRIPTION
   Checks if I/P buffer pool is allocated by IL Client or not.

   PARAMETERS
   None.

   RETURN VALUE
   true/false.

   ========================================================================== */
bool omx_vdec::allocate_input_done(void)
{
    bool bRet = false;
    unsigned i=0;

    if (m_inp_mem_ptr == NULL) {
        return bRet;
    }
    if (m_inp_mem_ptr ) {
        for (; i<drv_ctx.ip_buf.actualcount; i++) {
            if (BITMASK_ABSENT(&m_inp_bm_count,i)) {
                break;
            }
        }
    }
    if (i == drv_ctx.ip_buf.actualcount) {
        bRet = true;
    }
    if (i==drv_ctx.ip_buf.actualcount && m_inp_bEnabled) {
        m_inp_bPopulated = OMX_TRUE;
    }
    return bRet;
}
/* ======================================================================
   FUNCTION
   omx_vdec::AllocateOutputDone

   DESCRIPTION
   Checks if entire O/P buffer pool is allocated by IL Client or not.

   PARAMETERS
   None.

   RETURN VALUE
   true/false.

   ========================================================================== */
bool omx_vdec::allocate_output_done(void)
{
    bool bRet = false;
    unsigned j=0;

    if (m_out_mem_ptr == NULL) {
        return bRet;
    }

    if (m_out_mem_ptr) {
        for (; j < drv_ctx.op_buf.actualcount; j++) {
            if (BITMASK_ABSENT(&m_out_bm_count,j)) {
                break;
            }
        }
    }

    if (j == drv_ctx.op_buf.actualcount) {
        bRet = true;
        if (m_out_bEnabled)
            m_out_bPopulated = OMX_TRUE;
    }

    return bRet;
}

bool omx_vdec::allocate_output_extradata_done(void)
{
    bool bRet = false;
    unsigned j=0;
    unsigned nBufferCount = 0;

    nBufferCount = m_client_out_extradata_info.getBufferCount();

    if (!m_client_out_extradata_info.is_client_extradata_enabled()) {
        return true;
    }

    if (m_client_output_extradata_mem_ptr) {
        for (; j < nBufferCount; j++) {
            if (BITMASK_ABSENT(&m_out_extradata_bm_count,j)) {
                break;
            }
        }

        if (j == nBufferCount) {
            bRet = true;
            DEBUG_PRINT_HIGH("Allocate done for all extradata o/p buffers");
        }
    }

    return bRet;
}
/* ======================================================================
   FUNCTION
   omx_vdec::ReleaseDone

   DESCRIPTION
   Checks if IL client has released all the buffers.

   PARAMETERS
   None.

   RETURN VALUE
   true/false

   ========================================================================== */
bool omx_vdec::release_done(void)
{
    bool bRet = false;

    if (release_input_done()) {
        if (release_output_done()) {
            if (release_output_extradata_done()) {
                DEBUG_PRINT_HIGH("All ports buffers are released");
                bRet = true;
            }
        }
    }
    return bRet;
}


/* ======================================================================
   FUNCTION
   omx_vdec::ReleaseOutputDone

   DESCRIPTION
   Checks if IL client has released all the buffers.

   PARAMETERS
   None.

   RETURN VALUE
   true/false

   ========================================================================== */
bool omx_vdec::release_output_done(void)
{
    bool bRet = false;
    unsigned i=0,j=0;

    if (m_out_mem_ptr) {
        for (; j < drv_ctx.op_buf.actualcount ; j++) {
            if (BITMASK_PRESENT(&m_out_bm_count,j)) {
                break;
            }
        }
        if (j == drv_ctx.op_buf.actualcount) {
            m_out_bm_count = 0;
            bRet = true;
        }
    } else {
        m_out_bm_count = 0;
        bRet = true;
    }
    return bRet;
}
/* ======================================================================
   FUNCTION
   omx_vdec::ReleaseInputDone

   DESCRIPTION
   Checks if IL client has released all the buffers.

   PARAMETERS
   None.

   RETURN VALUE
   true/false

   ========================================================================== */
bool omx_vdec::release_input_done(void)
{
    bool bRet = false;
    unsigned i=0,j=0;

    if (m_inp_mem_ptr) {
        for (; j<drv_ctx.ip_buf.actualcount; j++) {
            if ( BITMASK_PRESENT(&m_inp_bm_count,j)) {
                break;
            }
        }
        if (j==drv_ctx.ip_buf.actualcount) {
            bRet = true;
        }
    } else {
        bRet = true;
    }
    return bRet;
}

bool omx_vdec::release_output_extradata_done(void)
{
    bool bRet = false;
    unsigned i=0,j=0, buffer_count=0;

    buffer_count = m_client_out_extradata_info.getBufferCount();
    DEBUG_PRINT_LOW("Value of m_client_output_extradata_mem_ptr %p buffer_count - %d",
            m_client_output_extradata_mem_ptr, buffer_count);

    if (m_client_output_extradata_mem_ptr) {
        for (; j<buffer_count; j++) {
            if ( BITMASK_PRESENT(&m_out_extradata_bm_count,j)) {
                break;
            }
        }
        if (j == buffer_count) {
            bRet = true;
        }
    } else {
        bRet = true;
    }
    return bRet;
}

OMX_ERRORTYPE omx_vdec::fill_buffer_done(OMX_HANDLETYPE hComp,
        OMX_BUFFERHEADERTYPE * buffer)
{
    VIDC_TRACE_NAME_HIGH("FBD");
    OMX_QCOM_PLATFORM_PRIVATE_PMEM_INFO *pPMEMInfo = NULL;
    OMX_BUFFERHEADERTYPE  *omx_base_address =
        client_buffers.is_color_conversion_enabled()?
                              m_intermediate_out_mem_ptr:m_out_mem_ptr;
    vdec_bufferpayload *omx_ptr_outputbuffer =
        client_buffers.is_color_conversion_enabled()?
                    drv_ctx.ptr_intermediate_outputbuffer:drv_ctx.ptr_outputbuffer;

    if (!buffer || (buffer - omx_base_address) >= (int)drv_ctx.op_buf.actualcount) {
        DEBUG_PRINT_ERROR("[FBD] ERROR in ptr(%p)", buffer);
        return OMX_ErrorBadParameter;
    } else if (output_flush_progress) {
        DEBUG_PRINT_LOW("FBD: Buffer (%p) flushed", buffer);
        buffer->nFilledLen = 0;
        buffer->nTimeStamp = 0;
        buffer->nFlags &= ~OMX_BUFFERFLAG_EXTRADATA;
        buffer->nFlags &= ~QOMX_VIDEO_BUFFERFLAG_EOSEQ;
        buffer->nFlags &= ~OMX_BUFFERFLAG_DATACORRUPT;
    }

    if (buffer->nFlags & QOMX_VIDEO_BUFFERFLAG_EOSEQ) {
        DEBUG_PRINT_HIGH("***************************************************");
        DEBUG_PRINT_HIGH("FillBufferDone: End Of Sequence Received");
        DEBUG_PRINT_HIGH("***************************************************");
    }

    if (buffer->nFlags & OMX_BUFFERFLAG_DATACORRUPT) {
        DEBUG_PRINT_HIGH("***************************************************");
        DEBUG_PRINT_HIGH("FillBufferDone: OMX_BUFFERFLAG_DATACORRUPT Received");
        DEBUG_PRINT_HIGH("***************************************************");
    }

    pending_output_buffers --;
    VIDC_TRACE_INT_LOW("FTB-pending", pending_output_buffers);

    if (buffer->nFlags & OMX_BUFFERFLAG_EOS) {
        DEBUG_PRINT_HIGH("Output EOS has been reached");
        if (!output_flush_progress)
            post_event((unsigned)NULL, (unsigned)NULL,
                    OMX_COMPONENT_GENERATE_EOS_DONE);

        if (psource_frame) {
            print_omx_buffer("EBD in FBD", psource_frame);
            m_cb.EmptyBufferDone(&m_cmp, m_app_data, psource_frame);
            psource_frame = NULL;
        }
        if (pdest_frame) {
            pdest_frame->nFilledLen = 0;
            m_input_free_q.insert_entry((unsigned long) pdest_frame,(unsigned)NULL,
                    (unsigned)NULL);
            pdest_frame = NULL;
        }
    }

#ifdef OUTPUT_EXTRADATA_LOG
    if (outputExtradataFile) {
        int buf_index = buffer - omx_base_address;
        if (buf_index < drv_ctx.extradata_info.count &&
                drv_ctx.extradata_info.ion[buf_index].uaddr)
            fwrite (drv_ctx.extradata_info.ion[buf_index].uaddr, 1,
                drv_ctx.extradata_info.buffer_size, outputExtradataFile);
    }
#endif

    /* For use buffer we need to copy the data */
    if (!output_flush_progress) {
        /* This is the error check for non-recoverable errros */
        bool is_duplicate_ts_valid = true;
        bool is_interlaced = (drv_ctx.interlace != VDEC_InterlaceFrameProgressive);

        if (output_capability == V4L2_PIX_FMT_MPEG4 ||
                output_capability == V4L2_PIX_FMT_MPEG2)
            is_duplicate_ts_valid = false;

        if (buffer->nFilledLen > 0) {
            time_stamp_dts.get_next_timestamp(buffer,
                    is_interlaced && is_duplicate_ts_valid && !is_mbaff);
        }
    }
    VIDC_TRACE_INT_LOW("FBD-TS", buffer->nTimeStamp / 1000);

    if (m_cb.FillBufferDone) {
        if (buffer->nFilledLen > 0) {
            set_frame_rate(buffer->nTimeStamp);

            proc_frms++;
            if (perf_flag) {
                if (1 == proc_frms) {
                    dec_time.stop();
                    latency = dec_time.processing_time_us() - latency;
                    DEBUG_PRINT_HIGH(">>> FBD Metrics: Latency(%.2f)mS", latency / 1e3);
                    dec_time.start();
                    fps_metrics.start();
                }
                if (buffer->nFlags & OMX_BUFFERFLAG_EOS) {
                    OMX_U64 proc_time = 0;
                    fps_metrics.stop();
                    proc_time = fps_metrics.processing_time_us();
                    DEBUG_PRINT_HIGH(">>> FBD Metrics: proc_frms(%u) proc_time(%.2f)S fps(%.2f)",
                            (unsigned int)proc_frms, (float)proc_time / 1e6,
                            (float)(1e6 * proc_frms) / proc_time);
                }
            }
        }
        if (buffer->nFlags & OMX_BUFFERFLAG_EOS) {
            prev_ts = LLONG_MAX;
            proc_frms = 0;
        }

        pPMEMInfo = (OMX_QCOM_PLATFORM_PRIVATE_PMEM_INFO *)
            ((OMX_QCOM_PLATFORM_PRIVATE_LIST *)
             buffer->pPlatformPrivate)->entryList->entry;
        OMX_BUFFERHEADERTYPE *il_buffer;
        il_buffer = client_buffers.get_il_buf_hdr(buffer);
        OMX_U32 current_framerate = (int)(drv_ctx.frame_rate.fps_numerator / drv_ctx.frame_rate.fps_denominator);

        if (il_buffer && il_buffer->nTimeStamp >= 0 && m_dec_hfr_fps > 0 && buffer->nFilledLen > 0) {
            uint64_t tsDeltaUs = llabs(il_buffer->nTimeStamp - m_prev_timestampUs);
            double vsyncUs = 1e6/m_dec_hfr_fps;
            double vsync_start = (static_cast<uint64_t>(il_buffer->nTimeStamp/vsyncUs)) * vsyncUs;
            double vsync_end = vsync_start + vsyncUs;
            bool render_frame = false;

            if ((static_cast<double>(il_buffer->nTimeStamp + tsDeltaUs) > (vsync_end + 1.0)) ||
                 !m_prev_timestampUs || il_buffer->nFlags & OMX_BUFFERFLAG_EOS) {
                render_frame = true;
            }
            // Render frames very close to boundaries of vsync interval
            if ((abs(static_cast<double>(il_buffer->nTimeStamp) - vsync_start) < 1.0) ||
                (abs(static_cast<double>(il_buffer->nTimeStamp) - vsync_end) < 1.0)) {
                render_frame = true;
            }
            // Render frames for which ts_Delta == 0, only if previous frame was rendered.
            if (tsDeltaUs == 0 && m_prev_frame_rendered) {
                render_frame = true;
            }
            if (!render_frame) {
                buffer->nFilledLen = 0;
                m_prev_frame_rendered = false;
            } else {
                m_prev_frame_rendered = true;
            }

            m_prev_timestampUs = il_buffer->nTimeStamp;
            DEBUG_PRINT_LOW(" -- %s Frame with bufferTs(%lld)", buffer->nFilledLen? "Rendering":"Dropping", il_buffer->nTimeStamp);
        }

        // add current framerate to gralloc meta data
        if ((buffer->nFilledLen > 0) && m_enable_android_native_buffers && omx_base_address) {
            // If valid fps was received, consider the same if less than dec hfr rate
            // Otherwise, calculate fps using fbd timestamps
            float refresh_rate = (m_fps_received >> 16) ? (m_fps_received >> 16) : current_framerate;

            if (m_dec_hfr_fps)
                refresh_rate = m_dec_hfr_fps;

            DEBUG_PRINT_LOW("frc set refresh_rate %f, frame %d", refresh_rate, proc_frms);
            OMX_U32 buf_index = buffer - omx_base_address;
            setMetaData((private_handle_t *)native_buffer[buf_index].privatehandle,
                         UPDATE_REFRESH_RATE, (void*)&refresh_rate);
        }

        if (il_buffer) {
            log_output_buffers(buffer);
            log_cc_output_buffers(il_buffer);
            if (dynamic_buf_mode) {
                unsigned int nPortIndex = 0;
                nPortIndex = buffer-omx_base_address;

                // Since we're passing around handles, adjust nFilledLen and nAllocLen
                // to size of the handle. Do it _after_ log_output_buffers which
                // requires the respective sizes to be accurate.

                buffer->nAllocLen = sizeof(struct VideoDecoderOutputMetaData);
                buffer->nFilledLen = buffer->nFilledLen ?
                        sizeof(struct VideoDecoderOutputMetaData) : 0;

                //Clear graphic buffer handles in dynamic mode
                if (nPortIndex < drv_ctx.op_buf.actualcount &&
                    nPortIndex < MAX_NUM_INPUT_OUTPUT_BUFFERS) {
                    native_buffer[nPortIndex].privatehandle = NULL;
                    native_buffer[nPortIndex].nativehandle = NULL;
                } else {
                    DEBUG_PRINT_ERROR("[FBD]Invalid native_buffer index: %d", nPortIndex);
                    return OMX_ErrorBadParameter;
                }
            }
            print_omx_buffer("FillBufferDone", buffer);
            m_cb.FillBufferDone (hComp,m_app_data,il_buffer);
        } else {
            DEBUG_PRINT_ERROR("Invalid buffer address from get_il_buf_hdr");
            return OMX_ErrorBadParameter;
        }
    } else {
        DEBUG_PRINT_ERROR("NULL m_cb.FillBufferDone");
        return OMX_ErrorBadParameter;
    }

    return OMX_ErrorNone;
}

OMX_ERRORTYPE omx_vdec::empty_buffer_done(OMX_HANDLETYPE         hComp,
        OMX_BUFFERHEADERTYPE* buffer)
{
    VIDC_TRACE_NAME_HIGH("EBD");
    int nBufferIndex = buffer - m_inp_mem_ptr;

    if (buffer == NULL || (nBufferIndex >= (int)drv_ctx.ip_buf.actualcount)) {
        DEBUG_PRINT_ERROR("empty_buffer_done: ERROR bufhdr = %p", buffer);
        return OMX_ErrorBadParameter;
    }

    pending_input_buffers--;
    VIDC_TRACE_INT_LOW("ETB-pending", pending_input_buffers);

    if (m_cb.EmptyBufferDone) {
        buffer->nFilledLen = 0;
        if (input_use_buffer == true) {
            buffer = &m_inp_heap_ptr[buffer-m_inp_mem_ptr];
        }

        /* Restore the FD that we over-wrote in ETB */
        if (m_input_pass_buffer_fd) {
            buffer->pBuffer = (OMX_U8*)(uintptr_t)drv_ctx.ptr_inputbuffer[nBufferIndex].pmem_fd;
        }

        print_omx_buffer("EmptyBufferDone", buffer);
        m_cb.EmptyBufferDone(hComp ,m_app_data, buffer);
    }
    return OMX_ErrorNone;
}

int omx_vdec::async_message_process (void *context, void* message)
{
    omx_vdec* omx = NULL;
    struct vdec_msginfo *vdec_msg = NULL;
    OMX_BUFFERHEADERTYPE* omxhdr = NULL;
    struct v4l2_buffer *v4l2_buf_ptr = NULL;
    struct v4l2_plane *plane = NULL;
    struct vdec_output_frameinfo *output_respbuf = NULL;
    int rc=1;
    bool reconfig_event_sent = false;
    if (context == NULL || message == NULL) {
        DEBUG_PRINT_ERROR("FATAL ERROR in omx_vdec::async_message_process NULL Check");
        return -1;
    }
    vdec_msg = (struct vdec_msginfo *)message;

    omx = reinterpret_cast<omx_vdec*>(context);

    switch (vdec_msg->msgcode) {

        case VDEC_MSG_EVT_HW_ERROR:
            omx->post_event ((unsigned)NULL, vdec_msg->status_code,\
                    OMX_COMPONENT_GENERATE_HARDWARE_ERROR);
            break;

        case VDEC_MSG_EVT_HW_OVERLOAD:
            omx->post_event ((unsigned)NULL, vdec_msg->status_code,\
                    OMX_COMPONENT_GENERATE_HARDWARE_OVERLOAD);
            break;

        case VDEC_MSG_EVT_HW_UNSUPPORTED:
            omx->post_event ((unsigned)NULL, vdec_msg->status_code,\
                    OMX_COMPONENT_GENERATE_UNSUPPORTED_SETTING);
            break;

        case VDEC_MSG_RESP_START_DONE:
            omx->post_event ((unsigned)NULL, vdec_msg->status_code,\
                    OMX_COMPONENT_GENERATE_START_DONE);
            break;

        case VDEC_MSG_RESP_STOP_DONE:
            omx->post_event ((unsigned)NULL, vdec_msg->status_code,\
                    OMX_COMPONENT_GENERATE_STOP_DONE);
            break;

        case VDEC_MSG_RESP_RESUME_DONE:
            omx->post_event ((unsigned)NULL, vdec_msg->status_code,\
                    OMX_COMPONENT_GENERATE_RESUME_DONE);
            break;

        case VDEC_MSG_RESP_PAUSE_DONE:
            omx->post_event ((unsigned)NULL, vdec_msg->status_code,\
                    OMX_COMPONENT_GENERATE_PAUSE_DONE);
            break;

        case VDEC_MSG_RESP_FLUSH_INPUT_DONE:
            omx->post_event ((unsigned)NULL, vdec_msg->status_code,\
                    OMX_COMPONENT_GENERATE_EVENT_INPUT_FLUSH);
            break;
        case VDEC_MSG_RESP_FLUSH_OUTPUT_DONE:
            omx->post_event ((unsigned)NULL, vdec_msg->status_code,\
                    OMX_COMPONENT_GENERATE_EVENT_OUTPUT_FLUSH);
            break;
        case VDEC_MSG_RESP_INPUT_FLUSHED:
        case VDEC_MSG_RESP_INPUT_BUFFER_DONE:

            /* omxhdr = (OMX_BUFFERHEADERTYPE* )
               vdec_msg->msgdata.input_frame_clientdata; */

            v4l2_buf_ptr = (v4l2_buffer*)vdec_msg->msgdata.input_frame_clientdata;
            if (omx->m_inp_mem_ptr == NULL || v4l2_buf_ptr == NULL ||
                v4l2_buf_ptr->index >= omx->drv_ctx.ip_buf.actualcount) {
                omxhdr = NULL;
                vdec_msg->status_code = VDEC_S_EFATAL;
                break;

            }
            omxhdr = omx->m_inp_mem_ptr + v4l2_buf_ptr->index;

            if (v4l2_buf_ptr->flags & V4L2_BUF_INPUT_UNSUPPORTED) {
                DEBUG_PRINT_HIGH("Unsupported input");
                omx->post_event ((unsigned)NULL, vdec_msg->status_code,\
                        OMX_COMPONENT_GENERATE_HARDWARE_ERROR);
            }
            if (v4l2_buf_ptr->flags & V4L2_BUF_FLAG_DATA_CORRUPT) {
                omxhdr->nFlags |= OMX_BUFFERFLAG_DATACORRUPT;
                vdec_msg->status_code = VDEC_S_INPUT_BITSTREAM_ERR;
            }
            if (omxhdr->nFlags & OMX_BUFFERFLAG_CODECCONFIG) {

                DEBUG_PRINT_LOW("Decrement codec_config buffer counter");
                android_atomic_dec(&omx->m_queued_codec_config_count);
                if ((android_atomic_add(0, &omx->m_queued_codec_config_count) == 0) &&
                    BITMASK_PRESENT(&omx->m_flags, OMX_COMPONENT_FLUSH_DEFERRED)) {
                    DEBUG_PRINT_LOW("sem post for CODEC CONFIG buffer");
                    sem_post(&omx->m_safe_flush);
                }
            }
            omx->post_event ((unsigned long)omxhdr,vdec_msg->status_code,
                    OMX_COMPONENT_GENERATE_EBD);
            break;
        case VDEC_MSG_RESP_OUTPUT_FLUSHED:
        case VDEC_MSG_RESP_OUTPUT_BUFFER_DONE: {
           v4l2_buf_ptr = (v4l2_buffer*)vdec_msg->msgdata.output_frame.client_data;

           OMX_BUFFERHEADERTYPE  *omx_base_address = omx->m_out_mem_ptr;
           vdec_bufferpayload *omx_ptr_outputbuffer = omx->drv_ctx.ptr_outputbuffer;
           vdec_output_frameinfo *omx_ptr_respbuffer = omx->drv_ctx.ptr_respbuffer;

           if (omx->client_buffers.is_color_conversion_enabled()) {
               omx_base_address = omx->m_intermediate_out_mem_ptr;
               omx_ptr_outputbuffer = omx->drv_ctx.ptr_intermediate_outputbuffer;
               omx_ptr_respbuffer = omx->drv_ctx.ptr_intermediate_respbuffer;
           }

           if (v4l2_buf_ptr == NULL || omx_base_address == NULL ||
               v4l2_buf_ptr->index >= omx->drv_ctx.op_buf.actualcount) {
               omxhdr = NULL;
               vdec_msg->status_code = VDEC_S_EFATAL;
               break;
           }
           plane = v4l2_buf_ptr->m.planes;
           omxhdr = omx_base_address + v4l2_buf_ptr->index;

           if (omxhdr && omxhdr->pOutputPortPrivate &&
               ((omxhdr - omx_base_address) < (int)omx->drv_ctx.op_buf.actualcount) &&
                   (((struct vdec_output_frameinfo *)omxhdr->pOutputPortPrivate
                     - omx_ptr_respbuffer) < (int)omx->drv_ctx.op_buf.actualcount)) {

               omxhdr->pMarkData = (OMX_PTR)(unsigned long)plane[0].reserved[MSM_VIDC_INPUT_TAG_1];
               omxhdr->hMarkTargetComponent = (OMX_HANDLETYPE)(unsigned long)plane[0].reserved[MSM_VIDC_INPUT_TAG_2];

               if (vdec_msg->msgdata.output_frame.len <=  omxhdr->nAllocLen) {
                   omxhdr->nFilledLen = vdec_msg->msgdata.output_frame.len;
               } else {
                   DEBUG_PRINT_ERROR("Invalid filled length = %u, set it as buffer size = %u",
                           (unsigned int)vdec_msg->msgdata.output_frame.len, omxhdr->nAllocLen);
                   omxhdr->nFilledLen = omxhdr->nAllocLen;
               }
                   omxhdr->nOffset = vdec_msg->msgdata.output_frame.offset;
                   omxhdr->nTimeStamp = vdec_msg->msgdata.output_frame.time_stamp;
                   omxhdr->nFlags = 0;

                   if (v4l2_buf_ptr->flags & V4L2_BUF_FLAG_EOS) {
                        omxhdr->nFlags |= OMX_BUFFERFLAG_EOS;
                        //rc = -1;
                   }
                   if (omxhdr->nFilledLen) {
                       omxhdr->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;
                   }
                   if (v4l2_buf_ptr->flags & V4L2_BUF_FLAG_READONLY) {
                        omxhdr->nFlags |= OMX_BUFFERFLAG_READONLY;
                        DEBUG_PRINT_LOW("F_B_D: READONLY BUFFER - REFERENCE WITH F/W fd = %d",
                                        omx_ptr_outputbuffer[v4l2_buf_ptr->index].pmem_fd);
                   }

                   if (v4l2_buf_ptr->flags & V4L2_BUF_FLAG_DATA_CORRUPT) {
                       omxhdr->nFlags |= OMX_BUFFERFLAG_DATACORRUPT;
                   }

                   output_respbuf = (struct vdec_output_frameinfo *)\
                            omxhdr->pOutputPortPrivate;
                   if (!output_respbuf) {
                     DEBUG_PRINT_ERROR("async_message_process: invalid output buf received");
                     return -1;
                   }
                   output_respbuf->len = vdec_msg->msgdata.output_frame.len;
                   output_respbuf->offset = vdec_msg->msgdata.output_frame.offset;

                   if (v4l2_buf_ptr->flags & V4L2_BUF_FLAG_KEYFRAME) {
                       output_respbuf->pic_type = PICTURE_TYPE_I;
                       omxhdr->nFlags |= OMX_BUFFERFLAG_SYNCFRAME;
                   }
                   if (v4l2_buf_ptr->flags & V4L2_BUF_FLAG_PFRAME) {
                       output_respbuf->pic_type = PICTURE_TYPE_P;
                   }
                   if (v4l2_buf_ptr->flags & V4L2_BUF_FLAG_BFRAME) {
                       output_respbuf->pic_type = PICTURE_TYPE_B;
                       omxhdr->nFlags |= QOMX_VIDEO_BUFFERFLAG_BFRAME;
                   }

                   if (v4l2_buf_ptr->flags & V4L2_BUF_FLAG_CODECCONFIG)
                       omxhdr->nFlags |= OMX_BUFFERFLAG_CODECCONFIG;

                   if (vdec_msg->msgdata.output_frame.len) {
                       DEBUG_PRINT_LOW("Processing extradata");
                       reconfig_event_sent = omx->handle_extradata(omxhdr);

                       if (omx->m_extradata_misr.output_crop_updated) {
                           DEBUG_PRINT_LOW("Read FBD crop from output extra data");
                           vdec_msg->msgdata.output_frame.framesize.left = omx->m_extradata_misr.output_crop_rect.nLeft;
                           vdec_msg->msgdata.output_frame.framesize.top = omx->m_extradata_misr.output_crop_rect.nTop;
                           vdec_msg->msgdata.output_frame.framesize.right = omx->m_extradata_misr.output_crop_rect.nWidth;
                           vdec_msg->msgdata.output_frame.framesize.bottom = omx->m_extradata_misr.output_crop_rect.nHeight;
                           vdec_msg->msgdata.output_frame.picsize.frame_width = omx->m_extradata_misr.output_width;
                           vdec_msg->msgdata.output_frame.picsize.frame_height = omx->m_extradata_misr.output_height;
                       }
                   }

                   vdec_msg->msgdata.output_frame.bufferaddr =
                       omx_ptr_outputbuffer[v4l2_buf_ptr->index].bufferaddr;

                   DEBUG_PRINT_LOW("[RespBufDone] Fd(%d) Buf(%p) Ts(%lld) PicType(%u) Flags (0x%x)"
                           " FillLen(%u) Crop: L(%u) T(%u) R(%u) B(%u)",
                           omx_ptr_outputbuffer[v4l2_buf_ptr->index].pmem_fd,
                           omxhdr, (long long)vdec_msg->msgdata.output_frame.time_stamp,
                           vdec_msg->msgdata.output_frame.pic_type, v4l2_buf_ptr->flags,
                           (unsigned int)vdec_msg->msgdata.output_frame.len,
                           vdec_msg->msgdata.output_frame.framesize.left,
                           vdec_msg->msgdata.output_frame.framesize.top,
                           vdec_msg->msgdata.output_frame.framesize.right,
                           vdec_msg->msgdata.output_frame.framesize.bottom);

                   /* Post event if resolution OR crop changed */
                   /* filled length will be changed if resolution changed */
                   /* Crop parameters can be changed even without resolution change */
                   if (omxhdr->nFilledLen
                       && ((omx->prev_n_filled_len != omxhdr->nFilledLen)
                       || (omx->drv_ctx.frame_size.left != vdec_msg->msgdata.output_frame.framesize.left)
                       || (omx->drv_ctx.frame_size.top != vdec_msg->msgdata.output_frame.framesize.top)
                       || (omx->drv_ctx.frame_size.right != vdec_msg->msgdata.output_frame.framesize.right)
                       || (omx->drv_ctx.frame_size.bottom != vdec_msg->msgdata.output_frame.framesize.bottom) )) {

                       DEBUG_PRINT_HIGH("Parameters Changed From: Len: %u, L: %u, T: %u, R: %u, B: %u --> Len: %u, L: %u, T: %u, R: %u, B: %u",
                               omx->prev_n_filled_len,
                               omx->drv_ctx.frame_size.left, omx->drv_ctx.frame_size.top,
                               omx->drv_ctx.frame_size.right, omx->drv_ctx.frame_size.bottom,
                               omxhdr->nFilledLen,
                               vdec_msg->msgdata.output_frame.framesize.left,
                               vdec_msg->msgdata.output_frame.framesize.top,
                               vdec_msg->msgdata.output_frame.framesize.right,
                               vdec_msg->msgdata.output_frame.framesize.bottom);

                       memcpy(&omx->drv_ctx.frame_size,
                               &vdec_msg->msgdata.output_frame.framesize,
                               sizeof(struct vdec_framesize));

                       if(!reconfig_event_sent) {
                           omx->post_event(OMX_CORE_OUTPUT_PORT_INDEX,
                                           OMX_IndexConfigCommonOutputCrop,
                                           OMX_COMPONENT_GENERATE_PORT_RECONFIG);
                           reconfig_event_sent = true;
                       }
                       /* Update C2D with new resolution */
                       omx->m_progressive = omx->drv_ctx.interlace == VDEC_InterlaceFrameProgressive;
                       if (!omx->client_buffers.update_buffer_req()) {
                           DEBUG_PRINT_ERROR("Setting C2D buffer requirements failed");
                       }
                   } else if (omxhdr->nFilledLen) {
                       /* Check whether interlaced info changed and update C2D */
                       int bProgressive = omx->drv_ctx.interlace == VDEC_InterlaceFrameProgressive;
                       if (omx->m_progressive != bProgressive) {
                           omx->m_progressive = bProgressive;
                           if (!omx->client_buffers.update_buffer_req()) {
                               DEBUG_PRINT_ERROR("Setting C2D buffer requirements failed");
                           }
                       }
                   }

                   if (omxhdr->nFilledLen)
                       omx->prev_n_filled_len = omxhdr->nFilledLen;

                   if (!omx->m_enable_android_native_buffers && omx->output_use_buffer && omxhdr->pBuffer &&
                       vdec_msg->msgdata.output_frame.bufferaddr)
                       memcpy ( omxhdr->pBuffer, (void *)
                               ((unsigned long)vdec_msg->msgdata.output_frame.bufferaddr +
                                (unsigned long)vdec_msg->msgdata.output_frame.offset),
                               vdec_msg->msgdata.output_frame.len);

               omx->post_event ((unsigned long)omxhdr, vdec_msg->status_code,
                        OMX_COMPONENT_GENERATE_FBD);

            } else if (vdec_msg->msgdata.output_frame.flags & OMX_BUFFERFLAG_EOS) {
                omx->post_event ((unsigned long)NULL, vdec_msg->status_code,
                        OMX_COMPONENT_GENERATE_EOS_DONE);
            } else {
                omx->post_event ((unsigned int)NULL, vdec_msg->status_code,
                        OMX_COMPONENT_GENERATE_HARDWARE_ERROR);
            }
            break;
        }
        case VDEC_MSG_EVT_CONFIG_CHANGED:
            DEBUG_PRINT_HIGH("Port settings changed");
            omx->m_reconfig_width = vdec_msg->msgdata.output_frame.picsize.frame_width;
            omx->m_reconfig_height = vdec_msg->msgdata.output_frame.picsize.frame_height;
            omx->isPortReconfigInsufficient = vdec_msg->msgdata.output_frame.flags;
            omx->post_event (OMX_CORE_OUTPUT_PORT_INDEX, OMX_IndexParamPortDefinition,
                    OMX_COMPONENT_GENERATE_PORT_RECONFIG);
            break;
        default:
            break;
    }
    return rc;
}

#ifdef USE_GBM
bool omx_vdec::alloc_map_gbm_memory(OMX_U32 w,OMX_U32 h,int dev_fd,
                 struct vdec_gbm *op_buf_gbm_info, int flag)
{

    uint32 flags = GBM_BO_USE_SCANOUT | GBM_BO_USE_RENDERING;
    struct gbm_device *gbm = NULL;
    struct gbm_bo *bo = NULL;
    int bo_fd = -1, meta_fd = -1;
    if (!op_buf_gbm_info || dev_fd < 0 ) {
        DEBUG_PRINT_ERROR("Invalid arguments to alloc_map_ion_memory");
        return FALSE;
    }

    gbm = gbm_create_device(dev_fd);
    if (gbm == NULL) {
       DEBUG_PRINT_ERROR("create gbm device failed");
       return FALSE;
    } else {
       DEBUG_PRINT_LOW( "Successfully created gbm device");
    }
    if (drv_ctx.output_format == VDEC_YUV_FORMAT_NV12_UBWC)
       flags |= GBM_BO_USAGE_UBWC_ALIGNED_QTI;

    DEBUG_PRINT_LOW("create NV12 gbm_bo with width=%d, height=%d", w, h);
    bo = gbm_bo_create(gbm, w, h,GBM_FORMAT_NV12,
              flags);

    if (bo == NULL) {
      DEBUG_PRINT_ERROR("Create bo failed");
      gbm_device_destroy(gbm);
      return FALSE;
    }

    bo_fd = gbm_bo_get_fd(bo);
    if (bo_fd < 0) {
      DEBUG_PRINT_ERROR("Get bo fd failed");
      gbm_bo_destroy(bo);
      gbm_device_destroy(gbm);
      return FALSE;
    }

    gbm_perform(GBM_PERFORM_GET_METADATA_ION_FD, bo, &meta_fd);
    if (meta_fd < 0) {
      DEBUG_PRINT_ERROR("Get bo meta fd failed");
      gbm_bo_destroy(bo);
      gbm_device_destroy(gbm);
      return FALSE;
    }
    op_buf_gbm_info->gbm = gbm;
    op_buf_gbm_info->bo = bo;
    op_buf_gbm_info->bo_fd = bo_fd;
    op_buf_gbm_info->meta_fd = meta_fd;

    DEBUG_PRINT_LOW("allocate gbm bo fd meta fd  %p %d %d",bo,bo_fd,meta_fd);
    return TRUE;
}

void omx_vdec::free_gbm_memory(struct vdec_gbm *buf_gbm_info)
{
    if(!buf_gbm_info) {
      DEBUG_PRINT_ERROR(" GBM: free called with invalid fd/allocdata");
      return;
    }
    DEBUG_PRINT_LOW("free gbm bo fd meta fd  %p %d %d",
           buf_gbm_info->bo,buf_gbm_info->bo_fd,buf_gbm_info->meta_fd);

    if (buf_gbm_info->bo)
       gbm_bo_destroy(buf_gbm_info->bo);
    buf_gbm_info->bo = NULL;

    if (buf_gbm_info->gbm)
       gbm_device_destroy(buf_gbm_info->gbm);
    buf_gbm_info->gbm = NULL;

    buf_gbm_info->bo_fd = -1;
    buf_gbm_info->meta_fd = -1;
}
#endif
#ifndef USE_ION
bool omx_vdec::align_pmem_buffers(int pmem_fd, OMX_U32 buffer_size,
        OMX_U32 alignment)
{
    struct pmem_allocation allocation;
    allocation.size = buffer_size;
    allocation.align = clip2(alignment);
    if (allocation.align < 4096) {
        allocation.align = 4096;
    }
    if (ioctl(pmem_fd, PMEM_ALLOCATE_ALIGNED, &allocation) < 0) {
        DEBUG_PRINT_ERROR("Aligment(%u) failed with pmem driver Sz(%lu)",
                allocation.align, allocation.size);
        return false;
    }
    return true;
}
#endif
#ifdef USE_ION
bool omx_vdec::alloc_map_ion_memory(OMX_U32 buffer_size, vdec_ion *ion_info, int flag)

{
    int rc = -EINVAL;
    int ion_dev_flag;
    struct vdec_ion ion_buf_info;

    if (!ion_info || buffer_size <= 0) {
        DEBUG_PRINT_ERROR("Invalid arguments to alloc_map_ion_memory");
        return false;
    }

    ion_info->dev_fd = ion_open();
    if (ion_info->dev_fd < 0) {
        DEBUG_PRINT_ERROR("opening ion device failed with ion_fd = %d", ion_info->dev_fd);
        return false;
    }

    ion_info->alloc_data.flags = flag;
    ion_info->alloc_data.len = buffer_size;

    ion_info->alloc_data.heap_id_mask = ION_HEAP(ION_SYSTEM_HEAP_ID);
    if (secure_mode && (ion_info->alloc_data.flags & ION_FLAG_SECURE)) {
        ion_info->alloc_data.heap_id_mask = ION_HEAP(MEM_HEAP_ID);
    }

    /* Use secure display cma heap for obvious reasons. */
    if (ion_info->alloc_data.flags & ION_FLAG_CP_BITSTREAM) {
        ion_info->alloc_data.heap_id_mask |= ION_HEAP(ION_SECURE_DISPLAY_HEAP_ID);
    }

    rc = ion_alloc_fd(ion_info->dev_fd, ion_info->alloc_data.len, 0,
                      ion_info->alloc_data.heap_id_mask, ion_info->alloc_data.flags,
                      &ion_info->data_fd);

    if (rc || ion_info->data_fd < 0) {
        DEBUG_PRINT_ERROR("ION ALLOC memory failed");
        ion_close(ion_info->dev_fd);
        ion_info->data_fd = -1;
        ion_info->dev_fd = -1;
        return false;
    }

    DEBUG_PRINT_HIGH("Alloc ion memory: fd (dev:%d data:%d) len %d flags %#x mask %#x",
                     ion_info->dev_fd, ion_info->data_fd, (unsigned int)ion_info->alloc_data.len,
                     (unsigned int)ion_info->alloc_data.flags,
                     (unsigned int)ion_info->alloc_data.heap_id_mask);

    return true;
}

void omx_vdec::free_ion_memory(struct vdec_ion *buf_ion_info)
{

    if (!buf_ion_info) {
        DEBUG_PRINT_ERROR("ION: free called with invalid fd/allocdata");
        return;
    }

    DEBUG_PRINT_HIGH("Free ion memory: fd (dev:%d data:%d) len %d flags %#x mask %#x",
        buf_ion_info->dev_fd, buf_ion_info->data_fd,
        (unsigned int)buf_ion_info->alloc_data.len,
        (unsigned int)buf_ion_info->alloc_data.flags,
        (unsigned int)buf_ion_info->alloc_data.heap_id_mask);

    if (buf_ion_info->data_fd >= 0) {
        close(buf_ion_info->data_fd);
        buf_ion_info->data_fd = -1;
    }
    if (buf_ion_info->dev_fd >= 0) {
        ion_close(buf_ion_info->dev_fd);
        buf_ion_info->dev_fd = -1;
    }
}

#endif
void omx_vdec::free_output_buffer_header(bool intermediate)
{
    DEBUG_PRINT_HIGH("ALL output buffers are freed/released");
    output_use_buffer = false;
    ouput_egl_buffers = false;

    OMX_BUFFERHEADERTYPE  **omx_base_address =
        intermediate?&m_intermediate_out_mem_ptr:&m_out_mem_ptr;
    vdec_bufferpayload **omx_ptr_outputbuffer =
        intermediate?&drv_ctx.ptr_intermediate_outputbuffer:&drv_ctx.ptr_outputbuffer;
    vdec_output_frameinfo **omx_ptr_respbuffer =
        intermediate?&drv_ctx.ptr_intermediate_respbuffer:&drv_ctx.ptr_respbuffer;
    vdec_ion **omx_op_buf_ion_info =
        intermediate?&drv_ctx.op_intermediate_buf_ion_info:&drv_ctx.op_buf_ion_info;
#ifdef USE_GBM
    vdec_gbm **omx_op_buf_gbm_info =
        intermediate?&drv_ctx.op_intermediate_buf_gbm_info:&drv_ctx.op_buf_gbm_info;
#endif

    if (*omx_base_address) {
        free (*omx_base_address);
        *omx_base_address = NULL;
    }

    if (m_platform_list) {
        free(m_platform_list);
        m_platform_list = NULL;
    }

    if (*omx_ptr_respbuffer) {
        free (*omx_ptr_respbuffer);
        *omx_ptr_respbuffer = NULL;
    }
    if (*omx_ptr_outputbuffer) {
        free (*omx_ptr_outputbuffer);
        *omx_ptr_outputbuffer = NULL;
    }
#ifdef USE_GBM
    if (*omx_op_buf_gbm_info) {
        DEBUG_PRINT_LOW("Free o/p gbm context");
        free(*omx_op_buf_gbm_info);
        *omx_op_buf_gbm_info = NULL;
    }
    if (drv_ctx.gbm_device_fd >= 0) {
       DEBUG_PRINT_LOW("Close gbm device");
       close(drv_ctx.gbm_device_fd);
       drv_ctx.gbm_device_fd = -1;
    }

#elif defined USE_ION
    if (*omx_op_buf_ion_info) {
        DEBUG_PRINT_LOW("Free o/p ion context");
        free(*omx_op_buf_ion_info);
        *omx_op_buf_ion_info = NULL;
    }
#endif
    if (intermediate == false && client_buffers.is_color_conversion_enabled()) {
        free_output_buffer_header(true);
    }
}

void omx_vdec::free_input_buffer_header()
{
    input_use_buffer = false;
    if (m_inp_mem_ptr) {
        DEBUG_PRINT_LOW("Free input pmem Pointer area");
        free (m_inp_mem_ptr);
        m_inp_mem_ptr = NULL;
    }
    /* We just freed all the buffer headers, every thing in m_input_free_q,
     * m_input_pending_q, pdest_frame, and psource_frame is now invalid */
    while (m_input_free_q.m_size) {
        unsigned long address, p2, id;
        m_input_free_q.pop_entry(&address, &p2, &id);
    }
    while (m_input_pending_q.m_size) {
        unsigned long address, p2, id;
        m_input_pending_q.pop_entry(&address, &p2, &id);
    }
    pdest_frame = NULL;
    psource_frame = NULL;
    if (drv_ctx.ptr_inputbuffer) {
        DEBUG_PRINT_LOW("Free Driver Context pointer");
        free (drv_ctx.ptr_inputbuffer);
        drv_ctx.ptr_inputbuffer = NULL;
    }
#ifdef USE_ION
    if (drv_ctx.ip_buf_ion_info) {
        DEBUG_PRINT_LOW("Free ion context");
        free(drv_ctx.ip_buf_ion_info);
        drv_ctx.ip_buf_ion_info = NULL;
    }
#endif
}

void omx_vdec::free_output_extradata_buffer_header()
{
    if (m_client_output_extradata_mem_ptr) {
        DEBUG_PRINT_LOW("Free extradata pmem Pointer area");
        free(m_client_output_extradata_mem_ptr);
        m_client_output_extradata_mem_ptr = NULL;
    }
}

int omx_vdec::stream_off(OMX_U32 port)
{
    enum v4l2_buf_type btype;
    int rc = 0;
    enum v4l2_ports v4l2_port = OUTPUT_PORT;
    struct v4l2_requestbuffers bufreq;

    if (port == OMX_CORE_INPUT_PORT_INDEX) {
        btype = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        v4l2_port = OUTPUT_PORT;
    } else if (port == OMX_CORE_OUTPUT_PORT_INDEX) {
        btype = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        v4l2_port = CAPTURE_PORT;
    } else if (port == OMX_ALL) {
        int rc_input = stream_off(OMX_CORE_INPUT_PORT_INDEX);
        int rc_output = stream_off(OMX_CORE_OUTPUT_PORT_INDEX);

        if (!rc_input)
            return rc_input;
        else
            return rc_output;
    }

    if (!streaming[v4l2_port]) {
        // already streamed off, warn and move on
        DEBUG_PRINT_HIGH("Warning: Attempting to stream off on %d port,"
                " which is already streamed off", v4l2_port);
        return 0;
    }

    DEBUG_PRINT_HIGH("Streaming off %d port", v4l2_port);

    rc = ioctl(drv_ctx.video_driver_fd, VIDIOC_STREAMOFF, &btype);
    if (rc) {
        /*TODO: How to handle this case */
        DEBUG_PRINT_ERROR("Failed to call streamoff on %d Port", v4l2_port);
    } else {
        streaming[v4l2_port] = false;
    }

    if (port == OMX_CORE_INPUT_PORT_INDEX) {
        bufreq.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    } else if (port == OMX_CORE_OUTPUT_PORT_INDEX) {
        bufreq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    }

    bufreq.memory = V4L2_MEMORY_USERPTR;
    bufreq.count = 0;
    rc = ioctl(drv_ctx.video_driver_fd,VIDIOC_REQBUFS, &bufreq);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed to release buffers on %d Port", v4l2_port);
    }
    return rc;
}

OMX_ERRORTYPE omx_vdec::get_buffer_req(vdec_allocatorproperty *buffer_prop)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    struct v4l2_control control;
    unsigned int buf_size = 0, extra_data_size = 0, default_extra_data_size = 0;
    unsigned int final_extra_data_size = 0;
    struct v4l2_format fmt;
    int ret = 0;
    DEBUG_PRINT_LOW("GetBufReq IN: ActCnt(%d) Size(%u)",
            buffer_prop->actualcount, (unsigned int)buffer_prop->buffer_size);

    if (buffer_prop->buffer_type == VDEC_BUFFER_TYPE_INPUT) {
        fmt.type =V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        fmt.fmt.pix_mp.pixelformat = output_capability;
        control.id = V4L2_CID_MIN_BUFFERS_FOR_OUTPUT;
    } else if (buffer_prop->buffer_type == VDEC_BUFFER_TYPE_OUTPUT) {
        fmt.type =V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.fmt.pix_mp.pixelformat = capture_capability;
        control.id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE;
    } else {
        eRet = OMX_ErrorBadParameter;
    }
    if (eRet == OMX_ErrorNone) {
        ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_G_CTRL, &control);
    }
    if (ret) {
        DEBUG_PRINT_ERROR("Requesting buffer requirements failed");
        /*TODO: How to handle this case */
        eRet = OMX_ErrorInsufficientResources;
        return eRet;
    }
    buffer_prop->actualcount = buffer_prop->mincount = control.value;
    DEBUG_PRINT_LOW("GetBufReq IN: ActCnt(%d) Size(%u)",
            buffer_prop->actualcount, (unsigned int)buffer_prop->buffer_size);

    ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_G_FMT, &fmt);

    if (fmt.type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
        drv_ctx.num_planes = fmt.fmt.pix_mp.num_planes;
    DEBUG_PRINT_HIGH("Buffer Size = %d, type = %d",fmt.fmt.pix_mp.plane_fmt[0].sizeimage, fmt.type);

    if (ret) {
        /*TODO: How to handle this case */
        DEBUG_PRINT_ERROR("Requesting buffer requirements failed");
        eRet = OMX_ErrorInsufficientResources;
    } else {
        int extra_idx = 0;

        buffer_prop->buffer_size = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
        buf_size = buffer_prop->buffer_size;
        extra_idx = EXTRADATA_IDX(drv_ctx.num_planes);
        if (extra_idx && (extra_idx < VIDEO_MAX_PLANES)) {
            extra_data_size =  fmt.fmt.pix_mp.plane_fmt[extra_idx].sizeimage;
        } else if (extra_idx >= VIDEO_MAX_PLANES) {
            DEBUG_PRINT_ERROR("Extradata index is more than allowed: %d", extra_idx);
            return OMX_ErrorBadParameter;
        }

        default_extra_data_size = VENUS_EXTRADATA_SIZE(
                drv_ctx.video_resolution.frame_height,
                drv_ctx.video_resolution.frame_width);
        final_extra_data_size = extra_data_size > default_extra_data_size ?
            extra_data_size : default_extra_data_size;

        final_extra_data_size = (final_extra_data_size + buffer_prop->alignment - 1) &
            (~(buffer_prop->alignment - 1));

        drv_ctx.extradata_info.count = buffer_prop->actualcount;
        drv_ctx.extradata_info.buffer_size = final_extra_data_size;
        buf_size = (buf_size + buffer_prop->alignment - 1)&(~(buffer_prop->alignment - 1));
        DEBUG_PRINT_LOW("GetBufReq UPDATE: ActCnt(%d) Size(%u) BufSize(%d)",
                buffer_prop->actualcount, (unsigned int)buffer_prop->buffer_size, buf_size);
        if (extra_data_size)
            DEBUG_PRINT_LOW("GetBufReq UPDATE: extradata: count(%d) size(%lu)",
                drv_ctx.extradata_info.count, drv_ctx.extradata_info.buffer_size);

        if (in_reconfig) // BufReq will be set to driver when port is disabled
            buffer_prop->buffer_size = buf_size;
        else if (buf_size != buffer_prop->buffer_size) {
            buffer_prop->buffer_size = buf_size;
            eRet = set_buffer_req(buffer_prop);
        }
    }
    DEBUG_PRINT_LOW("GetBufReq OUT: ActCnt(%d) Size(%u)",
            buffer_prop->actualcount, (unsigned int)buffer_prop->buffer_size);
    return eRet;
}

OMX_ERRORTYPE omx_vdec::set_buffer_req(vdec_allocatorproperty *buffer_prop)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    unsigned buf_size = 0;
    struct v4l2_format fmt, c_fmt;
    struct v4l2_requestbuffers bufreq;
    int ret = 0;
    DEBUG_PRINT_LOW("SetBufReq IN: ActCnt(%d) Size(%u)",
            buffer_prop->actualcount, (unsigned int)buffer_prop->buffer_size);
    buf_size = (buffer_prop->buffer_size + buffer_prop->alignment - 1)&(~(buffer_prop->alignment - 1));
    if (buf_size != buffer_prop->buffer_size) {
        DEBUG_PRINT_ERROR("Buffer size alignment error: Requested(%u) Required(%d)",
                (unsigned int)buffer_prop->buffer_size, buf_size);
        eRet = OMX_ErrorBadParameter;
    } else {
        memset(&fmt, 0x0, sizeof(struct v4l2_format));
        memset(&c_fmt, 0x0, sizeof(struct v4l2_format));
        fmt.fmt.pix_mp.height = drv_ctx.video_resolution.frame_height;
        fmt.fmt.pix_mp.width = drv_ctx.video_resolution.frame_width;
        fmt.fmt.pix_mp.plane_fmt[0].sizeimage = buf_size;

        if (buffer_prop->buffer_type == VDEC_BUFFER_TYPE_INPUT) {
            fmt.type =V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
            fmt.fmt.pix_mp.pixelformat = output_capability;
            DEBUG_PRINT_LOW("S_FMT: type %d wxh %dx%d size %d format %x",
                fmt.type, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
                fmt.fmt.pix_mp.plane_fmt[0].sizeimage, fmt.fmt.pix_mp.pixelformat);
            ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_S_FMT, &fmt);
        } else if (buffer_prop->buffer_type == VDEC_BUFFER_TYPE_OUTPUT) {
            c_fmt.type =V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            c_fmt.fmt.pix_mp.pixelformat = capture_capability;
            ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_G_FMT, &c_fmt);
            c_fmt.fmt.pix_mp.plane_fmt[0].sizeimage = buf_size;
            DEBUG_PRINT_LOW("S_FMT: type %d wxh %dx%d size %d format %x",
                c_fmt.type, c_fmt.fmt.pix_mp.width, c_fmt.fmt.pix_mp.height,
                c_fmt.fmt.pix_mp.plane_fmt[0].sizeimage, c_fmt.fmt.pix_mp.pixelformat);
            ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_S_FMT, &c_fmt);
        } else {
            eRet = OMX_ErrorBadParameter;
        }
        if (ret) {
            DEBUG_PRINT_ERROR("Setting buffer requirements (format) failed %d", ret);
            eRet = OMX_ErrorInsufficientResources;
        }

        bufreq.memory = V4L2_MEMORY_USERPTR;
        bufreq.count = buffer_prop->actualcount;
        if (buffer_prop->buffer_type == VDEC_BUFFER_TYPE_INPUT) {
            bufreq.type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        } else if (buffer_prop->buffer_type == VDEC_BUFFER_TYPE_OUTPUT) {
            bufreq.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        } else {
            eRet = OMX_ErrorBadParameter;
        }

        if (eRet == OMX_ErrorNone) {
            DEBUG_PRINT_LOW("REQBUFS: type %d count %d", bufreq.type, bufreq.count);
            ret = ioctl(drv_ctx.video_driver_fd,VIDIOC_REQBUFS, &bufreq);
        }

        if (ret) {
            DEBUG_PRINT_ERROR("Setting buffer requirements (reqbufs) failed %d", ret);
            /*TODO: How to handle this case */
            eRet = OMX_ErrorInsufficientResources;
        } else if (bufreq.count < buffer_prop->actualcount) {
            DEBUG_PRINT_ERROR("Driver refused to change the number of buffers"
                    " on v4l2 port %d to %d (prefers %d)", bufreq.type,
                    buffer_prop->actualcount, bufreq.count);
            eRet = OMX_ErrorInsufficientResources;
        } else {
            if (!client_buffers.update_buffer_req()) {
                DEBUG_PRINT_ERROR("Setting c2D buffer requirements failed");
                eRet = OMX_ErrorInsufficientResources;
            }
        }
    }
    return eRet;
}

OMX_ERRORTYPE omx_vdec::update_picture_resolution()
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    return eRet;
}

void omx_vdec::fix_drv_output_format()
{
    bool is_dest_format_non_ubwc = (capture_capability != V4L2_PIX_FMT_NV12_UBWC &&
                                    capture_capability != V4L2_PIX_FMT_NV12_TP10_UBWC);

    if (is_dest_format_non_ubwc){
        if (dpb_bit_depth == MSM_VIDC_BIT_DEPTH_10) {
            if(is_flexible_format){ // if flexible formats are expected, P010 is set for 10bit cases here
                drv_ctx.output_format = VDEC_YUV_FORMAT_P010_VENUS;
                capture_capability = V4L2_PIX_FMT_SDE_Y_CBCR_H2V2_P010_VENUS;
            }
        } else {
            drv_ctx.output_format = VDEC_YUV_FORMAT_NV12;
            capture_capability = V4L2_PIX_FMT_NV12;
        }
    } else {
        if (dpb_bit_depth == MSM_VIDC_BIT_DEPTH_10) {
            drv_ctx.output_format = VDEC_YUV_FORMAT_NV12_TP10_UBWC;
            capture_capability = V4L2_PIX_FMT_NV12_TP10_UBWC;
        } else {
            // 8 bit depth uses the default.
            // from 10 bit to 8 bit resolution change case
            // we can't rely on default value. we need to re-init these
            drv_ctx.output_format = VDEC_YUV_FORMAT_NV12_UBWC;
            capture_capability = V4L2_PIX_FMT_NV12_UBWC;
        }
    }
    DEBUG_PRINT_LOW("%s: Drv output format %#X Capture capability %#X",
                    __func__,
                    drv_ctx.output_format,
                    capture_capability);

}

OMX_ERRORTYPE omx_vdec::update_portdef(OMX_PARAM_PORTDEFINITIONTYPE *portDefn)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    struct v4l2_format fmt;
    struct v4l2_control control;
    if (!portDefn) {
        DEBUG_PRINT_ERROR("update_portdef: invalid params");
        return OMX_ErrorBadParameter;
    }
    portDefn->nVersion.nVersion = OMX_SPEC_VERSION;
    portDefn->nSize = sizeof(OMX_PARAM_PORTDEFINITIONTYPE);
    portDefn->eDomain    = OMX_PortDomainVideo;
    memset(&fmt, 0x0, sizeof(struct v4l2_format));
    if (0 == portDefn->nPortIndex) {
        int ret = 0;
        fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        fmt.fmt.pix_mp.pixelformat = output_capability;
        ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_G_FMT, &fmt);
        if (ret) {
            DEBUG_PRINT_ERROR("Get Resolution failed");
            return OMX_ErrorHardware;
        }
        drv_ctx.ip_buf.buffer_size = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;

        control.id = V4L2_CID_MIN_BUFFERS_FOR_OUTPUT;
        ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_G_CTRL, &control);
        if (ret) {
            DEBUG_PRINT_ERROR("Get input buffer count failed");
            return OMX_ErrorHardware;
        }
        drv_ctx.ip_buf.mincount = control.value;
        // update actualcount only if it is small than mincount
        if (drv_ctx.ip_buf.actualcount < drv_ctx.ip_buf.mincount){
             DEBUG_PRINT_LOW("Updated actualcount same as mincount");
             drv_ctx.ip_buf.actualcount = drv_ctx.ip_buf.mincount;
        }

        portDefn->eDir =  OMX_DirInput;
        portDefn->nBufferCountActual = drv_ctx.ip_buf.actualcount;
        portDefn->nBufferCountMin    = drv_ctx.ip_buf.mincount;
        portDefn->nBufferSize        = drv_ctx.ip_buf.buffer_size;
        portDefn->format.video.eColorFormat = OMX_COLOR_FormatUnused;
        portDefn->format.video.eCompressionFormat = eCompressionFormat;
        //for input port, always report the fps value set by client,
        //to distinguish whether client got valid fps from parser.
        portDefn->format.video.xFramerate = m_fps_received;
        portDefn->bEnabled   = m_inp_bEnabled;
        portDefn->bPopulated = m_inp_bPopulated;
    } else if (1 == portDefn->nPortIndex) {
        unsigned int buf_size = 0;
        int ret = 0;
        fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_G_FMT, &fmt);
        /* Check the return value */

        fmt.fmt.pix_mp.pixelformat = capture_capability;
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_S_FMT, &fmt);
        /* Check the return value */

        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.fmt.pix_mp.pixelformat = capture_capability;
        ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_G_FMT, &fmt);
        if (ret) {
            DEBUG_PRINT_ERROR("Get Resolution failed");
            return OMX_ErrorHardware;
        }
        drv_ctx.op_buf.buffer_size = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
        if (!client_buffers.update_buffer_req()) {
            DEBUG_PRINT_ERROR("client_buffers.update_buffer_req Failed");
            return OMX_ErrorHardware;
        }

        if (!client_buffers.get_buffer_req(buf_size)) {
            DEBUG_PRINT_ERROR("update buffer requirements");
            return OMX_ErrorHardware;
        }
        portDefn->nBufferSize = buf_size;
        portDefn->eDir =  OMX_DirOutput;
        portDefn->nBufferCountActual = drv_ctx.op_buf.actualcount;
        portDefn->nBufferCountMin    = drv_ctx.op_buf.mincount;
        portDefn->format.video.eCompressionFormat = OMX_VIDEO_CodingUnused;
        if (drv_ctx.frame_rate.fps_denominator > 0)
            portDefn->format.video.xFramerate = (drv_ctx.frame_rate.fps_numerator /
                drv_ctx.frame_rate.fps_denominator) << 16; //Q16 format
        else {
            DEBUG_PRINT_ERROR("Error: Divide by zero");
            return OMX_ErrorBadParameter;
        }
        portDefn->bEnabled   = m_out_bEnabled;
        portDefn->bPopulated = m_out_bPopulated;
        if (!client_buffers.get_color_format(portDefn->format.video.eColorFormat)) {
            DEBUG_PRINT_ERROR("Error in getting color format");
            return OMX_ErrorHardware;
        }
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.fmt.pix_mp.pixelformat = capture_capability;
    } else if (OMX_CORE_OUTPUT_EXTRADATA_INDEX == portDefn->nPortIndex) {
        portDefn->nBufferSize = m_client_out_extradata_info.getSize();
        portDefn->nBufferCountMin = MIN_NUM_INPUT_OUTPUT_EXTRADATA_BUFFERS;
        portDefn->nBufferCountActual = MIN_NUM_INPUT_OUTPUT_EXTRADATA_BUFFERS;
        portDefn->eDir =  OMX_DirOutput;
        portDefn->format.video.nFrameHeight =  drv_ctx.video_resolution.frame_height;
        portDefn->format.video.nFrameWidth  =  drv_ctx.video_resolution.frame_width;
        portDefn->format.video.nStride  = drv_ctx.video_resolution.stride;
        portDefn->format.video.nSliceHeight = drv_ctx.video_resolution.scan_lines;
        portDefn->format.video.eCompressionFormat = OMX_VIDEO_CodingUnused;
        portDefn->format.video.eColorFormat = OMX_COLOR_FormatUnused;
        DEBUG_PRINT_LOW(" get_parameter: Port idx %d nBufSize %u nBufCnt %u",
                (int)portDefn->nPortIndex,
                (unsigned int)portDefn->nBufferSize,
                (unsigned int)portDefn->nBufferCountActual);
        return eRet;
    } else {
        portDefn->eDir = OMX_DirMax;
        DEBUG_PRINT_LOW(" get_parameter: Bad Port idx %d",
                (int)portDefn->nPortIndex);
        eRet = OMX_ErrorBadPortIndex;
    }
    if (in_reconfig) {
        m_extradata_misr.output_crop_rect.nLeft = 0;
        m_extradata_misr.output_crop_rect.nTop = 0;
        m_extradata_misr.output_crop_rect.nWidth = fmt.fmt.pix_mp.width;
        m_extradata_misr.output_crop_rect.nHeight = fmt.fmt.pix_mp.height;
    }
    update_resolution(fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
        fmt.fmt.pix_mp.plane_fmt[0].bytesperline, fmt.fmt.pix_mp.plane_fmt[0].reserved[0]);

    portDefn->format.video.nFrameHeight =  drv_ctx.video_resolution.frame_height;
    portDefn->format.video.nFrameWidth  =  drv_ctx.video_resolution.frame_width;
    portDefn->format.video.nStride = drv_ctx.video_resolution.stride;
    portDefn->format.video.nSliceHeight = drv_ctx.video_resolution.scan_lines;

    /* OMX client can do tone mapping/post processing, Hence, OMX should produce
       YUV buffers in case client needs it. */
    if (drv_ctx.idr_only_decoding && portDefn->format.video.eColorFormat == OMX_COLOR_Format16bitRGB565 &&
       ((dpb_bit_depth == MSM_VIDC_BIT_DEPTH_10 && (m_thumbnail_yuv_output & THUMBNAIL_YUV420P_10BIT)) ||
        (dpb_bit_depth == MSM_VIDC_BIT_DEPTH_8 && (m_thumbnail_yuv_output & THUMBNAIL_YUV420P_8BIT)))) {
        DEBUG_PRINT_LOW("Change color format to YUV420 planar for thumbnail usecase");
        portDefn->format.video.eColorFormat = OMX_COLOR_FormatYUV420Planar;
        if (!client_buffers.set_color_format(portDefn->format.video.eColorFormat)) {
            DEBUG_PRINT_ERROR("Set color format failed");
            eRet = OMX_ErrorBadParameter;
        }
    }
    if ((portDefn->format.video.eColorFormat == OMX_COLOR_FormatYUV420Planar) ||
       (portDefn->format.video.eColorFormat == OMX_COLOR_FormatYUV420SemiPlanar)) {
           portDefn->format.video.nStride = ALIGN(drv_ctx.video_resolution.frame_width, 16);
           portDefn->format.video.nSliceHeight = drv_ctx.video_resolution.frame_height;
    } else if (portDefn->format.video.eColorFormat == OMX_COLOR_Format16bitRGB565) {
           portDefn->format.video.nStride = ALIGN(drv_ctx.video_resolution.frame_width, 32);
           portDefn->format.video.nSliceHeight = drv_ctx.video_resolution.frame_height;
    }
    DEBUG_PRINT_HIGH("update_portdef(%u): Width = %u Height = %u Stride = %d "
            "SliceHeight = %u eColorFormat = %d nBufSize %u nBufCnt %u",
            (unsigned int)portDefn->nPortIndex,
            (unsigned int)portDefn->format.video.nFrameWidth,
            (unsigned int)portDefn->format.video.nFrameHeight,
            (int)portDefn->format.video.nStride,
            (unsigned int)portDefn->format.video.nSliceHeight,
            (unsigned int)portDefn->format.video.eColorFormat,
            (unsigned int)portDefn->nBufferSize,
            (unsigned int)portDefn->nBufferCountActual);

    return eRet;
}

OMX_ERRORTYPE omx_vdec::allocate_output_headers(bool intermediate)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    OMX_BUFFERHEADERTYPE *bufHdr = NULL;
    unsigned i = 0;

    OMX_BUFFERHEADERTYPE  **omx_base_address =
                      intermediate?&m_intermediate_out_mem_ptr:&m_out_mem_ptr;
    vdec_bufferpayload **omx_ptr_outputbuffer =
        intermediate?&drv_ctx.ptr_intermediate_outputbuffer:&drv_ctx.ptr_outputbuffer;
    vdec_output_frameinfo **omx_ptr_respbuffer =
        intermediate?&drv_ctx.ptr_intermediate_respbuffer:&drv_ctx.ptr_respbuffer;
    vdec_ion **omx_op_buf_ion_info =
        intermediate?&drv_ctx.op_intermediate_buf_ion_info:&drv_ctx.op_buf_ion_info;

    if (!*omx_base_address) {
        DEBUG_PRINT_HIGH("Use o/p buffer case - Header List allocation, Cnt %d Sz %d",
            drv_ctx.op_buf.actualcount, (unsigned int)drv_ctx.op_buf.buffer_size);
        int nBufHdrSize        = 0;
        int nPlatformEntrySize = 0;
        int nPlatformListSize  = 0;
        int nPMEMInfoSize = 0;
        OMX_QCOM_PLATFORM_PRIVATE_LIST      *pPlatformList;
        OMX_QCOM_PLATFORM_PRIVATE_ENTRY     *pPlatformEntry;
        OMX_QCOM_PLATFORM_PRIVATE_PMEM_INFO *pPMEMInfo;

        nBufHdrSize        = drv_ctx.op_buf.actualcount *
            sizeof(OMX_BUFFERHEADERTYPE);
        nPMEMInfoSize      = drv_ctx.op_buf.actualcount *
            sizeof(OMX_QCOM_PLATFORM_PRIVATE_PMEM_INFO);
        nPlatformListSize  = drv_ctx.op_buf.actualcount *
            sizeof(OMX_QCOM_PLATFORM_PRIVATE_LIST);
        nPlatformEntrySize = drv_ctx.op_buf.actualcount *
            sizeof(OMX_QCOM_PLATFORM_PRIVATE_ENTRY);

        *omx_base_address = (OMX_BUFFERHEADERTYPE  *)calloc(nBufHdrSize,1);
        // Alloc mem for platform specific info
        char *pPtr=NULL;
        pPtr = (char*) calloc(nPlatformListSize + nPlatformEntrySize +
                nPMEMInfoSize,1);
        *omx_ptr_outputbuffer = (struct vdec_bufferpayload *)    \
                       calloc (sizeof(struct vdec_bufferpayload),
                               drv_ctx.op_buf.actualcount);
        *omx_ptr_respbuffer = (struct vdec_output_frameinfo  *)                         \
                     calloc (sizeof (struct vdec_output_frameinfo),
                             drv_ctx.op_buf.actualcount);
        if (!pPtr || !*omx_ptr_outputbuffer || !*omx_ptr_respbuffer) {
            DEBUG_PRINT_ERROR("allocate_output_headers: allocation failed");
            free(pPtr); pPtr = NULL;
            free(*omx_ptr_outputbuffer); *omx_ptr_outputbuffer = NULL;
            free(*omx_ptr_respbuffer); *omx_ptr_respbuffer = NULL;
            return OMX_ErrorInsufficientResources;
        }

#ifdef USE_ION
        *omx_op_buf_ion_info = (struct vdec_ion * ) \
                      calloc (sizeof(struct vdec_ion),drv_ctx.op_buf.actualcount);
        if (!*omx_op_buf_ion_info) {
            DEBUG_PRINT_ERROR("Failed to alloc output buffer ion info");
            free(pPtr); pPtr = NULL;
            free(*omx_ptr_outputbuffer); *omx_ptr_outputbuffer = NULL;
            free(*omx_ptr_respbuffer); *omx_ptr_respbuffer = NULL;
            return OMX_ErrorInsufficientResources;
        }
#endif

        if (*omx_base_address && pPtr && *omx_ptr_outputbuffer
            && *omx_ptr_respbuffer) {
            bufHdr          =  *omx_base_address;
            if (m_platform_list) {
                free(m_platform_list);
            }
            m_platform_list = (OMX_QCOM_PLATFORM_PRIVATE_LIST *)(pPtr);
            m_platform_entry= (OMX_QCOM_PLATFORM_PRIVATE_ENTRY *)
                (((char *) m_platform_list)  + nPlatformListSize);
            m_pmem_info     = (OMX_QCOM_PLATFORM_PRIVATE_PMEM_INFO *)
                (((char *) m_platform_entry) + nPlatformEntrySize);
            pPlatformList   = m_platform_list;
            pPlatformEntry  = m_platform_entry;
            pPMEMInfo       = m_pmem_info;

            DEBUG_PRINT_LOW("Memory Allocation Succeeded for OUT port%p", *omx_base_address);

            // Settting the entire storage nicely
            DEBUG_PRINT_LOW("bHdr %p OutMem %p PE %p",bufHdr,
                            *omx_base_address,pPlatformEntry);
            DEBUG_PRINT_LOW(" Pmem Info = %p",pPMEMInfo);
            for (i=0; i < drv_ctx.op_buf.actualcount ; i++) {
                bufHdr->nSize              = sizeof(OMX_BUFFERHEADERTYPE);
                bufHdr->nVersion.nVersion  = OMX_SPEC_VERSION;
                // Set the values when we determine the right HxW param
                bufHdr->nAllocLen          = 0;
                bufHdr->nFilledLen         = 0;
                bufHdr->pAppPrivate        = NULL;
                bufHdr->nOutputPortIndex   = OMX_CORE_OUTPUT_PORT_INDEX;
                pPlatformEntry->type       = OMX_QCOM_PLATFORM_PRIVATE_PMEM;
                pPlatformEntry->entry      = pPMEMInfo;
                // Initialize the Platform List
                pPlatformList->nEntries    = 1;
                pPlatformList->entryList   = pPlatformEntry;
                // Keep pBuffer NULL till vdec is opened
                bufHdr->pBuffer            = NULL;
                pPMEMInfo->offset          =  0;
                pPMEMInfo->pmem_fd = -1;
                bufHdr->pPlatformPrivate = pPlatformList;
                (*omx_ptr_outputbuffer)[i].pmem_fd = -1;
#ifdef USE_ION
                (*omx_op_buf_ion_info)[i].data_fd = -1;
                (*omx_op_buf_ion_info)[i].dev_fd = -1;
#endif
                /*Create a mapping between buffers*/
                bufHdr->pOutputPortPrivate = &(*omx_ptr_respbuffer)[i];
                (*omx_ptr_respbuffer)[i].client_data = (void *) \
                    &(*omx_ptr_outputbuffer)[i];
                // Move the buffer and buffer header pointers
                bufHdr++;
                pPMEMInfo++;
                pPlatformEntry++;
                pPlatformList++;
            }
        } else {
            DEBUG_PRINT_ERROR("Output buf mem alloc failed[0x%p][0x%p]",\
                              *omx_base_address, pPtr);
            if (*omx_base_address) {
                free(*omx_base_address);
                *omx_base_address = NULL;
            }
            if (pPtr) {
                free(pPtr);
                pPtr = NULL;
            }
            if (*omx_ptr_outputbuffer) {
                free(*omx_ptr_outputbuffer);
                *omx_ptr_outputbuffer = NULL;
            }
            if (*omx_ptr_respbuffer) {
                free(*omx_ptr_respbuffer);
                *omx_ptr_respbuffer = NULL;
            }
#ifdef USE_ION
            if (*omx_op_buf_ion_info) {
                DEBUG_PRINT_LOW("Free o/p ion context");
                free(*omx_op_buf_ion_info);
                *omx_op_buf_ion_info = NULL;
            }
#endif
            eRet =  OMX_ErrorInsufficientResources;
        }
    } else {
        eRet =  OMX_ErrorInsufficientResources;
    }

    if (intermediate == false &&
        eRet == OMX_ErrorNone &&
        client_buffers.is_color_conversion_enabled()) {
        eRet = allocate_output_headers(true);
    }

    return eRet;
}

void omx_vdec::complete_pending_buffer_done_cbs()
{
    unsigned long p1, p2, ident;
    omx_cmd_queue tmp_q, pending_bd_q;
    pthread_mutex_lock(&m_lock);
    // pop all pending GENERATE FDB from ftb queue
    while (m_ftb_q.m_size) {
        m_ftb_q.pop_entry(&p1,&p2,&ident);
        if (ident == OMX_COMPONENT_GENERATE_FBD) {
            pending_bd_q.insert_entry(p1,p2,ident);
        } else {
            tmp_q.insert_entry(p1,p2,ident);
        }
    }
    //return all non GENERATE FDB to ftb queue
    while (tmp_q.m_size) {
        tmp_q.pop_entry(&p1,&p2,&ident);
        m_ftb_q.insert_entry(p1,p2,ident);
    }
    // pop all pending GENERATE EDB from etb queue
    while (m_etb_q.m_size) {
        m_etb_q.pop_entry(&p1,&p2,&ident);
        if (ident == OMX_COMPONENT_GENERATE_EBD) {
            pending_bd_q.insert_entry(p1,p2,ident);
        } else {
            tmp_q.insert_entry(p1,p2,ident);
        }
    }
    //return all non GENERATE FDB to etb queue
    while (tmp_q.m_size) {
        tmp_q.pop_entry(&p1,&p2,&ident);
        m_etb_q.insert_entry(p1,p2,ident);
    }
    pthread_mutex_unlock(&m_lock);
    // process all pending buffer dones
    while (pending_bd_q.m_size) {
        pending_bd_q.pop_entry(&p1,&p2,&ident);
        switch (ident) {
            case OMX_COMPONENT_GENERATE_EBD:
                if (empty_buffer_done(&m_cmp, (OMX_BUFFERHEADERTYPE *)p1) != OMX_ErrorNone) {
                    DEBUG_PRINT_ERROR("ERROR: empty_buffer_done() failed!");
                    omx_report_error ();
                }
                break;

            case OMX_COMPONENT_GENERATE_FBD:
                if (fill_buffer_done(&m_cmp, (OMX_BUFFERHEADERTYPE *)p1) != OMX_ErrorNone ) {
                    DEBUG_PRINT_ERROR("ERROR: fill_buffer_done() failed!");
                    omx_report_error ();
                }
                break;
        }
    }
}

void omx_vdec::set_frame_rate(OMX_S64 act_timestamp)
{
    OMX_U32 new_frame_interval = 0;
    if (VALID_TS(act_timestamp) && VALID_TS(prev_ts) && act_timestamp != prev_ts
            && (llabs(act_timestamp - prev_ts) > 2000 || frm_int == 0)) {
        new_frame_interval = client_set_fps ? frm_int : llabs(act_timestamp - prev_ts);
        if (new_frame_interval != frm_int || frm_int == 0) {
            frm_int = new_frame_interval;
            if (frm_int) {
                drv_ctx.frame_rate.fps_numerator = 1e6;
                drv_ctx.frame_rate.fps_denominator = frm_int;
                DEBUG_PRINT_LOW("set_frame_rate: frm_int(%u) fps(%f)",
                        (unsigned int)frm_int, drv_ctx.frame_rate.fps_numerator /
                        (float)drv_ctx.frame_rate.fps_denominator);
                /* We need to report the difference between this FBD and the previous FBD
                 * back to the driver for clock scaling purposes. */
                struct v4l2_control control;

                control.id =  V4L2_CID_MPEG_VIDC_VIDEO_FRAME_RATE;
                control.value = drv_ctx.frame_rate.fps_numerator / drv_ctx.frame_rate.fps_denominator;
                control.value <<= 16;
                control.value |= (0x0000FFFF & (drv_ctx.frame_rate.fps_numerator % drv_ctx.frame_rate.fps_denominator));
                DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
                if (ioctl(drv_ctx.video_driver_fd, VIDIOC_S_CTRL, &control)) {
                   DEBUG_PRINT_ERROR("Unable to convey fps info to driver, \
                            performance might be affected");
                }
            }
        }
    }
    prev_ts = act_timestamp;
}

void omx_vdec::convert_color_space_info(OMX_U32 primaries,
    OMX_U32 transfer, OMX_U32 matrix, ColorAspects *aspects)
{
    switch (primaries) {
        case MSM_VIDC_BT709_5:
            aspects->mPrimaries = ColorAspects::PrimariesBT709_5;
            break;
        case MSM_VIDC_BT470_6_M:
            aspects->mPrimaries = ColorAspects::PrimariesBT470_6M;
            break;
        case MSM_VIDC_BT601_6_625:
            aspects->mPrimaries = ColorAspects::PrimariesBT601_6_625;
            break;
        case MSM_VIDC_BT601_6_525:
            aspects->mPrimaries = ColorAspects::PrimariesBT601_6_525;
            break;
        case MSM_VIDC_GENERIC_FILM:
            aspects->mPrimaries = ColorAspects::PrimariesGenericFilm;
            break;
        case MSM_VIDC_BT2020:
            aspects->mPrimaries = ColorAspects::PrimariesBT2020;
            break;
        case MSM_VIDC_UNSPECIFIED:
            //Client does not expect ColorAspects::PrimariesUnspecified, but rather the supplied default
        default:
            //aspects->mPrimaries = ColorAspects::PrimariesOther;
            aspects->mPrimaries = m_client_color_space.sAspects.mPrimaries;
            break;
    }

    switch (transfer) {
        case MSM_VIDC_TRANSFER_BT709_5:
        case MSM_VIDC_TRANSFER_601_6_525: // case MSM_VIDC_TRANSFER_601_6_625:
        case MSM_VIDC_TRANSFER_BT_2020_10:
        case MSM_VIDC_TRANSFER_BT_2020_12:
            aspects->mTransfer = ColorAspects::TransferSMPTE170M;
            break;
        case MSM_VIDC_TRANSFER_BT_470_6_M:
            aspects->mTransfer = ColorAspects::TransferGamma22;
            break;
        case MSM_VIDC_TRANSFER_BT_470_6_BG:
            aspects->mTransfer = ColorAspects::TransferGamma28;
            break;
        case MSM_VIDC_TRANSFER_SMPTE_240M:
            aspects->mTransfer = ColorAspects::TransferSMPTE240M;
            break;
        case MSM_VIDC_TRANSFER_LINEAR:
            aspects->mTransfer = ColorAspects::TransferLinear;
            break;
        case MSM_VIDC_TRANSFER_IEC_61966:
            aspects->mTransfer = ColorAspects::TransferXvYCC;
            break;
        case MSM_VIDC_TRANSFER_BT_1361:
            aspects->mTransfer = ColorAspects::TransferBT1361;
            break;
        case MSM_VIDC_TRANSFER_SRGB:
            aspects->mTransfer = ColorAspects::TransferSRGB;
            break;
        case MSM_VIDC_TRANSFER_SMPTE_ST2084:
            aspects->mTransfer = ColorAspects::TransferST2084;
            break;
        case MSM_VIDC_TRANSFER_HLG:
            aspects->mTransfer = ColorAspects::TransferHLG;
            break;
        default:
            //aspects->mTransfer = ColorAspects::TransferOther;
            aspects->mTransfer = m_client_color_space.sAspects.mTransfer;
            break;
    }

    switch (matrix) {
        case MSM_VIDC_MATRIX_BT_709_5:
            aspects->mMatrixCoeffs = ColorAspects::MatrixBT709_5;
            break;
        case MSM_VIDC_MATRIX_FCC_47:
            aspects->mMatrixCoeffs = ColorAspects::MatrixBT470_6M;
            break;
        case MSM_VIDC_MATRIX_601_6_625:
        case MSM_VIDC_MATRIX_601_6_525:
            aspects->mMatrixCoeffs = ColorAspects::MatrixBT601_6;
            break;
        case MSM_VIDC_MATRIX_SMPTE_240M:
            aspects->mMatrixCoeffs = ColorAspects::MatrixSMPTE240M;
            break;
        case MSM_VIDC_MATRIX_BT_2020:
            aspects->mMatrixCoeffs = ColorAspects::MatrixBT2020;
            break;
        case MSM_VIDC_MATRIX_BT_2020_CONST:
            aspects->mMatrixCoeffs = ColorAspects::MatrixBT2020Constant;
            break;
        default:
            //aspects->mMatrixCoeffs = ColorAspects::MatrixOther;
            aspects->mMatrixCoeffs = m_client_color_space.sAspects.mMatrixCoeffs;
            break;
    }
}

bool omx_vdec::handle_color_space_info(void *data)
{
    ColorAspects tempAspects;
    memset(&tempAspects, 0x0, sizeof(ColorAspects));
    ColorAspects *aspects = &tempAspects;

    switch(output_capability) {
        case V4L2_PIX_FMT_MPEG2:
            {
                struct msm_vidc_mpeg2_seqdisp_payload *seqdisp_payload;
                seqdisp_payload = (struct msm_vidc_mpeg2_seqdisp_payload *)data;

                /* Refer MPEG2 Spec @ Rec. ISO/IEC 13818-2, ITU-T Draft Rec. H.262 to
                 * understand this code */

                if (seqdisp_payload && seqdisp_payload->color_descp) {

                    convert_color_space_info(seqdisp_payload->color_primaries,
                            seqdisp_payload->transfer_char, seqdisp_payload->matrix_coeffs,
                            aspects);
                    /* MPEG2 seqdisp payload doesn't give range info. Hence assing the value
                     * set by client */
                    aspects->mRange = m_client_color_space.sAspects.mRange;
                    m_disp_hor_size = seqdisp_payload->disp_width;
                    m_disp_vert_size = seqdisp_payload->disp_height;
                }
            }
            break;
        case V4L2_PIX_FMT_H264:
        case V4L2_PIX_FMT_HEVC:
            {
                struct msm_vidc_vui_display_info_payload *display_info_payload;
                OMX_U32 range;
                display_info_payload = (struct msm_vidc_vui_display_info_payload*)data;

                /* Refer H264 Spec @ Rec. ITU-T H.264 (02/2014) to understand this code */

                if (display_info_payload->video_signal_present_flag) {
                    range = display_info_payload->video_full_range_flag;
                    aspects->mRange = range ? ColorAspects::RangeFull : ColorAspects::RangeLimited;
                    if (display_info_payload->color_description_present_flag) {
                        convert_color_space_info(display_info_payload->color_primaries,
                                display_info_payload->transfer_char,
                                display_info_payload->matrix_coeffs,
                                aspects);
                    }
                }
            }
            break;
        case V4L2_PIX_FMT_VP8:
            {
                struct msm_vidc_vpx_colorspace_payload *vpx_color_space_payload;
                vpx_color_space_payload = (struct msm_vidc_vpx_colorspace_payload*)data;
                /* Refer VP8 Data Format in latest VP8 spec and Decoding Guide November 2011
                 * to understand this code */

                if (vpx_color_space_payload->color_space == 0) {
                    aspects->mPrimaries = ColorAspects::PrimariesBT601_6_525;
                    aspects->mRange = ColorAspects::RangeLimited;
                    aspects->mTransfer = ColorAspects::TransferSMPTE170M;
                    aspects->mMatrixCoeffs = ColorAspects::MatrixBT601_6;
                } else {
                    DEBUG_PRINT_ERROR("Unsupported Color space for VP8");
                    break;
                }
            }
            break;
        case V4L2_PIX_FMT_VP9:
            {
                struct msm_vidc_vpx_colorspace_payload *vpx_color_space_payload;
                vpx_color_space_payload = (struct msm_vidc_vpx_colorspace_payload*)data;
                /* Refer VP9 Spec @ VP9 Bitstream & Decoding Process Specification - v0.6 31st March 2016
                 * to understand this code */

                switch(vpx_color_space_payload->color_space) {
                    case MSM_VIDC_CS_BT_601:
                        aspects->mMatrixCoeffs = ColorAspects::MatrixBT601_6;
                        aspects->mTransfer = ColorAspects::TransferSMPTE170M;
                        aspects->mPrimaries = ColorAspects::PrimariesBT601_6_625;
                        aspects->mRange = m_client_color_space.sAspects.mRange;
                        break;
                    case MSM_VIDC_CS_BT_709:
                        aspects->mMatrixCoeffs = ColorAspects::MatrixBT709_5;
                        aspects->mTransfer = ColorAspects::TransferSMPTE170M;
                        aspects->mPrimaries =  ColorAspects::PrimariesBT709_5;
                        aspects->mRange = m_client_color_space.sAspects.mRange;
                        break;
                    case MSM_VIDC_CS_SMPTE_170:
                        aspects->mMatrixCoeffs = ColorAspects::MatrixBT601_6;
                        aspects->mTransfer = ColorAspects::TransferSMPTE170M;
                        aspects->mPrimaries =  ColorAspects::PrimariesBT601_6_525;
                        aspects->mRange = m_client_color_space.sAspects.mRange;
                        break;
                    case MSM_VIDC_CS_SMPTE_240:
                        aspects->mMatrixCoeffs = ColorAspects::MatrixSMPTE240M;
                        aspects->mTransfer = ColorAspects::TransferSMPTE240M;
                        aspects->mPrimaries =  ColorAspects::PrimariesBT601_6_525;
                        aspects->mRange = m_client_color_space.sAspects.mRange;
                        break;
                    case MSM_VIDC_CS_BT_2020:
                        aspects->mMatrixCoeffs = ColorAspects::MatrixBT2020;
                        aspects->mTransfer = ColorAspects:: TransferSMPTE170M;
                        aspects->mPrimaries = ColorAspects::PrimariesBT2020;
                        aspects->mRange = m_client_color_space.sAspects.mRange;
                        break;
                    case MSM_VIDC_CS_RESERVED:
                        aspects->mMatrixCoeffs = ColorAspects::MatrixOther;
                        aspects->mTransfer = ColorAspects::TransferOther;
                        aspects->mPrimaries = ColorAspects::PrimariesOther;
                        aspects->mRange = m_client_color_space.sAspects.mRange;
                        break;
                    case MSM_VIDC_CS_RGB:
                        aspects->mMatrixCoeffs = ColorAspects::MatrixBT709_5;
                        aspects->mTransfer = ColorAspects::TransferSMPTE170M;
                        aspects->mPrimaries = ColorAspects::PrimariesOther;
                        aspects->mRange = m_client_color_space.sAspects.mRange;
                        break;
                    default:
                        break;
                }
            }
            break;
        default:
            break;
    }

    print_debug_color_aspects(aspects, "Bitstream");

    if (m_internal_color_space.sAspects.mPrimaries != aspects->mPrimaries ||
            m_internal_color_space.sAspects.mTransfer != aspects->mTransfer ||
            m_internal_color_space.sAspects.mMatrixCoeffs != aspects->mMatrixCoeffs ||
            m_internal_color_space.sAspects.mRange != aspects->mRange) {
        memcpy(&(m_internal_color_space.sAspects), aspects, sizeof(ColorAspects));

        DEBUG_PRINT_HIGH("Initiating PORT Reconfig due to Color Aspects Change");
        print_debug_color_aspects(&(m_internal_color_space.sAspects), "Internal");
        print_debug_color_aspects(&(m_client_color_space.sAspects), "Client");

        post_event(OMX_CORE_OUTPUT_PORT_INDEX,
                OMX_QTIIndexConfigDescribeColorAspects,
                OMX_COMPONENT_GENERATE_PORT_RECONFIG);
        return true;
    }
    return false;
}

bool omx_vdec::handle_content_light_level_info(void* data)
{
    struct msm_vidc_content_light_level_sei_payload *light_level_payload =
        (msm_vidc_content_light_level_sei_payload*)(data);

    if ((m_internal_hdr_info.sInfo.sType1.mMaxContentLightLevel != light_level_payload->nMaxContentLight) ||
        (m_internal_hdr_info.sInfo.sType1.mMaxFrameAverageLightLevel != light_level_payload->nMaxPicAverageLight)) {
        m_internal_hdr_info.sInfo.sType1.mMaxContentLightLevel = light_level_payload->nMaxContentLight;
        m_internal_hdr_info.sInfo.sType1.mMaxFrameAverageLightLevel = light_level_payload->nMaxPicAverageLight;
        return true;
    }
    return false;
}

bool omx_vdec::handle_mastering_display_color_info(void* data)
{
    struct msm_vidc_mastering_display_colour_sei_payload *mastering_display_payload =
        (msm_vidc_mastering_display_colour_sei_payload*)(data);
    HDRStaticInfo* hdr_info = &m_internal_hdr_info.sInfo;
    bool internal_disp_changed_flag = false;

    internal_disp_changed_flag |= (hdr_info->sType1.mG.x != mastering_display_payload->nDisplayPrimariesX[0]) ||
        (hdr_info->sType1.mG.y != mastering_display_payload->nDisplayPrimariesY[0]);
    internal_disp_changed_flag |= (hdr_info->sType1.mB.x != mastering_display_payload->nDisplayPrimariesX[1]) ||
        (hdr_info->sType1.mB.y != mastering_display_payload->nDisplayPrimariesY[1]);
    internal_disp_changed_flag |= (hdr_info->sType1.mR.x != mastering_display_payload->nDisplayPrimariesX[2]) ||
        (hdr_info->sType1.mR.y != mastering_display_payload->nDisplayPrimariesY[2]);

    internal_disp_changed_flag |= (hdr_info->sType1.mW.x != mastering_display_payload->nWhitePointX) ||
        (hdr_info->sType1.mW.y != mastering_display_payload->nWhitePointY);

    /* Maximum Display Luminance from the bitstream is in 0.0001 cd/m2 while the HDRStaticInfo extension
       requires it in cd/m2, so dividing by 10000 and rounding the value after division
    */
    uint16_t max_display_luminance_cd_m2 =
        static_cast<int>((mastering_display_payload->nMaxDisplayMasteringLuminance / LUMINANCE_DIV_FACTOR) + 0.5);
    internal_disp_changed_flag |= (hdr_info->sType1.mMaxDisplayLuminance != max_display_luminance_cd_m2) ||
        (hdr_info->sType1.mMinDisplayLuminance != mastering_display_payload->nMinDisplayMasteringLuminance);

    if (internal_disp_changed_flag) {
        hdr_info->sType1.mG.x = mastering_display_payload->nDisplayPrimariesX[0];
        hdr_info->sType1.mG.y = mastering_display_payload->nDisplayPrimariesY[0];
        hdr_info->sType1.mB.x = mastering_display_payload->nDisplayPrimariesX[1];
        hdr_info->sType1.mB.y = mastering_display_payload->nDisplayPrimariesY[1];
        hdr_info->sType1.mR.x = mastering_display_payload->nDisplayPrimariesX[2];
        hdr_info->sType1.mR.y = mastering_display_payload->nDisplayPrimariesY[2];
        hdr_info->sType1.mW.x = mastering_display_payload->nWhitePointX;
        hdr_info->sType1.mW.y = mastering_display_payload->nWhitePointY;

        hdr_info->sType1.mMaxDisplayLuminance = max_display_luminance_cd_m2;
        hdr_info->sType1.mMinDisplayLuminance = mastering_display_payload->nMinDisplayMasteringLuminance;
    }

    return internal_disp_changed_flag;
}

void omx_vdec::convert_color_aspects_to_metadata(ColorAspects& aspects, ColorMetaData &color_mdata)
{
    PrimariesMap::const_iterator primary_it = mPrimariesMap.find(aspects.mPrimaries);
    TransferMap::const_iterator transfer_it = mTransferMap.find(aspects.mTransfer);
    MatrixCoeffMap::const_iterator matrix_it = mMatrixCoeffMap.find(aspects.mMatrixCoeffs);
    RangeMap::const_iterator range_it = mColorRangeMap.find(aspects.mRange);

    if (primary_it == mPrimariesMap.end()) {
        DEBUG_PRINT_LOW("No mapping for %d in PrimariesMap, defaulting to unspecified", aspects.mPrimaries);
        color_mdata.colorPrimaries = (ColorPrimaries)2;
    } else {
        color_mdata.colorPrimaries = primary_it->second;
    }

    if (transfer_it == mTransferMap.end()) {
        DEBUG_PRINT_LOW("No mapping for %d in TransferMap, defaulting to unspecified", aspects.mTransfer);
        color_mdata.transfer = (GammaTransfer)2;
    } else {
        color_mdata.transfer = transfer_it->second;
    }

    if (matrix_it == mMatrixCoeffMap.end()) {
        DEBUG_PRINT_LOW("No mapping for %d in MatrixCoeffMap, defaulting to unspecified", aspects.mMatrixCoeffs);
        color_mdata.matrixCoefficients = (MatrixCoEfficients)2;
    } else {
        color_mdata.matrixCoefficients = matrix_it->second;
    }

    if (range_it == mColorRangeMap.end()) {
        DEBUG_PRINT_LOW("No mapping for %d in ColorRangeMap, defaulting to limited range", aspects.mRange);
        color_mdata.range = Range_Limited;
    } else {
        color_mdata.range = range_it->second;
    }
}

void omx_vdec::convert_hdr_info_to_metadata(HDRStaticInfo& hdr_info, ColorMetaData &color_mdata)
{
    HDRStaticInfo::Type1 zero_hdr_info;
    MasteringDisplay& mastering_display = color_mdata.masteringDisplayInfo;
    ContentLightLevel& content_light = color_mdata.contentLightLevel;
    bool hdr_info_enabled = false;
    memset(&zero_hdr_info, 0, sizeof(HDRStaticInfo::Type1));
    hdr_info_enabled = (memcmp(&hdr_info, &zero_hdr_info, sizeof(HDRStaticInfo::Type1))!= 0);

    if (hdr_info_enabled) {
        mastering_display.colorVolumeSEIEnabled = true;
        mastering_display.primaries.rgbPrimaries[0][0] = hdr_info.sType1.mR.x;
        mastering_display.primaries.rgbPrimaries[0][1] = hdr_info.sType1.mR.y;
        mastering_display.primaries.rgbPrimaries[1][0] = hdr_info.sType1.mG.x;
        mastering_display.primaries.rgbPrimaries[1][1] = hdr_info.sType1.mG.y;
        mastering_display.primaries.rgbPrimaries[2][0] = hdr_info.sType1.mB.x;
        mastering_display.primaries.rgbPrimaries[2][1] = hdr_info.sType1.mB.y;
        mastering_display.primaries.whitePoint[0] = hdr_info.sType1.mW.x;
        mastering_display.primaries.whitePoint[1] = hdr_info.sType1.mW.y;
        mastering_display.maxDisplayLuminance = hdr_info.sType1.mMaxDisplayLuminance;
        mastering_display.minDisplayLuminance = hdr_info.sType1.mMinDisplayLuminance;
        content_light.lightLevelSEIEnabled = true;
        content_light.maxContentLightLevel = hdr_info.sType1.mMaxContentLightLevel;
        content_light.minPicAverageLightLevel = hdr_info.sType1.mMaxFrameAverageLightLevel;
    }

}

void omx_vdec::get_preferred_color_aspects(ColorAspects& preferredColorAspects)
{
    OMX_U32 width = drv_ctx.video_resolution.frame_width;
    OMX_U32 height = drv_ctx.video_resolution.frame_height;

    // For VPX, use client-color if specified.
    // For the rest, try to use the stream-color if present
    bool preferClientColor = (output_capability == V4L2_PIX_FMT_VP8 ||
         output_capability == V4L2_PIX_FMT_VP9);

    const ColorAspects &preferredColor = preferClientColor ?
        m_client_color_space.sAspects : m_internal_color_space.sAspects;
    const ColorAspects &defaultColor = preferClientColor ?
        m_internal_color_space.sAspects : m_client_color_space.sAspects;

    if ((width >= 3840 || height >= 3840 || width * (int64_t)height >= 3840 * 1634) &&
        (m_client_color_space.sAspects.mPrimaries == ColorAspects::PrimariesBT2020) &&
        (dpb_bit_depth == MSM_VIDC_BIT_DEPTH_8)) {
        m_client_color_space.sAspects.mPrimaries = ColorAspects::PrimariesBT709_5;
        m_client_color_space.sAspects.mMatrixCoeffs = ColorAspects::MatrixBT709_5;
    }

    preferredColorAspects.mPrimaries = preferredColor.mPrimaries != ColorAspects::PrimariesUnspecified ?
        preferredColor.mPrimaries : defaultColor.mPrimaries;
    preferredColorAspects.mTransfer = preferredColor.mTransfer != ColorAspects::TransferUnspecified ?
        preferredColor.mTransfer : defaultColor.mTransfer;
    preferredColorAspects.mMatrixCoeffs = preferredColor.mMatrixCoeffs != ColorAspects::MatrixUnspecified ?
        preferredColor.mMatrixCoeffs : defaultColor.mMatrixCoeffs;
    preferredColorAspects.mRange = preferredColor.mRange != ColorAspects::RangeUnspecified ?
        preferredColor.mRange : defaultColor.mRange;

}

void omx_vdec::get_preferred_hdr_info(HDRStaticInfo& finalHDRInfo)
{
    bool preferClientHDR = (output_capability == V4L2_PIX_FMT_VP9);

    const HDRStaticInfo &preferredHDRInfo = preferClientHDR ?
        m_client_hdr_info.sInfo : m_internal_hdr_info.sInfo;
    const HDRStaticInfo &defaultHDRInfo = preferClientHDR ?
        m_internal_hdr_info.sInfo : m_client_hdr_info.sInfo;
    finalHDRInfo.sType1.mR = ((preferredHDRInfo.sType1.mR.x != 0) && (preferredHDRInfo.sType1.mR.y != 0)) ?
        preferredHDRInfo.sType1.mR : defaultHDRInfo.sType1.mR;
    finalHDRInfo.sType1.mG = ((preferredHDRInfo.sType1.mG.x != 0) && (preferredHDRInfo.sType1.mG.y != 0)) ?
        preferredHDRInfo.sType1.mG : defaultHDRInfo.sType1.mG;
    finalHDRInfo.sType1.mB = ((preferredHDRInfo.sType1.mB.x != 0) && (preferredHDRInfo.sType1.mB.y != 0)) ?
        preferredHDRInfo.sType1.mB : defaultHDRInfo.sType1.mB;
    finalHDRInfo.sType1.mW = ((preferredHDRInfo.sType1.mW.x != 0) && (preferredHDRInfo.sType1.mW.y != 0)) ?
        preferredHDRInfo.sType1.mW : defaultHDRInfo.sType1.mW;
    finalHDRInfo.sType1.mMaxDisplayLuminance = (preferredHDRInfo.sType1.mMaxDisplayLuminance != 0) ?
        preferredHDRInfo.sType1.mMaxDisplayLuminance : defaultHDRInfo.sType1.mMaxDisplayLuminance;
    finalHDRInfo.sType1.mMinDisplayLuminance = (preferredHDRInfo.sType1.mMinDisplayLuminance != 0) ?
        preferredHDRInfo.sType1.mMinDisplayLuminance : defaultHDRInfo.sType1.mMinDisplayLuminance;
    finalHDRInfo.sType1.mMaxContentLightLevel = (preferredHDRInfo.sType1.mMaxContentLightLevel != 0) ?
        preferredHDRInfo.sType1.mMaxContentLightLevel : defaultHDRInfo.sType1.mMaxContentLightLevel;
    finalHDRInfo.sType1.mMaxFrameAverageLightLevel = (preferredHDRInfo.sType1.mMaxFrameAverageLightLevel != 0) ?
        preferredHDRInfo.sType1.mMaxFrameAverageLightLevel : defaultHDRInfo.sType1.mMaxFrameAverageLightLevel;
}

void omx_vdec::set_histogram_metadata(private_handle_t *private_handle)
{
    if (m_hist_metadata.stat_len != VIDEO_HISTOGRAM_STATS_SIZE || !private_handle)
        return;

    m_hist_metadata.display_width = m_extradata_misr.output_crop_rect.nWidth;
    m_hist_metadata.display_height = m_extradata_misr.output_crop_rect.nHeight;
    m_hist_metadata.decode_width = m_extradata_misr.output_width;
    m_hist_metadata.decode_height = m_extradata_misr.output_height;

    if (setMetaData(private_handle, SET_VIDEO_HISTOGRAM_STATS, (void*)&m_hist_metadata))
        DEBUG_PRINT_HIGH("failed to set histogram video stats");

    m_hist_metadata.stat_len = 0;
}

bool omx_vdec::handle_extradata(OMX_BUFFERHEADERTYPE *p_buf_hdr)
{
    OMX_OTHER_EXTRADATATYPE *p_client_extra = NULL;
    OMX_TICKS time_stamp = 0;
    unsigned long consumed_len = 0;
    OMX_U32 recovery_sei_flags = 1;
    int enable = OMX_InterlaceFrameProgressive;
    bool internal_hdr_info_changed_flag = false;
    bool reconfig_event_sent = false;
    char *p_extradata = NULL;
    OMX_OTHER_EXTRADATATYPE *data = NULL;
    ColorMetaData color_mdata;
    private_handle_t *private_handle = NULL;

    OMX_BUFFERHEADERTYPE  *omx_base_address =
        client_buffers.is_color_conversion_enabled()?
                            m_intermediate_out_mem_ptr:m_out_mem_ptr;
    memset(&color_mdata, 0, sizeof(color_mdata));

    int buf_index = p_buf_hdr - omx_base_address;
    if (buf_index >= drv_ctx.extradata_info.count) {
        DEBUG_PRINT_ERROR("handle_extradata: invalid index(%d) max(%d)",
                buf_index, drv_ctx.extradata_info.count);
        return reconfig_event_sent;
    }
    if (m_enable_android_native_buffers)
        private_handle = (private_handle_t *)native_buffer[buf_index].privatehandle;

    if (!drv_ctx.extradata_info.ion[buf_index].uaddr) {
        DEBUG_PRINT_HIGH("NULL drv_ctx.extradata_info.ion[buf_index].uaddr");
        return reconfig_event_sent;
    }

    if (m_client_output_extradata_mem_ptr &&
        m_client_out_extradata_info.getSize() >= drv_ctx.extradata_info.buffer_size) {
        p_client_extra = (OMX_OTHER_EXTRADATATYPE *)((m_client_output_extradata_mem_ptr + buf_index)->pBuffer);
    }

    p_extradata = drv_ctx.extradata_info.ion[buf_index].uaddr;
    if (p_extradata && p_client_extra) {
        memcpy(p_client_extra, p_extradata, drv_ctx.extradata_info.buffer_size);
        p_buf_hdr->nFlags |= OMX_BUFFERFLAG_EXTRADATA;
    }

    m_extradata_misr.output_crop_updated = OMX_FALSE;
    data = (struct OMX_OTHER_EXTRADATATYPE *)p_extradata;
    if (data) {
        bool is_hdr10_plus_info_found = false;
        while ((((consumed_len + sizeof(struct OMX_OTHER_EXTRADATATYPE)) <
                drv_ctx.extradata_info.buffer_size) && ((consumed_len + data->nSize) <
                drv_ctx.extradata_info.buffer_size))
                && (data->eType != (OMX_EXTRADATATYPE)MSM_VIDC_EXTRADATA_NONE)) {
            DEBUG_PRINT_LOW("handle_extradata: eType = 0x%x", data->eType);
            switch ((unsigned long)data->eType) {
                case MSM_VIDC_EXTRADATA_INTERLACE_VIDEO:
                    struct msm_vidc_interlace_payload *payload;
                    payload = (struct msm_vidc_interlace_payload *)(void *)data->data;
                    if (payload) {
                        DEBUG_PRINT_LOW("Interlace format %#x", payload->format);
                        enable = OMX_InterlaceFrameProgressive;
                        is_mbaff = payload->format & MSM_VIDC_INTERLACE_FRAME_MBAFF;
                        switch (payload->format & 0x1F) {
                            case MSM_VIDC_INTERLACE_FRAME_PROGRESSIVE:
                                drv_ctx.interlace = VDEC_InterlaceFrameProgressive;
                                break;
                            case MSM_VIDC_INTERLACE_INTERLEAVE_FRAME_TOPFIELDFIRST:
                                drv_ctx.interlace = VDEC_InterlaceInterleaveFrameTopFieldFirst;
                                enable = OMX_InterlaceInterleaveFrameTopFieldFirst;
                                break;
                            case MSM_VIDC_INTERLACE_INTERLEAVE_FRAME_BOTTOMFIELDFIRST:
                                drv_ctx.interlace = VDEC_InterlaceInterleaveFrameBottomFieldFirst;
                                enable = OMX_InterlaceInterleaveFrameBottomFieldFirst;
                                break;
                            case MSM_VIDC_INTERLACE_FRAME_TOPFIELDFIRST:
                                drv_ctx.interlace = VDEC_InterlaceFrameTopFieldFirst;
                                enable = OMX_InterlaceFrameTopFieldFirst;
                                break;
                            case MSM_VIDC_INTERLACE_FRAME_BOTTOMFIELDFIRST:
                                drv_ctx.interlace = VDEC_InterlaceFrameBottomFieldFirst;
                                enable = OMX_InterlaceFrameBottomFieldFirst;
                                break;
                            default:
                                DEBUG_PRINT_LOW("default case - set to progressive");
                                drv_ctx.interlace = VDEC_InterlaceFrameProgressive;
                        }
                    }

                    if (m_enable_android_native_buffers) {
                        DEBUG_PRINT_LOW("setMetaData INTERLACED format:%d enable:%d",
                                        payload->format, enable);

                        setMetaData(private_handle, PP_PARAM_INTERLACED, (void*)&enable);
                    }
                    break;
                case MSM_VIDC_EXTRADATA_TIMESTAMP:
                    struct msm_vidc_ts_payload *time_stamp_payload;
                    time_stamp_payload = (struct msm_vidc_ts_payload *)(void *)data->data;
                    time_stamp = time_stamp_payload->timestamp_lo;
                    time_stamp |= ((unsigned long long)time_stamp_payload->timestamp_hi << 32);
                    p_buf_hdr->nTimeStamp = time_stamp;
                    break;
                case MSM_VIDC_EXTRADATA_INDEX:
                    int *etype;
                    etype  = (int *)(void *)data->data;
                    if (etype && *etype == MSM_VIDC_EXTRADATA_OUTPUT_CROP) {
                        struct msm_vidc_output_crop_payload *output_crop_payload;
                        output_crop_payload = (struct msm_vidc_output_crop_payload *)(++etype);
                        if (output_crop_payload) {
                            m_extradata_misr.output_crop_rect.nLeft = output_crop_payload->left;
                            m_extradata_misr.output_crop_rect.nTop = output_crop_payload->top;
                            m_extradata_misr.output_crop_rect.nWidth = output_crop_payload->left + output_crop_payload->display_width;
                            m_extradata_misr.output_crop_rect.nHeight = output_crop_payload->top + output_crop_payload->display_height;
                            m_extradata_misr.output_width = output_crop_payload->width;
                            m_extradata_misr.output_height = output_crop_payload->height;
                            m_extradata_misr.output_crop_updated = OMX_TRUE;
                            for(unsigned int m=0; m<output_crop_payload->misr_info[0].misr_set; m++) {
                            DEBUG_PRINT_HIGH("MISR0: %x %x %x %x\n",
                                output_crop_payload->misr_info[0].misr_dpb_luma[m],
                                output_crop_payload->misr_info[0].misr_dpb_chroma[m],
                                output_crop_payload->misr_info[0].misr_opb_luma[m],
                                output_crop_payload->misr_info[0].misr_opb_chroma[m]);
                            }
                            for(unsigned int m=0; m< output_crop_payload->misr_info[1].misr_set; m++) {
                                DEBUG_PRINT_HIGH("MISR1: %x %x %x %x\n",
                                                 output_crop_payload->misr_info[1].misr_dpb_luma[m],
                                                 output_crop_payload->misr_info[1].misr_dpb_chroma[m],
                                                 output_crop_payload->misr_info[1].misr_opb_luma[m],
                                                 output_crop_payload->misr_info[1].misr_opb_chroma[m]);
                            }
                            memcpy(m_extradata_misr.misr_info, output_crop_payload->misr_info, 2 * sizeof(msm_vidc_misr_info));
                        }
                    }
                    break;
                case MSM_VIDC_EXTRADATA_RECOVERY_POINT_SEI:
                    struct msm_vidc_recoverysei_payload *recovery_sei_payload;
                    recovery_sei_payload = (struct msm_vidc_recoverysei_payload *)(void *)data->data;
                    recovery_sei_flags = recovery_sei_payload->flags;
                    if (recovery_sei_flags != MSM_VIDC_FRAME_RECONSTRUCTION_CORRECT) {
                        p_buf_hdr->nFlags |= OMX_BUFFERFLAG_DATACORRUPT;
                        DEBUG_PRINT_HIGH("***************************************************");
                        DEBUG_PRINT_HIGH("FillBufferDone: OMX_BUFFERFLAG_DATACORRUPT Received");
                        DEBUG_PRINT_HIGH("***************************************************");
                    }
                    break;
                case MSM_VIDC_EXTRADATA_MPEG2_SEQDISP:
                case MSM_VIDC_EXTRADATA_VUI_DISPLAY_INFO:
                case MSM_VIDC_EXTRADATA_VPX_COLORSPACE_INFO:
                    reconfig_event_sent |= handle_color_space_info((void *)data->data);
                    break;
                case MSM_VIDC_EXTRADATA_STREAM_USERDATA:
                    if(output_capability == V4L2_PIX_FMT_HEVC) {
                        struct msm_vidc_stream_userdata_payload* userdata_payload = (struct msm_vidc_stream_userdata_payload*)data->data;
                        // Remove the size of type from msm_vidc_stream_userdata_payload
                        uint32_t payload_len = data->nDataSize - sizeof(userdata_payload->type);
                        if ((data->nDataSize < sizeof(userdata_payload->type)) ||
                            (payload_len > HDR_DYNAMIC_META_DATA_SZ)) {
                            DEBUG_PRINT_ERROR("Invalid User extradata size %u for HDR10+", data->nDataSize);
                        } else {
// enable setting metadata via gralloc handle
//#if HDR10_SETMETADATA_ENABLE
                            color_mdata.dynamicMetaDataValid = true;
                            color_mdata.dynamicMetaDataLen = payload_len;
                            memcpy(color_mdata.dynamicMetaDataPayload, userdata_payload->data, payload_len);
                            DEBUG_PRINT_HIGH("Copied %u bytes of HDR10+ extradata", payload_len);
//#endif
                            if(!is_hdr10_plus_info_found) {
                               store_hevc_hdr10plusinfo(payload_len, userdata_payload);
                               is_hdr10_plus_info_found = true;
                            }
                        }
                    }
                    break;
                case MSM_VIDC_EXTRADATA_CONTENT_LIGHT_LEVEL_SEI:
                    internal_hdr_info_changed_flag |= handle_content_light_level_info((void*)data->data);
                    break;
                case MSM_VIDC_EXTRADATA_MASTERING_DISPLAY_COLOUR_SEI:
                    internal_hdr_info_changed_flag |= handle_mastering_display_color_info((void*)data->data);
                    break;
                case MSM_VIDC_EXTRADATA_HDR_HIST:
                    if (data->nDataSize != VIDEO_HISTOGRAM_STATS_SIZE) {
                        DEBUG_PRINT_ERROR("Invalid HDR histogram extradata size %u", data->nDataSize);
                        m_hist_metadata.stat_len = 0;
                    } else {
                        memcpy(m_hist_metadata.stats_info, data->data, VIDEO_HISTOGRAM_STATS_SIZE);
                        m_hist_metadata.stat_len = VIDEO_HISTOGRAM_STATS_SIZE;
                        m_hist_metadata.frame_type = (p_buf_hdr->nFlags & OMX_BUFFERFLAG_SYNCFRAME) ? QD_SYNC_FRAME : 0;
                    }
                    break;
                case MSM_VIDC_EXTRADATA_NUM_CONCEALED_MB:
                case MSM_VIDC_EXTRADATA_FRAME_RATE:
                case MSM_VIDC_EXTRADATA_FRAME_QP:
                case MSM_VIDC_EXTRADATA_FRAME_BITS_INFO:
                case MSM_VIDC_EXTRADATA_S3D_FRAME_PACKING:
                case MSM_VIDC_EXTRADATA_PANSCAN_WINDOW:
                    // skip unused extra data
                    break;
                default:
                    // skip unrecognized extra data instead of return error.
                    DEBUG_PRINT_LOW("Unrecognized extradata");
                    break;
            }
            consumed_len += data->nSize;
            data = (OMX_OTHER_EXTRADATATYPE *)((char *)data + data->nSize);
        }

        if(internal_hdr_info_changed_flag) {
            print_debug_hdr_color_info(&(m_internal_hdr_info.sInfo), "Internal");
            print_debug_hdr_color_info(&(m_client_hdr_info.sInfo), "Client");
            if(!reconfig_event_sent) {
                DEBUG_PRINT_HIGH("Initiating PORT Reconfig due to HDR Info Change");
                post_event(OMX_CORE_OUTPUT_PORT_INDEX,
                           OMX_QTIIndexConfigDescribeHDRColorInfo,
                           OMX_COMPONENT_GENERATE_PORT_RECONFIG);
                reconfig_event_sent = true;
            }
        }

        if (m_enable_android_native_buffers) {
                ColorAspects final_color_aspects;
                HDRStaticInfo final_hdr_info;
                memset(&final_color_aspects, 0, sizeof(final_color_aspects));
                memset(&final_hdr_info, 0, sizeof(final_hdr_info));
                get_preferred_color_aspects(final_color_aspects);

                /* For VP8, always set the metadata on gralloc handle to 601-LR */
                if (output_capability == V4L2_PIX_FMT_VP8) {
                    final_color_aspects.mPrimaries = ColorAspects::PrimariesBT601_6_525;
                    final_color_aspects.mRange = ColorAspects::RangeLimited;
                    final_color_aspects.mTransfer = ColorAspects::TransferSMPTE170M;
                    final_color_aspects.mMatrixCoeffs = ColorAspects::MatrixBT601_6;
                }
                get_preferred_hdr_info(final_hdr_info);
// enable setting metadata via gralloc handle
//#if HDR10_SETMETADATA_ENABLE
                convert_hdr_info_to_metadata(final_hdr_info, color_mdata);
                convert_hdr10plusinfo_to_metadata(p_buf_hdr->pMarkData, color_mdata);
                convert_color_aspects_to_metadata(final_color_aspects, color_mdata);
                print_debug_hdr_color_info_mdata(&color_mdata);
                print_debug_hdr10plus_metadata(color_mdata);
                setMetaData(private_handle, COLOR_METADATA, (void*)&color_mdata);
//#endif
                set_histogram_metadata(private_handle);
        }

    }

    return reconfig_event_sent;
}

OMX_ERRORTYPE omx_vdec::enable_extradata(OMX_U64 requested_extradata)
{
    OMX_ERRORTYPE ret = OMX_ErrorNone;
    struct v4l2_control control;
    if (m_state != OMX_StateLoaded) {
        DEBUG_PRINT_ERROR("ERROR: enable extradata allowed in Loaded state only");
        return OMX_ErrorIncorrectStateOperation;
    }
    DEBUG_PRINT_HIGH("NOTE: requested[%u]", (unsigned int)requested_extradata);


    // Extradata default is enabled ib kernel by default
    control.id = V4L2_CID_MPEG_VIDC_VIDEO_EXTRADATA;
    control.value = requested_extradata;
    if (ioctl(drv_ctx.video_driver_fd, VIDIOC_S_CTRL, &control)) {
        DEBUG_PRINT_HIGH("Failed to enable extradata %#llX", requested_extradata);
    } else {
        DEBUG_PRINT_INFO("Extradata %#llX is enabled successfully.", requested_extradata);
    }

    ret = get_buffer_req(&drv_ctx.op_buf);

    return ret;
}

OMX_ERRORTYPE  omx_vdec::allocate_desc_buffer(OMX_U32 index)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    if (index >= drv_ctx.ip_buf.actualcount) {
        DEBUG_PRINT_ERROR("ERROR:Desc Buffer Index not found");
        return OMX_ErrorInsufficientResources;
    }
    if (m_desc_buffer_ptr == NULL) {
        m_desc_buffer_ptr = (desc_buffer_hdr*) \
                    calloc( (sizeof(desc_buffer_hdr)),
                            drv_ctx.ip_buf.actualcount);
        if (m_desc_buffer_ptr == NULL) {
            DEBUG_PRINT_ERROR("m_desc_buffer_ptr Allocation failed ");
            return OMX_ErrorInsufficientResources;
        }
    }

    m_desc_buffer_ptr[index].buf_addr = (unsigned char *)malloc (DESC_BUFFER_SIZE * sizeof(OMX_U8));
    if (m_desc_buffer_ptr[index].buf_addr == NULL) {
        DEBUG_PRINT_ERROR("desc buffer Allocation failed ");
        return OMX_ErrorInsufficientResources;
    }

    return eRet;
}

void omx_vdec::insert_demux_addr_offset(OMX_U32 address_offset)
{
    DEBUG_PRINT_LOW("Inserting address offset (%u) at idx (%u)", (unsigned int)address_offset,(unsigned int)m_demux_entries);
    if (m_demux_entries < 8192) {
        m_demux_offsets[m_demux_entries++] = address_offset;
    }
    return;
}

void omx_vdec::extract_demux_addr_offsets(OMX_BUFFERHEADERTYPE *buf_hdr)
{
    OMX_U32 bytes_to_parse = buf_hdr->nFilledLen;
    OMX_U8 *buf = buf_hdr->pBuffer + buf_hdr->nOffset;
    OMX_U32 index = 0;

    m_demux_entries = 0;

    while (index < bytes_to_parse) {
        if ( ((buf[index] == 0x00) && (buf[index+1] == 0x00) &&
                    (buf[index+2] == 0x00) && (buf[index+3] == 0x01)) ||
                ((buf[index] == 0x00) && (buf[index+1] == 0x00) &&
                 (buf[index+2] == 0x01)) ) {
            //Found start code, insert address offset
            insert_demux_addr_offset(index);
            if (buf[index+2] == 0x01) // 3 byte start code
                index += 3;
            else                      //4 byte start code
                index += 4;
        } else
            index++;
    }
    DEBUG_PRINT_LOW("Extracted (%u) demux entry offsets", (unsigned int)m_demux_entries);
    return;
}

OMX_ERRORTYPE omx_vdec::handle_demux_data(OMX_BUFFERHEADERTYPE *p_buf_hdr)
{
    //fix this, handle 3 byte start code, vc1 terminator entry
    OMX_U8 *p_demux_data = NULL;
    OMX_U32 desc_data = 0;
    OMX_U32 start_addr = 0;
    OMX_U32 nal_size = 0;
    OMX_U32 suffix_byte = 0;
    OMX_U32 demux_index = 0;
    OMX_U32 buffer_index = 0;

    if (m_desc_buffer_ptr == NULL) {
        DEBUG_PRINT_ERROR("m_desc_buffer_ptr is NULL. Cannot append demux entries.");
        return OMX_ErrorBadParameter;
    }

    buffer_index = p_buf_hdr - ((OMX_BUFFERHEADERTYPE *)m_inp_mem_ptr);
    if (buffer_index > drv_ctx.ip_buf.actualcount) {
        DEBUG_PRINT_ERROR("handle_demux_data:Buffer index is incorrect (%u)", (unsigned int)buffer_index);
        return OMX_ErrorBadParameter;
    }

    p_demux_data = (OMX_U8 *) m_desc_buffer_ptr[buffer_index].buf_addr;

    if ( ((OMX_U8*)p_demux_data == NULL) ||
            ((m_demux_entries * 16) + 1) > DESC_BUFFER_SIZE) {
        DEBUG_PRINT_ERROR("Insufficient buffer. Cannot append demux entries.");
        return OMX_ErrorBadParameter;
    } else {
        for (; demux_index < m_demux_entries; demux_index++) {
            desc_data = 0;
            start_addr = m_demux_offsets[demux_index];
            if (p_buf_hdr->pBuffer[m_demux_offsets[demux_index] + 2] == 0x01) {
                suffix_byte = p_buf_hdr->pBuffer[m_demux_offsets[demux_index] + 3];
            } else {
                suffix_byte = p_buf_hdr->pBuffer[m_demux_offsets[demux_index] + 4];
            }
            if (demux_index < (m_demux_entries - 1)) {
                nal_size = m_demux_offsets[demux_index + 1] - m_demux_offsets[demux_index] - 2;
            } else {
                nal_size = p_buf_hdr->nFilledLen - m_demux_offsets[demux_index] - 2;
            }
            DEBUG_PRINT_LOW("Start_addr(0x%x), suffix_byte(0x%x),nal_size(%u),demux_index(%u)",
                    (unsigned int)start_addr,
                    (unsigned int)suffix_byte,
                    (unsigned int)nal_size,
                    (unsigned int)demux_index);
            desc_data = (start_addr >> 3) << 1;
            desc_data |= (start_addr & 7) << 21;
            desc_data |= suffix_byte << 24;

            memcpy(p_demux_data, &desc_data, sizeof(OMX_U32));
            memcpy(p_demux_data + 4, &nal_size, sizeof(OMX_U32));
            memset(p_demux_data + 8, 0, sizeof(OMX_U32));
            memset(p_demux_data + 12, 0, sizeof(OMX_U32));

            p_demux_data += 16;
        }
        //Add zero word to indicate end of descriptors
        memset(p_demux_data, 0, sizeof(OMX_U32));

        m_desc_buffer_ptr[buffer_index].desc_data_size = (m_demux_entries * 16) + sizeof(OMX_U32);
        DEBUG_PRINT_LOW("desc table data size=%u", (unsigned int)m_desc_buffer_ptr[buffer_index].desc_data_size);
    }
    memset(m_demux_offsets, 0, ( sizeof(OMX_U32) * 8192) );
    m_demux_entries = 0;
    DEBUG_PRINT_LOW("Demux table complete!");
    return OMX_ErrorNone;
}

void omx_vdec::allocate_color_convert_buf::enable_color_conversion(bool enable) {
    if (!omx) {
        DEBUG_PRINT_HIGH("Invalid omx_vdec");
        return;
    }

    if (!omx->in_reconfig)
        this->enabled = enable;

    omx->c2d_enable_pending = enable;
}

omx_vdec::allocate_color_convert_buf::allocate_color_convert_buf()
{
    enabled = false;
    client_buffers_disabled = false;
    omx = NULL;
    init_members();
    ColorFormat = OMX_COLOR_FormatMax;
    dest_format = YCbCr420P;
    m_c2d_width = 0;
    m_c2d_height = 0;

    mMapOutput2DriverColorFormat[VDEC_YUV_FORMAT_NV12][-1] =
                        QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m;
    mMapOutput2DriverColorFormat[VDEC_YUV_FORMAT_NV12][VDEC_CODECTYPE_MVC] =
                        QOMX_COLOR_FORMATYUV420PackedSemiPlanar32mMultiView;
    mMapOutput2DriverColorFormat[VDEC_YUV_FORMAT_NV12_UBWC][-1] =
                        QOMX_COLOR_FORMATYUV420PackedSemiPlanar32mCompressed;
    mMapOutput2DriverColorFormat[VDEC_YUV_FORMAT_NV12_TP10_UBWC][-1] =
                     QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m10bitCompressed;
    mMapOutput2DriverColorFormat[VDEC_YUV_FORMAT_P010_VENUS][-1] =
                     QOMX_COLOR_FORMATYUV420SemiPlanarP010Venus;

    mMapOutput2Convert.insert( {
            {VDEC_YUV_FORMAT_NV12, NV12_128m},
            {VDEC_YUV_FORMAT_NV12_UBWC, NV12_UBWC},
            {VDEC_YUV_FORMAT_NV12_TP10_UBWC, TP10_UBWC},
            {VDEC_YUV_FORMAT_P010_VENUS, YCbCr420_VENUS_P010},
        });
}

void omx_vdec::allocate_color_convert_buf::set_vdec_client(void *client)
{
    omx = reinterpret_cast<omx_vdec*>(client);
}

void omx_vdec::allocate_color_convert_buf::init_members()
{
    allocated_count = 0;
    buffer_size_req = 0;
    buffer_alignment_req = 0;
    m_c2d_width = m_c2d_height = 0;
    memset(m_platform_list_client,0,sizeof(m_platform_list_client));
    memset(m_platform_entry_client,0,sizeof(m_platform_entry_client));
    memset(m_pmem_info_client,0,sizeof(m_pmem_info_client));
    memset(m_out_mem_ptr_client,0,sizeof(m_out_mem_ptr_client));
#ifdef USE_ION
    memset(op_buf_ion_info,0,sizeof(m_platform_entry_client));
#endif
    for (int i = 0; i < MAX_COUNT; i++)
        pmem_fd[i] = -1;
}

bool omx_vdec::allocate_color_convert_buf::update_buffer_req()
{
    bool status = true;
    unsigned int src_size = 0, destination_size = 0;
    unsigned int height, width;
    struct v4l2_format fmt;
    bool is_interlaced = false;
    OMX_COLOR_FORMATTYPE drv_color_format;

    if (!omx) {
        DEBUG_PRINT_ERROR("Invalid client in color convert");
        return false;
    }
    if (!enabled) {
        DEBUG_PRINT_HIGH("No color conversion required");
        return true;
    }
    pthread_mutex_lock(&omx->c_lock);

    ColorSubMapping::const_iterator
        found =  mMapOutput2Convert.find(omx->drv_ctx.output_format);
    if (found == mMapOutput2Convert.end()) {
        DEBUG_PRINT_HIGH("%s: Could not find the color conversion "
                         "mapping for %#X. Setting to default NV12",
                         __func__, omx->drv_ctx.output_format);
        src_format = NV12_128m;
    } else {
        src_format = (ColorConvertFormat) found->second;;
    }

    memset(&fmt, 0x0, sizeof(struct v4l2_format));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.pixelformat = omx->capture_capability;
    ioctl(omx->drv_ctx.video_driver_fd, VIDIOC_G_FMT, &fmt);
    width = fmt.fmt.pix_mp.width;
    height =  fmt.fmt.pix_mp.height;

    bool resolution_upgrade = (height > m_c2d_height ||
            width > m_c2d_width);
    if (omx->drv_ctx.output_format != VDEC_YUV_FORMAT_NV12)
        is_interlaced = omx->m_progressive != MSM_VIDC_PIC_STRUCT_PROGRESSIVE;
    if (resolution_upgrade) {
        // resolution upgraded ? ensure we are yet to allocate;
        // failing which, c2d buffers will never be reallocated and bad things will happen
        if (allocated_count > 0) {
            DEBUG_PRINT_ERROR("Cannot change C2D buffer requirements with %d active allocations",
                    allocated_count);
            status = false;
        }
    }

    if (status != false) {
        if (omx->drv_ctx.output_format != VDEC_YUV_FORMAT_NV12 &&
            (ColorFormat != OMX_COLOR_FormatYUV420Planar &&
             ColorFormat != OMX_COLOR_FormatYUV420SemiPlanar &&
             ColorFormat != OMX_COLOR_Format16bitRGB565 &&
             ColorFormat != (OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m)) {
            DEBUG_PRINT_ERROR("update_buffer_req: Unsupported color conversion");
            status = false;
        } else {
            ColorSubMapping::const_iterator
                found =  mMapOutput2Convert.find(
                                                 omx->drv_ctx.output_format);
            if (found == mMapOutput2Convert.end()) {
                src_format = NV12_128m;
            } else {
                src_format = (ColorConvertFormat) found->second;;
            }

            if (c2dcc.isPropChanged(width, is_interlaced ? (height+1)/2 : height,
                                    width, height, src_format, dest_format, 0, 0)) {
                DEBUG_PRINT_INFO("C2D: Set Resolution, Interlace(%s) Conversion(%#X -> %#X)"
                                 " src(%dX%d) dest(%dX%d)",
                                 is_interlaced ? "true": "false",
                                 src_format, dest_format, width,
                                 is_interlaced ? (height+1)/2 : height,
                                 width, height);
                status = c2dcc.setResolution(width,
                                             is_interlaced ? (height+1)/2 : height,
                                             width, height,
                                             src_format, dest_format,
                                             0, 0);
                if (status) {
                    src_size = c2dcc.getBuffSize(C2D_INPUT);
                    destination_size = c2dcc.getBuffSize(C2D_OUTPUT);

                    if (!src_size || src_size > omx->drv_ctx.op_buf.buffer_size ||
                        !destination_size) {
                        DEBUG_PRINT_ERROR("ERROR: Size mismatch in C2D src_size %d"
                                          "driver size %u destination size %d",
                                          src_size, (unsigned int)omx->drv_ctx.op_buf.buffer_size,
                                          destination_size);
                        buffer_size_req = 0;
                        // TODO: make this fatal. Driver is not supposed to quote size
                        //  smaller than what C2D needs !!
                    } else {
                        buffer_size_req = destination_size;
                        m_c2d_height = height;
                        m_c2d_width = width;
                    }
                }
            }
        }
    }
    pthread_mutex_unlock(&omx->c_lock);
    return status;
}

bool omx_vdec::allocate_color_convert_buf::set_color_format(
        OMX_COLOR_FORMATTYPE dest_color_format)
{
    bool status = true, drv_colorformat_c2d_enable = false;
    bool dest_color_format_c2d_enable = false;
    OMX_COLOR_FORMATTYPE drv_color_format = OMX_COLOR_FormatUnused;
    if (!omx) {
        DEBUG_PRINT_ERROR("Invalid client in color convert");
        return false;
    }
    pthread_mutex_lock(&omx->c_lock);
    status = get_color_format (drv_color_format);

    drv_colorformat_c2d_enable = (drv_color_format != dest_color_format) &&
        (drv_color_format != (OMX_COLOR_FORMATTYPE)
                QOMX_COLOR_FORMATYUV420PackedSemiPlanar32mMultiView) &&
        (drv_color_format != (OMX_COLOR_FORMATTYPE)
                QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m10bitCompressed) &&
        (drv_color_format != (OMX_COLOR_FORMATTYPE)
                QOMX_COLOR_FORMATYUV420SemiPlanarP010Venus);

    dest_color_format_c2d_enable = (dest_color_format != (OMX_COLOR_FORMATTYPE)
            QOMX_COLOR_FORMATYUV420PackedSemiPlanar32mCompressed) &&
            (dest_color_format != (OMX_COLOR_FORMATTYPE)
                QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m10bitCompressed);

    if (status && drv_colorformat_c2d_enable && dest_color_format_c2d_enable) {
        DEBUG_PRINT_LOW("Enabling C2D");
        if (dest_color_format == OMX_COLOR_FormatYUV420Planar ||
            dest_color_format == OMX_COLOR_FormatYUV420SemiPlanar ||
            dest_color_format == OMX_COLOR_Format16bitRGB565) {
            ColorFormat = dest_color_format;
            if (dest_color_format == OMX_COLOR_FormatYUV420Planar) {
                   dest_format = YCbCr420P;
            } else if( dest_color_format == OMX_COLOR_FormatYUV420SemiPlanar) {
                    dest_format = YCbCr420SP;
            } else if(dest_color_format == OMX_COLOR_Format16bitRGB565) {
                    dest_format = RGB565;
            } else {
                   dest_format = NV12_128m;
            }
            enable_color_conversion(true);
        } else {
            DEBUG_PRINT_ERROR("Unsupported output color format for c2d (%d)",
                              dest_color_format);
            status = false;
            enable_color_conversion(false);
        }
    } else {
        enable_color_conversion(false);
    }
    pthread_mutex_unlock(&omx->c_lock);
    return status;
}

OMX_BUFFERHEADERTYPE* omx_vdec::allocate_color_convert_buf::get_il_buf_hdr
(OMX_BUFFERHEADERTYPE *bufadd)
{
    if (!omx) {
        DEBUG_PRINT_ERROR("Invalid param get_buf_hdr");
        return NULL;
    }
    if (!is_color_conversion_enabled())
        return bufadd;

    OMX_BUFFERHEADERTYPE  *omx_base_address =
              is_color_conversion_enabled()?
                    omx->m_intermediate_out_mem_ptr:omx->m_out_mem_ptr;

    unsigned index = 0;
    index = bufadd - omx_base_address;
    if (index < omx->drv_ctx.op_buf.actualcount) {
        m_out_mem_ptr_client[index].nFlags = (bufadd->nFlags & OMX_BUFFERFLAG_EOS);
        m_out_mem_ptr_client[index].nTimeStamp = bufadd->nTimeStamp;

        omx->m_out_mem_ptr[index].nFlags = (bufadd->nFlags & OMX_BUFFERFLAG_EOS);
        omx->m_out_mem_ptr[index].nTimeStamp = bufadd->nTimeStamp;
        bool status = false;
        if (!omx->in_reconfig && !omx->output_flush_progress && bufadd->nFilledLen) {
            pthread_mutex_lock(&omx->c_lock);

            DEBUG_PRINT_INFO("C2D: Start color convertion");
            cache_invalidate(omx->drv_ctx.ptr_outputbuffer[index].pmem_fd);
            status = c2dcc.convertC2D(
                             omx->drv_ctx.ptr_intermediate_outputbuffer[index].pmem_fd,
                             bufadd->pBuffer, bufadd->pBuffer,
                             omx->drv_ctx.ptr_outputbuffer[index].pmem_fd,
                             omx->m_out_mem_ptr[index].pBuffer,
                             omx->m_out_mem_ptr[index].pBuffer);
            cache_invalidate(omx->drv_ctx.ptr_outputbuffer[index].pmem_fd);
            if (!status) {
                DEBUG_PRINT_ERROR("Failed color conversion %d", status);
                m_out_mem_ptr_client[index].nFilledLen = 0;
                omx->m_out_mem_ptr[index].nFilledLen = 0;
                pthread_mutex_unlock(&omx->c_lock);
                return &omx->m_out_mem_ptr[index];
            } else {
                unsigned int filledLen = 0;
                c2dcc.getBuffFilledLen(C2D_OUTPUT, filledLen);
                m_out_mem_ptr_client[index].nFilledLen = filledLen;
                omx->m_out_mem_ptr[index].nFilledLen = filledLen;
            }
            pthread_mutex_unlock(&omx->c_lock);
        } else {
            m_out_mem_ptr_client[index].nFilledLen = 0;
            omx->m_out_mem_ptr[index].nFilledLen = 0;
        }
        return &omx->m_out_mem_ptr[index];
    }
    DEBUG_PRINT_ERROR("Index messed up in the get_il_buf_hdr");
    return NULL;
}

    bool omx_vdec::allocate_color_convert_buf::get_buffer_req
(unsigned int &buffer_size)
{
    bool status = true;
    pthread_mutex_lock(&omx->c_lock);
     /* Whenever port mode is set to kPortModeDynamicANWBuffer, Video Frameworks
        always uses VideoNativeMetadata and OMX recives buffer type as
        grallocsource via storeMetaDataInBuffers_l API. The buffer_size
        will be communicated to frameworks via IndexParamPortdefinition. */
    if (!enabled)
        buffer_size = omx->dynamic_buf_mode ? sizeof(struct VideoNativeMetadata) :
                      omx->drv_ctx.op_buf.buffer_size;
    else {
        buffer_size = c2dcc.getBuffSize(C2D_OUTPUT);
    }
    pthread_mutex_unlock(&omx->c_lock);
    return status;
}

OMX_ERRORTYPE omx_vdec::allocate_color_convert_buf::set_buffer_req(
        OMX_U32 buffer_size, OMX_U32 actual_count)
{
    OMX_U32 expectedSize = enabled ? buffer_size_req : omx->dynamic_buf_mode ?
            sizeof(struct VideoDecoderOutputMetaData) : omx->drv_ctx.op_buf.buffer_size;
    if (buffer_size < expectedSize) {
        DEBUG_PRINT_ERROR("OP Requirements: Client size(%u) insufficient v/s requested(%u)",
                buffer_size, expectedSize);
        return OMX_ErrorBadParameter;
    }
    if (actual_count < omx->drv_ctx.op_buf.mincount) {
        DEBUG_PRINT_ERROR("OP Requirements: Client count(%u) insufficient v/s requested(%u)",
                actual_count, omx->drv_ctx.op_buf.mincount);
        return OMX_ErrorBadParameter;
    }

    if (enabled) {
        // disallow changing buffer size/count while we have active allocated buffers
        if (allocated_count > 0) {
            DEBUG_PRINT_ERROR("Cannot change C2D buffer size from %u to %u with %d active allocations",
                    buffer_size_req, buffer_size, allocated_count);
            return OMX_ErrorInvalidState;
        }

        buffer_size_req = buffer_size;
    } else {
        if (buffer_size > omx->drv_ctx.op_buf.buffer_size) {
            omx->drv_ctx.op_buf.buffer_size = buffer_size;
        }
    }

    omx->drv_ctx.op_buf.actualcount = actual_count;
    omx->drv_ctx.extradata_info.count = omx->drv_ctx.op_buf.actualcount;

    return omx->set_buffer_req(&(omx->drv_ctx.op_buf));
}

bool omx_vdec::is_component_secure()
{
    return secure_mode;
}

bool omx_vdec::allocate_color_convert_buf::get_color_format(OMX_COLOR_FORMATTYPE &dest_color_format)
{
    bool status = true;
    if (!enabled) {
        for (auto& x: mMapOutput2DriverColorFormat) {
            DecColorMapping::const_iterator
                found = mMapOutput2DriverColorFormat.find(omx->drv_ctx.output_format);
            if (found == mMapOutput2DriverColorFormat.end()) {
                status = false;
            } else {
                ColorSubMapping::const_iterator
                    subFound = found->second.find(omx->drv_ctx.decoder_format);
                if (subFound == found->second.end()) {
                    dest_color_format = (OMX_COLOR_FORMATTYPE)
                                             found->second.find(-1)->second;
                } else {
                    dest_color_format = (OMX_COLOR_FORMATTYPE) subFound->second;
                }
            }
        }
    } else {
        if (ColorFormat == OMX_COLOR_FormatYUV420Planar ||
            ColorFormat == OMX_COLOR_FormatYUV420SemiPlanar ||
            ColorFormat == OMX_COLOR_Format16bitRGB565 ||
            ColorFormat == (OMX_COLOR_FORMATTYPE) QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m) {
            dest_color_format = ColorFormat;
        } else {
            status = false;
        }
    }
    return status;
}

void omx_vdec::send_codec_config()
{
    if (codec_config_flag) {
        unsigned long p1 = 0, p2 = 0;
        unsigned long p3 = 0, p4 = 0;
        unsigned long ident = 0, ident2 = 0;
        pthread_mutex_lock(&m_lock);
        DEBUG_PRINT_LOW("\n Check Queue for codec_config buffer \n");
        while (m_etb_q.m_size) {
            m_etb_q.pop_entry(&p1,&p2,&ident);
            if (ident == OMX_COMPONENT_GENERATE_ETB) {
                if (((OMX_BUFFERHEADERTYPE *)p2)->nFlags & OMX_BUFFERFLAG_CODECCONFIG) {
                    while (m_ftb_q.m_size) {
                        m_ftb_q.pop_entry(&p3,&p4,&ident2);
                        if (ident2 == OMX_COMPONENT_GENERATE_FTB) {
                            pthread_mutex_unlock(&m_lock);
                            if (fill_this_buffer_proxy((OMX_HANDLETYPE)p3,\
                                        (OMX_BUFFERHEADERTYPE *)p4) != OMX_ErrorNone) {
                                DEBUG_PRINT_ERROR("\n fill_this_buffer_proxy failure");
                                omx_report_error ();
                            }
                            pthread_mutex_lock(&m_lock);
                        } else if (ident2 == OMX_COMPONENT_GENERATE_FBD) {
                            fill_buffer_done(&m_cmp,(OMX_BUFFERHEADERTYPE *)p3);
                        }
                    }
                    pthread_mutex_unlock(&m_lock);
                    if (empty_this_buffer_proxy((OMX_HANDLETYPE)p1,\
                                (OMX_BUFFERHEADERTYPE *)p2) != OMX_ErrorNone) {
                        DEBUG_PRINT_ERROR("\n empty_this_buffer_proxy failure");
                        omx_report_error ();
                    }
                    pthread_mutex_lock(&m_lock);
                } else {
                    pending_input_buffers++;
                    VIDC_TRACE_INT_LOW("ETB-pending", pending_input_buffers);
                    DEBUG_PRINT_LOW("\n Flush Input OMX_COMPONENT_GENERATE_ETB %p, pending_input_buffers %d",
                            (OMX_BUFFERHEADERTYPE *)p2, pending_input_buffers);
                    empty_buffer_done(&m_cmp,(OMX_BUFFERHEADERTYPE *)p2);
                }
            } else if (ident == OMX_COMPONENT_GENERATE_EBD) {
                DEBUG_PRINT_LOW("\n Flush Input OMX_COMPONENT_GENERATE_EBD %p",
                        (OMX_BUFFERHEADERTYPE *)p1);
                empty_buffer_done(&m_cmp,(OMX_BUFFERHEADERTYPE *)p1);
            }
        }
        pthread_mutex_unlock(&m_lock);
    }
}

omx_vdec::perf_control::perf_control()
{
    m_perf_control_enable = 0;
    m_perf_lib = NULL;
    m_perf_handle = 0;
    m_perf_lock_acquire = NULL;
    m_perf_lock_release = NULL;
}

omx_vdec::perf_control::~perf_control()
{
    if (!m_perf_control_enable)
        return;

    if (m_perf_handle && m_perf_lock_release) {
        m_perf_lock_release(m_perf_handle);
        DEBUG_PRINT_LOW("perflock released");
    }
    if (m_perf_lib) {
        dlclose(m_perf_lib);
    }
}

int omx_vdec::perf_control::perf_lock_acquire()
{
    int arg[2];
    if (!m_perf_control_enable)
        return 0;

    if (!m_perf_lib) {
        DEBUG_PRINT_ERROR("no perf control library");
        return -1;
    }
    if (!m_perf_lock_acquire) {
        DEBUG_PRINT_ERROR("NULL perflock acquire");
        return -1;
    }
    if (m_perf_handle) {
        DEBUG_PRINT_LOW("perflock already acquired");
        return 0;
    }
    DEBUG_PRINT_HIGH("perflock acquire");
    arg[0] = MPCTLV3_VIDEO_DECODE_PB_HINT;
    arg[1] = 1;
    m_perf_handle = m_perf_lock_acquire(0, 0, arg, sizeof(arg) / sizeof(int));
    if (m_perf_handle < 0) {
        DEBUG_PRINT_ERROR("perflock acquire failed with error %d", m_perf_handle);
        m_perf_handle = 0;
        return -1;
    }
    return 0;
}

void omx_vdec::perf_control::perf_lock_release()
{
    if (!m_perf_control_enable)
        return;

    if (!m_perf_lib) {
        DEBUG_PRINT_ERROR("no perf control library");
        return;
    }
    if (!m_perf_lock_release) {
        DEBUG_PRINT_ERROR("NULL perflock release");
        return;
    }
    if (!m_perf_handle) {
        DEBUG_PRINT_LOW("perflock already released");
        return;
    }
    DEBUG_PRINT_HIGH("perflock release");
    m_perf_lock_release(m_perf_handle);
    m_perf_handle = 0;
}

bool omx_vdec::perf_control::load_perf_library()
{
    char perf_lib_path[PROPERTY_VALUE_MAX] = {0};

    if (!m_perf_control_enable) {
        DEBUG_PRINT_HIGH("perf control is not enabled");
        return false;
    }
    if (m_perf_lib) {
        DEBUG_PRINT_HIGH("perf lib already opened");
        return true;
    }

    if((property_get("ro.vendor.extension_library", perf_lib_path, NULL) <= 0)) {
        DEBUG_PRINT_ERROR("vendor library not set in ro.vendor.extension_library");
        goto handle_err;
    }

    if ((m_perf_lib = dlopen(perf_lib_path, RTLD_NOW)) == NULL) {
        DEBUG_PRINT_ERROR("Failed to open %s : %s",perf_lib_path, dlerror());
        goto handle_err;
    } else {
        m_perf_lock_acquire = (perf_lock_acquire_t)dlsym(m_perf_lib, "perf_lock_acq");
        if (m_perf_lock_acquire == NULL) {
            DEBUG_PRINT_ERROR("Failed to load symbol: perf_lock_acq");
            goto handle_err;
        }
        m_perf_lock_release = (perf_lock_release_t)dlsym(m_perf_lib, "perf_lock_rel");
        if (m_perf_lock_release == NULL) {
            DEBUG_PRINT_ERROR("Failed to load symbol: perf_lock_rel");
            goto handle_err;
        }
    }
    return true;

handle_err:
    if (m_perf_lib) {
        dlclose(m_perf_lib);
    }
    m_perf_lib = NULL;
    m_perf_lock_acquire = NULL;
    m_perf_lock_release = NULL;
    return false;
}

//static
OMX_ERRORTYPE omx_vdec::describeColorFormat(OMX_PTR pParam)
{
#ifndef FLEXYUV_SUPPORTED
    return OMX_ErrorUndefined;
#else

    if (pParam == NULL) {
        DEBUG_PRINT_ERROR("describeColorFormat: invalid params");
        return OMX_ErrorBadParameter;
    }

    DescribeColorFormatParams *params = (DescribeColorFormatParams*)pParam;

    MediaImage *img = &(params->sMediaImage);
    switch(params->eColorFormat) {
        case static_cast <OMX_COLOR_FORMATTYPE> (QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m):
        {
            img->mType = MediaImage::MEDIA_IMAGE_TYPE_YUV;
            img->mNumPlanes = 3;
            // mWidth and mHeight represent the W x H of the largest plane
            // In our case, this happens to be the Stride x Scanlines of Y plane
            img->mWidth = params->nFrameWidth;
            img->mHeight = params->nFrameHeight;
            size_t planeWidth = VENUS_Y_STRIDE(COLOR_FMT_NV12, params->nFrameWidth);
            size_t planeHeight = VENUS_Y_SCANLINES(COLOR_FMT_NV12, params->nFrameHeight);
            img->mBitDepth = 8;
            //Plane 0 (Y)
            img->mPlane[MediaImage::Y].mOffset = 0;
            img->mPlane[MediaImage::Y].mColInc = 1;
            img->mPlane[MediaImage::Y].mRowInc = planeWidth; //same as stride
            img->mPlane[MediaImage::Y].mHorizSubsampling = 1;
            img->mPlane[MediaImage::Y].mVertSubsampling = 1;
            //Plane 1 (U)
            img->mPlane[MediaImage::U].mOffset = planeWidth * planeHeight;
            img->mPlane[MediaImage::U].mColInc = 2;           //interleaved UV
            img->mPlane[MediaImage::U].mRowInc =
                    VENUS_UV_STRIDE(COLOR_FMT_NV12, params->nFrameWidth);
            img->mPlane[MediaImage::U].mHorizSubsampling = 2;
            img->mPlane[MediaImage::U].mVertSubsampling = 2;
            //Plane 2 (V)
            img->mPlane[MediaImage::V].mOffset = planeWidth * planeHeight + 1;
            img->mPlane[MediaImage::V].mColInc = 2;           //interleaved UV
            img->mPlane[MediaImage::V].mRowInc =
                    VENUS_UV_STRIDE(COLOR_FMT_NV12, params->nFrameWidth);
            img->mPlane[MediaImage::V].mHorizSubsampling = 2;
            img->mPlane[MediaImage::V].mVertSubsampling = 2;
            break;
        }

        case OMX_COLOR_FormatYUV420Planar:
        case OMX_COLOR_FormatYUV420SemiPlanar:
            // We need not describe the standard OMX linear formats as these are
            // understood by client. Fail this deliberately to let client fill-in
            return OMX_ErrorUnsupportedSetting;

        default:
            // Rest all formats which are non-linear cannot be described
            DEBUG_PRINT_LOW("color-format %x is not flexible", params->eColorFormat);
            img->mType = MediaImage::MEDIA_IMAGE_TYPE_UNKNOWN;
            return OMX_ErrorNone;
    };

    DEBUG_PRINT_LOW("NOTE: Describe color format : %x", params->eColorFormat);
    DEBUG_PRINT_LOW("  FrameWidth x FrameHeight : %d x %d", params->nFrameWidth, params->nFrameHeight);
    DEBUG_PRINT_LOW("  YWidth x YHeight : %d x %d", img->mWidth, img->mHeight);
    for (size_t i = 0; i < img->mNumPlanes; ++i) {
        DEBUG_PRINT_LOW("  Plane[%zu] : offset=%d / xStep=%d / yStep = %d",
                i, img->mPlane[i].mOffset, img->mPlane[i].mColInc, img->mPlane[i].mRowInc);
    }
    return OMX_ErrorNone;
#endif //FLEXYUV_SUPPORTED
}

bool omx_vdec::prefetch_buffers(unsigned long prefetch_count,
        unsigned long prefetch_size, unsigned ioctl_code, unsigned ion_flag)
{
    struct ion_prefetch_data prefetch_data;
    struct ion_prefetch_regions regions;
    __u64 sizes[prefetch_count];
    int rc, ion_fd = ion_open();
    if (ion_fd < 0) {
        DEBUG_PRINT_ERROR("%s: Ion fd open failed : %d", __func__, ion_fd);
        return false;
    }

    DEBUG_PRINT_HIGH("%s: prefetch_count : %lu, prefetch_size : %lu, ioctl : %u",
            __func__, prefetch_count, prefetch_size, ioctl_code);
    for (uint32_t i = 0; i < prefetch_count; i++) {
        sizes[i] = prefetch_size;
    }

    regions.nr_sizes = prefetch_count;
    regions.sizes = (__u64) sizes;
    regions.vmid = ion_flag;

    prefetch_data.nr_regions = 1;
    prefetch_data.regions = (__u64) &regions;
    prefetch_data.heap_id = ION_HEAP(ION_SECURE_HEAP_ID);

    rc = ioctl(ion_fd, ioctl_code, &prefetch_data);
    if (rc) {
        DEBUG_PRINT_ERROR("%s: Prefetch ioctl failed ioctl : %u, rc : %d, errno : %d",
                __func__, ioctl_code, rc, errno);
        rc = false;
    } else {
        rc = true;
    }

    close(ion_fd);
    return rc;
}

bool omx_vdec::store_vp9_hdr10plusinfo(DescribeHDR10PlusInfoParams *hdr10plusdata)
{
    struct hdr10plusInfo metadata;

    if (!hdr10plusdata) {
        DEBUG_PRINT_ERROR("hdr10plus info not present");
        return false;
    }

    if (hdr10plusdata->nParamSize > MAX_HDR10PLUSINFO_SIZE ||
            hdr10plusdata->nParamSize < 1) {
        DEBUG_PRINT_ERROR("Invalid hdr10plus metadata size %u", hdr10plusdata->nParamSize);
        return false;
    }

    if (output_capability != V4L2_PIX_FMT_VP9) {
        DEBUG_PRINT_ERROR("DescribeHDR10PlusInfoParams is not supported for %d codec",
            output_capability);
        return false;
    }

    memset(&metadata, 0, sizeof(struct hdr10plusInfo));
    metadata.nSize = hdr10plusdata->nSize;
    metadata.nVersion = hdr10plusdata->nVersion;
    metadata.nPortIndex = hdr10plusdata->nPortIndex;
    metadata.nParamSize = hdr10plusdata->nParamSize;
    metadata.nParamSizeUsed = hdr10plusdata->nParamSizeUsed;
    memcpy(metadata.payload, hdr10plusdata->nValue , hdr10plusdata->nParamSizeUsed);
    metadata.is_new = true;

    /*
     * For the first setconfig, set the timestamp as zero. For
     * the remaining, set the timestamp equal to previous
     * etb timestamp + 1 to know this hdr10plus data arrived
     * after previous etb.
     */
    if (m_etb_count) {
        metadata.timestamp = m_etb_timestamp + 1;
    }

    pthread_mutex_lock(&m_hdr10pluslock);
    DEBUG_PRINT_LOW("add hdr10plus info to the list with timestamp %lld and size %u",
        metadata.timestamp, metadata.nParamSizeUsed);
    m_hdr10pluslist.push_back(metadata);
    pthread_mutex_unlock(&m_hdr10pluslock);

    return true;
}

bool omx_vdec::store_hevc_hdr10plusinfo(uint32_t payload_size,
    msm_vidc_stream_userdata_payload *hdr10plusdata)
{
    struct hdr10plusInfo metadata;

    if (!hdr10plusdata) {
        DEBUG_PRINT_ERROR("hdr10plus info not present");
        return false;
    }

    if (payload_size > MAX_HDR10PLUSINFO_SIZE ||
            payload_size < 1) {
        DEBUG_PRINT_ERROR("Invalid hdr10plus metadata size %u", payload_size);
        return false;
    }

    if (output_capability != V4L2_PIX_FMT_HEVC) {
        DEBUG_PRINT_ERROR("msm_vidc_stream_userdata_payload is not supported for %d codec",
            output_capability);
        return false;
    }

    memset(&metadata, 0, sizeof(struct hdr10plusInfo));
    metadata.nParamSizeUsed = payload_size;
    memcpy(metadata.payload, hdr10plusdata->data , payload_size);
    metadata.is_new = false;
    if (m_etb_count) {
        metadata.timestamp = m_etb_timestamp + 1;
    }

    pthread_mutex_lock(&m_hdr10pluslock);
    DEBUG_PRINT_LOW("add hevc hdr10plus info to the list with size %u", payload_size);
    m_hdr10pluslist.push_back(metadata);
    pthread_mutex_unlock(&m_hdr10pluslock);

    return true;
}

void omx_vdec::update_hdr10plusinfo_cookie_using_timestamp(OMX_PTR markdata, OMX_TICKS timestamp)
{
    std::list<hdr10plusInfo>::reverse_iterator iter;
    unsigned int found = 0;
    unsigned int cookie = (unsigned int)(unsigned long)markdata;
    bool is_list_empty = false;

    if (output_capability != V4L2_PIX_FMT_VP9 &&
        output_capability != V4L2_PIX_FMT_HEVC)
        return;

    pthread_mutex_lock(&m_hdr10pluslock);
    is_list_empty = m_hdr10pluslist.empty();
    pthread_mutex_unlock(&m_hdr10pluslock);

    if (is_list_empty) {
        DEBUG_PRINT_HIGH("update_hdr10plusinfo_cookie_using_timestamp: hdr10plusinfo list is empty!");
        return;
    }
    /*
     * look for the hdr10plus data which has timestamp nearest and
     * lower than the etb timestamp, we should not take the
     * hdr10plus data which has the timestamp greater than etb timestamp.
     */
    pthread_mutex_lock(&m_hdr10pluslock);
    iter = m_hdr10pluslist.rbegin();
    while (iter != m_hdr10pluslist.rend()) {
        if (iter->timestamp <= timestamp && iter->is_new) {
            found++;
            if (found == 1) {
                iter->cookie = cookie;
                iter->is_new = false;
                DEBUG_PRINT_LOW("Cookie value %u stored in hdr10plus list with timestamp %lld, size %u",
                    iter->cookie, iter->timestamp, iter->nParamSizeUsed);
            }
        }
        iter++;
    }
    pthread_mutex_unlock(&m_hdr10pluslock);

    if(found > 1)
        DEBUG_PRINT_HIGH("Multiple hdr10plus data not expected. Continue with the latest");
}

void omx_vdec::convert_hdr10plusinfo_to_metadata(OMX_PTR markdata, ColorMetaData &colorData)
{
    std::list<hdr10plusInfo>::iterator iter;
    unsigned int cookie = (unsigned int)(unsigned long)markdata;
    bool is_list_empty = false;

    if (output_capability != V4L2_PIX_FMT_VP9 &&
        output_capability != V4L2_PIX_FMT_HEVC)
        return;

    pthread_mutex_lock(&m_hdr10pluslock);
    is_list_empty = m_hdr10pluslist.empty();
    pthread_mutex_unlock(&m_hdr10pluslock);

    if (is_list_empty) {
        DEBUG_PRINT_HIGH("convert_hdr10plusinfo_to_metadata: hdr10plusinfo list is empty!");
        return;
    }

    pthread_mutex_lock(&m_hdr10pluslock);
    iter = m_hdr10pluslist.begin();
    while (iter != m_hdr10pluslist.end()) {
        if (iter->cookie == cookie && !iter->is_new) {
            colorData.dynamicMetaDataValid = true;
            colorData.dynamicMetaDataLen = iter->nParamSizeUsed;
            memcpy(colorData.dynamicMetaDataPayload, iter->payload,
                iter->nParamSizeUsed);
            DEBUG_PRINT_LOW("found hdr10plus metadata for cookie %u with timestamp %lld, size %u",
                cookie, iter->timestamp, colorData.dynamicMetaDataLen);
            break;
        }
        iter++;
    }
    pthread_mutex_unlock(&m_hdr10pluslock);
}

void omx_vdec::remove_hdr10plusinfo_using_cookie(OMX_PTR markdata)
{
    std::list<hdr10plusInfo>::iterator iter;
    unsigned int cookie = (unsigned int)(unsigned long)markdata;
    bool is_list_empty = false;

    if (output_capability != V4L2_PIX_FMT_VP9 &&
        output_capability != V4L2_PIX_FMT_HEVC)
        return;

    pthread_mutex_lock(&m_hdr10pluslock);
    is_list_empty = m_hdr10pluslist.empty();
    pthread_mutex_unlock(&m_hdr10pluslock);

    if (is_list_empty) {
        DEBUG_PRINT_HIGH("remove_hdr10plusinfo_using_cookie: hdr10plusinfo list is empty!");
        return;
    }

    pthread_mutex_lock(&m_hdr10pluslock);
    iter = m_hdr10pluslist.begin();
    while (iter != m_hdr10pluslist.end()) {
        if (iter->cookie == cookie && !iter->is_new) {
            iter = m_hdr10pluslist.erase(iter);
            DEBUG_PRINT_LOW("removed hdr10plusinfo from the list for the cookie %u", cookie);
            break;
        }
        iter++;
    }
    pthread_mutex_unlock(&m_hdr10pluslock);
}

void omx_vdec::clear_hdr10plusinfo()
{
    bool is_list_empty = false;

    if (output_capability != V4L2_PIX_FMT_VP9 &&
        output_capability != V4L2_PIX_FMT_HEVC)
        return;

    pthread_mutex_lock(&m_hdr10pluslock);
    is_list_empty = m_hdr10pluslist.empty();
    pthread_mutex_unlock(&m_hdr10pluslock);

    if (is_list_empty) {
        DEBUG_PRINT_HIGH("clear_hdr10plusinfo: hdr10plusinfo list is empty!");
        return;
    }

    pthread_mutex_lock(&m_hdr10pluslock);
    m_hdr10pluslist.clear();
    pthread_mutex_unlock(&m_hdr10pluslock);
}

void omx_vdec::get_hdr10plusinfo(DescribeHDR10PlusInfoParams *hdr10plusdata)
{
    bool is_list_empty = false;

    if (output_capability != V4L2_PIX_FMT_VP9 &&
        output_capability != V4L2_PIX_FMT_HEVC)
        return;

    pthread_mutex_lock(&m_hdr10pluslock);
    is_list_empty = m_hdr10pluslist.empty();
    pthread_mutex_unlock(&m_hdr10pluslock);

    if (is_list_empty) {
        DEBUG_PRINT_HIGH("get_hdr10plusinfo: hdr10plusinfo list is empty!");
        return;
    }

    pthread_mutex_lock(&m_hdr10pluslock);
    hdr10plusInfo item = m_hdr10pluslist.front();
    hdr10plusdata->nParamSizeUsed = item.nParamSizeUsed;
    memcpy(hdr10plusdata->nValue, item.payload,item.nParamSizeUsed);
    DEBUG_PRINT_LOW("found hdr10plus metadata with timestamp %lld, size %u",
        item.timestamp, item.nParamSizeUsed);
    m_hdr10pluslist.pop_front();
    pthread_mutex_unlock(&m_hdr10pluslock);
}

void omx_vdec::print_hdr10plusinfo(DescribeHDR10PlusInfoParams *hdr10plusdata)
{
         DEBUG_PRINT_LOW("HDR10+ frameworks path valid data length: %d", hdr10plusdata->nParamSizeUsed);
        for (uint32_t i = 0 ; i < hdr10plusdata->nParamSizeUsed && i+3 < 1024; i=i+4) {
            DEBUG_PRINT_LOW("HDR10+ mdata: %02X %02X %02X %02X", hdr10plusdata->nValue[i],
            hdr10plusdata->nValue[i+1],
            hdr10plusdata->nValue[i+2],
            hdr10plusdata->nValue[i+3]);
    }
}

void perf_metrics::start()
{
    if (!active) {
        start_time = get_act_time();
        active = true;
    }
}

void perf_metrics::stop()
{
    OMX_U64 stop_time = get_act_time();
    if (active) {
        proc_time += (stop_time - start_time);
        active = false;
    }
}

void perf_metrics::end(OMX_U32 units_cntr)
{
    stop();
    DEBUG_PRINT_LOW("--> Processing time : [%.2f] Sec", (float)proc_time / 1e6);
    if (units_cntr) {
        DEBUG_PRINT_LOW("--> Avrg proc time  : [%.2f] mSec", proc_time / (float)(units_cntr * 1e3));
    }
}

void perf_metrics::reset()
{
    start_time = 0;
    proc_time = 0;
    active = false;
}

OMX_U64 perf_metrics::get_act_time()
{
    struct timeval act_time = {0, 0};
    gettimeofday(&act_time, NULL);
    return (act_time.tv_usec + act_time.tv_sec * 1e6);
}

OMX_U64 perf_metrics::processing_time_us()
{
    return proc_time;
}

// No code beyond this !

// inline import of vendor-extensions implementation
#include "omx_vdec_extensions.hpp"
