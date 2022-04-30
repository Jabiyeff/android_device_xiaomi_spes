/*--------------------------------------------------------------------------
Copyright (c) 2010 - 2020, The Linux Foundation. All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

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
#ifndef __OMX_VDEC_H__
#define __OMX_VDEC_H__
/*============================================================================
                            O p e n M A X   Component
                                Video Decoder

*//** @file comx_vdec.h
  This module contains the class definition for openMAX decoder component.

*//*========================================================================*/

//////////////////////////////////////////////////////////////////////////////
//                             Include Files
//////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <cstddef>
#include <dlfcn.h>
#include <cutils/atomic.h>
#include <qdMetaData.h>
#include <color_metadata.h>
#define STRINGIFY_ENUMS
#include "media/hardware/VideoAPI.h"
#include "media/hardware/HardwareAPI.h"
#include <unordered_map>
#include <media/msm_media_info.h>
#include <OMX_Core.h>
#include <list>

#include <linux/msm_ion.h>
#if TARGET_ION_ABI_VERSION >= 2
#include <ion/ion.h>
#include <linux/dma-buf.h>
#else
#include <linux/ion.h>
#endif

#include "C2DColorConverter.h"

static ptrdiff_t x;

extern "C" {
#include <utils/Log.h>
}

#ifdef _ANDROID_
#undef LOG_TAG
#define LOG_TAG "OMX-VDEC-1080P"

#ifdef USE_ION
#include <linux/msm_ion.h>
//#include <binder/MemoryHeapIon.h>
//#else
#endif

#ifdef USE_GBM
#include "gbm.h"
#include "gbm_priv.h"
#endif

#ifndef NATIVE_BASE_DISABLE
#include <nativebase/nativebase.h>
#endif
#include <linux/videodev2.h>
#define VALID_TS(ts)      ((ts < LLONG_MAX)? true : false)
#include <poll.h>
#define TIMEOUT 5000
#endif // _ANDROID_

#if defined (_ANDROID_HONEYCOMB_) || defined (_ANDROID_ICS_)
#define STRINGIFY_ENUMS
#include <media/hardware/HardwareAPI.h>
#endif

#include <unistd.h>

#if defined (_ANDROID_ICS_)
#include <gralloc_priv.h>
#endif

#include <pthread.h>
#ifndef PC_DEBUG
#include <semaphore.h>
#endif
#include "OMX_Core.h"
#include "OMX_QCOMExtns.h"
#include "OMX_Skype_VideoExtensions.h"
#include "OMX_VideoExt.h"
#include "OMX_IndexExt.h"
#include "qc_omx_component.h"
#include "media/msm_vidc_utils.h"
#include "extra_data_handler.h"
#include "ts_parser.h"
#include "vidc_debug.h"
#include "vidc_common.h"
#include "vidc_vendor_extensions.h"
extern "C" {
    OMX_API void * get_omx_component_factory_fn(void);
}

//////////////////////////////////////////////////////////////////////////////
//                       Module specific globals
//////////////////////////////////////////////////////////////////////////////
#define OMX_SPEC_VERSION  0x00000101
#define OMX_INIT_STRUCT(_s_, _name_)         \
    memset((_s_), 0x0, sizeof(_name_));      \
(_s_)->nSize = sizeof(_name_);               \
(_s_)->nVersion.nVersion = OMX_SPEC_VERSION  \


//////////////////////////////////////////////////////////////////////////////
//               Macros
//////////////////////////////////////////////////////////////////////////////
#define PrintFrameHdr(bufHdr) DEBUG_PRINT("bufHdr %x buf %x size %d TS %d\n",\
        (unsigned) bufHdr,\
        (unsigned)((OMX_BUFFERHEADERTYPE *)bufHdr)->pBuffer,\
        (unsigned)((OMX_BUFFERHEADERTYPE *)bufHdr)->nFilledLen,\
        (unsigned)((OMX_BUFFERHEADERTYPE *)bufHdr)->nTimeStamp)

#define DEFAULT_FPS 30
#define MAX_SUPPORTED_FPS 960
#define DEFAULT_WIDTH_ALIGNMENT 128
#define DEFAULT_HEIGHT_ALIGNMENT 32

#define SZ_4K 0x1000
#define SZ_1M 0x100000

#define Log2(number, power)  { OMX_U32 temp = number; power = 0; while( (0 == (temp & 0x1)) &&  power < 16) { temp >>=0x1; power++; } }
#define Q16ToFraction(q,num,den) { OMX_U32 power; Log2(q,power);  num = q >> power; den = 0x1 << (16 - power); }
#define EXTRADATA_IDX(__num_planes) ((__num_planes) ? (__num_planes) - 1 : 0)
#undef ALIGN
#define ALIGN(x, to_align) ((((unsigned) x) + (to_align - 1)) & ~(to_align - 1))

// Y=16(0-9bits), Cb(10-19bits)=Cr(20-29bits)=128, black by default
#define DEFAULT_VIDEO_CONCEAL_COLOR_BLACK 0x8020010

#define OMX_CORE_CONTROL_CMDQ_SIZE   100
#define OMX_CORE_QCIF_HEIGHT         144
#define OMX_CORE_QCIF_WIDTH          176
#define OMX_CORE_VGA_HEIGHT          480
#define OMX_CORE_VGA_WIDTH           640
#define OMX_CORE_WVGA_HEIGHT         480
#define OMX_CORE_WVGA_WIDTH          800
#define OMX_CORE_FWVGA_HEIGHT        480
#define OMX_CORE_FWVGA_WIDTH         864

#define DESC_BUFFER_SIZE (8192 * 16)

#ifdef _ANDROID_
#define MAX_NUM_INPUT_OUTPUT_BUFFERS VIDEO_MAX_FRAME
#endif

// Aligning to MAX NUM INPUT & OUTPUT
#define MIN_NUM_INPUT_OUTPUT_EXTRADATA_BUFFERS MAX_NUM_INPUT_OUTPUT_BUFFERS

/* STATUS CODES */
/* Base value for status codes */
#define VDEC_S_BASE	0x40000000
/* Success */
#define VDEC_S_SUCCESS	(VDEC_S_BASE)
/* General failure */
#define VDEC_S_EFAIL	(VDEC_S_BASE + 1)
/* Fatal irrecoverable  failure. Need to  tear down session. */
#define VDEC_S_EFATAL   (VDEC_S_BASE + 2)
/* Error with input bistream */
#define VDEC_S_INPUT_BITSTREAM_ERR (VDEC_S_BASE + 3)

#define VDEC_MSG_BASE	0x0000000
/* Codes to identify asynchronous message responses and events that driver
  wants to communicate to the app.*/
#define VDEC_MSG_RESP_INPUT_BUFFER_DONE	(VDEC_MSG_BASE + 1)
#define VDEC_MSG_RESP_OUTPUT_BUFFER_DONE	(VDEC_MSG_BASE + 2)
#define VDEC_MSG_RESP_INPUT_FLUSHED	(VDEC_MSG_BASE + 3)
#define VDEC_MSG_RESP_OUTPUT_FLUSHED	(VDEC_MSG_BASE + 4)
#define VDEC_MSG_RESP_FLUSH_INPUT_DONE	(VDEC_MSG_BASE + 5)
#define VDEC_MSG_RESP_FLUSH_OUTPUT_DONE	(VDEC_MSG_BASE + 6)
#define VDEC_MSG_RESP_START_DONE	(VDEC_MSG_BASE + 7)
#define VDEC_MSG_RESP_STOP_DONE	(VDEC_MSG_BASE + 8)
#define VDEC_MSG_RESP_PAUSE_DONE	(VDEC_MSG_BASE + 9)
#define VDEC_MSG_RESP_RESUME_DONE	(VDEC_MSG_BASE + 10)
#define VDEC_MSG_EVT_CONFIG_CHANGED	(VDEC_MSG_BASE + 11)
#define VDEC_MSG_EVT_HW_ERROR	(VDEC_MSG_BASE + 12)
#define VDEC_MSG_EVT_INFO_FIELD_DROPPED	(VDEC_MSG_BASE + 13)
#define VDEC_MSG_EVT_HW_OVERLOAD	(VDEC_MSG_BASE + 14)
#define VDEC_MSG_EVT_MAX_CLIENTS	(VDEC_MSG_BASE + 15)
#define VDEC_MSG_EVT_HW_UNSUPPORTED	(VDEC_MSG_BASE + 16)

using namespace android;

enum port_indexes {
    OMX_CORE_INPUT_PORT_INDEX        =0,
    OMX_CORE_OUTPUT_PORT_INDEX       =1,
    OMX_CORE_INPUT_EXTRADATA_INDEX   =2,
    OMX_CORE_OUTPUT_EXTRADATA_INDEX  =3
};

enum vdec_codec {
	VDEC_CODECTYPE_H264 = 0x1,
	VDEC_CODECTYPE_H263 = 0x2,
	VDEC_CODECTYPE_MPEG4 = 0x3,
	VDEC_CODECTYPE_DIVX_3 = 0x4,
	VDEC_CODECTYPE_DIVX_4 = 0x5,
	VDEC_CODECTYPE_DIVX_5 = 0x6,
	VDEC_CODECTYPE_DIVX_6 = 0x7,
	VDEC_CODECTYPE_XVID = 0x8,
	VDEC_CODECTYPE_MPEG1 = 0x9,
	VDEC_CODECTYPE_MPEG2 = 0xa,
	VDEC_CODECTYPE_VC1 = 0xb,
	VDEC_CODECTYPE_VC1_RCV = 0xc,
	VDEC_CODECTYPE_HEVC = 0xd,
	VDEC_CODECTYPE_MVC = 0xe,
	VDEC_CODECTYPE_VP8 = 0xf,
	VDEC_CODECTYPE_VP9 = 0x10,
};

enum vdec_output_format {
	VDEC_YUV_FORMAT_NV12 = 0x1,
	VDEC_YUV_FORMAT_TILE_4x2 = 0x2,
	VDEC_YUV_FORMAT_NV12_UBWC = 0x3,
	VDEC_YUV_FORMAT_NV12_TP10_UBWC = 0x4,
	VDEC_YUV_FORMAT_P010_VENUS = 0x5,
};

enum vdec_interlaced_format {
    VDEC_InterlaceFrameProgressive = 0x1,
    VDEC_InterlaceInterleaveFrameTopFieldFirst = 0x2,
    VDEC_InterlaceInterleaveFrameBottomFieldFirst = 0x4,
    VDEC_InterlaceFrameTopFieldFirst = 0x8,
    VDEC_InterlaceFrameBottomFieldFirst = 0x10,
};

enum vdec_output_order {
	VDEC_ORDER_DISPLAY = 0x1,
	VDEC_ORDER_DECODE = 0x2
};

struct vdec_framesize {
	uint32_t   left;
	uint32_t   top;
	uint32_t   right;
	uint32_t   bottom;
};

struct vdec_picsize {
	uint32_t frame_width;
	uint32_t frame_height;
	uint32_t stride;
	uint32_t scan_lines;
};

enum vdec_buffer {
	VDEC_BUFFER_TYPE_INPUT,
	VDEC_BUFFER_TYPE_OUTPUT
};

struct vdec_allocatorproperty {
	enum vdec_buffer buffer_type;
	uint32_t mincount;
	uint32_t maxcount;
	uint32_t actualcount;
	size_t buffer_size;
	uint32_t alignment;
	uint32_t buf_poolid;
	size_t meta_buffer_size;
};

struct vdec_bufferpayload {
	void *bufferaddr;
	size_t buffer_len;
	int pmem_fd;
	size_t offset;
	size_t mmaped_size;
};

enum vdec_picture {
	PICTURE_TYPE_I,
	PICTURE_TYPE_P,
	PICTURE_TYPE_B,
	PICTURE_TYPE_BI,
	PICTURE_TYPE_SKIP,
	PICTURE_TYPE_IDR,
	PICTURE_TYPE_UNKNOWN
};

struct vdec_misrinfo {
        uint32_t misr_dpb_luma;
        uint32_t misr_dpb_chroma;
        uint32_t misr_opb_luma;
        uint32_t misr_opb_chroma;
};

struct vdec_output_frameinfo {
	void *bufferaddr;
	size_t offset;
	size_t len;
	uint32_t flags;
	uint64_t time_stamp;
	enum vdec_picture pic_type;
	void *client_data;
	struct vdec_picsize picsize;
	struct vdec_framesize framesize;
};

union vdec_msgdata {
	struct vdec_output_frameinfo output_frame;
	void *input_frame_clientdata;
};

struct vdec_msginfo {
	uint32_t status_code;
	uint32_t msgcode;
	union vdec_msgdata msgdata;
	size_t msgdatasize;
};

struct vdec_framerate {
	unsigned long fps_denominator;
	unsigned long fps_numerator;
};

struct hdr10plusInfo {
    bool is_new;
    unsigned int cookie;
    OMX_TICKS timestamp;
    OMX_U32 nSize;
    OMX_VERSIONTYPE nVersion;
    OMX_U32 nPortIndex;
    OMX_U32 nParamSize;
    OMX_U32 nParamSizeUsed;
    OMX_U8 payload[MAX_HDR10PLUSINFO_SIZE];
};

#ifdef USE_ION
struct vdec_ion {
    int dev_fd;
    struct ion_allocation_data alloc_data;
    int data_fd;
    char *uaddr;
};
#endif

#ifdef USE_GBM
struct vdec_gbm {
    int gbm_device_fd;
    struct gbm_device *gbm;
    struct gbm_bo *bo;
    unsigned long bo_fd;
    unsigned long meta_fd;
};
#endif


struct extradata_buffer_info {
    unsigned long buffer_size;
    int count;
#ifdef USE_ION
    struct vdec_ion ion[VIDEO_MAX_FRAME];
#endif
};

struct video_driver_context {
    int video_driver_fd;
    enum vdec_codec decoder_format;
    enum vdec_output_format output_format;
    enum vdec_interlaced_format interlace;
    enum vdec_output_order picture_order;
    struct vdec_framesize frame_size;
    struct vdec_picsize video_resolution;
    struct vdec_allocatorproperty ip_buf;
    struct vdec_allocatorproperty op_buf;
    struct vdec_bufferpayload *ptr_inputbuffer;
    struct vdec_bufferpayload *ptr_outputbuffer;
    struct vdec_output_frameinfo *ptr_respbuffer;
    struct vdec_bufferpayload *ptr_intermediate_outputbuffer;
    struct vdec_output_frameinfo *ptr_intermediate_respbuffer;
#ifdef USE_ION
    struct vdec_ion *ip_buf_ion_info;
    struct vdec_ion *op_buf_ion_info;
    struct vdec_ion *op_intermediate_buf_ion_info;
    struct vdec_ion h264_mv;
    struct vdec_ion meta_buffer;
    struct vdec_ion meta_buffer_iommu;
#endif
#ifdef USE_GBM
    int gbm_device_fd;
    struct vdec_gbm *op_buf_gbm_info;
    struct vdec_gbm *op_intermediate_buf_gbm_info;
#endif
    struct vdec_framerate frame_rate;
    bool timestamp_adjust;
    char kind[128];
    bool idr_only_decoding;
    unsigned disable_dmx;
    struct extradata_buffer_info extradata_info;
    int num_planes;
};

struct video_decoder_capability {
    unsigned int min_width;
    unsigned int max_width;
    unsigned int min_height;
    unsigned int max_height;
};

struct dynamic_buf_list {
    long fd;
    long dup_fd;
    OMX_U32 offset;
    OMX_U32 ref_count;
    void *buffaddr;
    long mapped_size;
};

struct extradata_misr_info {
    OMX_BOOL output_crop_updated;
    OMX_CONFIG_RECTTYPE output_crop_rect;
    OMX_U32 output_width;
    OMX_U32 output_height;
    OMX_QCOM_MISR_INFO misr_info[2];
};

struct reconfig_client_data {
    OMX_U32 width;
    OMX_U32 height;
    OMX_U32 dpb_bit_depth;
    OMX_U32 m_progressive;
    bool isPortReconfigInsufficient;
};

struct reconfig_client_crop_data {
    OMX_U32 width;
    OMX_U32 height;
    OMX_U32 left;
    OMX_U32 top;
    bool isPortReconfigInsufficient;
};

typedef std::unordered_map <int, int> ColorSubMapping;
typedef std::unordered_map <int, ColorSubMapping> DecColorMapping;
typedef std::unordered_map <enum ColorAspects::Primaries, ColorPrimaries> PrimariesMap;
typedef std::unordered_map <enum ColorAspects::Transfer, GammaTransfer> TransferMap;
typedef std::unordered_map <enum ColorAspects::MatrixCoeffs, MatrixCoEfficients> MatrixCoeffMap;
typedef std::unordered_map <enum ColorAspects::Range, ColorRange> RangeMap;

class perf_metrics
{
    public:
        perf_metrics() :
            start_time(0),
            proc_time(0),
            active(false) {
            };
        ~perf_metrics() {};
        void start();
        void stop();
        void end(OMX_U32 units_cntr = 0);
        void reset();
        OMX_U64 processing_time_us();
    private:
        inline OMX_U64 get_act_time();
        OMX_U64 start_time;
        OMX_U64 proc_time;
        bool active;
};

// OMX video decoder class
class omx_vdec: public qc_omx_component
{

    public:
        omx_vdec();  // constructor
        virtual ~omx_vdec();  // destructor

        static int async_message_process (void *context, void* message);
        static void process_event_cb(void *ctxt);

        OMX_ERRORTYPE allocate_buffer(
                OMX_HANDLETYPE hComp,
                OMX_BUFFERHEADERTYPE **bufferHdr,
                OMX_U32 port,
                OMX_PTR appData,
                OMX_U32 bytes
                );


        OMX_ERRORTYPE component_deinit(OMX_HANDLETYPE hComp);

        OMX_ERRORTYPE component_init(OMX_STRING role);

        OMX_ERRORTYPE component_role_enum(
                OMX_HANDLETYPE hComp,
                OMX_U8 *role,
                OMX_U32 index
                );

        OMX_ERRORTYPE component_tunnel_request(
                OMX_HANDLETYPE hComp,
                OMX_U32 port,
                OMX_HANDLETYPE  peerComponent,
                OMX_U32 peerPort,
                OMX_TUNNELSETUPTYPE *tunnelSetup
                );

        OMX_ERRORTYPE empty_this_buffer(
                OMX_HANDLETYPE hComp,
                OMX_BUFFERHEADERTYPE *buffer
                );



        OMX_ERRORTYPE fill_this_buffer(
                OMX_HANDLETYPE hComp,
                OMX_BUFFERHEADERTYPE *buffer
                );


        OMX_ERRORTYPE free_buffer(
                OMX_HANDLETYPE hComp,
                OMX_U32 port,
                OMX_BUFFERHEADERTYPE *buffer
                );

        OMX_ERRORTYPE get_component_version(
                OMX_HANDLETYPE hComp,
                OMX_STRING componentName,
                OMX_VERSIONTYPE *componentVersion,
                OMX_VERSIONTYPE *specVersion,
                OMX_UUIDTYPE *componentUUID
                );

        OMX_ERRORTYPE get_config(
                OMX_HANDLETYPE hComp,
                OMX_INDEXTYPE configIndex,
                OMX_PTR configData
                );

        OMX_ERRORTYPE get_extension_index(
                OMX_HANDLETYPE hComp,
                OMX_STRING paramName,
                OMX_INDEXTYPE *indexType
                );

        OMX_ERRORTYPE get_parameter(OMX_HANDLETYPE hComp,
                OMX_INDEXTYPE  paramIndex,
                OMX_PTR        paramData);

        OMX_ERRORTYPE get_state(OMX_HANDLETYPE hComp,
                OMX_STATETYPE *state);



        OMX_ERRORTYPE send_command(OMX_HANDLETYPE  hComp,
                OMX_COMMANDTYPE cmd,
                OMX_U32         param1,
                OMX_PTR         cmdData);


        OMX_ERRORTYPE set_callbacks(OMX_HANDLETYPE   hComp,
                OMX_CALLBACKTYPE *callbacks,
                OMX_PTR          appData);

        OMX_ERRORTYPE set_config(OMX_HANDLETYPE hComp,
                OMX_INDEXTYPE  configIndex,
                OMX_PTR        configData);

        OMX_ERRORTYPE set_parameter(OMX_HANDLETYPE hComp,
                OMX_INDEXTYPE  paramIndex,
                OMX_PTR        paramData);

        OMX_ERRORTYPE use_buffer(OMX_HANDLETYPE      hComp,
                OMX_BUFFERHEADERTYPE **bufferHdr,
                OMX_U32              port,
                OMX_PTR              appData,
                OMX_U32              bytes,
                OMX_U8               *buffer);

        OMX_ERRORTYPE  use_input_heap_buffers(
                OMX_HANDLETYPE            hComp,
                OMX_BUFFERHEADERTYPE** bufferHdr,
                OMX_U32                   port,
                OMX_PTR                   appData,
                OMX_U32                   bytes,
                OMX_U8*                   buffer);

        OMX_ERRORTYPE use_EGL_image(OMX_HANDLETYPE     hComp,
                OMX_BUFFERHEADERTYPE **bufferHdr,
                OMX_U32              port,
                OMX_PTR              appData,
                void *               eglImage);
        void complete_pending_buffer_done_cbs();
        struct video_driver_context drv_ctx;
        int m_poll_efd;
        char *ion_map(int fd, int len);
        OMX_ERRORTYPE ion_unmap(int fd, void *bufaddr, int len);
        OMX_ERRORTYPE allocate_extradata();
        void free_extradata();
        int update_resolution(int width, int height, int stride, int scan_lines);
        Signal signal;
        pthread_t msg_thread_id;
        pthread_t async_thread_id;
        bool is_component_secure();
        OMX_BUFFERHEADERTYPE* get_omx_output_buffer_header(int index);
        int dpb_bit_depth;
        bool check_supported_flexible_formats(OMX_COLOR_FORMATTYPE required_format);
        bool is_flexible_format;//To save status if required format is flexible color formats
        bool async_thread_force_stop;
        volatile bool message_thread_stop;
        struct extradata_misr_info m_extradata_misr;
        int m_progressive;
        bool is_mbaff;

        enum color_space_type {
            BT2020 = 0,
            EXCEPT_BT2020,
            UNKNOWN
        };
        enum color_space_type m_color_space;

        /*
         * Class variables to track client set profile and level
         * via OMX_QTIIndexParamClientConfiguredMaxProfileLevelForSufficiency
         */
        bool isPortReconfigInsufficient;
        inline int get_session_codec_type();

    private:
        // Bit Positions
        enum flags_bit_positions {
            // Defer transition to IDLE
            OMX_COMPONENT_IDLE_PENDING            =0x1,
            // Defer transition to LOADING
            OMX_COMPONENT_LOADING_PENDING         =0x2,
            // First  Buffer Pending
            OMX_COMPONENT_FIRST_BUFFER_PENDING    =0x3,
            // Second Buffer Pending
            OMX_COMPONENT_SECOND_BUFFER_PENDING   =0x4,
            // Defer transition to Enable
            OMX_COMPONENT_INPUT_ENABLE_PENDING    =0x5,
            // Defer transition to Enable
            OMX_COMPONENT_OUTPUT_ENABLE_PENDING   =0x6,
            // Defer transition to Disable
            OMX_COMPONENT_INPUT_DISABLE_PENDING   =0x7,
            // Defer transition to Disable
            OMX_COMPONENT_OUTPUT_DISABLE_PENDING  =0x8,
            //defer flush notification
            OMX_COMPONENT_OUTPUT_FLUSH_PENDING    =0x9,
            OMX_COMPONENT_INPUT_FLUSH_PENDING    =0xA,
            OMX_COMPONENT_PAUSE_PENDING          =0xB,
            OMX_COMPONENT_EXECUTE_PENDING        =0xC,
            OMX_COMPONENT_OUTPUT_FLUSH_IN_DISABLE_PENDING =0xD,
            OMX_COMPONENT_DISABLE_OUTPUT_DEFERRED=0xE,
            OMX_COMPONENT_FLUSH_DEFERRED = 0xF
        };

        // Deferred callback identifiers
        enum {
            //Event Callbacks from the vdec component thread context
            OMX_COMPONENT_GENERATE_EVENT       = 0x1,
            //Buffer Done callbacks from the vdec component thread context
            OMX_COMPONENT_GENERATE_BUFFER_DONE = 0x2,
            //Frame Done callbacks from the vdec component thread context
            OMX_COMPONENT_GENERATE_FRAME_DONE  = 0x3,
            //Buffer Done callbacks from the vdec component thread context
            OMX_COMPONENT_GENERATE_FTB         = 0x4,
            //Frame Done callbacks from the vdec component thread context
            OMX_COMPONENT_GENERATE_ETB         = 0x5,
            //Command
            OMX_COMPONENT_GENERATE_COMMAND     = 0x6,
            //Push-Pending Buffers
            OMX_COMPONENT_PUSH_PENDING_BUFS    = 0x7,
            // Empty Buffer Done callbacks
            OMX_COMPONENT_GENERATE_EBD         = 0x8,
            //Flush Event Callbacks from the vdec component thread context
            OMX_COMPONENT_GENERATE_EVENT_FLUSH       = 0x9,
            OMX_COMPONENT_GENERATE_EVENT_INPUT_FLUSH = 0x0A,
            OMX_COMPONENT_GENERATE_EVENT_OUTPUT_FLUSH = 0x0B,
            OMX_COMPONENT_GENERATE_FBD = 0xC,
            OMX_COMPONENT_GENERATE_START_DONE = 0xD,
            OMX_COMPONENT_GENERATE_PAUSE_DONE = 0xE,
            OMX_COMPONENT_GENERATE_RESUME_DONE = 0xF,
            OMX_COMPONENT_GENERATE_STOP_DONE = 0x10,
            OMX_COMPONENT_GENERATE_HARDWARE_ERROR = 0x11,
            OMX_COMPONENT_GENERATE_ETB_ARBITRARY = 0x12,
            OMX_COMPONENT_GENERATE_PORT_RECONFIG = 0x13,
            OMX_COMPONENT_GENERATE_EOS_DONE = 0x14,
            OMX_COMPONENT_GENERATE_INFO_PORT_RECONFIG = 0x15,
            OMX_COMPONENT_GENERATE_INFO_FIELD_DROPPED = 0x16,
            OMX_COMPONENT_GENERATE_UNSUPPORTED_SETTING = 0x17,
            OMX_COMPONENT_GENERATE_HARDWARE_OVERLOAD = 0x18,
            OMX_COMPONENT_GENERATE_ION_PREFETCH_PIXEL = 0x19,
            OMX_COMPONENT_GENERATE_ION_PREFETCH_NON_PIXEL = 0x1A,
            OMX_COMPONENT_CLOSE_MSG = 0x1B
        };

        enum vc1_profile_type {
            VC1_SP_MP_RCV = 1,
            VC1_AP = 2
        };

        enum v4l2_ports {
            CAPTURE_PORT,
            OUTPUT_PORT,
            MAX_PORT
        };

        struct omx_event {
            unsigned long param1;
            unsigned long param2;
            unsigned long id;
        };

        struct omx_cmd_queue {
            omx_event m_q[OMX_CORE_CONTROL_CMDQ_SIZE];
            unsigned long m_read;
            unsigned long m_write;
            unsigned long m_size;

            omx_cmd_queue();
            ~omx_cmd_queue();
            bool insert_entry(unsigned long p1, unsigned long p2, unsigned long id);
            bool pop_entry(unsigned long *p1,unsigned long *p2, unsigned long *id);
            // get msgtype of the first ele from the queue
            unsigned get_q_msg_type();

        };
        struct v4l2_capability cap;
#ifdef _ANDROID_
        struct ts_entry {
            OMX_TICKS timestamp;
            bool valid;
        };

        struct ts_arr_list {
            ts_entry m_ts_arr_list[MAX_NUM_INPUT_OUTPUT_BUFFERS];

            ts_arr_list();
            ~ts_arr_list();

            bool insert_ts(OMX_TICKS ts);
            bool pop_min_ts(OMX_TICKS &ts);
            bool reset_ts_list();
        };
#endif

        struct desc_buffer_hdr {
            OMX_U8 *buf_addr;
            OMX_U32 desc_data_size;
        };
        bool allocate_done(void);
        bool allocate_input_done(void);
        bool allocate_output_done(void);
        bool allocate_output_extradata_done(void);

        OMX_ERRORTYPE free_input_buffer(OMX_BUFFERHEADERTYPE *bufferHdr);
        OMX_ERRORTYPE free_input_buffer(unsigned int bufferindex,
                OMX_BUFFERHEADERTYPE *pmem_bufferHdr);
        OMX_ERRORTYPE free_output_buffer(OMX_BUFFERHEADERTYPE *bufferHdr,
                                         bool                 intermediate=false);
        void free_output_buffer_header(bool intermediate=false);
        void free_input_buffer_header();
        void free_output_extradata_buffer_header();

        OMX_ERRORTYPE allocate_input_heap_buffer(OMX_HANDLETYPE       hComp,
                OMX_BUFFERHEADERTYPE **bufferHdr,
                OMX_U32              port,
                OMX_PTR              appData,
                OMX_U32              bytes);


        OMX_ERRORTYPE allocate_input_buffer(OMX_HANDLETYPE       hComp,
                OMX_BUFFERHEADERTYPE **bufferHdr,
                OMX_U32              port,
                OMX_PTR              appData,
                OMX_U32              bytes);

        OMX_ERRORTYPE allocate_output_buffer(OMX_HANDLETYPE       hComp,
                OMX_BUFFERHEADERTYPE **bufferHdr,
                OMX_U32 port,OMX_PTR appData,
                OMX_U32              bytes,
                bool                 intermediate=false,
                int                  index=-1);
        OMX_ERRORTYPE use_output_buffer(OMX_HANDLETYPE hComp,
                OMX_BUFFERHEADERTYPE   **bufferHdr,
                OMX_U32                port,
                OMX_PTR                appData,
                OMX_U32                bytes,
                OMX_U8                 *buffer);
        OMX_ERRORTYPE use_client_output_extradata_buffer(OMX_HANDLETYPE hComp,
                OMX_BUFFERHEADERTYPE   **bufferHdr,
                OMX_U32                port,
                OMX_PTR                appData,
                OMX_U32                bytes,
                OMX_U8                 *buffer);
        OMX_ERRORTYPE get_supported_profile_level(OMX_VIDEO_PARAM_PROFILELEVELTYPE *profileLevelType);

        OMX_ERRORTYPE allocate_desc_buffer(OMX_U32 index);
        OMX_ERRORTYPE allocate_output_headers(bool intermediate=false);
        OMX_ERRORTYPE allocate_client_output_extradata_headers();
        bool execute_omx_flush(OMX_U32);
        bool execute_output_flush();
        bool execute_input_flush();
        void notify_flush_done(void *ctxt);
        OMX_ERRORTYPE empty_buffer_done(OMX_HANDLETYPE hComp,
                OMX_BUFFERHEADERTYPE * buffer);

        OMX_ERRORTYPE fill_buffer_done(OMX_HANDLETYPE hComp,
                OMX_BUFFERHEADERTYPE * buffer);
        OMX_ERRORTYPE empty_this_buffer_proxy(OMX_HANDLETYPE       hComp,
                OMX_BUFFERHEADERTYPE *buffer);

        OMX_ERRORTYPE empty_this_buffer_proxy_arbitrary(OMX_HANDLETYPE hComp,
                OMX_BUFFERHEADERTYPE *buffer
                );

        OMX_ERRORTYPE push_input_buffer (OMX_HANDLETYPE hComp);
        OMX_ERRORTYPE push_input_sc_codec (OMX_HANDLETYPE hComp);
        OMX_ERRORTYPE push_input_h264 (OMX_HANDLETYPE hComp);
        OMX_ERRORTYPE push_input_hevc (OMX_HANDLETYPE hComp);

        OMX_ERRORTYPE fill_this_buffer_proxy(OMX_HANDLETYPE       hComp,
                OMX_BUFFERHEADERTYPE *buffer);
        bool release_done();

        bool release_output_done();
        bool release_input_done();
        bool release_output_extradata_done();
        OMX_ERRORTYPE get_buffer_req(vdec_allocatorproperty *buffer_prop);
        OMX_ERRORTYPE set_buffer_req(vdec_allocatorproperty *buffer_prop);
        OMX_ERRORTYPE start_port_reconfig();
        OMX_ERRORTYPE update_picture_resolution();
        int stream_off(OMX_U32 port);
        void adjust_timestamp(OMX_S64 &act_timestamp);
        void set_frame_rate(OMX_S64 act_timestamp);
        bool handle_extradata(OMX_BUFFERHEADERTYPE *p_buf_hdr);
        void convert_color_space_info(OMX_U32 primaries,
            OMX_U32 transfer, OMX_U32 matrix,
            ColorAspects *aspects);
        bool handle_color_space_info(void *data);
        bool handle_content_light_level_info(void* data);
        bool handle_mastering_display_color_info(void* data);
        OMX_ERRORTYPE enable_extradata(OMX_U64 requested_extradata);
        OMX_ERRORTYPE update_portdef(OMX_PARAM_PORTDEFINITIONTYPE *portDefn);
        void fix_drv_output_format();
        void insert_demux_addr_offset(OMX_U32 address_offset);
        void extract_demux_addr_offsets(OMX_BUFFERHEADERTYPE *buf_hdr);
        OMX_ERRORTYPE handle_demux_data(OMX_BUFFERHEADERTYPE *buf_hdr);
        OMX_U32 count_MB_in_extradata(OMX_OTHER_EXTRADATATYPE *extra);
        void set_histogram_metadata(private_handle_t *private_handle);
        struct VideoHistogramMetadata m_hist_metadata;

        bool align_pmem_buffers(int pmem_fd, OMX_U32 buffer_size,
                OMX_U32 alignment);
#ifdef USE_ION
        bool alloc_map_ion_memory(OMX_U32 buffer_size, vdec_ion *ion_info, int flag);
        void free_ion_memory(struct vdec_ion *buf_ion_info);
#endif

#ifdef USE_GBM
        bool alloc_map_gbm_memory(OMX_U32 w,OMX_U32 h,int gbm_device_fd,
              struct vdec_gbm *gbm_info, int flag);
        void free_gbm_memory(struct vdec_gbm *buf_gbm_info);
#endif



        OMX_ERRORTYPE send_command_proxy(OMX_HANDLETYPE  hComp,
                OMX_COMMANDTYPE cmd,
                OMX_U32         param1,
                OMX_PTR         cmdData);
        bool post_event( unsigned long p1,
                unsigned long p2,
                unsigned long id
                   );
        inline int clip2(int x) {
            x = x -1;
            x = x | x >> 1;
            x = x | x >> 2;
            x = x | x >> 4;
            x = x | x >> 16;
            x = x + 1;
            return x;
        }

        OMX_ERRORTYPE vdec_alloc_h264_mv();
        void vdec_dealloc_h264_mv();
        OMX_ERRORTYPE vdec_alloc_meta_buffers();
        void vdec_dealloc_meta_buffers();

        inline void omx_report_error () {
            if (m_cb.EventHandler && !m_error_propogated && m_state != OMX_StateLoaded) {
                DEBUG_PRINT_ERROR("ERROR: Sending OMX_ErrorHardware to Client");
                m_error_propogated = true;
                m_cb.EventHandler(&m_cmp,m_app_data,
                        OMX_EventError,OMX_ErrorHardware,0,NULL);
            }
        }

        inline void omx_report_unsupported_setting () {
            if (m_cb.EventHandler && !m_error_propogated && m_state != OMX_StateLoaded) {
                DEBUG_PRINT_ERROR(
                        "ERROR: Sending OMX_ErrorUnsupportedSetting to Client");
                m_error_propogated = true;
                m_cb.EventHandler(&m_cmp, m_app_data,
                        OMX_EventError, OMX_ErrorUnsupportedSetting, 0, NULL);
            }
        }
        inline void omx_report_hw_overload () {
            if (m_cb.EventHandler && !m_error_propogated && m_state != OMX_StateLoaded) {
                DEBUG_PRINT_ERROR(
                        "ERROR: Sending OMX_ErrorInsufficientResources to Client");
                m_error_propogated = true;
                m_cb.EventHandler(&m_cmp, m_app_data,
                        OMX_EventError, OMX_ErrorInsufficientResources, 0, NULL);
            }
        }

#if defined (_ANDROID_HONEYCOMB_) || defined (_ANDROID_ICS_)
        OMX_ERRORTYPE use_android_native_buffer(OMX_IN OMX_HANDLETYPE hComp, OMX_PTR data);
#endif
#if defined (_ANDROID_ICS_)
        struct nativebuffer {
            native_handle_t *nativehandle;
            private_handle_t *privatehandle;
            int inuse;
        };
        nativebuffer native_buffer[MAX_NUM_INPUT_OUTPUT_BUFFERS];
#endif

        //*************************************************************
        //*******************MEMBER VARIABLES *************************
        //*************************************************************
        pthread_mutex_t       m_lock;
        pthread_mutex_t       c_lock;
        pthread_mutex_t       buf_lock;
        //sem to handle the minimum procesing of commands
        sem_t                 m_cmd_lock;
        sem_t                 m_safe_flush;
        bool              m_error_propogated;
        // compression format
        OMX_VIDEO_CODINGTYPE eCompressionFormat;
        // OMX State
        OMX_STATETYPE m_state;
        // Application data
        OMX_PTR m_app_data;
        // Application callbacks
        OMX_CALLBACKTYPE m_cb;
        OMX_PRIORITYMGMTTYPE m_priority_mgm ;
        OMX_PARAM_BUFFERSUPPLIERTYPE m_buffer_supplier;
        // fill this buffer queue
        omx_cmd_queue         m_ftb_q;
        // Command Q for rest of the events
        omx_cmd_queue         m_cmd_q;
        omx_cmd_queue         m_etb_q;
        // Input memory pointer
        OMX_BUFFERHEADERTYPE  *m_inp_mem_ptr;
        // Output memory pointer
        OMX_BUFFERHEADERTYPE  *m_out_mem_ptr;
        // Output memory pointer
        OMX_BUFFERHEADERTYPE  *m_intermediate_out_mem_ptr;
        // Client extradata memory pointer
        OMX_BUFFERHEADERTYPE  *m_client_output_extradata_mem_ptr;
        // number of input bitstream error frame count
        unsigned int m_inp_err_count;

        pthread_mutex_t m_hdr10pluslock;
        std::list<hdr10plusInfo> m_hdr10pluslist;
#ifdef _ANDROID_
        // Timestamp list
        ts_arr_list           m_timestamp_list;
#endif

        char m_platform_name[PROPERTY_VALUE_MAX] = {0};
        bool input_flush_progress;
        bool output_flush_progress;
        bool input_use_buffer;
        bool output_use_buffer;
        bool ouput_egl_buffers;
        OMX_BOOL m_use_output_pmem;

        int pending_input_buffers;
        int pending_output_buffers;
        // bitmask array size for output side
        uint64_t m_out_bm_count;
        // bitmask array size for input side
        uint64_t m_inp_bm_count;
        // bitmask array size for extradata
        uint64_t m_out_extradata_bm_count;
        //Input port Populated
        OMX_BOOL m_inp_bPopulated;
        //Output port Populated
        OMX_BOOL m_out_bPopulated;
        // encapsulate the waiting states.
        uint64_t m_flags;

        OMX_U32 m_etb_count;
        OMX_TICKS m_etb_timestamp;
        // store I/P PORT state
        OMX_BOOL m_inp_bEnabled;
        // store O/P PORT state
        OMX_BOOL m_out_bEnabled;
        OMX_U32 m_in_alloc_cnt;
        OMX_U8                m_cRole[OMX_MAX_STRINGNAME_SIZE];
        // Platform specific details
        OMX_QCOM_PLATFORM_PRIVATE_LIST      *m_platform_list;
        OMX_QCOM_PLATFORM_PRIVATE_ENTRY     *m_platform_entry;
        OMX_QCOM_PLATFORM_PRIVATE_PMEM_INFO *m_pmem_info;
        // SPS+PPS sent as part of set_config
        OMX_VENDOR_EXTRADATATYPE            m_vendor_config;

        omx_cmd_queue m_input_pending_q;
        omx_cmd_queue m_input_free_q;
        OMX_BUFFERHEADERTYPE  *psource_frame;
        OMX_BUFFERHEADERTYPE  *pdest_frame;
        OMX_BUFFERHEADERTYPE  *m_inp_heap_ptr;
        OMX_BUFFERHEADERTYPE  **m_phdr_pmem_ptr;
        unsigned int m_heap_inp_bm_count;
        bool first_frame_meta;
        unsigned frame_count;
        unsigned nal_count;
        unsigned nal_length;
        int first_frame;
        unsigned char *first_buffer;
        int first_frame_size;
        unsigned char m_hwdevice_name[80];
        FILE *m_device_file_ptr;
        enum vc1_profile_type m_vc1_profile;
        OMX_U32 m_demux_offsets[8192];
        OMX_U32 m_demux_entries;
        OMX_U32 m_disp_hor_size;
        OMX_U32 m_disp_vert_size;
        OMX_S64 prev_ts;
        OMX_U32 frm_int;
        OMX_U32 m_fps_received;

        struct vdec_allocatorproperty op_buf_rcnfg;
        bool in_reconfig;
        bool c2d_enable_pending;
        OMX_NATIVE_WINDOWTYPE m_display_id;
        OMX_U32 m_client_extradata;
#ifdef _ANDROID_
        bool perf_flag;
        OMX_U32 proc_frms, latency;
        perf_metrics fps_metrics;
        perf_metrics dec_time;
        bool m_enable_android_native_buffers;
        bool m_use_android_native_buffers;
        bool m_disable_dynamic_buf_mode;
#endif

        struct meta_buffer {
            unsigned char* buffer;
            int size;
            int count;
            int pmem_fd;
            int pmem_fd_iommu;
            int offset;
        };
        meta_buffer meta_buff;
        OMX_PARAM_PORTDEFINITIONTYPE m_port_def;
        omx_time_stamp_reorder time_stamp_dts;
        desc_buffer_hdr *m_desc_buffer_ptr;
        bool secure_mode;
        bool allocate_native_handle;
        bool external_meta_buffer;
        bool external_meta_buffer_iommu;
        OMX_QCOM_EXTRADATA_FRAMEINFO *m_extradata;
        bool codec_config_flag;
        uint32_t capture_capability;
        int output_capability;
        bool streaming[MAX_PORT];
        OMX_FRAMESIZETYPE framesize;
        OMX_CONFIG_RECTTYPE rectangle;
        OMX_U32 prev_n_filled_len;
        struct custom_buffersize {
            OMX_U32 input_buffersize;
        } m_custom_buffersize;
        OMX_ERRORTYPE power_module_register();
        OMX_ERRORTYPE power_module_deregister();
        bool msg_thread_created;
        bool async_thread_created;

        OMX_VIDEO_PARAM_PROFILELEVELTYPE clientSet_profile_level;
        QOMX_EXTNINDEX_VIDEO_LOW_LATENCY_MODE m_sParamLowLatency;

        //variables to handle dynamic buffer mode
        bool dynamic_buf_mode;
        OMX_U32 m_reconfig_width;
        OMX_U32 m_reconfig_height;
        bool m_decode_order_mode;

        bool m_input_pass_buffer_fd;
        DescribeColorAspectsParams m_client_color_space;
        DescribeColorAspectsParams m_internal_color_space;

        // HDRStaticInfo defined in HardwareAPI.h
        DescribeHDRStaticInfoParams m_client_hdr_info;
        DescribeHDRStaticInfoParams m_internal_hdr_info;

        bool m_disable_ubwc_mode;

        unsigned int m_fill_output_msg;
        bool client_set_fps;
        unsigned int stereo_output_mode;
        class allocate_color_convert_buf
        {
            public:
                allocate_color_convert_buf();
                ~allocate_color_convert_buf() {};
                void set_vdec_client(void *);
                void update_client();
                bool set_color_format(OMX_COLOR_FORMATTYPE dest_color_format);
                bool get_color_format(OMX_COLOR_FORMATTYPE &dest_color_format);
                bool update_buffer_req();
                bool get_buffer_req(unsigned int &buffer_size);
                OMX_ERRORTYPE set_buffer_req(OMX_U32 buffer_size, OMX_U32 actual_count);
                OMX_BUFFERHEADERTYPE* get_il_buf_hdr();
                OMX_BUFFERHEADERTYPE* get_il_buf_hdr(OMX_BUFFERHEADERTYPE *input_hdr);
                OMX_BUFFERHEADERTYPE* convert(OMX_BUFFERHEADERTYPE *header);
                OMX_BUFFERHEADERTYPE* queue_buffer(OMX_BUFFERHEADERTYPE *header);
                OMX_ERRORTYPE allocate_buffers_color_convert(OMX_HANDLETYPE hComp,
                        OMX_BUFFERHEADERTYPE **bufferHdr,OMX_U32 port,OMX_PTR appData,
                        OMX_U32 bytes);
                OMX_ERRORTYPE free_output_buffer(OMX_BUFFERHEADERTYPE *bufferHdr);
                bool is_color_conversion_enabled() {return enabled;}
                void enable_color_conversion(bool enable);
                bool is_client_buffers_disabled() {return client_buffers_disabled;}
                void set_client_buffers_disabled(bool val) {client_buffers_disabled = val;}
                // Returns a specific C2D state, if true current buffers should undergo C2D conversion
                // otherwise C2D is in quasi enabled state where in C2D src and dst formats are set but
                // color conversion doesnt take place. This state is needed to handle color conversion
                // of interlaced clips during port reconfig.
                bool client_buffers_invalid() {return (!enabled || client_buffers_disabled);}
            private:
#define MAX_COUNT MAX_NUM_INPUT_OUTPUT_BUFFERS
                omx_vdec *omx;
                bool enabled;
                bool client_buffers_disabled;
                OMX_COLOR_FORMATTYPE ColorFormat;
                void init_members();
                bool color_convert_mode;
                ColorConvertFormat dest_format;
                ColorConvertFormat src_format;
                C2DColorConverter c2dcc;
                unsigned int allocated_count;
                unsigned int buffer_size_req;
                unsigned int buffer_alignment_req;
                OMX_U32 m_c2d_width;
                OMX_U32 m_c2d_height;
                OMX_QCOM_PLATFORM_PRIVATE_LIST      m_platform_list_client[MAX_COUNT];
                OMX_QCOM_PLATFORM_PRIVATE_ENTRY     m_platform_entry_client[MAX_COUNT];
                OMX_QCOM_PLATFORM_PRIVATE_PMEM_INFO m_pmem_info_client[MAX_COUNT];
                OMX_BUFFERHEADERTYPE  m_out_mem_ptr_client[MAX_COUNT];
                DecColorMapping mMapOutput2DriverColorFormat;
                ColorSubMapping mMapOutput2Convert;
#ifdef USE_ION
                struct vdec_ion op_buf_ion_info[MAX_COUNT];
#endif
#ifdef USE_GBM
                struct vdec_gbm op_buf_gbm_info[MAX_COUNT];
#endif

                unsigned char *pmem_baseaddress[MAX_COUNT];
                int pmem_fd[MAX_COUNT];
        };
        allocate_color_convert_buf client_buffers;
        struct video_decoder_capability m_decoder_capability;
        struct debug_cap m_debug;
        int log_input_buffers(const char *, int, uint64_t, int);
        int log_output_buffers(OMX_BUFFERHEADERTYPE *);
        int log_cc_output_buffers(OMX_BUFFERHEADERTYPE *);
        void send_codec_config();
        OMX_TICKS m_prev_timestampUs;
        bool m_prev_frame_rendered;
        int32_t m_dec_hfr_fps;
        int32_t m_dec_secure_prefetch_size_internal;
        int32_t m_dec_secure_prefetch_size_output;
        int32_t m_arb_mode_override;
        int32_t m_disable_hdr;
        int32_t m_dec_output_rate;
        volatile int32_t m_queued_codec_config_count;
        OMX_U32 current_perf_level;
        int32_t m_thumbnail_yuv_output;

        class perf_control {
            typedef int (*perf_lock_acquire_t)(int, int, int*, int);
            typedef int (*perf_lock_release_t)(int);

            private:
                void *m_perf_lib;
                int m_perf_handle;
                perf_lock_acquire_t m_perf_lock_acquire;
                perf_lock_release_t m_perf_lock_release;

            public:
                perf_control();
                ~perf_control();
                bool load_perf_library();
                int perf_lock_acquire();
                void perf_lock_release();
                int m_perf_control_enable;
        };
        perf_control m_perf_control;

        static OMX_COLOR_FORMATTYPE getPreferredColorFormatNonSurfaceMode(OMX_U32 index) {
            //On Android, we default to standard YUV formats for non-surface use-cases
            //where apps prefer known color formats.
            OMX_COLOR_FORMATTYPE formatsNonSurfaceMode[] = {
                [0] = OMX_COLOR_FormatYUV420SemiPlanar,
                [1] = OMX_COLOR_FormatYUV420Planar,
                [2] = (OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m,
                [3] = (OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420PackedSemiPlanar32mMultiView,
                [4] = (OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420PackedSemiPlanar32mCompressed,
                [5] = OMX_COLOR_Format16bitRGB565,
            };
            return (index < sizeof(formatsNonSurfaceMode) / sizeof(OMX_COLOR_FORMATTYPE)) ?
                formatsNonSurfaceMode[index] : OMX_COLOR_FormatMax;
        }

        OMX_COLOR_FORMATTYPE getPreferredColorFormatDefaultMode(OMX_U32 index) {
            //for surface mode (normal playback), advertise native/accelerated formats first
            OMX_COLOR_FORMATTYPE format = (OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m;

            if (!m_disable_ubwc_mode) {
                OMX_COLOR_FORMATTYPE formatsDefault[] = {
                    [0] = (OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420PackedSemiPlanar32mCompressed,
                    [1] = (OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m,
                    [2] = OMX_COLOR_FormatYUV420SemiPlanar,
                    [3] = OMX_COLOR_FormatYUV420Planar,
                    [4] = (OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420PackedSemiPlanar32mMultiView,
                    [5] = OMX_COLOR_Format16bitRGB565,
                };
                format = (index < sizeof(formatsDefault) / sizeof(OMX_COLOR_FORMATTYPE)) ?
                    formatsDefault[index] : OMX_COLOR_FormatMax;
            } else {
                OMX_COLOR_FORMATTYPE formatsDefault[] = {
                    [0] = (OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m,
                    [1] = OMX_COLOR_FormatYUV420SemiPlanar,
                    [2] = OMX_COLOR_FormatYUV420Planar,
                    [3] = (OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420PackedSemiPlanar32mMultiView,
                };
                format = (index < sizeof(formatsDefault) / sizeof(OMX_COLOR_FORMATTYPE)) ?
                    formatsDefault[index] : OMX_COLOR_FormatMax;
            }
            return format;
        }

        static OMX_ERRORTYPE describeColorFormat(OMX_PTR params);
        bool prefetch_buffers(unsigned long prefetch_count,
                    unsigned long prefetch_size, unsigned ioctl_code,
                    unsigned ion_flag);
        unsigned char m_prefetch_done;
        client_extradata_info m_client_out_extradata_info;

        OMX_ERRORTYPE get_vendor_extension_config(
                OMX_CONFIG_ANDROID_VENDOR_EXTENSIONTYPE *ext);
        OMX_ERRORTYPE set_vendor_extension_config(
                OMX_CONFIG_ANDROID_VENDOR_EXTENSIONTYPE *ext);

        void init_vendor_extensions (VendorExtensionStore&);

        // list of extensions is not mutable after initialization
        const VendorExtensionStore mVendorExtensionStore;

        // Map of ColorAspects (VideoAPI.h) to ColorMetaData (color_metadata.h)
        PrimariesMap mPrimariesMap;
        TransferMap mTransferMap;
        MatrixCoeffMap mMatrixCoeffMap;
        RangeMap mColorRangeMap;

        void init_color_aspects_map();
        void convert_color_aspects_to_metadata(ColorAspects& aspects, ColorMetaData &color_mdata);
        void convert_hdr_info_to_metadata(HDRStaticInfo& hdr_info, ColorMetaData &color_mdata);
        void get_preferred_color_aspects(ColorAspects& preferredColorAspects);
        void get_preferred_hdr_info(HDRStaticInfo& preferredHDRInfo);
        bool vdec_query_cap(struct v4l2_queryctrl &cap);
        bool store_vp9_hdr10plusinfo(DescribeHDR10PlusInfoParams *hdr10plusinfo);
        bool store_hevc_hdr10plusinfo(uint32_t payload_size,
            msm_vidc_stream_userdata_payload *hdr10plusdata);
        void update_hdr10plusinfo_cookie_using_timestamp(OMX_PTR cookie, OMX_TICKS timestamp);
        void convert_hdr10plusinfo_to_metadata(OMX_PTR cookie, ColorMetaData &colorData);
        void remove_hdr10plusinfo_using_cookie(OMX_PTR cookie);
        void clear_hdr10plusinfo();
        void get_hdr10plusinfo(DescribeHDR10PlusInfoParams *hdr10plusdata);
        void print_hdr10plusinfo(DescribeHDR10PlusInfoParams *hdr10plusdata);
public:
        bool m_buffer_error;
        OMX_VIDEO_PARAM_PROFILELEVELTYPE get_clientSet_profile_level() {
            return clientSet_profile_level;
        }

};

enum instance_state {
    MSM_VIDC_CORE_UNINIT_DONE = 0x0001,
    MSM_VIDC_CORE_INIT,
    MSM_VIDC_CORE_INIT_DONE,
    MSM_VIDC_OPEN,
    MSM_VIDC_OPEN_DONE,
    MSM_VIDC_LOAD_RESOURCES,
    MSM_VIDC_LOAD_RESOURCES_DONE,
    MSM_VIDC_START,
    MSM_VIDC_START_DONE,
    MSM_VIDC_STOP,
    MSM_VIDC_STOP_DONE,
    MSM_VIDC_RELEASE_RESOURCES,
    MSM_VIDC_RELEASE_RESOURCES_DONE,
    MSM_VIDC_CLOSE,
    MSM_VIDC_CLOSE_DONE,
    MSM_VIDC_CORE_UNINIT,
};

enum vidc_resposes_id {
    MSM_VIDC_DECODER_FLUSH_DONE = 0x11,
    MSM_VIDC_DECODER_EVENT_CHANGE,
};

#endif // __OMX_VDEC_H__
