/*--------------------------------------------------------------------------
Copyright (c) 2014-2020, The Linux Foundation. All rights reserved.

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
#include "omx_swvenc_mpeg4.h"

/* def: StoreMetaDataInBuffersParams */
#include <media/hardware/HardwareAPI.h>

/* def: VENUS_BUFFER_SIZE, VENUS_Y_STRIDE etc */
#include <media/msm_media_info.h>

/* def: private_handle_t*/
#include <gralloc_priv.h>

#include "PlatformConfig.h"

/* use GraphicBuffer for rotation */
#include <ui/GraphicBufferAllocator.h>
#include <gralloc.h>

/* def: GET_VT_TIMESTAMP */
#include <qdMetaData.h>

/*----------------------------------------------------------------------------
 * Preprocessor Definitions and Constants
 * -------------------------------------------------------------------------*/
#define OMX_SPEC_VERSION 0x00000101
#define OMX_INIT_STRUCT(_s_, _name_)             \
    memset((_s_), 0x0, sizeof(_name_));          \
    (_s_)->nSize = sizeof(_name_);               \
    (_s_)->nVersion.nVersion = OMX_SPEC_VERSION

#define ENTER_FUNC() DEBUG_PRINT_HIGH("ENTERING: %s",__FUNCTION__)
#define EXIT_FUNC()  DEBUG_PRINT_HIGH("EXITING: %s",__FUNCTION__)
#define RETURN(x)    EXIT_FUNC(); return x;
#undef ALIGN
#define ALIGN(value,alignment) (((value) + (alignment-1)) & (~(alignment-1)))

#define BUFFER_LOG_LOC "/data/vendor/media"

/* factory function executed by the core to create instances */
void *get_omx_component_factory_fn(void)
{
    RETURN((new omx_venc));
}

omx_venc::omx_venc()
{
    ENTER_FUNC();

    char property_value[PROPERTY_VALUE_MAX] = {0};

    memset(&m_debug,0,sizeof(m_debug));

    property_value[0] = '\0';
    property_get("vendor.vidc.debug.level", property_value, "1");
    debug_level = atoi(property_value);

    Platform::Config::getInt32(Platform::vidc_enc_log_in,
            (int32_t *)&m_debug.in_buffer_log, 0);
    Platform::Config::getInt32(Platform::vidc_enc_log_out,
            (int32_t *)&m_debug.out_buffer_log, 0);

    property_value[0] = '\0';
    property_get("vendor.vidc.enc.log.in", property_value, "0");
    m_debug.in_buffer_log = atoi(property_value);

    property_value[0] = '\0';
    property_get("vendor.vidc.enc.log.in.rotated", property_value, "0");
    m_debug.in_buffer_rotated_log = atoi(property_value);

    property_value[0] = '\0';
    property_get("vendor.vidc.enc.log.out", property_value, "0");
    m_debug.out_buffer_log = atoi(property_value);

    snprintf(m_debug.log_loc, PROPERTY_VALUE_MAX, "%s", BUFFER_LOG_LOC);
    property_value[0] = '\0';
    property_get("vendor.vidc.log.loc", property_value, "");
    if (*property_value)
    {
       strlcpy(m_debug.log_loc, property_value, PROPERTY_VALUE_MAX);
    }

    memset(meta_buffer_hdr,0,sizeof(meta_buffer_hdr));
    meta_mode_enable = false;
    memset(meta_buffer_hdr,0,sizeof(meta_buffer_hdr));
    memset(meta_buffers,0,sizeof(meta_buffers));
    memset(opaque_buffer_hdr,0,sizeof(opaque_buffer_hdr));
    mUseProxyColorFormat = false;
    get_syntaxhdr_enable = false;
    m_bSeqHdrRequested = false;
    m_bDimensionsNeedFlip = false;
    m_bIsRotationSupported = false;
    m_bIsInFrameSizeSet = false;
    m_bIsOutFrameSizeSet = false;
    m_bIsInFlipDone = false;
    m_bIsOutFlipDone = false;
    m_bUseAVTimerTimestamps = false;
    m_bIsIntraperiodSet = false;
    m_pIpbuffers = nullptr;
    set_format = false;
    update_offset = true;
    m_ubwc_supported = false;
    EXIT_FUNC();
}

omx_venc::~omx_venc()
{
    ENTER_FUNC();
    get_syntaxhdr_enable = false;
    EXIT_FUNC();
}

OMX_ERRORTYPE omx_venc::component_init(OMX_STRING role)
{
    ENTER_FUNC();

    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    SWVENC_STATUS Ret = SWVENC_S_SUCCESS;
    SWVENC_CALLBACK callBackInfo;
    OMX_VIDEO_CODINGTYPE codec_type;
    SWVENC_PROPERTY Prop;

    strlcpy((char *)m_nkind,role,OMX_MAX_STRINGNAME_SIZE);
    secure_session = false;

    if (!strncmp( (char *)m_nkind,"OMX.qcom.video.encoder.mpeg4sw",
                  OMX_MAX_STRINGNAME_SIZE))
    {
        strlcpy((char *)m_cRole, "video_encoder.mpeg4",\
                OMX_MAX_STRINGNAME_SIZE);
        codec_type = OMX_VIDEO_CodingMPEG4;
        m_codec = SWVENC_CODEC_MPEG4;
    }
    else if (!strncmp( (char *)m_nkind,"OMX.qcom.video.encoder.h263sw",
                  OMX_MAX_STRINGNAME_SIZE))
    {
        strlcpy((char *)m_cRole, "video_encoder.h263",\
                OMX_MAX_STRINGNAME_SIZE);
        codec_type = OMX_VIDEO_CodingH263;
        m_codec = SWVENC_CODEC_H263;
    }
    else
    {
        DEBUG_PRINT_ERROR("ERROR: Unknown Component");
        eRet = OMX_ErrorInvalidComponentName;
        RETURN(eRet);
    }

#ifdef ENABLE_GET_SYNTAX_HDR
    get_syntaxhdr_enable = true;
    DEBUG_PRINT_HIGH("Get syntax header enabled");
#endif

    callBackInfo.pfn_empty_buffer_done    = swvenc_empty_buffer_done_cb;
    callBackInfo.pfn_fill_buffer_done     = swvenc_fill_buffer_done_cb;
    callBackInfo.pfn_event_notification   = swvenc_handle_event_cb;
    callBackInfo.p_client                 = (void*)this;

    SWVENC_STATUS sRet = swvenc_init(&m_hSwVenc, m_codec, &callBackInfo);
    if (sRet != SWVENC_S_SUCCESS)
    {
        DEBUG_PRINT_ERROR("swvenc_init returned %d, ret insufficient resources",
         sRet);
        RETURN(OMX_ErrorInsufficientResources);
    }

    sRet = swvenc_check_inst_load(m_hSwVenc);
    if (sRet != SWVENC_S_SUCCESS)
    {
        DEBUG_PRINT_ERROR("swvenc_init returned %d, ret insufficient resources",
         sRet);
        RETURN(OMX_ErrorInsufficientResources);
    }

    m_stopped = true;

    //Intialise the OMX layer variables
    memset(&m_pCallbacks,0,sizeof(OMX_CALLBACKTYPE));

    OMX_INIT_STRUCT(&m_sPortParam, OMX_PORT_PARAM_TYPE);
    m_sPortParam.nPorts = 0x2;
    m_sPortParam.nStartPortNumber = (OMX_U32) PORT_INDEX_IN;

    OMX_INIT_STRUCT(&m_sPortParam_audio, OMX_PORT_PARAM_TYPE);
    m_sPortParam_audio.nPorts = 0;
    m_sPortParam_audio.nStartPortNumber = 0;

    OMX_INIT_STRUCT(&m_sPortParam_img, OMX_PORT_PARAM_TYPE);
    m_sPortParam_img.nPorts = 0;
    m_sPortParam_img.nStartPortNumber = 0;

    OMX_INIT_STRUCT(&m_sParamBitrate, OMX_VIDEO_PARAM_BITRATETYPE);
    m_sParamBitrate.nPortIndex = (OMX_U32) PORT_INDEX_OUT;
    m_sParamBitrate.eControlRate = OMX_Video_ControlRateVariableSkipFrames;
    m_sParamBitrate.nTargetBitrate = 64000;

    OMX_INIT_STRUCT(&m_sConfigBitrate, OMX_VIDEO_CONFIG_BITRATETYPE);
    m_sConfigBitrate.nPortIndex = (OMX_U32) PORT_INDEX_OUT;
    m_sConfigBitrate.nEncodeBitrate = 64000;

    OMX_INIT_STRUCT(&m_sConfigFramerate, OMX_CONFIG_FRAMERATETYPE);
    m_sConfigFramerate.nPortIndex = (OMX_U32) PORT_INDEX_OUT;
    m_sConfigFramerate.xEncodeFramerate = 30 << 16;

    OMX_INIT_STRUCT(&m_sConfigIntraRefreshVOP, OMX_CONFIG_INTRAREFRESHVOPTYPE);
    m_sConfigIntraRefreshVOP.nPortIndex = (OMX_U32) PORT_INDEX_OUT;
    m_sConfigIntraRefreshVOP.IntraRefreshVOP = OMX_FALSE;

    OMX_INIT_STRUCT(&m_sConfigFrameRotation, OMX_CONFIG_ROTATIONTYPE);
    m_sConfigFrameRotation.nPortIndex = (OMX_U32) PORT_INDEX_IN;
    m_sConfigFrameRotation.nRotation = 0;

    OMX_INIT_STRUCT(&m_sSessionQuantization, OMX_VIDEO_PARAM_QUANTIZATIONTYPE);
    m_sSessionQuantization.nPortIndex = (OMX_U32) PORT_INDEX_OUT;
    m_sSessionQuantization.nQpI = 9;
    m_sSessionQuantization.nQpP = 6;
    m_sSessionQuantization.nQpB = 2;

    OMX_INIT_STRUCT(&m_sSessionQPRange, OMX_QCOM_VIDEO_PARAM_IPB_QPRANGETYPE);
    m_sSessionQPRange.nPortIndex = (OMX_U32) PORT_INDEX_OUT;
    m_sSessionQPRange.minIQP = 2;
    m_sSessionQPRange.minPQP = 2;
    m_sSessionQPRange.minBQP = 2;

    OMX_INIT_STRUCT(&m_sParamProfileLevel, OMX_VIDEO_PARAM_PROFILELEVELTYPE);
    m_sParamProfileLevel.nPortIndex = (OMX_U32) PORT_INDEX_OUT;

    OMX_INIT_STRUCT(&m_sIntraperiod, QOMX_VIDEO_INTRAPERIODTYPE);
    m_sIntraperiod.nPortIndex = (OMX_U32) PORT_INDEX_OUT;
    m_sIntraperiod.nPFrames = (m_sConfigFramerate.xEncodeFramerate * 2) - 1;

    OMX_INIT_STRUCT(&m_sErrorCorrection, OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE);
    m_sErrorCorrection.nPortIndex = (OMX_U32) PORT_INDEX_OUT;
    m_sErrorCorrection.bEnableDataPartitioning = OMX_FALSE;
    m_sErrorCorrection.bEnableHEC = OMX_FALSE;
    m_sErrorCorrection.bEnableResync = OMX_FALSE;
    m_sErrorCorrection.bEnableRVLC = OMX_FALSE;
    m_sErrorCorrection.nResynchMarkerSpacing = 0;

    OMX_INIT_STRUCT(&m_sIntraRefresh, OMX_VIDEO_PARAM_INTRAREFRESHTYPE);
    m_sIntraRefresh.nPortIndex = (OMX_U32) PORT_INDEX_OUT;
    m_sIntraRefresh.eRefreshMode = OMX_VIDEO_IntraRefreshMax;

    if (codec_type == OMX_VIDEO_CodingMPEG4)
    {
        m_sParamProfileLevel.eProfile = (OMX_U32) OMX_VIDEO_MPEG4ProfileSimple;
        m_sParamProfileLevel.eLevel = (OMX_U32) OMX_VIDEO_MPEG4Level0;
    } else if (codec_type == OMX_VIDEO_CodingH263)
    {
        m_sParamProfileLevel.eProfile = (OMX_U32) OMX_VIDEO_H263ProfileBaseline;
        m_sParamProfileLevel.eLevel = (OMX_U32) OMX_VIDEO_H263Level10;
    }

    /* set the profile and level */
    Ret = swvenc_set_profile_level(m_sParamProfileLevel.eProfile,
                m_sParamProfileLevel.eLevel);
    if (Ret != SWVENC_S_SUCCESS)
    {
       DEBUG_PRINT_ERROR("%s, swvenc_set_rc_mode failed (%d)",
         __FUNCTION__, Ret);
       RETURN(OMX_ErrorUndefined);
    }

    // Initialize the video parameters for input port
    OMX_INIT_STRUCT(&m_sInPortDef, OMX_PARAM_PORTDEFINITIONTYPE);
    m_sInPortDef.nPortIndex= (OMX_U32) PORT_INDEX_IN;
    m_sInPortDef.bEnabled = OMX_TRUE;
    m_sInPortDef.bPopulated = OMX_FALSE;
    m_sInPortDef.eDomain = OMX_PortDomainVideo;
    m_sInPortDef.eDir = OMX_DirInput;
    m_sInPortDef.format.video.cMIMEType = (char *)"YUV420";
    m_sInPortDef.format.video.nFrameWidth = OMX_CORE_QCIF_WIDTH;
    m_sInPortDef.format.video.nFrameHeight = OMX_CORE_QCIF_HEIGHT;
    m_sInPortDef.format.video.nStride = OMX_CORE_QCIF_WIDTH;
    m_sInPortDef.format.video.nSliceHeight = OMX_CORE_QCIF_HEIGHT;
    m_sInPortDef.format.video.nBitrate = 64000;
    m_sInPortDef.format.video.xFramerate = 15 << 16;
    m_sInPortDef.format.video.eColorFormat = (OMX_COLOR_FORMATTYPE)
        QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m;
    m_sInPortDef.format.video.eCompressionFormat =  OMX_VIDEO_CodingUnused;

    /* set the frame size */
    Prop.id = SWVENC_PROPERTY_ID_FRAME_SIZE;
    Prop.info.frame_size.height = m_sInPortDef.format.video.nFrameHeight;
    Prop.info.frame_size.width  = m_sInPortDef.format.video.nFrameWidth;

    Ret = swvenc_setproperty(m_hSwVenc, &Prop);
    if (Ret != SWVENC_S_SUCCESS)
    {
       DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
         __FUNCTION__, Ret);
       RETURN(OMX_ErrorUnsupportedSetting);
    }

    /* set the frame attributes */
    Prop.id = SWVENC_PROPERTY_ID_FRAME_ATTRIBUTES;
    Prop.info.frame_attributes.stride_luma = m_sInPortDef.format.video.nStride;
    Prop.info.frame_attributes.stride_chroma = m_sInPortDef.format.video.nStride;
    Prop.info.frame_attributes.offset_luma = 0;
    Prop.info.frame_attributes.offset_chroma =
      (m_sInPortDef.format.video.nSliceHeight * m_sInPortDef.format.video.nStride);
    Prop.info.frame_attributes.size = (Prop.info.frame_attributes.offset_chroma * 3) >> 1;

    Ret = swvenc_setproperty(m_hSwVenc, &Prop);
    if (Ret != SWVENC_S_SUCCESS)
    {
       DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
         __FUNCTION__, Ret);
       RETURN(OMX_ErrorUndefined);
    }

    Ret = swvenc_get_buffer_req(&m_sInPortDef.nBufferCountMin,
              &m_sInPortDef.nBufferCountActual,
              &m_sInPortDef.nBufferSize,
              &m_sInPortDef.nBufferAlignment,
              PORT_INDEX_IN);
    if (Ret != SWVENC_S_SUCCESS)
    {
       DEBUG_PRINT_ERROR("ERROR: %s, swvenc_get_buffer_req failed (%d)", __FUNCTION__,
          Ret);
       RETURN(OMX_ErrorUndefined);
    }

    // Initialize the video parameters for output port
    OMX_INIT_STRUCT(&m_sOutPortDef, OMX_PARAM_PORTDEFINITIONTYPE);
    m_sOutPortDef.nPortIndex = (OMX_U32) PORT_INDEX_OUT;
    m_sOutPortDef.bEnabled = OMX_TRUE;
    m_sOutPortDef.bPopulated = OMX_FALSE;
    m_sOutPortDef.eDomain = OMX_PortDomainVideo;
    m_sOutPortDef.eDir = OMX_DirOutput;
    m_sOutPortDef.format.video.nFrameWidth = OMX_CORE_QCIF_WIDTH;
    m_sOutPortDef.format.video.nFrameHeight = OMX_CORE_QCIF_HEIGHT;
    m_sOutPortDef.format.video.nBitrate = 64000;
    m_sOutPortDef.format.video.xFramerate = 15 << 16;
    m_sOutPortDef.format.video.eColorFormat =  OMX_COLOR_FormatUnused;
    if (codec_type == OMX_VIDEO_CodingMPEG4)
    {
        m_sOutPortDef.format.video.eCompressionFormat =  OMX_VIDEO_CodingMPEG4;
    }
    else if (codec_type == OMX_VIDEO_CodingH263)
    {
        m_sOutPortDef.format.video.eCompressionFormat =  OMX_VIDEO_CodingH263;
    }

    Ret = swvenc_get_buffer_req(&m_sOutPortDef.nBufferCountMin,
              &m_sOutPortDef.nBufferCountActual,
              &m_sOutPortDef.nBufferSize,
              &m_sOutPortDef.nBufferAlignment,
              PORT_INDEX_OUT);
    if (Ret != SWVENC_S_SUCCESS)
    {
       DEBUG_PRINT_ERROR("ERROR: %s, swvenc_get_buffer_req failed (%d)", __FUNCTION__,
          Ret);
       RETURN(OMX_ErrorUndefined);
    }

    // Initialize the video color format for input port
    OMX_INIT_STRUCT(&m_sInPortFormat, OMX_VIDEO_PARAM_PORTFORMATTYPE);
    m_sInPortFormat.nPortIndex = (OMX_U32) PORT_INDEX_IN;
    m_sInPortFormat.nIndex = 0;
    m_sInPortFormat.eColorFormat = (OMX_COLOR_FORMATTYPE)
        QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m;
    m_sInPortFormat.eCompressionFormat = OMX_VIDEO_CodingUnused;

    // Initialize the compression format for output port
    OMX_INIT_STRUCT(&m_sOutPortFormat, OMX_VIDEO_PARAM_PORTFORMATTYPE);
    m_sOutPortFormat.nPortIndex = (OMX_U32) PORT_INDEX_OUT;
    m_sOutPortFormat.nIndex = 0;
    m_sOutPortFormat.eColorFormat = OMX_COLOR_FormatUnused;
    if (codec_type == OMX_VIDEO_CodingMPEG4)
    {
        m_sOutPortFormat.eCompressionFormat =  OMX_VIDEO_CodingMPEG4;
    } else if (codec_type == OMX_VIDEO_CodingH263)
    {
        m_sOutPortFormat.eCompressionFormat =  OMX_VIDEO_CodingH263;
    }

    // mandatory Indices for kronos test suite
    OMX_INIT_STRUCT(&m_sPriorityMgmt, OMX_PRIORITYMGMTTYPE);

    OMX_INIT_STRUCT(&m_sInBufSupplier, OMX_PARAM_BUFFERSUPPLIERTYPE);
    m_sInBufSupplier.nPortIndex = (OMX_U32) PORT_INDEX_IN;

    OMX_INIT_STRUCT(&m_sOutBufSupplier, OMX_PARAM_BUFFERSUPPLIERTYPE);
    m_sOutBufSupplier.nPortIndex = (OMX_U32) PORT_INDEX_OUT;

    OMX_INIT_STRUCT(&m_sConfigQP, OMX_QCOM_VIDEO_CONFIG_QP);
    m_sConfigQP.nPortIndex = (OMX_U32) PORT_INDEX_OUT;

    // mp4 specific init
    OMX_INIT_STRUCT(&m_sParamMPEG4, OMX_VIDEO_PARAM_MPEG4TYPE);
    m_sParamMPEG4.nPortIndex = (OMX_U32) PORT_INDEX_OUT;
    m_sParamMPEG4.eProfile = OMX_VIDEO_MPEG4ProfileSimple;
    m_sParamMPEG4.eLevel = OMX_VIDEO_MPEG4Level0;
    m_sParamMPEG4.nSliceHeaderSpacing = 0;
    m_sParamMPEG4.bSVH = OMX_FALSE;
    m_sParamMPEG4.bGov = OMX_FALSE;
    // 2 second intra period for default outport fps
    if(m_sOutPortFormat.xFramerate)
    m_sParamMPEG4.nPFrames = (m_sOutPortFormat.xFramerate * 2 - 1);

    m_sParamMPEG4.bACPred = OMX_TRUE;
    // delta = 2 @ 15 fps
    m_sParamMPEG4.nTimeIncRes = 30;
    // pframe and iframe
    m_sParamMPEG4.nAllowedPictureTypes = 2;
    // number of video packet headers per vop
    m_sParamMPEG4.nHeaderExtension = 1;
    m_sParamMPEG4.bReversibleVLC = OMX_FALSE;

    // h263 specific init
    OMX_INIT_STRUCT(&m_sParamH263, OMX_VIDEO_PARAM_H263TYPE);
    m_sParamH263.nPortIndex = (OMX_U32) PORT_INDEX_OUT;
    // 2 second intra period for default outport fps
    if(m_sOutPortFormat.xFramerate)
    m_sParamH263.nPFrames = (m_sOutPortFormat.xFramerate * 2 - 1);

    m_sParamH263.nBFrames = 0;
    m_sParamH263.eProfile = OMX_VIDEO_H263ProfileBaseline;
    m_sParamH263.eLevel = OMX_VIDEO_H263Level10;
    m_sParamH263.bPLUSPTYPEAllowed = OMX_FALSE;
    m_sParamH263.nAllowedPictureTypes = 2;
    m_sParamH263.bForceRoundingTypeToZero = OMX_TRUE;
    m_sParamH263.nPictureHeaderRepetition = 0;
    m_sParamH263.nGOBHeaderInterval = 1;

    // av-timer init (for ims-vt)
    OMX_INIT_STRUCT(&m_sParamAVTimerTimestampMode, QOMX_ENABLETYPE);
    m_sParamAVTimerTimestampMode.bEnable = OMX_FALSE;

    m_state                   = OMX_StateLoaded;
    m_sExtraData = 0;
    //m_sParamConsumerUsage     |= (OMX_U32)GRALLOC_USAGE_SW_READ_OFTEN;

    if (codec_type == OMX_VIDEO_CodingMPEG4)
    {
        m_capability.max_height = OMX_CORE_720P_HEIGHT;
        m_capability.max_width = OMX_CORE_720P_WIDTH;
    }
    else if (codec_type == OMX_VIDEO_CodingH263)
    {
        m_capability.max_height = OMX_CORE_FWVGA_HEIGHT;
        m_capability.max_width = OMX_CORE_FWVGA_WIDTH;
    }

    m_capability.min_height = 32;
    m_capability.min_width = 32;

    if (eRet == OMX_ErrorNone)
    {
        if (pthread_create(&msg_thread_id,0, message_thread_enc, this) < 0)
        {
            eRet = OMX_ErrorInsufficientResources;
            msg_thread_created = false;
        }
        else
        {
            msg_thread_created = true;
        }
    }

    DEBUG_PRINT_HIGH("Component_init return value = 0x%x", eRet);

    EXIT_FUNC();

    {
        VendorExtensionStore *extStore = const_cast<VendorExtensionStore *>(&mVendorExtensionStore);
        init_sw_vendor_extensions(*extStore);
        mVendorExtensionStore.dumpExtensions((const char *)m_nkind);
    }

    RETURN(eRet);
}

OMX_ERRORTYPE  omx_venc::set_parameter
(
    OMX_IN OMX_HANDLETYPE hComp,
    OMX_IN OMX_INDEXTYPE  paramIndex,
    OMX_IN OMX_PTR        paramData
)
{
    ENTER_FUNC();

    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    SWVENC_STATUS Ret  = SWVENC_S_SUCCESS;
    SWVENC_PROPERTY Prop;
    bool bResult;
    unsigned int y_stride, y_scanlines;

    (void)hComp;

    if (m_state == OMX_StateInvalid)
    {
        DEBUG_PRINT_ERROR("ERROR: Set Param in Invalid State");
        RETURN(OMX_ErrorInvalidState);
    }
    if (paramData == NULL)
    {
        DEBUG_PRINT_ERROR("ERROR: Get Param in Invalid paramData");
        RETURN(OMX_ErrorBadParameter);
    }

    /* set_parameter can be called in loaded state or disabled port */
    if ( (m_state == OMX_StateLoaded) ||
         (m_sInPortDef.bEnabled == OMX_FALSE) ||
         (m_sOutPortDef.bEnabled == OMX_FALSE)
       )
    {
        DEBUG_PRINT_LOW("Set Parameter called in valid state");
    }
    else
    {
        DEBUG_PRINT_ERROR("ERROR: Set Parameter called in Invalid State");
        RETURN(OMX_ErrorIncorrectStateOperation);
    }

    switch ((int)paramIndex)
    {
        case OMX_IndexParamPortDefinition:
        {
            OMX_PARAM_PORTDEFINITIONTYPE *portDefn;
            portDefn = (OMX_PARAM_PORTDEFINITIONTYPE *) paramData;
            DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamPortDefinition H= %d, W = %d",
                    (int)portDefn->format.video.nFrameHeight,
                    (int)portDefn->format.video.nFrameWidth);

            if (PORT_INDEX_IN == portDefn->nPortIndex)
            {
                if (!dev_is_video_session_supported(portDefn->format.video.nFrameWidth,
                            portDefn->format.video.nFrameHeight))
                {
                    DEBUG_PRINT_ERROR("video session not supported");
                    omx_report_unsupported_setting();
                    RETURN(OMX_ErrorUnsupportedSetting);
                }
                DEBUG_PRINT_LOW("i/p actual cnt requested = %u", portDefn->nBufferCountActual);
                DEBUG_PRINT_LOW("i/p min cnt requested = %u", portDefn->nBufferCountMin);
                DEBUG_PRINT_LOW("i/p buffersize requested = %u", portDefn->nBufferSize);
                if (portDefn->nBufferCountMin > portDefn->nBufferCountActual)
                {
                    DEBUG_PRINT_ERROR("ERROR: (In_PORT) Min buffers (%u) > actual count (%u)",
                            portDefn->nBufferCountMin, portDefn->nBufferCountActual);
                    RETURN(OMX_ErrorUnsupportedSetting);
                }

                // don't update frame size if it's unchanged
                if (m_sInPortDef.format.video.nFrameWidth != portDefn->format.video.nFrameWidth
                        || m_sInPortDef.format.video.nFrameHeight != portDefn->format.video.nFrameHeight) {
                    /* set the frame size */
                    Prop.id = SWVENC_PROPERTY_ID_FRAME_SIZE;
                    Prop.info.frame_size.height = portDefn->format.video.nFrameHeight;
                    Prop.info.frame_size.width  = portDefn->format.video.nFrameWidth;

                    Ret = swvenc_setproperty(m_hSwVenc, &Prop);
                    if (Ret != SWVENC_S_SUCCESS)
                    {
                        DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
                                __FUNCTION__, Ret);
                    RETURN(OMX_ErrorUnsupportedSetting);
                    }
                }

                /* set the input frame-rate */
                if (portDefn->format.video.xFramerate != 0)
                {
                   Ret = swvenc_set_frame_rate(portDefn->format.video.xFramerate >> 16);
                   if (Ret != SWVENC_S_SUCCESS)
                   {
                      DEBUG_PRINT_ERROR("%s, swvenc_set_frame_rate failed (%d)",
                        __FUNCTION__, Ret);
                      RETURN(OMX_ErrorUnsupportedSetting);
                   }
                }

                /* set the frame attributes */
                /*Align stride and scanline to worst case*/
                /*------------------------------------------------------------------------------------------
                *           [Color Format]                   [Stride Alignment]        [Scanline Alignment]
                * QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m       512 or 128                  512 or 32
                * OMX_COLOR_FormatYUV420SemiPlanar                 512                         512
                * QOMX_COLOR_FormatYVU420SemiPlanar                16                          16
                * HAL_PIXEL_FORMAT_NV21_ZSL                        64                          64
                *------------------------------------------------------------------------------------------*/
                y_stride = SWVENC_Y_STRIDE(COLOR_FMT_NV12,portDefn->format.video.nFrameWidth);
                //Slice height doesn't get updated so chroma offset calculation becomes incorrect .
                //Using FrameHeight Instead , just for omx-test-app .
                y_scanlines = SWVENC_Y_SCANLINES(COLOR_FMT_NV12_ZSL,portDefn->format.video.nFrameHeight);
                Prop.id = SWVENC_PROPERTY_ID_FRAME_ATTRIBUTES;
                Prop.info.frame_attributes.stride_luma = y_stride;
                Prop.info.frame_attributes.stride_chroma = y_stride;
                Prop.info.frame_attributes.offset_luma = 0;
                Prop.info.frame_attributes.offset_chroma = y_scanlines * y_stride;
                Prop.info.frame_attributes.size = SWVENC_BUFFER_SIZE(COLOR_FMT_NV12_ZSL,
                                                                     portDefn->format.video.nFrameWidth,
                                                                     portDefn->format.video.nFrameHeight);
                Ret = swvenc_setproperty(m_hSwVenc, &Prop);
                if (Ret != SWVENC_S_SUCCESS)
                {
                   DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
                     __FUNCTION__, Ret);
                   RETURN(OMX_ErrorUnsupportedSetting);
                }

                DEBUG_PRINT_LOW("i/p previous actual cnt = %u", m_sInPortDef.nBufferCountActual);
                DEBUG_PRINT_LOW("i/p previous min cnt = %u", m_sInPortDef.nBufferCountMin);
                DEBUG_PRINT_LOW("i/p previous buffersize = %u", m_sInPortDef.nBufferSize);

                memcpy(&m_sInPortDef, portDefn,sizeof(OMX_PARAM_PORTDEFINITIONTYPE));

                /* update the input buffer requirement */
                Ret = swvenc_get_buffer_req(&m_sInPortDef.nBufferCountMin,
                        &m_sInPortDef.nBufferCountActual,
                        &m_sInPortDef.nBufferSize,
                        &m_sInPortDef.nBufferAlignment,
                        portDefn->nPortIndex);
                if (Ret != SWVENC_S_SUCCESS)
                {
                   DEBUG_PRINT_ERROR("ERROR: %s, swvenc_get_buffer_req failed (%d)", __FUNCTION__,
                      Ret);
                   RETURN(OMX_ErrorUndefined);
                }

                if (portDefn->nBufferCountActual > m_sInPortDef.nBufferCountActual)
                {
                   m_sInPortDef.nBufferCountActual = portDefn->nBufferCountActual;
                }
                if (portDefn->nBufferSize > m_sInPortDef.nBufferSize)
                {
                   m_sInPortDef.nBufferSize = portDefn->nBufferSize;
                }

                DEBUG_PRINT_LOW("i/p new actual cnt = %u", m_sInPortDef.nBufferCountActual);
                DEBUG_PRINT_LOW("i/p new min cnt = %u", m_sInPortDef.nBufferCountMin);
                DEBUG_PRINT_LOW("i/p new buffersize = %u", m_sInPortDef.nBufferSize);

                // when rotation is setting before portdefinition, if need flip dimensions
                // in port flip will be set here
                if (m_bDimensionsNeedFlip && !m_bIsInFlipDone) {
                    DEBUG_PRINT_HIGH("flip in port dimension(for swcodec) in portdefinition");
                    OMX_ERRORTYPE err = swvenc_do_flip_inport();
                    if (err != OMX_ErrorNone) {
                        DEBUG_PRINT_ERROR("%s, swvenc_do_flip_inport falied (%d)",
                                __FUNCTION__, err);
                        RETURN(err);
                    }
                    m_bIsInFlipDone = true;
                }
                m_bIsInFrameSizeSet = true;
            }
            else if (PORT_INDEX_OUT == portDefn->nPortIndex)
            {
                DEBUG_PRINT_LOW("o/p actual cnt requested = %u", portDefn->nBufferCountActual);
                DEBUG_PRINT_LOW("o/p min cnt requested = %u", portDefn->nBufferCountMin);
                DEBUG_PRINT_LOW("o/p buffersize requested = %u", portDefn->nBufferSize);
                if (portDefn->nBufferCountMin > portDefn->nBufferCountActual)
                {
                    DEBUG_PRINT_ERROR("ERROR: (Out_PORT) Min buffers (%u) > actual count (%u)",
                            portDefn->nBufferCountMin, portDefn->nBufferCountActual);
                    RETURN(OMX_ErrorUnsupportedSetting);
                }

                /* set the output bit-rate */
                Ret = swvenc_set_bit_rate(portDefn->format.video.nBitrate);
                if (Ret != SWVENC_S_SUCCESS)
                {
                   DEBUG_PRINT_ERROR("%s, swvenc_set_bit_rate failed (%d)",
                     __FUNCTION__, Ret);
                   RETURN(OMX_ErrorUnsupportedSetting);
                }

                DEBUG_PRINT_LOW("o/p previous actual cnt = %u", m_sOutPortDef.nBufferCountActual);
                DEBUG_PRINT_LOW("o/p previous min cnt = %u", m_sOutPortDef.nBufferCountMin);
                DEBUG_PRINT_LOW("o/p previous buffersize = %u", m_sOutPortDef.nBufferSize);

                /* set the buffer requirement */
                bResult = dev_set_buf_req(&portDefn->nBufferCountMin,
                  &portDefn->nBufferCountActual,
                  &portDefn->nBufferSize,
                  portDefn->nPortIndex);
                if (bResult != true)
                {
                   DEBUG_PRINT_ERROR("%s, dev_set_buf_req failed",
                     __FUNCTION__);
                   RETURN(OMX_ErrorUnsupportedSetting);
                }
                memcpy(&m_sOutPortDef, portDefn,sizeof(OMX_PARAM_PORTDEFINITIONTYPE));

                /* update the output buffer requirement */
                Ret = swvenc_get_buffer_req(&m_sOutPortDef.nBufferCountMin,
                        &m_sOutPortDef.nBufferCountActual,
                        &m_sOutPortDef.nBufferSize,
                        &m_sOutPortDef.nBufferAlignment,
                        portDefn->nPortIndex);
                if (Ret != SWVENC_S_SUCCESS)
                {
                   DEBUG_PRINT_ERROR("ERROR: %s, swvenc_get_buffer_req failed (%d)", __FUNCTION__,
                      Ret);
                   RETURN(OMX_ErrorUndefined);
                }

                if (portDefn->nBufferCountActual > m_sOutPortDef.nBufferCountActual)
                {
                   m_sOutPortDef.nBufferCountActual = portDefn->nBufferCountActual;
                }
                if (portDefn->nBufferSize > m_sOutPortDef.nBufferSize)
                {
                   m_sOutPortDef.nBufferSize = portDefn->nBufferSize;
                }

                DEBUG_PRINT_LOW("o/p new actual cnt = %u", m_sOutPortDef.nBufferCountActual);
                DEBUG_PRINT_LOW("o/p new min cnt = %u", m_sOutPortDef.nBufferCountMin);
                DEBUG_PRINT_LOW("o/p new buffersize = %u", m_sOutPortDef.nBufferSize);
                // when rotation is setting before portdefinition, if need flip dimensions
                // out port flip will be set here
                if (m_bDimensionsNeedFlip && !m_bIsOutFlipDone) {
                    DEBUG_PRINT_HIGH("flip out port dimension in portdefinition");
                    OMX_ERRORTYPE err = swvenc_do_flip_outport();
                    m_bIsOutFlipDone = true;
                    DEBUG_PRINT_HIGH("Out Port Definition: rotation (%d), flipped WxH (%d x %d)",
                            m_sConfigFrameRotation.nRotation,
                            m_sOutPortDef.format.video.nFrameWidth,
                            m_sOutPortDef.format.video.nFrameHeight);
                }
                m_bIsOutFrameSizeSet = true;
            }
            else
            {
                DEBUG_PRINT_ERROR("ERROR: Set_parameter: Bad Port idx %d",
                        (int)portDefn->nPortIndex);
                eRet = OMX_ErrorBadPortIndex;
            }
            m_sConfigFramerate.xEncodeFramerate = portDefn->format.video.xFramerate;
            m_sConfigBitrate.nEncodeBitrate = portDefn->format.video.nBitrate;
            m_sParamBitrate.nTargetBitrate = portDefn->format.video.nBitrate;
            break;
        }

        case OMX_IndexParamVideoPortFormat:
        {
            OMX_VIDEO_PARAM_PORTFORMATTYPE *portFmt =
                (OMX_VIDEO_PARAM_PORTFORMATTYPE *)paramData;
            DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamVideoPortFormat %d",
                    portFmt->eColorFormat);
            SWVENC_COLOR_FORMAT color_format;

            /* set the driver with the corresponding values */
            if (PORT_INDEX_IN == portFmt->nPortIndex)
            {
                if (portFmt->eColorFormat ==
                    ((OMX_COLOR_FORMATTYPE) QOMX_COLOR_FormatAndroidOpaque))
                {
                    /* meta_mode = 2 (kMetadataBufferTypeGrallocSource) */
                    m_sInPortFormat.eColorFormat =
                        (OMX_COLOR_FORMATTYPE) QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m;
                    color_format = SWVENC_COLOR_FORMAT_NV12;
                    mUseProxyColorFormat = true;
                    m_input_msg_id = OMX_COMPONENT_GENERATE_ETB_OPQ;
                }
                else
                {
                    m_sInPortFormat.eColorFormat = portFmt->eColorFormat;
                    if ((portFmt->eColorFormat == OMX_COLOR_FormatYUV420SemiPlanar) ||
                        (portFmt->eColorFormat ==
                         ((OMX_COLOR_FORMATTYPE) QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m)))
                    {
                        color_format = SWVENC_COLOR_FORMAT_NV12;
                    }
                    else if (portFmt->eColorFormat ==
                             ((OMX_COLOR_FORMATTYPE) QOMX_COLOR_FormatYVU420SemiPlanar))
                    {
                        color_format = SWVENC_COLOR_FORMAT_NV21;
                    }
                    else
                    {
                        DEBUG_PRINT_ERROR("%s: OMX_IndexParamVideoPortFormat %d invalid",
                                          __FUNCTION__,
                                          portFmt->eColorFormat);
                        RETURN(OMX_ErrorBadParameter);
                    }
                    m_input_msg_id = OMX_COMPONENT_GENERATE_ETB;
                    mUseProxyColorFormat = false;
                }
                m_sInPortDef.format.video.eColorFormat = m_sInPortFormat.eColorFormat;
                /* set the input color format */
                Prop.id = SWVENC_PROPERTY_ID_COLOR_FORMAT;
                Prop.info.color_format = color_format;
                Ret = swvenc_setproperty(m_hSwVenc, &Prop);
                if (Ret != SWVENC_S_SUCCESS)
                {
                   DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
                     __FUNCTION__, Ret);
                   RETURN(OMX_ErrorUnsupportedSetting);
                }

                /* set the input frame-rate */
                if (portFmt->xFramerate != 0)
                {
                   Ret = swvenc_set_frame_rate(portFmt->xFramerate >> 16);
                   if (Ret != SWVENC_S_SUCCESS)
                   {
                      DEBUG_PRINT_ERROR("%s, swvenc_set_frame_rate failed (%d)",
                        __FUNCTION__, Ret);
                      //RETURN(OMX_ErrorUnsupportedSetting);
                   }
                   m_sInPortFormat.xFramerate = portFmt->xFramerate;
                }
            }
            break;
        }

        case OMX_IndexParamVideoInit:
        {
            OMX_PORT_PARAM_TYPE* pParam = (OMX_PORT_PARAM_TYPE*)(paramData);
            DEBUG_PRINT_LOW("Set OMX_IndexParamVideoInit called");
            break;
        }

        case OMX_IndexParamVideoBitrate:
        {
            OMX_VIDEO_PARAM_BITRATETYPE* pParam = (OMX_VIDEO_PARAM_BITRATETYPE*)paramData;
            DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamVideoBitrate");

            /* set the output bit-rate */
            Ret = swvenc_set_bit_rate(pParam->nTargetBitrate);
            if (Ret != SWVENC_S_SUCCESS)
            {
               DEBUG_PRINT_ERROR("%s, swvenc_set_bit_rate failed (%d)",
                 __FUNCTION__, Ret);
               RETURN(OMX_ErrorUnsupportedSetting);
            }

            /* set the RC-mode */
            Ret = swvenc_set_rc_mode(pParam->eControlRate);
            if (Ret != SWVENC_S_SUCCESS)
            {
               DEBUG_PRINT_ERROR("%s, swvenc_set_rc_mode failed (%d)",
                 __FUNCTION__, Ret);
               RETURN(OMX_ErrorUnsupportedSetting);
            }

            m_sParamBitrate.nTargetBitrate = pParam->nTargetBitrate;
            m_sParamBitrate.eControlRate = pParam->eControlRate;
            m_sConfigBitrate.nEncodeBitrate = pParam->nTargetBitrate;
            m_sInPortDef.format.video.nBitrate = pParam->nTargetBitrate;
            m_sOutPortDef.format.video.nBitrate = pParam->nTargetBitrate;
            DEBUG_PRINT_LOW("bitrate = %u", m_sOutPortDef.format.video.nBitrate);
            break;
        }

        case OMX_IndexParamVideoMpeg4:
        {
            OMX_VIDEO_PARAM_MPEG4TYPE* pParam = (OMX_VIDEO_PARAM_MPEG4TYPE*)paramData;

            DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamVideoMpeg4");

            if (pParam->nBFrames)
            {
                DEBUG_PRINT_ERROR("Warning: B frames not supported");
            }

            /* set the intra period */
            if (!m_bIsIntraperiodSet)
            {
                DEBUG_PRINT_LOW("pParam->nPFrames : %d", pParam->nPFrames);
                Ret = swvenc_set_intra_period(pParam->nPFrames,pParam->nBFrames);
                if (Ret != SWVENC_S_SUCCESS)
                {
                    DEBUG_PRINT_ERROR("%s, swvenc_set_intra_period failed (%d)",
                        __FUNCTION__, Ret);
                    RETURN(OMX_ErrorUnsupportedSetting);
                }
                else
                {
                    m_sIntraperiod.nPFrames = pParam->nPFrames;
                    m_sIntraperiod.nBFrames = pParam->nBFrames;
                    m_bIsIntraperiodSet = true;
                }
            }

            /* set profile/level */
            if (pParam->eProfile && pParam->eLevel)
            {
                DEBUG_PRINT_LOW("pParam->eProfile : %d, pParam->eLevel : %d", pParam->eProfile, pParam->eLevel);
                Ret = swvenc_set_profile_level(pParam->eProfile, pParam->eLevel);
                if (Ret != SWVENC_S_SUCCESS)
                {
                    DEBUG_PRINT_ERROR("%sm swvenc_set_profile_level failed (%d)",
                            __FUNCTION__, Ret);
                    RETURN(OMX_ErrorUnsupportedSetting);
                }
                else
                {
                    m_sParamProfileLevel.eProfile = pParam->eProfile;
                    m_sParamProfileLevel.eLevel = pParam->eLevel;
                }
            }

            /*set slice config */
            if (pParam->nSliceHeaderSpacing > 0)
            {
                SWVENC_PROPERTY Prop;
                Prop.id = SWVENC_PROPERTY_ID_SLICE_CONFIG;
                Prop.info.slice_config.mode = SWVENC_SLICE_MODE_MB;
                Prop.info.slice_config.size = pParam->nSliceHeaderSpacing;
                Ret = swvenc_setproperty(m_hSwVenc, &Prop);
                if (Ret != SWVENC_S_SUCCESS)
                {
                    DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
                        __FUNCTION__, Ret);
                    RETURN(OMX_ErrorUndefined);
                }
                else
                {
                    m_sParamMPEG4.nSliceHeaderSpacing = pParam->nSliceHeaderSpacing;
                }
            }
            // NOTE: m_sParamMPEG4.eProfile/eLevel may be overwritten to 0 if client didn't set them
            memcpy(&m_sParamMPEG4, pParam, sizeof(struct OMX_VIDEO_PARAM_MPEG4TYPE));
            break;
        }

        case OMX_IndexParamVideoH263:
        {
            OMX_VIDEO_PARAM_H263TYPE* pParam = (OMX_VIDEO_PARAM_H263TYPE*)paramData;

            DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamVideoH263");

            /* set the intra period */
            if (!m_bIsIntraperiodSet)
            {
                Ret = swvenc_set_intra_period(pParam->nPFrames,pParam->nBFrames);
                if (Ret != SWVENC_S_SUCCESS)
                {
                    DEBUG_PRINT_ERROR("%s, swvenc_set_intra_period failed (%d)",
                        __FUNCTION__, Ret);
                    RETURN(OMX_ErrorUnsupportedSetting);
                }
                else
                {
                    m_sIntraperiod.nPFrames = pParam->nPFrames;
                    m_sIntraperiod.nBFrames = pParam->nBFrames;
                    m_bIsIntraperiodSet = true;
                }
            }

            /* set profile/level */
            if (pParam->eProfile && pParam->eLevel)
            {
                DEBUG_PRINT_LOW("pParam->eProfile : %d, pParam->eLevel : %d", pParam->eProfile, pParam->eLevel);
                Ret = swvenc_set_profile_level(pParam->eProfile, pParam->eLevel);
                if (Ret != SWVENC_S_SUCCESS)
                {
                    DEBUG_PRINT_ERROR("%sm swvenc_set_profile_level failed (%d)",
                            __FUNCTION__, Ret);
                    RETURN(OMX_ErrorUnsupportedSetting);
                }
                else
                {
                    m_sParamProfileLevel.eProfile = pParam->eProfile;
                    m_sParamProfileLevel.eLevel = pParam->eLevel;
                }
            }

            // NOTE: m_sParamH263.eProfile/eLevel may be overwritten to 0 if client didn't set them
            memcpy(&m_sParamH263,pParam, sizeof(struct OMX_VIDEO_PARAM_H263TYPE));
            break;
        }

        case OMX_IndexParamVideoProfileLevelCurrent:
        {
            OMX_VIDEO_PARAM_PROFILELEVELTYPE* pParam = (OMX_VIDEO_PARAM_PROFILELEVELTYPE*)paramData;

            DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamVideoProfileLevelCurrent");

            /* set the profile and level */
            Ret = swvenc_set_profile_level(pParam->eProfile,pParam->eLevel);
            if (Ret != SWVENC_S_SUCCESS)
            {
               DEBUG_PRINT_ERROR("%s, swvenc_set_rc_mode failed (%d)",
                 __FUNCTION__, Ret);
               RETURN(OMX_ErrorUnsupportedSetting);
            }


            m_sParamProfileLevel.eProfile = pParam->eProfile;
            m_sParamProfileLevel.eLevel = pParam->eLevel;

            if (SWVENC_CODEC_MPEG4 == m_codec)
            {
                m_sParamMPEG4.eProfile = (OMX_VIDEO_MPEG4PROFILETYPE)m_sParamProfileLevel.eProfile;
                m_sParamMPEG4.eLevel = (OMX_VIDEO_MPEG4LEVELTYPE)m_sParamProfileLevel.eLevel;
                DEBUG_PRINT_LOW("MPEG4 profile = %d, level = %d", m_sParamMPEG4.eProfile,
                        m_sParamMPEG4.eLevel);
            }
            else if (SWVENC_CODEC_H263 == m_codec)
            {
                m_sParamH263.eProfile = (OMX_VIDEO_H263PROFILETYPE)m_sParamProfileLevel.eProfile;
                m_sParamH263.eLevel = (OMX_VIDEO_H263LEVELTYPE)m_sParamProfileLevel.eLevel;
                DEBUG_PRINT_LOW("H263 profile = %d, level = %d", m_sParamH263.eProfile,
                        m_sParamH263.eLevel);
            }
            break;
        }

        case OMX_IndexParamStandardComponentRole:
        {
            OMX_PARAM_COMPONENTROLETYPE *comp_role;
            comp_role = (OMX_PARAM_COMPONENTROLETYPE *) paramData;
            DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamStandardComponentRole %s",
                    comp_role->cRole);

            if ((m_state == OMX_StateLoaded)&&
                    !BITMASK_PRESENT(&m_flags,OMX_COMPONENT_IDLE_PENDING))
            {
                DEBUG_PRINT_LOW("Set Parameter called in valid state");
            }
            else
            {
                DEBUG_PRINT_ERROR("Set Parameter called in Invalid State");
                RETURN(OMX_ErrorIncorrectStateOperation);
            }

            if (SWVENC_CODEC_MPEG4 == m_codec)
            {
                if (!strncmp((const char*)comp_role->cRole,"video_encoder.mpeg4",OMX_MAX_STRINGNAME_SIZE))
                {
                    strlcpy((char*)m_cRole,"video_encoder.mpeg4",OMX_MAX_STRINGNAME_SIZE);
                }
                else
                {
                    DEBUG_PRINT_ERROR("ERROR: Setparameter: unknown Index %s", comp_role->cRole);
                    eRet = OMX_ErrorUnsupportedSetting;
                }
            }
            else if (SWVENC_CODEC_H263 == m_codec)
            {
                if (!strncmp((const char*)comp_role->cRole,"video_encoder.h263",OMX_MAX_STRINGNAME_SIZE))
                {
                    strlcpy((char*)m_cRole,"video_encoder.h263",OMX_MAX_STRINGNAME_SIZE);
                }
                else
                {
                    DEBUG_PRINT_ERROR("ERROR: Setparameter: unknown Index %s", comp_role->cRole);
                    eRet =OMX_ErrorUnsupportedSetting;
                }
            }
            else
            {
                DEBUG_PRINT_ERROR("ERROR: Setparameter: unknown param %s", m_nkind);
                eRet = OMX_ErrorInvalidComponentName;
            }
            break;
        }

        case OMX_IndexParamPriorityMgmt:
        {
            DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamPriorityMgmt");
            if (m_state != OMX_StateLoaded) {
                DEBUG_PRINT_ERROR("ERROR: Set Parameter called in Invalid State");
                RETURN(OMX_ErrorIncorrectStateOperation);
            }
            OMX_PRIORITYMGMTTYPE *priorityMgmtype = (OMX_PRIORITYMGMTTYPE*) paramData;
            DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamPriorityMgmt %u",
                    priorityMgmtype->nGroupID);

            DEBUG_PRINT_LOW("set_parameter: priorityMgmtype %u",
                    priorityMgmtype->nGroupPriority);

            m_sPriorityMgmt.nGroupID = priorityMgmtype->nGroupID;
            m_sPriorityMgmt.nGroupPriority = priorityMgmtype->nGroupPriority;

            break;
        }

        case OMX_IndexParamCompBufferSupplier:
        {
            DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamCompBufferSupplier");
            OMX_PARAM_BUFFERSUPPLIERTYPE *bufferSupplierType = (OMX_PARAM_BUFFERSUPPLIERTYPE*) paramData;
            DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamCompBufferSupplier %d",
                    bufferSupplierType->eBufferSupplier);
            if ( (bufferSupplierType->nPortIndex == 0) ||
                 (bufferSupplierType->nPortIndex ==1)
               )
            {
                m_sInBufSupplier.eBufferSupplier = bufferSupplierType->eBufferSupplier;
            }
            else
            {
                eRet = OMX_ErrorBadPortIndex;
            }

            break;

        }

        case OMX_IndexParamVideoQuantization:
        {
            // this is applicable only for RC-off case
            DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamVideoQuantization");
            OMX_VIDEO_PARAM_QUANTIZATIONTYPE *session_qp = (OMX_VIDEO_PARAM_QUANTIZATIONTYPE*) paramData;
            if (session_qp->nPortIndex == PORT_INDEX_OUT)
            {
                Prop.id = SWVENC_PROPERTY_ID_QP;
                Prop.info.qp.qp_i = session_qp->nQpI;
                Prop.info.qp.qp_p = session_qp->nQpP;
                Prop.info.qp.qp_b = session_qp->nQpB;

                Ret = swvenc_setproperty(m_hSwVenc, &Prop);
                if (Ret != SWVENC_S_SUCCESS)
                {
                   DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
                     __FUNCTION__, Ret);
                   RETURN(OMX_ErrorUnsupportedSetting);
                }

                m_sSessionQuantization.nQpI = session_qp->nQpI;
                m_sSessionQuantization.nQpP = session_qp->nQpP;
                m_sSessionQuantization.nQpB = session_qp->nQpB;
            }
            else
            {
                DEBUG_PRINT_ERROR("ERROR: Unsupported port Index for Session QP setting");
                eRet = OMX_ErrorBadPortIndex;
            }
            break;
        }

        case OMX_QcomIndexParamVideoIPBQPRange:
        {
            DEBUG_PRINT_LOW("set_parameter: OMX_QcomIndexParamVideoIPBQPRange");
            OMX_QCOM_VIDEO_PARAM_IPB_QPRANGETYPE *session_qp_range = (OMX_QCOM_VIDEO_PARAM_IPB_QPRANGETYPE*) paramData;
            if (session_qp_range->nPortIndex == PORT_INDEX_OUT)
            {
                Prop.id = SWVENC_PROPERTY_ID_QP_RANGE;
                Prop.info.qp_range.min_qp_packed = ((session_qp_range->minBQP << 16) |
                                                    (session_qp_range->minPQP <<  8) |
                                                    (session_qp_range->minIQP <<  0));
                Prop.info.qp_range.max_qp_packed = ((session_qp_range->maxBQP << 16) |
                                                    (session_qp_range->maxPQP <<  8) |
                                                    (session_qp_range->maxIQP <<  0));

                Ret = swvenc_setproperty(m_hSwVenc, &Prop);
                if (Ret != SWVENC_S_SUCCESS)
                {
                   DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
                     __FUNCTION__, Ret);
                   RETURN(OMX_ErrorUnsupportedSetting);
                }

                m_sSessionQPRange.minIQP = session_qp_range->minIQP;
                m_sSessionQPRange.maxIQP = session_qp_range->maxIQP;
                m_sSessionQPRange.minPQP = session_qp_range->minPQP;
                m_sSessionQPRange.maxPQP = session_qp_range->maxPQP;
                m_sSessionQPRange.minBQP = session_qp_range->minBQP;
                m_sSessionQPRange.maxBQP = session_qp_range->maxBQP;
            }
            else
            {
                DEBUG_PRINT_ERROR("ERROR: Unsupported port Index for Session QP range setting");
                eRet = OMX_ErrorBadPortIndex;
            }
            break;
        }

        case OMX_QcomIndexPortDefn:
        {
            OMX_QCOM_PARAM_PORTDEFINITIONTYPE* pParam =
                (OMX_QCOM_PARAM_PORTDEFINITIONTYPE*)paramData;
            DEBUG_PRINT_LOW("set_parameter: OMX_QcomIndexPortDefn");
            if (pParam->nPortIndex == (OMX_U32)PORT_INDEX_IN)
            {
                if (pParam->nMemRegion > OMX_QCOM_MemRegionInvalid &&
                        pParam->nMemRegion < OMX_QCOM_MemRegionMax)
                {
                    m_use_input_pmem = OMX_TRUE;
                }
                else
                {
                    m_use_input_pmem = OMX_FALSE;
                }
            }
            else if (pParam->nPortIndex == (OMX_U32)PORT_INDEX_OUT)
            {
                if (pParam->nMemRegion > OMX_QCOM_MemRegionInvalid &&
                        pParam->nMemRegion < OMX_QCOM_MemRegionMax)
                {
                    m_use_output_pmem = OMX_TRUE;
                }
                else
                {
                    m_use_output_pmem = OMX_FALSE;
                }
            }
            else
            {
                DEBUG_PRINT_ERROR("ERROR: SetParameter called on unsupported Port Index for QcomPortDefn");
                RETURN(OMX_ErrorBadPortIndex);
            }
            break;
        }

        case OMX_IndexParamVideoErrorCorrection:
        {
            DEBUG_PRINT_LOW("OMX_IndexParamVideoErrorCorrection");
            OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE* pParam =
                (OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE*)paramData;

            /* HEC */
            if (m_codec == SWVENC_CODEC_MPEG4)
            {
               Prop.id = SWVENC_PROPERTY_ID_MPEG4_HEC;
               Prop.info.mpeg4_hec = pParam->bEnableHEC;

               Ret = swvenc_setproperty(m_hSwVenc, &Prop);
               if (Ret != SWVENC_S_SUCCESS)
               {
                  DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
                    __FUNCTION__, Ret);
                  RETURN(OMX_ErrorUndefined);
               }

               /* Data partitioning */
               Prop.id = SWVENC_PROPERTY_ID_MPEG4_DP;
               Prop.info.mpeg4_dp = pParam->bEnableDataPartitioning;

               Ret = swvenc_setproperty(m_hSwVenc, &Prop);
               if (Ret != SWVENC_S_SUCCESS)
               {
                  DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
                    __FUNCTION__, Ret);
                  RETURN(OMX_ErrorUndefined);
               }
            }

            /* RVLC */
            if (pParam->bEnableRVLC)
            {
               DEBUG_PRINT_ERROR("%s, RVLC not support", __FUNCTION__);
            }

            /* Re-sync Marker */
            Prop.id = SWVENC_PROPERTY_ID_SLICE_CONFIG;
            if ( (m_codec != SWVENC_CODEC_H263) && (pParam->bEnableDataPartitioning) )
            {
               DEBUG_PRINT_ERROR("DataPartioning are not Supported for this codec");
               break;
            }
            if ( (m_codec != SWVENC_CODEC_H263) && (pParam->nResynchMarkerSpacing) )
            {
               Prop.info.slice_config.mode = SWVENC_SLICE_MODE_BYTE;
               Prop.info.slice_config.size = ALIGN(pParam->nResynchMarkerSpacing, 8) >> 3; //slice size is defined in bits
               Ret = swvenc_setproperty(m_hSwVenc, &Prop);
               if (Ret != SWVENC_S_SUCCESS)
               {
                  DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
                    __FUNCTION__, Ret);
                  RETURN(OMX_ErrorUndefined);
               }
            }
            else if ( (SWVENC_CODEC_H263 == m_codec) && (pParam->bEnableResync) )
            {
               Prop.info.slice_config.mode = SWVENC_SLICE_MODE_GOB;
               Prop.info.slice_config.size = 0;
               Ret = swvenc_setproperty(m_hSwVenc, &Prop);
               if (Ret != SWVENC_S_SUCCESS)
               {
                  DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
                    __FUNCTION__, Ret);
                  RETURN(OMX_ErrorUndefined);
               }
            }
            else
            {
               Prop.info.slice_config.mode = SWVENC_SLICE_MODE_OFF;
               Prop.info.slice_config.size = 0;
            }

            memcpy(&m_sErrorCorrection,pParam, sizeof(m_sErrorCorrection));
            break;
        }

        case OMX_IndexParamVideoIntraRefresh:
        {
            DEBUG_PRINT_LOW("set_param:OMX_IndexParamVideoIntraRefresh");
            OMX_VIDEO_PARAM_INTRAREFRESHTYPE* pParam =
                (OMX_VIDEO_PARAM_INTRAREFRESHTYPE*)paramData;

            Ret = swvenc_set_intra_refresh(pParam);
            if (Ret != SWVENC_S_SUCCESS)
            {
               DEBUG_PRINT_ERROR("%s, swvenc_set_intra_refresh failed (%d)",
                 __FUNCTION__, Ret);
               RETURN(OMX_ErrorUnsupportedSetting);
            }

            memcpy(&m_sIntraRefresh, pParam, sizeof(m_sIntraRefresh));
            break;
        }

        case OMX_QcomIndexParamVideoMetaBufferMode:
        {
            StoreMetaDataInBuffersParams *pParam =
                (StoreMetaDataInBuffersParams*)paramData;
            DEBUG_PRINT_HIGH("set_parameter:OMX_QcomIndexParamVideoMetaBufferMode: "
                    "port_index = %u, meta_mode = %d", pParam->nPortIndex, pParam->bStoreMetaData);

            if (pParam->nPortIndex == PORT_INDEX_IN)
            {
                if (pParam->bStoreMetaData != meta_mode_enable)
                {
                    meta_mode_enable = pParam->bStoreMetaData;
                    if (!meta_mode_enable)
                    {
                        Ret = swvenc_get_buffer_req(&m_sOutPortDef.nBufferCountMin,
                                 &m_sOutPortDef.nBufferCountActual,
                                 &m_sOutPortDef.nBufferSize,
                                 &m_sOutPortDef.nBufferAlignment,
                                 m_sOutPortDef.nPortIndex);
                        if (Ret != SWVENC_S_SUCCESS)
                        {
                           DEBUG_PRINT_ERROR("ERROR: %s, swvenc_get_buffer_req failed (%d)", __FUNCTION__,
                              Ret);
                           eRet = OMX_ErrorUndefined;
                           break;
                        }
                    }
                }
            }
            else if (pParam->nPortIndex == PORT_INDEX_OUT && secure_session)
            {
                if (pParam->bStoreMetaData != meta_mode_enable)
                {
                    meta_mode_enable = pParam->bStoreMetaData;
                }
            }
            else
            {
                if (pParam->bStoreMetaData)
                {
                    DEBUG_PRINT_ERROR("set_parameter: metamode is "
                            "valid for input port only");
                    eRet = OMX_ErrorUnsupportedIndex;
                }
            }
        }
        break;

        case OMX_QcomIndexParamIndexExtraDataType:
        {
            DEBUG_PRINT_HIGH("set_parameter: OMX_QcomIndexParamIndexExtraDataType");
            QOMX_INDEXEXTRADATATYPE *pParam = (QOMX_INDEXEXTRADATATYPE *)paramData;
            OMX_U32 mask = 0;

            if (pParam->nIndex == (OMX_INDEXTYPE)OMX_ExtraDataVideoEncoderSliceInfo)
            {
                if (pParam->nPortIndex == PORT_INDEX_OUT)
                {
                    mask = VEN_EXTRADATA_SLICEINFO;

                    DEBUG_PRINT_HIGH("SliceInfo extradata %s",
                            ((pParam->bEnabled == OMX_TRUE) ? "enabled" : "disabled"));
                }
                else
                {
                    DEBUG_PRINT_ERROR("set_parameter: Slice information is "
                            "valid for output port only");
                    eRet = OMX_ErrorUnsupportedIndex;
                    break;
                }
            }
            else if (pParam->nIndex == (OMX_INDEXTYPE)OMX_ExtraDataVideoEncoderMBInfo)
            {
                if (pParam->nPortIndex == PORT_INDEX_OUT)
                {
                    mask = VEN_EXTRADATA_MBINFO;

                    DEBUG_PRINT_HIGH("MBInfo extradata %s",
                            ((pParam->bEnabled == OMX_TRUE) ? "enabled" : "disabled"));
                }
                else
                {
                    DEBUG_PRINT_ERROR("set_parameter: MB information is "
                            "valid for output port only");
                    eRet = OMX_ErrorUnsupportedIndex;
                    break;
                }
            }
            else
            {
                DEBUG_PRINT_ERROR("set_parameter: unsupported extrdata index (%x)",
                        pParam->nIndex);
                eRet = OMX_ErrorUnsupportedIndex;
                break;
            }


            if (pParam->bEnabled == OMX_TRUE)
            {
                m_sExtraData |= mask;
            }
            else
            {
                m_sExtraData &= ~mask;
            }

            #if 0
            // TBD: add setprop to swvenc once the support is added
            if (handle->venc_set_param((OMX_PTR)!!(m_sExtraData & mask),
                        (OMX_INDEXTYPE)pParam->nIndex) != true)
            {
                DEBUG_PRINT_ERROR("ERROR: Setting Extradata (%x) failed", pParam->nIndex);
                RETURN(OMX_ErrorUnsupportedSetting);
            }
            else
            #endif
            {
                m_sOutPortDef.nPortIndex = PORT_INDEX_OUT;
                bResult = dev_get_buf_req(&m_sOutPortDef.nBufferCountMin,
                        &m_sOutPortDef.nBufferCountActual,
                        &m_sOutPortDef.nBufferSize,
                        m_sOutPortDef.nPortIndex);
                if (false == bResult)
                {
                   DEBUG_PRINT_ERROR("dev_get_buf_req failed");
                   eRet = OMX_ErrorUndefined;
                   break;
                }

                DEBUG_PRINT_HIGH("updated out_buf_req: buffer cnt=%u, "
                        "count min=%u, buffer size=%u",
                        m_sOutPortDef.nBufferCountActual,
                        m_sOutPortDef.nBufferCountMin,
                        m_sOutPortDef.nBufferSize);
            }
            break;
        }

        case OMX_QcomIndexEnableH263PlusPType:
        {
            QOMX_EXTNINDEX_PARAMTYPE* pParam =
                (QOMX_EXTNINDEX_PARAMTYPE*)paramData;
            DEBUG_PRINT_LOW("OMX_QcomIndexEnableH263PlusPType");
            if (pParam->nPortIndex == PORT_INDEX_OUT)
            {
                DEBUG_PRINT_ERROR("ERROR: Request for setting PlusPType failed");
                RETURN(OMX_ErrorUnsupportedSetting);
            }
            else
            {
                DEBUG_PRINT_ERROR("ERROR: OMX_QcomIndexEnableH263PlusPType "
                        "called on wrong port(%u)", pParam->nPortIndex);
                RETURN(OMX_ErrorBadPortIndex);
            }
            break;
        }

        case QOMX_IndexParamVideoInitialQp:
        {
            // TBD: applicable to RC-on case only
            DEBUG_PRINT_ERROR("ERROR: Setting Initial QP for RC-on case");
            RETURN(OMX_ErrorNone);
            break;
        }

        case OMX_QTIIndexParamEnableAVTimerTimestamps:
        {
            VALIDATE_OMX_PARAM_DATA(paramData, QOMX_ENABLETYPE);
            QOMX_ENABLETYPE *pParam = (QOMX_ENABLETYPE *)paramData;
            m_bUseAVTimerTimestamps = pParam->bEnable == OMX_TRUE;
            DEBUG_PRINT_INFO("AVTimer timestamps %s", m_bUseAVTimerTimestamps ? "enabled" : "disabled");
            break;
        }

        default:
        {
            DEBUG_PRINT_ERROR("ERROR: Setparameter: unknown param %d", paramIndex);
            eRet = OMX_ErrorUnsupportedIndex;
            break;
        }
    }

    RETURN(eRet);
}

OMX_ERRORTYPE  omx_venc::set_config
(
   OMX_IN OMX_HANDLETYPE      hComp,
   OMX_IN OMX_INDEXTYPE configIndex,
   OMX_IN OMX_PTR        configData
)
{
    ENTER_FUNC();

    SWVENC_STATUS SwStatus;

    (void)hComp;

    if (configData == NULL)
    {
        DEBUG_PRINT_ERROR("ERROR: param is null");
        RETURN(OMX_ErrorBadParameter);
    }

    if (m_state == OMX_StateInvalid)
    {
        DEBUG_PRINT_ERROR("ERROR: config called in Invalid state");
        RETURN(OMX_ErrorIncorrectStateOperation);
    }

    switch ((int)configIndex)
    {
        case OMX_IndexConfigVideoBitrate:
        {
            OMX_VIDEO_CONFIG_BITRATETYPE* pParam =
                reinterpret_cast<OMX_VIDEO_CONFIG_BITRATETYPE*>(configData);
            DEBUG_PRINT_HIGH("set_config(): OMX_IndexConfigVideoBitrate (%u)", pParam->nEncodeBitrate);

            if (pParam->nPortIndex == PORT_INDEX_OUT)
            {
                SwStatus = swvenc_set_bit_rate(pParam->nEncodeBitrate);
                if (SwStatus != SWVENC_S_SUCCESS)
                {
                   DEBUG_PRINT_ERROR("%s, swvenc_set_bit_rate failed (%d)",
                     __FUNCTION__, SwStatus);
                   RETURN(OMX_ErrorUnsupportedSetting);
                }

                m_sConfigBitrate.nEncodeBitrate = pParam->nEncodeBitrate;
                m_sParamBitrate.nTargetBitrate = pParam->nEncodeBitrate;
                m_sOutPortDef.format.video.nBitrate = pParam->nEncodeBitrate;
            }
            else
            {
                DEBUG_PRINT_ERROR("ERROR: Unsupported port index: %u", pParam->nPortIndex);
                RETURN(OMX_ErrorBadPortIndex);
            }
            break;
        }
        case OMX_IndexConfigVideoFramerate:
        {
            OMX_CONFIG_FRAMERATETYPE* pParam =
                reinterpret_cast<OMX_CONFIG_FRAMERATETYPE*>(configData);
            DEBUG_PRINT_HIGH("set_config(): OMX_IndexConfigVideoFramerate (0x%x)", pParam->xEncodeFramerate);

            if (pParam->nPortIndex == PORT_INDEX_OUT)
            {
                SwStatus = swvenc_set_frame_rate(pParam->xEncodeFramerate >> 16);
                if (SwStatus != SWVENC_S_SUCCESS)
                {
                   DEBUG_PRINT_ERROR("%s, swvenc_set_frame_rate failed (%d)",
                     __FUNCTION__, SwStatus);
                   RETURN(OMX_ErrorUnsupportedSetting);
                }

                m_sConfigFramerate.xEncodeFramerate = pParam->xEncodeFramerate;
                m_sOutPortDef.format.video.xFramerate = pParam->xEncodeFramerate;
                m_sOutPortFormat.xFramerate = pParam->xEncodeFramerate;
            }
            else
            {
                DEBUG_PRINT_ERROR("ERROR: Unsupported port index: %u", pParam->nPortIndex);
                RETURN(OMX_ErrorBadPortIndex);
            }
            break;
        }
        case QOMX_IndexConfigVideoIntraperiod:
        {
            QOMX_VIDEO_INTRAPERIODTYPE* pParam =
                reinterpret_cast<QOMX_VIDEO_INTRAPERIODTYPE*>(configData);
            DEBUG_PRINT_HIGH("set_config(): QOMX_IndexConfigVideoIntraperiod");

            if (pParam->nPortIndex == PORT_INDEX_OUT)
            {
                if (pParam->nBFrames > 0)
                {
                    DEBUG_PRINT_ERROR("B frames not supported");
                    RETURN(OMX_ErrorUnsupportedSetting);
                }

                DEBUG_PRINT_HIGH("Old: P/B frames = %u/%u, New: P/B frames = %u/%u",
                        m_sIntraperiod.nPFrames, m_sIntraperiod.nBFrames,
                        pParam->nPFrames, pParam->nBFrames);
                if (m_sIntraperiod.nBFrames != pParam->nBFrames)
                {
                    DEBUG_PRINT_HIGH("Dynamically changing B-frames not supported");
                    RETURN(OMX_ErrorUnsupportedSetting);
                }

                /* set the intra period */
                SwStatus = swvenc_set_intra_period(pParam->nPFrames,pParam->nBFrames);
                if (SwStatus != SWVENC_S_SUCCESS)
                {
                   DEBUG_PRINT_ERROR("%s, swvenc_set_intra_period failed (%d)",
                     __FUNCTION__, SwStatus);
                   RETURN(OMX_ErrorUnsupportedSetting);
                }

                m_sIntraperiod.nPFrames = pParam->nPFrames;
                m_sIntraperiod.nBFrames = pParam->nBFrames;
                m_sIntraperiod.nIDRPeriod = pParam->nIDRPeriod;

                if (m_sOutPortFormat.eCompressionFormat == OMX_VIDEO_CodingMPEG4)
                {
                    m_sParamMPEG4.nPFrames = pParam->nPFrames;
                    if (m_sParamMPEG4.eProfile != OMX_VIDEO_MPEG4ProfileSimple)
                    {
                        m_sParamMPEG4.nBFrames = pParam->nBFrames;
                    }
                    else
                    {
                        m_sParamMPEG4.nBFrames = 0;
                    }
                }
                else if (m_sOutPortFormat.eCompressionFormat == OMX_VIDEO_CodingH263)
                {
                    m_sParamH263.nPFrames = pParam->nPFrames;
                }
            }
            else
            {
                DEBUG_PRINT_ERROR("ERROR: (QOMX_IndexConfigVideoIntraperiod) Unsupported port index: %u", pParam->nPortIndex);
                RETURN(OMX_ErrorBadPortIndex);
            }

            break;
        }
        case OMX_IndexConfigVideoIntraVOPRefresh:
        {
            OMX_CONFIG_INTRAREFRESHVOPTYPE* pParam =
                reinterpret_cast<OMX_CONFIG_INTRAREFRESHVOPTYPE*>(configData);
            DEBUG_PRINT_HIGH("set_config(): OMX_IndexConfigVideoIntraVOPRefresh");

            if (pParam->nPortIndex == PORT_INDEX_OUT)
            {

                SWVENC_PROPERTY Prop;

                Prop.id = SWVENC_PROPERTY_ID_IFRAME_REQUEST;

                SwStatus = swvenc_setproperty(m_hSwVenc, &Prop);
                if (SwStatus != SWVENC_S_SUCCESS)
                {
                   DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
                     __FUNCTION__, SwStatus);
                   RETURN(OMX_ErrorUnsupportedSetting);
                }

                m_sConfigIntraRefreshVOP.IntraRefreshVOP = pParam->IntraRefreshVOP;
            }
            else
            {
                DEBUG_PRINT_ERROR("ERROR: Unsupported port index: %u", pParam->nPortIndex);
                RETURN(OMX_ErrorBadPortIndex);
            }
            break;
        }
        case OMX_IndexConfigCommonRotate:
        {
            if (m_codec == SWVENC_CODEC_H263) {
                OMX_CONFIG_ROTATIONTYPE *pParam =
                    reinterpret_cast<OMX_CONFIG_ROTATIONTYPE *>(configData);
                DEBUG_PRINT_HIGH("set_config(): OMX_IndexConfigCommonRotate");
                m_bIsRotationSupported = true;

                // XXX: diffrent from h/w encoder rotation, h/w encoder only need to update out
                // port info. For h/w encoder, rotation is processed in h/w encoder firmware, this
                // is after ETB, so input info doesn't change. While s/w encoder rotation is
                // processed before ETB, so need to change in port info.
                if (pParam->nPortIndex != PORT_INDEX_IN) {
                    DEBUG_PRINT_ERROR("ERROR: Unsupported port index: %u",
                            (unsigned int)pParam->nPortIndex);
                    RETURN(OMX_ErrorBadPortIndex);
                }
                if (pParam->nRotation == 0 ||
                        pParam->nRotation == 90 ||
                        pParam->nRotation == 180 ||
                        pParam->nRotation == 270) {
                    DEBUG_PRINT_HIGH("set_config(): Rotation Angle %u",
                            (unsigned int)pParam->nRotation);
                    if (m_pIpbuffers == nullptr) {
                        // m_pIpbuffers is used to store original ipbuffer, because after rotation,
                        // will send new rotated ipbuffer to encoder, in EBD will also get new
                        // ipbuffer. so we can restore original ipbuffer from to m_pIpbuffers and
                        // return it to framework
                        m_pIpbuffers = new SWVENC_IPBUFFER[m_sInPortDef.nBufferCountActual];
                    }
                    if (m_pIpbuffers == nullptr) {
                        DEBUG_PRINT_ERROR("create ipbuffer array failed");
                        return OMX_ErrorUndefined;
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Unsupported Rotation Angle %u",
                            (unsigned int)pParam->nRotation);
                    RETURN(OMX_ErrorUnsupportedSetting);
                }
                if (m_sConfigFrameRotation.nRotation == pParam->nRotation) {
                    DEBUG_PRINT_HIGH("set_config(): rotation (%d) not changed", pParam->nRotation);
                    break;
                }

                OMX_S32 rotation_diff = pParam->nRotation - m_sConfigFrameRotation.nRotation;
                if (rotation_diff < 0)
                    rotation_diff = -rotation_diff;
                if (rotation_diff == 90 || rotation_diff == 270) {
                    // in the case that rotation angle is 90 or 270 degree, if original buffer size
                    // is 640x480, after rotation, rotated buffer size will be 480x640, so need to
                    // flip dimensions in such cases.
                    m_bDimensionsNeedFlip = true;
                    OMX_ERRORTYPE err = OMX_ErrorNone;
                    // flip and set new dimensions must be called after real dimension set
                    if (m_bIsInFrameSizeSet && !m_bIsInFlipDone) {
                        err = swvenc_do_flip_inport();
                        if (err != OMX_ErrorNone) {
                            DEBUG_PRINT_ERROR("set_config(): flipping failed");
                            RETURN(err);
                        }

                        m_bIsInFlipDone = true;
                    } else {
                        DEBUG_PRINT_HIGH("set_config(): in port frame size isn't set, will do flip later");
                    }
                    if (m_bIsOutFrameSizeSet && !m_bIsOutFlipDone) {
                        err = swvenc_do_flip_outport();
                        m_bIsOutFlipDone = true;
                        DEBUG_PRINT_HIGH("set_config(): out port flip done, rotation (%d), flipped WxH (%d x %d)",
                                pParam->nRotation,
                                m_sOutPortDef.format.video.nFrameWidth,
                                m_sOutPortDef.format.video.nFrameHeight);
                    } else {
                        DEBUG_PRINT_HIGH("set_config(): out port frame size isn't set, will do flip later");
                    }
                } else {
                    m_bDimensionsNeedFlip = false;
                    DEBUG_PRINT_HIGH("set_config(): rotation (%d), no need to flip WxH",
                            pParam->nRotation);
                }

                // save rotation angle
                m_sConfigFrameRotation.nRotation = pParam->nRotation;
                break;
            } else {
                DEBUG_PRINT_ERROR("ERROR: rotation is not supported for current codec");
                RETURN(OMX_ErrorUnsupportedSetting);


            }
        }
        case OMX_IndexConfigAndroidVendorExtension:
        {
            OMX_CONFIG_ANDROID_VENDOR_EXTENSIONTYPE *ext =
                reinterpret_cast<OMX_CONFIG_ANDROID_VENDOR_EXTENSIONTYPE *>(configData);
            OMX_ERRORTYPE err = set_vendor_extension_config(ext);
            RETURN(err);
        }
        default:
            DEBUG_PRINT_ERROR("ERROR: unsupported index %d", (int) configIndex);
            RETURN(OMX_ErrorUnsupportedSetting);
            break;
    }

    EXIT_FUNC();

    RETURN(OMX_ErrorNone);
}

OMX_ERRORTYPE omx_venc::swvenc_do_flip_inport() {
    ENTER_FUNC();
    OMX_U32 inWidth = m_sInPortDef.format.video.nFrameWidth;
    OMX_U32 inHeight = m_sInPortDef.format.video.nFrameHeight;

    // set new dimensions to encoder
    SWVENC_PROPERTY Prop;
    SWVENC_STATUS Ret = SWVENC_S_SUCCESS;
    Prop.id = SWVENC_PROPERTY_ID_FRAME_SIZE;
    Prop.info.frame_size.height = inWidth;
    Prop.info.frame_size.width = inHeight;

    DEBUG_PRINT_HIGH("setting flipped dimensions to swencoder, WxH (%d x %d)",
            Prop.info.frame_size.width, Prop.info.frame_size.height);
    Ret = swvenc_setproperty(m_hSwVenc, &Prop);
    if (Ret != SWVENC_S_SUCCESS) {
        // currently, set dimensions to encoder can only be called when encoder is
        // in init state, while setVendorParameter() in ACodec can be called when
        // OMX component is in Executing state, in this case, encoder is in ready
        // state, will report unsupported error.
        DEBUG_PRINT_ERROR("ERROR: setting new dimension to encoder failed (%d)",
                Ret);
        return OMX_ErrorUnsupportedSetting;
    }

    // don't flip in port dimensions m_sInPortDef.format.video.nFrameWidth(mFrameHeight)
    // app may require this dimensions by get_parameter

    // update attributes, here dimensions are flipped, so use inHeight for calculating
    // stride, inWidth for scanlines, and swapp parameters in venus size calculation
    int stride = SWVENC_Y_STRIDE(COLOR_FMT_NV12, inHeight);
    int scanlines = SWVENC_Y_SCANLINES(COLOR_FMT_NV12, inWidth);
    Prop.id = SWVENC_PROPERTY_ID_FRAME_ATTRIBUTES;
    Prop.info.frame_attributes.stride_luma = stride;
    Prop.info.frame_attributes.stride_chroma = stride;
    Prop.info.frame_attributes.offset_luma = 0;
    Prop.info.frame_attributes.offset_chroma = scanlines * stride;
    Prop.info.frame_attributes.size =
        SWVENC_BUFFER_SIZE(COLOR_FMT_NV12_ZSL, inHeight, inWidth);

    Ret = swvenc_setproperty(m_hSwVenc, &Prop);
    if (Ret != SWVENC_S_SUCCESS) {
        DEBUG_PRINT_ERROR("ERROR: update frame attributes failed (%d)", Ret);
        return OMX_ErrorUnsupportedSetting;
    }

    // till now, attributes of omx input port is different from sw encoder input port,
    // omx input port stores original attributes, sw encoder input port stores flipped
    // attributes. no need to update buffer requirements from sw encoder here, but need
    // to update in output port, omx output port should also store flipped attrinutes

    EXIT_FUNC();
    return OMX_ErrorNone;
}

OMX_ERRORTYPE omx_venc::swvenc_do_flip_outport() {
    ENTER_FUNC();
    // for out port, no need to set dimensions to encoder
    OMX_U32 outWidth = m_sOutPortDef.format.video.nFrameWidth;
    OMX_U32 outHeight = m_sOutPortDef.format.video.nFrameHeight;

    // update out port info
    m_sOutPortDef.format.video.nFrameWidth = outHeight;
    m_sOutPortDef.format.video.nFrameHeight = outWidth;

    // attributes in sw encoder has been updated after flipping dimensions, so need to update
    // omx out port buffer requirements, they should have the same attributes
    DEBUG_PRINT_LOW("flip outport, o/p previous actual cnt = %u", m_sOutPortDef.nBufferCountActual);
    DEBUG_PRINT_LOW("flip outport, o/p previous min cnt = %u", m_sOutPortDef.nBufferCountMin);
    DEBUG_PRINT_LOW("flip outport, o/p previous buffersize = %u", m_sOutPortDef.nBufferSize);

    SWVENC_STATUS Ret = SWVENC_S_SUCCESS;
    Ret = swvenc_get_buffer_req(&m_sOutPortDef.nBufferCountMin,
            &m_sOutPortDef.nBufferCountActual,
            &m_sOutPortDef.nBufferSize,
            &m_sOutPortDef.nBufferAlignment,
            PORT_INDEX_OUT);
    if (Ret != SWVENC_S_SUCCESS) {
        DEBUG_PRINT_ERROR("ERROR: %s, flip outport swvenc_get_buffer_req failed(%d)", __FUNCTION__,
                Ret);
        return OMX_ErrorUndefined;
    }

    DEBUG_PRINT_LOW("flip outport, o/p new actual cnt = %u", m_sOutPortDef.nBufferCountActual);
    DEBUG_PRINT_LOW("flip outport, o/p new min cnt = %u", m_sOutPortDef.nBufferCountMin);
    DEBUG_PRINT_LOW("flip outport, o/p new buffersize = %u", m_sOutPortDef.nBufferSize);

    EXIT_FUNC();
    return OMX_ErrorNone;
}

bool omx_venc::swvenc_do_rotate(int fd, SWVENC_IPBUFFER & ipbuffer, OMX_U32 index) {
    // declarations and definitions of variables rotation needs
    private_handle_t *privateHandle = nullptr;

    int s_width = m_sInPortDef.format.video.nFrameWidth;
    int s_height = m_sInPortDef.format.video.nFrameHeight;
    int d_width = m_bDimensionsNeedFlip ? s_height : s_width;
    int d_height = m_bDimensionsNeedFlip ? s_width : s_height;

    uint32_t rotation = m_sConfigFrameRotation.nRotation;

    uint32_t usage = GraphicBuffer::USAGE_HW_TEXTURE | GraphicBuffer::USAGE_SW_READ_OFTEN |
        GraphicBuffer::USAGE_SW_WRITE_OFTEN | GraphicBuffer::USAGE_HW_RENDER;
    uint32_t dstusage = GraphicBuffer::USAGE_HW_TEXTURE | GraphicBuffer::USAGE_HW_VIDEO_ENCODER |
        GraphicBuffer::USAGE_HW_RENDER | GraphicBuffer::USAGE_SW_READ_OFTEN |
        GraphicBuffer::USAGE_SW_WRITE_OFTEN;

    int src_stride = SWVENC_Y_STRIDE(COLOR_FMT_NV12, s_width);
    int src_scanlines = SWVENC_Y_SCANLINES(COLOR_FMT_NV12, s_height);
    int src_size = SWVENC_BUFFER_SIZE(COLOR_FMT_NV12_ZSL, s_width, s_height);
    int dst_size = SWVENC_BUFFER_SIZE(COLOR_FMT_NV12_ZSL, d_width, d_height);

    uint32_t format = HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS;

    // privateHandle is created for creating GraphicBuffer in rotation case
    privateHandle = new private_handle_t(fd, ipbuffer.size, usage, BUFFER_TYPE_VIDEO, format,
            src_stride, src_scanlines);
    if (privateHandle == nullptr) {
        DEBUG_PRINT_ERROR("failed to create private handle");
        return false;
    }

    sp<GraphicBuffer> srcBuffer = new GraphicBuffer(s_width, s_height, format, 1, usage,
            src_stride, (native_handle_t *)privateHandle, false);
    if (srcBuffer.get() == NULL) {
        DEBUG_PRINT_ERROR("create source buffer failed");
        swvenc_delete_pointer(privateHandle);
        return false;
    }

    // reuse dstBuffer
    if (dstBuffer.get() == NULL) {
        dstBuffer = new GraphicBuffer(d_width, d_height, format, dstusage);
    }
    if (dstBuffer.get() == NULL) {
        DEBUG_PRINT_ERROR("create destination buffer failed");
        swvenc_delete_pointer(privateHandle);
        return false;
    }
    SWVENC_STATUS ret = swvenc_rotateFrame(s_width, s_height, d_height, d_width,
            rotation, srcBuffer->getNativeBuffer(), dstBuffer->getNativeBuffer());

    if (ret == SWVENC_S_SUCCESS) {
        void *buf = nullptr;
        if (dstBuffer->lock(dstusage, &buf) == 0 && buf != nullptr) {
            DEBUG_PRINT_HIGH("store original ipbuffer[p_buffer(%p), size(%d), filled_length(%d)], new ipbuffer[p_buffer(%p), size(%d), filled_length(%d)]",
                    ipbuffer.p_buffer,
                    ipbuffer.size,
                    ipbuffer.filled_length,
                    (unsigned char *)buf,
                    dst_size,
                    dst_size);
            if (index >= m_sInPortDef.nBufferCountActual) {
                DEBUG_PRINT_ERROR("incorrect buffer index");
                swvenc_delete_pointer(privateHandle);
                return false;
            }
            m_pIpbuffers[index].size = ipbuffer.size;
            m_pIpbuffers[index].filled_length = ipbuffer.filled_length;
            m_pIpbuffers[index].p_buffer = ipbuffer.p_buffer;
            ipbuffer.size = dst_size;
            ipbuffer.filled_length = dst_size;
            ipbuffer.p_buffer = (unsigned char *)buf;
            dstBuffer->unlock();
            DEBUG_PRINT_HIGH("copy rotated buffer successfully");
        } else {
            DEBUG_PRINT_ERROR("copy rotated buffer failed");
            swvenc_delete_pointer(privateHandle);
            return false;
        }
    } else {
        DEBUG_PRINT_ERROR("rotate failed");
        swvenc_delete_pointer(privateHandle);
        return false;
    }

    swvenc_delete_pointer(privateHandle);
    return true;
}

OMX_ERRORTYPE  omx_venc::component_deinit(OMX_IN OMX_HANDLETYPE hComp)
{
    ENTER_FUNC();

    OMX_U32 i = 0;
    DEBUG_PRINT_HIGH("omx_venc(): Inside component_deinit()");

    (void)hComp;

    if (m_bIsRotationSupported) {
        swvenc_rotation_deinit();
        if (m_pIpbuffers != nullptr) {
            delete [] m_pIpbuffers;
        }
    }

    if (OMX_StateLoaded != m_state)
    {
        DEBUG_PRINT_ERROR("WARNING:Rxd DeInit,OMX not in LOADED state %d",
                m_state);
    }
    if (m_out_mem_ptr)
    {
        DEBUG_PRINT_LOW("Freeing the Output Memory");
        for (i=0; i< m_sOutPortDef.nBufferCountActual; i++ )
        {
            free_output_buffer (&m_out_mem_ptr[i]);
        }
        free(m_out_mem_ptr);
        m_out_mem_ptr = NULL;
    }

    /* Check if the input buffers have to be cleaned up */
    if ( m_inp_mem_ptr && !meta_mode_enable )
    {
        DEBUG_PRINT_LOW("Freeing the Input Memory");
        for (i=0; i<m_sInPortDef.nBufferCountActual; i++)
        {
            free_input_buffer (&m_inp_mem_ptr[i]);
        }

        free(m_inp_mem_ptr);
        m_inp_mem_ptr = NULL;
    }

    /* Reset counters in msg queues */
    m_ftb_q.m_size=0;
    m_cmd_q.m_size=0;
    m_etb_q.m_size=0;
    m_ftb_q.m_read = m_ftb_q.m_write =0;
    m_cmd_q.m_read = m_cmd_q.m_write =0;
    m_etb_q.m_read = m_etb_q.m_write =0;

    DEBUG_PRINT_HIGH("Calling swvenc_deinit()");
    swvenc_deinit(m_hSwVenc);

    if (msg_thread_created) {
        msg_thread_created = false;
        msg_thread_stop = true;
        post_message(this, OMX_COMPONENT_CLOSE_MSG);
        DEBUG_PRINT_HIGH("omx_video: Waiting on Msg Thread exit");
        pthread_join(msg_thread_id,NULL);
    }
    DEBUG_PRINT_HIGH("OMX_Venc:Component Deinit");

    RETURN(OMX_ErrorNone);
}

OMX_U32 omx_venc::dev_stop(void)
{
    ENTER_FUNC();

    SWVENC_STATUS Ret;

    if (false == m_stopped)
    {
       Ret = swvenc_stop(m_hSwVenc);
       if (Ret != SWVENC_S_SUCCESS)
       {
          DEBUG_PRINT_ERROR("%s, swvenc_stop failed (%d)",
            __FUNCTION__, Ret);
          RETURN(-1);
       }
       set_format = false;
       m_stopped = true;

       /* post STOP_DONE event as start is synchronus */
       post_event (0, OMX_ErrorNone, OMX_COMPONENT_GENERATE_STOP_DONE);
    }

    RETURN(0);
}

OMX_U32 omx_venc::dev_pause(void)
{
    ENTER_FUNC();
    // nothing to be done for sw encoder

    RETURN(true);
}

OMX_U32 omx_venc::dev_resume(void)
{
    ENTER_FUNC();
    // nothing to be done for sw encoder

    RETURN(true);
}

OMX_U32 omx_venc::dev_start(void)
{
   ENTER_FUNC();
   SWVENC_STATUS Ret;
   Ret = swvenc_start(m_hSwVenc);
   if (Ret != SWVENC_S_SUCCESS)
   {
      DEBUG_PRINT_ERROR("%s, swvenc_start failed (%d)",
        __FUNCTION__, Ret);
      RETURN(-1);
   }

   m_stopped = false;
   if (m_bIsRotationSupported){
       Ret = swvenc_rotation_init();
       if (Ret == SWVENC_S_UNSUPPORTED) {
           DEBUG_PRINT_ERROR("ERROR: Rotation not supported for this target");
           m_bIsRotationSupported = false;
        }
   }
   RETURN(0);
}

OMX_U32 omx_venc::dev_flush(unsigned port)
{
   ENTER_FUNC();
   SWVENC_STATUS Ret;

   (void)port;
   Ret = swvenc_flush(m_hSwVenc);
   if (Ret != SWVENC_S_SUCCESS)
   {
      DEBUG_PRINT_ERROR("%s, swvenc_flush failed (%d)",
        __FUNCTION__, Ret);
      RETURN(-1);
   }

   RETURN(0);
}

OMX_U32 omx_venc::dev_start_done(void)
{
   ENTER_FUNC();

   /* post START_DONE event as start is synchronus */
   post_event (0, OMX_ErrorNone, OMX_COMPONENT_GENERATE_START_DONE);

   RETURN(0);
}

OMX_U32 omx_venc::dev_set_message_thread_id(pthread_t tid)
{
    ENTER_FUNC();

    // nothing to be done for sw encoder
    (void)tid;

    RETURN(true);
}

bool omx_venc::dev_use_buf(unsigned port)
{
    ENTER_FUNC();
    (void)port;
    RETURN(true);
}

bool omx_venc::dev_handle_empty_eos_buffer(void)
{
    ENTER_FUNC();
    SWVENC_STATUS Ret;
    SWVENC_IPBUFFER ipbuffer;
    ipbuffer.p_buffer = NULL;
    ipbuffer.filled_length =0;
    ipbuffer.flags = SWVENC_FLAG_EOS;
    Ret = swvenc_emptythisbuffer(m_hSwVenc, &ipbuffer);
    if (Ret != SWVENC_S_SUCCESS)
    {
       DEBUG_PRINT_ERROR("%s, swvenc_emptythisbuffer failed (%d)",
         __FUNCTION__, Ret);
       RETURN(false);
    }
    RETURN(true);
}

bool omx_venc::dev_free_buf(void *buf_addr,unsigned port)
{
    ENTER_FUNC();

    (void)buf_addr;
    (void)port;

    RETURN(true);
}

bool omx_venc::dev_empty_buf
(
    void *buffer,
    void *pmem_data_buf,
    unsigned index,
    unsigned fd
)
{
    ENTER_FUNC();

    SWVENC_STATUS Ret;
    SWVENC_IPBUFFER ipbuffer;
    OMX_BUFFERHEADERTYPE *bufhdr = (OMX_BUFFERHEADERTYPE *)buffer;
    unsigned int size = 0, filled_length, offset = 0;
    SWVENC_COLOR_FORMAT color_format;
    SWVENC_PROPERTY prop;

    (void)pmem_data_buf;
    (void)index;

    if (meta_mode_enable)
    {
        LEGACY_CAM_METADATA_TYPE *meta_buf = NULL;
        meta_buf = (LEGACY_CAM_METADATA_TYPE *)bufhdr->pBuffer;
        if(m_sInPortDef.format.video.eColorFormat == ((OMX_COLOR_FORMATTYPE) QOMX_COLOR_FormatAndroidOpaque))
        {
            DEBUG_PRINT_LOW("dev_empty_buf: color_format is QOMX_COLOR_FormatAndroidOpaque");
            set_format = true;
        }
        if(!meta_buf)
        {
            if (!bufhdr->nFilledLen && (bufhdr->nFlags & OMX_BUFFERFLAG_EOS))
            {
                ipbuffer.p_buffer= bufhdr->pBuffer;
                ipbuffer.size = bufhdr->nAllocLen;
                ipbuffer.filled_length = bufhdr->nFilledLen;
                DEBUG_PRINT_LOW("dev_empty_buf: empty EOS buffer");
            }
            else
            {
                return false;
            }
        }
        else
        {
            if (meta_buf->buffer_type == LEGACY_CAM_SOURCE)
            {
                offset = meta_buf->meta_handle->data[1];
                size = meta_buf->meta_handle->data[2];
                if (set_format && (meta_buf->meta_handle->numFds + meta_buf->meta_handle->numInts > 5))
                {
                    m_sInPortFormat.eColorFormat = (OMX_COLOR_FORMATTYPE)meta_buf->meta_handle->data[5];
                }
                ipbuffer.p_buffer = (unsigned char *)mmap(NULL, size, PROT_READ|PROT_WRITE,MAP_SHARED, fd, offset);
                if (ipbuffer.p_buffer == MAP_FAILED)
                {
                    DEBUG_PRINT_ERROR("mmap() failed for fd %d of size %d",fd,size);
                    RETURN(false);
                }
                ipbuffer.size = size;
                ipbuffer.filled_length = size;
            }
            else if (meta_buf->buffer_type == kMetadataBufferTypeGrallocSource)
            {
                VideoGrallocMetadata *meta_buf = (VideoGrallocMetadata *)bufhdr->pBuffer;
                private_handle_t *handle = (private_handle_t *)meta_buf->pHandle;
                size = handle->size;
                if (m_bUseAVTimerTimestamps) {
                    uint64_t avTimerTimestampNs = bufhdr->nTimeStamp * 1000;
                    if (getMetaData(handle, GET_VT_TIMESTAMP, &avTimerTimestampNs) == 0
                            && avTimerTimestampNs > 0) {
                        bufhdr->nTimeStamp = avTimerTimestampNs / 1000;
                        DEBUG_PRINT_LOW("AVTimer TS: %llu us", (unsigned long long)bufhdr->nTimeStamp);
                    }
                }
                if (set_format)
                {
                    DEBUG_PRINT_LOW("color format = 0x%x",handle->format);
                    if (((OMX_COLOR_FORMATTYPE)handle->format) != m_sInPortFormat.eColorFormat)
                    {
                        if(handle->format == HAL_PIXEL_FORMAT_NV12_ENCODEABLE)
                        {
                            DEBUG_PRINT_LOW("HAL_PIXEL_FORMAT_NV12_ENCODEABLE ");
                            m_sInPortFormat.eColorFormat = (OMX_COLOR_FORMATTYPE)
                                QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m;
                        }
                        else if(handle->format == HAL_PIXEL_FORMAT_NV21_ZSL)
                        {
                            /* HAL_PIXEL_FORMAT_NV21_ZSL format is NV21 format with 64,64 alignment,
                               this format support is added to address a CTS issue and OEM test app issue
                               which is trigerring this input format*/
                            DEBUG_PRINT_LOW("HAL_PIXEL_FORMAT_NV21_ZSL ");
                            m_sInPortFormat.eColorFormat = (OMX_COLOR_FORMATTYPE)
                                HAL_PIXEL_FORMAT_NV21_ZSL;
                        }
                        else if (handle->format == QOMX_COLOR_FormatYVU420SemiPlanar)
                        {
                            DEBUG_PRINT_LOW("HAL_PIXEL_FORMAT_NV21_ENCODEABLE ");
                            m_sInPortFormat.eColorFormat = (OMX_COLOR_FORMATTYPE)
                                QOMX_COLOR_FormatYVU420SemiPlanar;
                        }
                        else
                        {
                            DEBUG_PRINT_ERROR("%s: OMX_IndexParamVideoPortFormat 0x%x invalid",
                                              __FUNCTION__,handle->format);
                            RETURN(false);
                        }
                    }
                }
                ipbuffer.p_buffer = (unsigned char *)mmap(NULL, size, PROT_READ|PROT_WRITE,MAP_SHARED, fd, offset);
                if (ipbuffer.p_buffer == MAP_FAILED)
                {
                    DEBUG_PRINT_ERROR("mmap() failed for fd %d of size %d",fd,size);
                    RETURN(false);
                }
                ipbuffer.size = size;
                ipbuffer.filled_length = size;
            }
            else
            {
                //handles the use case for surface encode
                ipbuffer.p_buffer = bufhdr->pBuffer;
                ipbuffer.size = bufhdr->nAllocLen;
                ipbuffer.filled_length = bufhdr->nFilledLen;
            }
            if (set_format)
            {
                set_format = false;
                m_sInPortDef.format.video.eColorFormat = m_sInPortFormat.eColorFormat;
            }
        }
    }
    else
    {
        ipbuffer.p_buffer = bufhdr->pBuffer;
        ipbuffer.size = bufhdr->nAllocLen;
        ipbuffer.filled_length = bufhdr->nFilledLen;
    }
    if(update_offset)
    {
        update_offset = false;
        Ret = swvenc_set_color_format(m_sInPortFormat.eColorFormat);
        if (Ret != SWVENC_S_SUCCESS)
        {
            DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
                             __FUNCTION__, Ret);
            RETURN(false);
        }
    }
    ipbuffer.flags = 0;
    if (bufhdr->nFlags & OMX_BUFFERFLAG_EOS)
    {
      ipbuffer.flags |= SWVENC_FLAG_EOS;
    }
    ipbuffer.timestamp = bufhdr->nTimeStamp;
    ipbuffer.p_client_data = (unsigned char *)bufhdr;

    DEBUG_PRINT_LOW("ETB: p_buffer (%p) size (%d) filled_len (%d) flags (0x%X) timestamp (%lld) clientData (%p)",
      ipbuffer.p_buffer,
      ipbuffer.size,
      ipbuffer.filled_length,
      (unsigned int)ipbuffer.flags,
      ipbuffer.timestamp,
      ipbuffer.p_client_data);

    if (m_debug.in_buffer_log)
    {
       // dump before rotation, un-rotated buffer
       swvenc_input_log_buffers((const char*)ipbuffer.p_buffer, ipbuffer.filled_length);
    }

    if (m_bIsRotationSupported && m_sConfigFrameRotation.nRotation != 0) {
        if(!swvenc_do_rotate((int)fd, ipbuffer, (OMX_U32)index)) {
            DEBUG_PRINT_ERROR("rotate failed");
            return OMX_ErrorUndefined;
        }
        if (m_debug.in_buffer_rotated_log) {
            // dump after rotation, rotated buffer
            DEBUG_PRINT_ERROR("dump rotated");
            swvenc_input_log_rotated_buffers((const char*)ipbuffer.p_buffer, ipbuffer.filled_length);
        }
    }

    Ret = swvenc_emptythisbuffer(m_hSwVenc, &ipbuffer);
    if (Ret != SWVENC_S_SUCCESS)
    {
       DEBUG_PRINT_ERROR("%s, swvenc_emptythisbuffer failed (%d)",
         __FUNCTION__, Ret);
       RETURN(false);
    }

    RETURN(true);
}

bool omx_venc::dev_fill_buf
(
    void *buffer,
    void *pmem_data_buf,
    unsigned index,
    unsigned fd
)
{
    ENTER_FUNC();

    SWVENC_STATUS Ret;

    SWVENC_OPBUFFER opbuffer;
    OMX_BUFFERHEADERTYPE *bufhdr = (OMX_BUFFERHEADERTYPE *)buffer;

    (void)pmem_data_buf;
    (void)index;
    (void)fd;

    opbuffer.p_buffer = bufhdr->pBuffer;
    opbuffer.size = bufhdr->nAllocLen;
    opbuffer.filled_length = bufhdr->nFilledLen;
    opbuffer.flags = bufhdr->nFlags;
    opbuffer.timestamp = bufhdr->nTimeStamp;
    opbuffer.p_client_data = (unsigned char *)bufhdr;
    opbuffer.frame_type = SWVENC_FRAME_TYPE_I;

    DEBUG_PRINT_LOW("FTB: p_buffer (%p) size (%d) filled_len (%d) flags (0x%X) timestamp (%lld) clientData (%p)",
      opbuffer.p_buffer,
      opbuffer.size,
      opbuffer.filled_length,
      opbuffer.flags,
      opbuffer.timestamp,
      opbuffer.p_client_data);

    if ( false == m_bSeqHdrRequested)
    {
      if (dev_get_seq_hdr(opbuffer.p_buffer, opbuffer.size, &opbuffer.filled_length))
      {
         bufhdr->nFilledLen = opbuffer.filled_length;
         bufhdr->nOffset = 0;
         bufhdr->nTimeStamp = 0;
         bufhdr->nFlags = OMX_BUFFERFLAG_CODECCONFIG;

         DEBUG_PRINT_LOW("sending FBD with codec config");
         m_bSeqHdrRequested = true;
         post_event ((unsigned long)bufhdr,0,OMX_COMPONENT_GENERATE_FBD);
      }
      else
      {
         DEBUG_PRINT_ERROR("ERROR: couldn't get sequence header");
         post_event(OMX_EventError,OMX_ErrorUndefined,OMX_COMPONENT_GENERATE_EVENT);
      }
    }
    else
    {
       Ret = swvenc_fillthisbuffer(m_hSwVenc, &opbuffer);
       if (Ret != SWVENC_S_SUCCESS)
       {
          DEBUG_PRINT_ERROR("%s, swvenc_fillthisbuffer failed (%d)",
            __FUNCTION__, Ret);
          RETURN(false);
       }
    }

    RETURN(true);
}

bool omx_venc::dev_get_seq_hdr
(
   void *buffer,
   unsigned size,
   unsigned *hdrlen
)
{
   ENTER_FUNC();

   SWVENC_STATUS Ret;
   SWVENC_OPBUFFER Buffer;

   Buffer.p_buffer = (unsigned char*) buffer;
   Buffer.size = size;

   Ret = swvenc_getsequenceheader(m_hSwVenc, &Buffer);
   if (Ret != SWVENC_S_SUCCESS)
   {
      DEBUG_PRINT_ERROR("%s, swvenc_getsequenceheader failed (%d)",
        __FUNCTION__, Ret);
      RETURN(false);
   }

   *hdrlen = Buffer.filled_length;

   RETURN(true);
}

bool omx_venc::dev_get_capability_ltrcount
(
   OMX_U32 *min,
   OMX_U32 *max,
   OMX_U32 *step_size
)
{
    ENTER_FUNC();

    (void)min;
    (void)max;
    (void)step_size;

    DEBUG_PRINT_ERROR("Get Capability LTR Count is not supported");

    RETURN(false);
}

bool omx_venc::dev_get_vui_timing_info(OMX_U32 *enabled)
{
    ENTER_FUNC();

    (void)enabled;
    DEBUG_PRINT_ERROR("Get vui timing information is not supported");

    RETURN(false);
}

bool omx_venc::dev_get_peak_bitrate(OMX_U32 *peakbitrate)
{
    //TBD: store the peak bitrate in class and return here;
    ENTER_FUNC();

    (void)peakbitrate;
    DEBUG_PRINT_ERROR("Get peak bitrate is not supported");

    RETURN(false);
}

bool omx_venc::dev_get_batch_size(OMX_U32 *size)
{
    ENTER_FUNC();

    (void)size;

    DEBUG_PRINT_ERROR("Get batch size is not supported");

    RETURN(false);
}

OMX_ERRORTYPE omx_venc::dev_get_supported_profile_level(OMX_VIDEO_PARAM_PROFILELEVELTYPE *profileLevelType)
{
    ENTER_FUNC();
    OMX_ERRORTYPE eRet = OMX_ErrorNone;

   if (profileLevelType == NULL)
   {
        DEBUG_PRINT_ERROR("p_profilelevel = NULL");
        return OMX_ErrorBadParameter;
   }

    if (profileLevelType->nPortIndex == 1) {
        if (m_sOutPortDef.format.video.eCompressionFormat == OMX_VIDEO_CodingH263)
        {
            if (profileLevelType->nProfileIndex == 0)
            {
                profileLevelType->eProfile = OMX_VIDEO_H263ProfileBaseline;
                profileLevelType->eLevel   = OMX_VIDEO_H263Level70;

                DEBUG_PRINT_HIGH("H.263 baseline profile, level 70");
            }
            else
            {
                DEBUG_PRINT_LOW("dev_get_supported_profile_level:nProfileIndex ret NoMore %u",
                    (unsigned int)profileLevelType->nProfileIndex);
                eRet = OMX_ErrorNoMore;
            }
        }
        else if (m_sOutPortDef.format.video.eCompressionFormat == OMX_VIDEO_CodingMPEG4)
        {
            if (profileLevelType->nProfileIndex == 0)
            {
                profileLevelType->eProfile = OMX_VIDEO_MPEG4ProfileSimple;
                profileLevelType->eLevel   = OMX_VIDEO_MPEG4Level5;

                DEBUG_PRINT_LOW("MPEG-4 simple profile, level 5");
            }
            else
            {
                DEBUG_PRINT_LOW("dev_get_supported_profile_level:nProfileIndex ret NoMore %u",
                    (unsigned int)profileLevelType->nProfileIndex);
                 eRet = OMX_ErrorNoMore;
            }
        }
        else
        {
            DEBUG_PRINT_ERROR("get_parameter: dev_get_supported_profile_level ret NoMore");
            eRet = OMX_ErrorNoMore;
        }
    }
    else
    {
        DEBUG_PRINT_ERROR("get_parameter: dev_get_supported_profile_level should be queried on Input port only %u",
            (unsigned int)profileLevelType->nPortIndex);
        eRet = OMX_ErrorBadPortIndex;
    }
    return eRet;
}

bool omx_venc::dev_get_supported_color_format(unsigned index, OMX_U32 *colorFormat) {
    // we support two formats
    // index 0 - Venus flavour of YUV420SP
    // index 1 - opaque which internally maps to YUV420SP
    // index 2 - vannilla YUV420SP
    // this can be extended in the future
    int supportedFormats[] = {
        [0] = QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m,
        [1] = QOMX_COLOR_FormatYVU420SemiPlanar,
        [2] = QOMX_COLOR_FormatAndroidOpaque,
        [3] = OMX_COLOR_FormatYUV420SemiPlanar,
    };

    if (index > (sizeof(supportedFormats)/sizeof(*supportedFormats) - 1))
        return false;
    *colorFormat = supportedFormats[index];
    return true;
}

bool omx_venc::dev_loaded_start()
{
   ENTER_FUNC();
   RETURN(true);
}

bool omx_venc::dev_loaded_stop()
{
   ENTER_FUNC();
   RETURN(true);
}

bool omx_venc::dev_loaded_start_done()
{
   ENTER_FUNC();
   RETURN(true);
}

bool omx_venc::dev_loaded_stop_done()
{
   ENTER_FUNC();
   RETURN(true);
}

bool omx_venc::is_streamon_done(OMX_U32 port)
{
    if (PORT_INDEX_OUT <= port)
        ENTER_FUNC();
    RETURN(false);
}

bool omx_venc::dev_get_buf_req(OMX_U32 *min_buff_count,
        OMX_U32 *actual_buff_count,
        OMX_U32 *buff_size,
        OMX_U32 port)
{
   ENTER_FUNC();

   bool bRet = true;
   OMX_PARAM_PORTDEFINITIONTYPE *PortDef;

   if (PORT_INDEX_IN == port)
   {
     PortDef = &m_sInPortDef;
   }
   else if (PORT_INDEX_OUT == port)
   {
     PortDef = &m_sOutPortDef;
   }
   else
   {
     DEBUG_PRINT_ERROR("ERROR: %s, Unsupported parameter", __FUNCTION__);
     bRet = false;
   }

   if (true == bRet)
   {
      *min_buff_count = PortDef->nBufferCountMin;
      *actual_buff_count = PortDef->nBufferCountActual;
      *buff_size = PortDef->nBufferSize;
   }

   RETURN(true);
}

bool omx_venc::dev_set_buf_req
(
   OMX_U32 const *min_buff_count,
   OMX_U32 const *actual_buff_count,
   OMX_U32 const *buff_size,
   OMX_U32 port
)
{
   ENTER_FUNC();

   SWVENC_STATUS Ret;
   OMX_PARAM_PORTDEFINITIONTYPE *PortDef;

   (void)min_buff_count;
   if (PORT_INDEX_IN == port)
   {
     PortDef = &m_sInPortDef;
   }
   else if (PORT_INDEX_OUT == port)
   {
     PortDef = &m_sOutPortDef;
   }
   else
   {
     DEBUG_PRINT_ERROR("ERROR: %s, Unsupported parameter", __FUNCTION__);
     RETURN(false);
   }

   if (*actual_buff_count < PortDef->nBufferCountMin)
   {
      DEBUG_PRINT_ERROR("ERROR: %s, (actual,min) buffer count (%d, %d)",
         __FUNCTION__, *actual_buff_count, PortDef->nBufferCountMin);
      RETURN(false);
   }
   if (false == meta_mode_enable)
   {
      if (*buff_size < PortDef->nBufferSize)
      {
          DEBUG_PRINT_ERROR("ERROR: %s, (new,old) buffer count (%d, %d)",
             __FUNCTION__, *actual_buff_count, PortDef->nBufferCountMin);
          RETURN(false);
      }
   }

   RETURN(true);
}

bool omx_venc::dev_is_video_session_supported(OMX_U32 width, OMX_U32 height)
{
   ENTER_FUNC();

   if ( (width * height < m_capability.min_width *  m_capability.min_height) ||
        (width * height > m_capability.max_width *  m_capability.max_height)
      )
   {
       DEBUG_PRINT_ERROR(
         "Unsupported Resolution WxH = (%u)x(%u) Supported Range = min (%d)x(%d) - max (%d)x(%d)",
         width, height,
         m_capability.min_width, m_capability.min_height,
         m_capability.max_width, m_capability.max_height);
       RETURN(false);
   }

   RETURN(true);
}

bool omx_venc::dev_buffer_ready_to_queue(OMX_BUFFERHEADERTYPE *buffer)
{
   ENTER_FUNC();

   (void)buffer;
   RETURN(true);
}
int omx_venc::dev_handle_output_extradata(void *buffer, int fd)
{
   ENTER_FUNC();

   (void)buffer;
   (void)fd;

   RETURN(true);
}

int omx_venc::dev_handle_input_extradata(void *buffer, int fd, int index)
{
   ENTER_FUNC();

   (void)buffer;
   (void)fd;
   (void)index;

   RETURN(true);
}

void omx_venc::dev_set_extradata_cookie(void *buffer)
{
   ENTER_FUNC();

   (void)buffer;
}

int omx_venc::dev_set_format(int color)
{
   ENTER_FUNC();

   (void)color;

   RETURN(true);
    //return handle->venc_set_format(color);
}

bool omx_venc::dev_get_dimensions(OMX_U32 index, OMX_U32 *width, OMX_U32 *height)
{
   ENTER_FUNC();

   (void)index;
   (void)width;
   (void)height;

   RETURN(true);
}

bool omx_venc::dev_is_meta_mode()
{
   ENTER_FUNC();

   RETURN(true);
}

bool omx_venc::dev_is_avtimer_needed()
{
   ENTER_FUNC();

   RETURN(true);
}

bool omx_venc::dev_color_align(OMX_BUFFERHEADERTYPE *buffer,
                OMX_U32 width, OMX_U32 height)
{
    ENTER_FUNC();

    if(secure_session) {
        DEBUG_PRINT_ERROR("Cannot align colors in secure session.");
        RETURN(false);
    }
    return swvenc_color_align(buffer, width,height);
}

bool omx_venc::is_secure_session()
{
    ENTER_FUNC();

    RETURN(secure_session);
}

bool omx_venc::dev_get_output_log_flag()
{
    ENTER_FUNC();

    RETURN(m_debug.out_buffer_log == 1);
}

int omx_venc::dev_output_log_buffers(const char *buffer, int bufferlen, uint64_t ts)
{
    (void) ts;
    ENTER_FUNC();

    if (m_debug.out_buffer_log && !m_debug.outfile)
    {
        int size = 0;
        int width = m_sOutPortDef.format.video.nFrameWidth;
        int height = m_sOutPortDef.format.video.nFrameHeight;
        if(SWVENC_CODEC_MPEG4 == m_codec)
        {
           size = snprintf(m_debug.outfile_name, PROPERTY_VALUE_MAX,
              "%s/output_enc_%d_%d_%p.m4v",
              m_debug.log_loc, width, height, this);
        }
        else if(SWVENC_CODEC_H263 == m_codec)
        {
           size = snprintf(m_debug.outfile_name, PROPERTY_VALUE_MAX,
              "%s/output_enc_%d_%d_%p.263",
              m_debug.log_loc, width, height, this);
        }
        if ((size > PROPERTY_VALUE_MAX) || (size < 0))
        {
           DEBUG_PRINT_ERROR("Failed to open output file: %s for logging as size:%d",
                              m_debug.outfile_name, size);
           RETURN(-1);
        }
        DEBUG_PRINT_LOW("output filename = %s", m_debug.outfile_name);
        m_debug.outfile = fopen(m_debug.outfile_name, "ab");
        if (!m_debug.outfile)
        {
           DEBUG_PRINT_ERROR("Failed to open output file: %s for logging errno:%d",
                             m_debug.outfile_name, errno);
           m_debug.outfile_name[0] = '\0';
           RETURN(-1);
        }
    }
    if (m_debug.outfile && buffer && bufferlen)
    {
        DEBUG_PRINT_LOW("%s buffer length: %d", __func__, bufferlen);
        fwrite(buffer, bufferlen, 1, m_debug.outfile);
    }

    RETURN(0);
}

int omx_venc::swvenc_input_log_buffers(const char *buffer, int bufferlen)
{
   int width = m_sInPortDef.format.video.nFrameWidth;
   int height = m_sInPortDef.format.video.nFrameHeight;
   int stride = SWVENC_Y_STRIDE(COLOR_FMT_NV12, width);
   int scanlines = SWVENC_Y_SCANLINES(COLOR_FMT_NV12, height);
   char *temp = (char*)buffer;

   if (!m_debug.infile)
   {
       int size = snprintf(m_debug.infile_name, PROPERTY_VALUE_MAX,
                      "%s/input_enc_%d_%d_%p.yuv",
                      m_debug.log_loc, width, height, this);
       if ((size > PROPERTY_VALUE_MAX) || (size < 0))
       {
           DEBUG_PRINT_ERROR("Failed to open input file: %s for logging size:%d",
                              m_debug.infile_name, size);
           RETURN(-1);
       }
       DEBUG_PRINT_LOW("input filename = %s", m_debug.infile_name);
       m_debug.infile = fopen (m_debug.infile_name, "ab");
       if (!m_debug.infile)
       {
           DEBUG_PRINT_HIGH("Failed to open input file: %s for logging",
              m_debug.infile_name);
           m_debug.infile_name[0] = '\0';
           RETURN(-1);
       }
   }
   if (m_debug.infile && buffer && bufferlen)
   {
       DEBUG_PRINT_LOW("%s buffer length: %d", __func__, bufferlen);
       for (int i = 0; i < height; i++)
       {
          fwrite(temp, width, 1, m_debug.infile);
          temp += stride;
       }
       temp = (char*)(buffer + (stride * scanlines));
       for(int i = 0; i < height/2; i++)
       {
          fwrite(temp, width, 1, m_debug.infile);
          temp += stride;
      }
   }

   RETURN(0);
}

int omx_venc::swvenc_input_log_rotated_buffers(const char *buffer, int bufferlen)
{
   int width = m_sInPortDef.format.video.nFrameWidth;
   int height = m_sInPortDef.format.video.nFrameHeight;
   if (m_bIsInFlipDone) {
       auto v = width;
       width = height;
       height = v;
   }
   int stride = SWVENC_Y_STRIDE(COLOR_FMT_NV12, width);
   int scanlines = SWVENC_Y_SCANLINES(COLOR_FMT_NV12, height);
   char *temp = (char*)buffer;

   if (!m_debug.inrotatedfile)
   {
       int size = snprintf(m_debug.inrotatedfile_name, PROPERTY_VALUE_MAX,
                      "%s/input_enc_rotated_%d_%d_%p.yuv",
                      m_debug.log_loc, width, height, this);
       if ((size > PROPERTY_VALUE_MAX) || (size < 0))
       {
           DEBUG_PRINT_ERROR("Failed to open input rotated file: %s for logging size:%d",
                              m_debug.inrotatedfile_name, size);
           RETURN(-1);
       }
       DEBUG_PRINT_LOW("input rotated filename = %s", m_debug.inrotatedfile_name);
       m_debug.inrotatedfile = fopen (m_debug.inrotatedfile_name, "ab");
       if (!m_debug.inrotatedfile)
       {
           DEBUG_PRINT_HIGH("Failed to open input rotated file: %s for logging",
              m_debug.inrotatedfile_name);
           m_debug.inrotatedfile_name[0] = '\0';
           RETURN(-1);
       }
   }
   if (m_debug.inrotatedfile && buffer && bufferlen)
   {
       DEBUG_PRINT_LOW("%s buffer length: %d", __func__, bufferlen);
       for (int i = 0; i < height; i++)
       {
          fwrite(temp, width, 1, m_debug.inrotatedfile);
          temp += stride;
       }
       temp = (char*)(buffer + (stride * scanlines));
       for(int i = 0; i < height/2; i++)
       {
          fwrite(temp, width, 1, m_debug.inrotatedfile);
          temp += stride;
      }
   }

   RETURN(0);
}

int omx_venc::dev_extradata_log_buffers(char *buffer, int index, bool input)
{
   ENTER_FUNC();

   (void)buffer;
   (void)index;
   (void)input;

   RETURN(true);
    //return handle->venc_extradata_log_buffers(buffer);
}

SWVENC_STATUS omx_venc::swvenc_get_buffer_req
(
   OMX_U32 *min_buff_count,
   OMX_U32 *actual_buff_count,
   OMX_U32 *buff_size,
   OMX_U32 *buff_alignment,
   OMX_U32 port
)
{
    ENTER_FUNC();

    SWVENC_PROPERTY Prop;
    SWVENC_STATUS Ret;
    OMX_PARAM_PORTDEFINITIONTYPE *PortDef;

    Prop.id = SWVENC_PROPERTY_ID_BUFFER_REQ;
    if (PORT_INDEX_IN == port)
    {
      Prop.info.buffer_req.type = SWVENC_BUFFER_INPUT;
    }
    else if (PORT_INDEX_OUT == port)
    {
      Prop.info.buffer_req.type = SWVENC_BUFFER_OUTPUT;
    }
    else
    {
      DEBUG_PRINT_ERROR("ERROR: %s, Unsupported parameter", __FUNCTION__);
      RETURN(SWVENC_S_INVALID_PARAMETERS);
    }

    Ret = swvenc_getproperty(m_hSwVenc, &Prop);
    if (Ret != SWVENC_S_SUCCESS)
    {
       DEBUG_PRINT_ERROR("ERROR: %s, swvenc_setproperty failed (%d)", __FUNCTION__,
          Ret);
       RETURN(SWVENC_S_INVALID_PARAMETERS);
    }

    *buff_size = Prop.info.buffer_req.size;
    *min_buff_count = Prop.info.buffer_req.mincount;
    *actual_buff_count = Prop.info.buffer_req.mincount;
    *buff_alignment = Prop.info.buffer_req.alignment;

    RETURN(Ret);
}

SWVENC_STATUS omx_venc::swvenc_empty_buffer_done_cb
(
    SWVENC_HANDLE    swvenc,
    SWVENC_IPBUFFER *p_ipbuffer,
    void            *p_client
)
{
    ENTER_FUNC();

    (void)swvenc;
    SWVENC_STATUS eRet = SWVENC_S_SUCCESS;
    omx_venc *omx = reinterpret_cast<omx_venc*>(p_client);

    if (p_ipbuffer == NULL)
    {
        eRet = SWVENC_S_FAILURE;
    }
    else
    {
        omx->swvenc_empty_buffer_done(p_ipbuffer);
    }
    return eRet;
}

SWVENC_STATUS omx_venc::swvenc_empty_buffer_done
(
    SWVENC_IPBUFFER *p_ipbuffer
)
{
    SWVENC_STATUS eRet = SWVENC_S_SUCCESS;
    OMX_ERRORTYPE error = OMX_ErrorNone;
    OMX_BUFFERHEADERTYPE* omxhdr = NULL;

    //omx_video *omx = reinterpret_cast<omx_video*>(p_client);
    if (!p_ipbuffer) {
        DEBUG_PRINT_ERROR("EBD: null buffer");
        return SWVENC_S_NULL_POINTER;
    }

    omxhdr = (OMX_BUFFERHEADERTYPE*)p_ipbuffer->p_client_data;

    DEBUG_PRINT_LOW("EBD: clientData (%p)", p_ipbuffer->p_client_data);

    if ( (omxhdr == NULL) ||
         ( ((OMX_U32)(omxhdr - m_inp_mem_ptr) >m_sInPortDef.nBufferCountActual) &&
           ((OMX_U32)(omxhdr - meta_buffer_hdr) >m_sInPortDef.nBufferCountActual)
         )
       )
    {
        omxhdr = NULL;
        error = OMX_ErrorUndefined;
    }

    if (m_pIpbuffers != nullptr) {
        int index = omxhdr - ((mUseProxyColorFormat && !mUsesColorConversion) ? meta_buffer_hdr : m_inp_mem_ptr);
        DEBUG_PRINT_HIGH("restore ipbuffer[p_buffer(%p), size(%d), filled_length(%d)] to original ipbuffer[p_buffer(%p), size(%d), filled_length(%d)]",
                p_ipbuffer->p_buffer,
                p_ipbuffer->size,
                p_ipbuffer->filled_length,
                m_pIpbuffers[index].p_buffer,
                m_pIpbuffers[index].size,
                m_pIpbuffers[index].filled_length);
        p_ipbuffer->size = m_pIpbuffers[index].size;
        p_ipbuffer->filled_length = m_pIpbuffers[index].filled_length;
        p_ipbuffer->p_buffer = m_pIpbuffers[index].p_buffer;
    }

    if (omxhdr != NULL)
    {
        // unmap the input buffer->pBuffer
        omx_release_meta_buffer(omxhdr);
#ifdef _ANDROID_ICS_
        if (meta_mode_enable)
        {
           LEGACY_CAM_METADATA_TYPE *meta_buf = NULL;
           unsigned int size = 0;
           meta_buf = (LEGACY_CAM_METADATA_TYPE *)omxhdr->pBuffer;
           if (meta_buf)
           {
              if (meta_buf->buffer_type == LEGACY_CAM_SOURCE)
              {
                  size = meta_buf->meta_handle->data[2];
              }
              else if (meta_buf->buffer_type == kMetadataBufferTypeGrallocSource)
              {
                  VideoGrallocMetadata *meta_buf = (VideoGrallocMetadata *)omxhdr->pBuffer;
                  private_handle_t *handle = (private_handle_t *)meta_buf->pHandle;
                  size = handle->size;
              }
           }

           DEBUG_PRINT_HIGH("Unmapping pBuffer <%p> size <%d>", p_ipbuffer->p_buffer, size);
           if (-1 == munmap(p_ipbuffer->p_buffer, size))
               DEBUG_PRINT_HIGH("Unmap failed");
        }
#endif
        post_event ((unsigned long)omxhdr,error,OMX_COMPONENT_GENERATE_EBD);
    }
    RETURN(eRet);
}

SWVENC_STATUS omx_venc::swvenc_fill_buffer_done_cb
(
    SWVENC_HANDLE    swvenc,
    SWVENC_OPBUFFER *p_opbuffer,
    void            *p_client
)
{
    ENTER_FUNC();

    SWVENC_STATUS eRet = SWVENC_S_SUCCESS;
    OMX_ERRORTYPE error = OMX_ErrorNone;
    OMX_BUFFERHEADERTYPE* omxhdr = NULL;
    omx_video *omx = reinterpret_cast<omx_video*>(p_client);

    (void)swvenc;

    if (p_opbuffer != NULL)
    {
        omxhdr = (OMX_BUFFERHEADERTYPE*)p_opbuffer->p_client_data;
    }

    if ( (p_opbuffer != NULL) &&
         ((OMX_U32)(omxhdr - omx->m_out_mem_ptr)  < omx->m_sOutPortDef.nBufferCountActual)
       )
    {
        DEBUG_PRINT_LOW("FBD: clientData (%p) buffer (%p) filled_lengh (%d) flags (0x%x) ts (%lld)",
          p_opbuffer->p_client_data,
          p_opbuffer->p_buffer,
          p_opbuffer->filled_length,
          p_opbuffer->flags,
          p_opbuffer->timestamp);

        if (p_opbuffer->filled_length <=  omxhdr->nAllocLen)
        {
            omxhdr->pBuffer = p_opbuffer->p_buffer;
            omxhdr->nFilledLen = p_opbuffer->filled_length;
            omxhdr->nOffset = 0;
            omxhdr->nTimeStamp = p_opbuffer->timestamp;
            omxhdr->nFlags = 0;
            if (SWVENC_FRAME_TYPE_I == p_opbuffer->frame_type)
            {
               omxhdr->nFlags |= OMX_BUFFERFLAG_SYNCFRAME;
            }
            if (SWVENC_FLAG_EOS & p_opbuffer->flags)
            {
               omxhdr->nFlags |= OMX_BUFFERFLAG_EOS;
            }
            if(omxhdr->nFilledLen)
            {
               omxhdr->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;
            }
            DEBUG_PRINT_LOW("o/p flag = 0x%x", omxhdr->nFlags);

            /* Use buffer case */
            if (omx->output_use_buffer && !omx->m_use_output_pmem)
            {
                DEBUG_PRINT_LOW("memcpy() for o/p Heap UseBuffer");
                memcpy( omxhdr->pBuffer,
                        (p_opbuffer->p_buffer),
                        p_opbuffer->filled_length );
            }
        }
        else
        {
            omxhdr->nFilledLen = 0;
        }

    }
    else
    {
        omxhdr = NULL;
        error = OMX_ErrorUndefined;
    }

    omx->post_event ((unsigned long)omxhdr,error,OMX_COMPONENT_GENERATE_FBD);

    RETURN(eRet);
}

SWVENC_STATUS omx_venc::swvenc_handle_event_cb
(
    SWVENC_HANDLE swvenc,
    SWVENC_EVENT  event,
    void         *p_client
)
{
    ENTER_FUNC();

    SWVENC_STATUS eRet = SWVENC_S_SUCCESS;
    omx_video *omx = reinterpret_cast<omx_video*>(p_client);

    OMX_BUFFERHEADERTYPE* omxhdr = NULL;

    (void)swvenc;

    if (omx == NULL || p_client == NULL)
    {
        DEBUG_PRINT_ERROR("ERROR: %s invalid i/p params", __FUNCTION__);
        RETURN(SWVENC_S_NULL_POINTER);
    }

    DEBUG_PRINT_LOW("swvenc_handle_event_cb - event = %d", event);

    switch (event)
    {
        case SWVENC_EVENT_FLUSH_DONE:
        {
           DEBUG_PRINT_ERROR("SWVENC_EVENT_FLUSH_DONE input_flush_progress %d output_flush_progress %d",
            omx->input_flush_progress, omx->output_flush_progress);
           if (omx->input_flush_progress)
           {
               omx->post_event ((unsigned)NULL, SWVENC_S_SUCCESS,
                  OMX_COMPONENT_GENERATE_EVENT_INPUT_FLUSH);
           }
           if (omx->output_flush_progress)
           {
               omx->post_event ((unsigned)NULL, SWVENC_S_SUCCESS,
                  OMX_COMPONENT_GENERATE_EVENT_OUTPUT_FLUSH);
           }
           break;
        }

        case SWVENC_EVENT_FATAL_ERROR:
        {
           DEBUG_PRINT_ERROR("ERROR: SWVENC_EVENT_FATAL_ERROR");
           omx->omx_report_error();
           break;
        }

        default:
            DEBUG_PRINT_HIGH("Unknown event received : %d", event);
            break;
    }

    RETURN(eRet);
}

SWVENC_STATUS omx_venc::swvenc_set_rc_mode
(
    OMX_VIDEO_CONTROLRATETYPE eControlRate
)
{
    ENTER_FUNC();

    SWVENC_STATUS Ret = SWVENC_S_SUCCESS;
    SWVENC_RC_MODE rc_mode;
    SWVENC_PROPERTY Prop;

    switch (eControlRate)
    {
        case OMX_Video_ControlRateDisable:
            rc_mode = SWVENC_RC_MODE_NONE;
            break;
        case OMX_Video_ControlRateVariableSkipFrames:
            rc_mode = SWVENC_RC_MODE_VBR_VFR;
            break;
        case OMX_Video_ControlRateVariable:
            rc_mode = SWVENC_RC_MODE_VBR_CFR;
            break;
        case OMX_Video_ControlRateConstantSkipFrames:
            rc_mode = SWVENC_RC_MODE_CBR_VFR;
            break;
        case OMX_Video_ControlRateConstant:
            rc_mode = SWVENC_RC_MODE_CBR_CFR;
            break;
        default:
            DEBUG_PRINT_ERROR("ERROR: UNKNOWN RC MODE");
            Ret = SWVENC_S_FAILURE;
            break;
    }

    if (SWVENC_S_SUCCESS == Ret)
    {
        Prop.id = SWVENC_PROPERTY_ID_RC_MODE;
        Prop.info.rc_mode = rc_mode;
        Ret = swvenc_setproperty(m_hSwVenc, &Prop);
        if (Ret != SWVENC_S_SUCCESS)
        {
           DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
             __FUNCTION__, Ret);
           RETURN(SWVENC_S_FAILURE);
        }
    }

    RETURN(Ret);
}

SWVENC_STATUS omx_venc::swvenc_set_profile_level
(
    OMX_U32 eProfile,
    OMX_U32 eLevel
)
{
    ENTER_FUNC();

    SWVENC_STATUS Ret = SWVENC_S_SUCCESS;
    SWVENC_PROPERTY Prop;
    SWVENC_PROFILE Profile;
    SWVENC_LEVEL Level;

    /* set the profile */
    if (SWVENC_CODEC_MPEG4 == m_codec)
    {
       switch (eProfile)
       {
          case OMX_VIDEO_MPEG4ProfileSimple:
             Profile.mpeg4 = SWVENC_PROFILE_MPEG4_SIMPLE;
             break;
          case OMX_VIDEO_MPEG4ProfileAdvancedSimple:
             Profile.mpeg4 = SWVENC_PROFILE_MPEG4_ADVANCED_SIMPLE;
             break;
          default:
             DEBUG_PRINT_ERROR("ERROR: UNKNOWN PROFILE");
             Ret = SWVENC_S_FAILURE;
             break;
       }
       switch (eLevel)
       {
          case OMX_VIDEO_MPEG4Level0:
             Level.mpeg4 = SWVENC_LEVEL_MPEG4_0;
             break;
          case OMX_VIDEO_MPEG4Level0b:
             Level.mpeg4 = SWVENC_LEVEL_MPEG4_0B;
             break;
          case OMX_VIDEO_MPEG4Level1:
             Level.mpeg4 = SWVENC_LEVEL_MPEG4_1;
             break;
          case OMX_VIDEO_MPEG4Level2:
             Level.mpeg4 = SWVENC_LEVEL_MPEG4_2;
             break;
          case OMX_VIDEO_MPEG4Level3:
             Level.mpeg4 = SWVENC_LEVEL_MPEG4_3;
             break;
          case OMX_VIDEO_MPEG4Level4:
             Level.mpeg4 = SWVENC_LEVEL_MPEG4_4;
             break;
          case OMX_VIDEO_MPEG4Level4a:
             Level.mpeg4 = SWVENC_LEVEL_MPEG4_4A;
             break;
          case OMX_VIDEO_MPEG4Level5:
             Level.mpeg4 = SWVENC_LEVEL_MPEG4_5;
             break;
          default:
             DEBUG_PRINT_ERROR("ERROR: UNKNOWN LEVEL");
             Ret = SWVENC_S_FAILURE;
             break;
       }
    }
    else if (SWVENC_CODEC_H263 == m_codec)
    {
       switch (eProfile)
       {
          case OMX_VIDEO_H263ProfileBaseline:
             Profile.h263 = SWVENC_PROFILE_H263_BASELINE;
             break;
          default:
             DEBUG_PRINT_ERROR("ERROR: UNKNOWN PROFILE");
             Ret = SWVENC_S_FAILURE;
             break;
       }
       switch (eLevel)
       {
          case OMX_VIDEO_H263Level10:
             Level.h263 = SWVENC_LEVEL_H263_10;
             break;
          case OMX_VIDEO_H263Level20:
             Level.h263 = SWVENC_LEVEL_H263_20;
             break;
          case OMX_VIDEO_H263Level30:
             Level.h263 = SWVENC_LEVEL_H263_30;
             break;
          case OMX_VIDEO_H263Level40:
             Level.h263 = SWVENC_LEVEL_H263_40;
             break;
          case OMX_VIDEO_H263Level45:
             Level.h263 = SWVENC_LEVEL_H263_45;
             break;
          case OMX_VIDEO_H263Level50:
             Level.h263 = SWVENC_LEVEL_H263_50;
             break;
          case OMX_VIDEO_H263Level60:
             Level.h263 = SWVENC_LEVEL_H263_60;
             break;
          case OMX_VIDEO_H263Level70:
             Level.h263 = SWVENC_LEVEL_H263_70;
             break;
          default:
             DEBUG_PRINT_ERROR("ERROR: UNKNOWN LEVEL");
             Ret = SWVENC_S_FAILURE;
             break;
       }
    }
    else
    {
      DEBUG_PRINT_ERROR("ERROR: UNSUPPORTED CODEC");
      Ret = SWVENC_S_FAILURE;
    }

    if (SWVENC_S_SUCCESS == Ret)
    {
       Prop.id = SWVENC_PROPERTY_ID_PROFILE;
       Prop.info.profile = Profile;

       /* set the profile */
       Ret = swvenc_setproperty(m_hSwVenc, &Prop);
       if (Ret != SWVENC_S_SUCCESS)
       {
          DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
            __FUNCTION__, Ret);
          RETURN(SWVENC_S_FAILURE);
       }

       /* set the level */
       Prop.id = SWVENC_PROPERTY_ID_LEVEL;
       Prop.info.level = Level;

       Ret = swvenc_setproperty(m_hSwVenc, &Prop);
       if (Ret != SWVENC_S_SUCCESS)
       {
          DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
            __FUNCTION__, Ret);
          RETURN(SWVENC_S_FAILURE);
       }
    }

    RETURN(Ret);
}

SWVENC_STATUS omx_venc::swvenc_set_intra_refresh
(
    OMX_VIDEO_PARAM_INTRAREFRESHTYPE *IntraRefresh
)
{
   ENTER_FUNC();

   SWVENC_STATUS Ret = SWVENC_S_SUCCESS;
   SWVENC_IR_CONFIG ir_config;
   SWVENC_PROPERTY Prop;

   switch (IntraRefresh->eRefreshMode)
   {
      case OMX_VIDEO_IntraRefreshCyclic:
        Prop.info.ir_config.mode = SWVENC_IR_MODE_CYCLIC;
        break;
      case OMX_VIDEO_IntraRefreshAdaptive:
         Prop.info.ir_config.mode = SWVENC_IR_MODE_ADAPTIVE;
        break;
      case OMX_VIDEO_IntraRefreshBoth:
         Prop.info.ir_config.mode = SWVENC_IR_MODE_CYCLIC_ADAPTIVE;
        break;
      case OMX_VIDEO_IntraRefreshRandom:
         Prop.info.ir_config.mode = SWVENC_IR_MODE_RANDOM;
        break;
      default:
         DEBUG_PRINT_ERROR("ERROR: UNKNOWN INTRA REFRESH MODE");
         Ret = SWVENC_S_FAILURE;
         break;
   }

   if (SWVENC_S_SUCCESS == Ret)
   {
       Prop.id = SWVENC_PROPERTY_ID_IR_CONFIG;
       Prop.info.ir_config.cir_mbs = IntraRefresh->nCirMBs;

       Ret = swvenc_setproperty(m_hSwVenc, &Prop);
       if (Ret != SWVENC_S_SUCCESS)
       {
          DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
            __FUNCTION__, Ret);
          Ret = SWVENC_S_FAILURE;
       }
   }

   RETURN(Ret);
}

SWVENC_STATUS omx_venc::swvenc_set_frame_rate
(
    OMX_U32 nFrameRate
)
{
   ENTER_FUNC();

   SWVENC_STATUS Ret = SWVENC_S_SUCCESS;
   SWVENC_PROPERTY Prop;

   Prop.id = SWVENC_PROPERTY_ID_FRAME_RATE;
   Prop.info.frame_rate = nFrameRate;

   Ret = swvenc_setproperty(m_hSwVenc, &Prop);
   if (Ret != SWVENC_S_SUCCESS)
   {
      DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
        __FUNCTION__, Ret);
      Ret = SWVENC_S_FAILURE;
   }

   RETURN(Ret);
}

SWVENC_STATUS omx_venc::swvenc_set_bit_rate
(
    OMX_U32 nTargetBitrate
)
{
   ENTER_FUNC();

   SWVENC_STATUS Ret = SWVENC_S_SUCCESS;
   SWVENC_PROPERTY Prop;

   Prop.id = SWVENC_PROPERTY_ID_TARGET_BITRATE;
   Prop.info.target_bitrate = nTargetBitrate;

   Ret = swvenc_setproperty(m_hSwVenc, &Prop);
   if (Ret != SWVENC_S_SUCCESS)
   {
      DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
        __FUNCTION__, Ret);
      Ret = SWVENC_S_FAILURE;
   }

   RETURN(Ret);
}

SWVENC_STATUS omx_venc::swvenc_set_intra_period
(
    OMX_U32 nPFrame,
    OMX_U32 nBFrame
)
{
   ENTER_FUNC();

   SWVENC_STATUS Ret = SWVENC_S_SUCCESS;
   SWVENC_PROPERTY Prop;

   Prop.id = SWVENC_PROPERTY_ID_INTRA_PERIOD;
   Prop.info.intra_period.pframes = nPFrame;
   Prop.info.intra_period.bframes = nBFrame;

   Ret = swvenc_setproperty(m_hSwVenc, &Prop);
   if (Ret != SWVENC_S_SUCCESS)
   {
      DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
        __FUNCTION__, Ret);
      Ret = SWVENC_S_FAILURE;
   }

   RETURN(Ret);
}

bool omx_venc::swvenc_color_align(OMX_BUFFERHEADERTYPE *buffer, OMX_U32 width,
                        OMX_U32 height)
{
    OMX_U32 y_stride,y_scanlines,uv_scanlines,plane_size_y,plane_size_uv,src_chroma_offset;
    y_stride = SWVENC_Y_STRIDE(COLOR_FMT_NV12,width);
    y_scanlines = SWVENC_Y_SCANLINES(COLOR_FMT_NV12,height);
    src_chroma_offset = width * height;
    OMX_U32 buffersize = SWVENC_BUFFER_SIZE(COLOR_FMT_NV12,width,height);
    if (buffer->nAllocLen >= buffersize) {
        OMX_U8* src_buf = buffer->pBuffer, *dst_buf = buffer->pBuffer;
        //Do chroma first, so that we can convert it in-place
        src_buf += width * height;
        dst_buf += y_stride * y_scanlines;
        for (int line = height / 2 - 1; line >= 0; --line) {
            memmove(dst_buf + line * y_stride,
                    src_buf + line * width,
                    width);
        }

        dst_buf = src_buf = buffer->pBuffer;
        //Copy the Y next
        for (int line = height - 1; line > 0; --line) {
            memmove(dst_buf + line * y_stride,
                    src_buf + line * width,
                    width);
        }
    } else {
        DEBUG_PRINT_ERROR("Failed to align Chroma. from %u to %u : \
                Insufficient bufferLen=%u v/s Required=%u",
                (unsigned int)(width*height), (unsigned int)src_chroma_offset, (unsigned int)buffer->nAllocLen,
                buffersize);
        return false;
    }

    return true;
}

SWVENC_STATUS omx_venc::swvenc_set_color_format
(
   OMX_COLOR_FORMATTYPE color_format
)
{
    ENTER_FUNC();
    SWVENC_STATUS Ret = SWVENC_S_SUCCESS;
    SWVENC_COLOR_FORMAT swvenc_color_format;
    SWVENC_PROPERTY Prop;
    if (color_format == ((OMX_COLOR_FORMATTYPE)QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m))
    {
        DEBUG_PRINT_ERROR("QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m");
        swvenc_color_format = SWVENC_COLOR_FORMAT_NV12;
        Prop.id = SWVENC_PROPERTY_ID_FRAME_ATTRIBUTES;
        Prop.info.frame_attributes.stride_luma = SWVENC_Y_STRIDE(COLOR_FMT_NV12, m_sOutPortDef.format.video.nFrameWidth);
        Prop.info.frame_attributes.stride_chroma = SWVENC_Y_STRIDE(COLOR_FMT_NV12, m_sOutPortDef.format.video.nFrameWidth);
        Prop.info.frame_attributes.offset_luma = 0;
        Prop.info.frame_attributes.offset_chroma = ((SWVENC_Y_STRIDE(COLOR_FMT_NV12, m_sOutPortDef.format.video.nFrameWidth)) * (SWVENC_Y_SCANLINES(COLOR_FMT_NV12, m_sOutPortDef.format.video.nFrameHeight)));
        Ret = swvenc_setproperty(m_hSwVenc, &Prop);
        if (Ret != SWVENC_S_SUCCESS)
        {
            DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
                __FUNCTION__, Ret);
            Ret = SWVENC_S_FAILURE;
        }
    }
    else if(color_format == OMX_COLOR_FormatYUV420SemiPlanar)
    {
        swvenc_color_format = SWVENC_COLOR_FORMAT_NV12;
        Prop.id = SWVENC_PROPERTY_ID_FRAME_ATTRIBUTES;
        Prop.info.frame_attributes.stride_luma = SWVENC_Y_STRIDE(COLOR_FMT_NV12, m_sInPortDef.format.video.nFrameWidth);
        Prop.info.frame_attributes.stride_chroma = SWVENC_Y_STRIDE(COLOR_FMT_NV12, m_sInPortDef.format.video.nFrameWidth);
        Prop.info.frame_attributes.offset_luma = 0;
        Prop.info.frame_attributes.offset_chroma = ((SWVENC_Y_STRIDE(COLOR_FMT_NV12, m_sInPortDef.format.video.nFrameWidth)) * (SWVENC_Y_SCANLINES(COLOR_FMT_NV12, m_sInPortDef.format.video.nFrameHeight)));
        Ret = swvenc_setproperty(m_hSwVenc, &Prop);
        if (Ret != SWVENC_S_SUCCESS)
        {
            DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
                __FUNCTION__, Ret);
            Ret = SWVENC_S_FAILURE;
        }
    }
    else if (color_format == ((OMX_COLOR_FORMATTYPE)QOMX_COLOR_FormatYVU420SemiPlanar))
    {
        swvenc_color_format = SWVENC_COLOR_FORMAT_NV21;
        Prop.id = SWVENC_PROPERTY_ID_FRAME_ATTRIBUTES;
        Prop.info.frame_attributes.stride_luma = ALIGN(m_sInPortDef.format.video.nFrameWidth,16);
        Prop.info.frame_attributes.stride_chroma = ALIGN(m_sInPortDef.format.video.nFrameWidth,16);
        Prop.info.frame_attributes.offset_luma = 0;
        Prop.info.frame_attributes.offset_chroma = ((ALIGN(m_sInPortDef.format.video.nFrameWidth,16)) * (ALIGN(m_sInPortDef.format.video.nFrameHeight,16)));
        Ret = swvenc_setproperty(m_hSwVenc, &Prop);
        if (Ret != SWVENC_S_SUCCESS)
        {
            DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
                __FUNCTION__, Ret);
            Ret = SWVENC_S_FAILURE;
        }
    }
    else if (color_format == ((OMX_COLOR_FORMATTYPE) HAL_PIXEL_FORMAT_NV21_ZSL))
    {
        DEBUG_PRINT_ERROR("HAL_PIXEL_FORMAT_NV21_ZSL");
        swvenc_color_format = SWVENC_COLOR_FORMAT_NV21;
        Prop.id = SWVENC_PROPERTY_ID_FRAME_ATTRIBUTES;
        Prop.info.frame_attributes.stride_luma = SWVENC_Y_STRIDE(COLOR_FMT_NV12_ZSL, m_sInPortDef.format.video.nFrameWidth);
        Prop.info.frame_attributes.stride_chroma = SWVENC_Y_STRIDE(COLOR_FMT_NV12_ZSL, m_sInPortDef.format.video.nFrameWidth);
        Prop.info.frame_attributes.offset_luma = 0;
        Prop.info.frame_attributes.offset_chroma = ((SWVENC_Y_STRIDE(COLOR_FMT_NV12_ZSL, m_sInPortDef.format.video.nFrameWidth)) * (SWVENC_Y_SCANLINES(COLOR_FMT_NV12_ZSL, m_sInPortDef.format.video.nFrameHeight)));
        Ret = swvenc_setproperty(m_hSwVenc, &Prop);
        if (Ret != SWVENC_S_SUCCESS)
        {
            DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
                __FUNCTION__, Ret);
            Ret = SWVENC_S_FAILURE;
        }
    }
    else
    {
        DEBUG_PRINT_ERROR("%s: color_format %d invalid",__FUNCTION__,color_format);
        RETURN(SWVENC_S_FAILURE);
    }
    /* set the input color format */
    Prop.id = SWVENC_PROPERTY_ID_COLOR_FORMAT;
    Prop.info.color_format = swvenc_color_format;
    Ret = swvenc_setproperty(m_hSwVenc, &Prop);
    if (Ret != SWVENC_S_SUCCESS)
    {
        DEBUG_PRINT_ERROR("%s, swvenc_setproperty failed (%d)",
            __FUNCTION__, Ret);
        Ret = SWVENC_S_FAILURE;
    }
    RETURN(Ret);
}

// don't use init_vendor_extensions() from omx_video_extensions.hpp, sw component doesn't support
// all the vendor extensions like hw component
void omx_venc::init_sw_vendor_extensions(VendorExtensionStore &store) {
    ADD_EXTENSION("qti-ext-enc-preprocess-rotate", OMX_IndexConfigCommonRotate, OMX_DirOutput)
    ADD_PARAM_END("angle", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-timestamp-source-avtimer", OMX_QTIIndexParamEnableAVTimerTimestamps,
            OMX_DirOutput)
    ADD_PARAM_END("enable", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-bitrate-mode", OMX_IndexParamVideoBitrate, OMX_DirOutput)
    ADD_PARAM_END("value", OMX_AndroidVendorValueInt32)
}

