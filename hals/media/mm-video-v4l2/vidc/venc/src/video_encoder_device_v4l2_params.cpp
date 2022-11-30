/*--------------------------------------------------------------------------
Copyright (c) 2010-2020, The Linux Foundation. All rights reserved.

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

#include "video_encoder_device_v4l2.h"
#include "omx_video_encoder.h"

#undef LOG_TAG
#define LOG_TAG "OMX-VENC: venc_dev"

void venc_dev::venc_get_consumer_usage(OMX_U32* usage)
{

    OMX_U32 eProfile = 0;
    bool hevc = venc_get_hevc_profile(&eProfile);

    /* Initialize to zero & update as per required color format */
    *usage = 0;

    /* Configure UBWC as default if target supports */
#ifdef _UBWC_
    *usage |= GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
#endif

    if (hevc &&
        (eProfile == (OMX_U32)OMX_VIDEO_HEVCProfileMain10HDR10 ||
         eProfile == (OMX_U32)OMX_VIDEO_HEVCProfileMain10 ||
         eProfile == (OMX_U32)OMX_VIDEO_HEVCProfileMain10HDR10Plus)) {
        DEBUG_PRINT_INFO("Setting 10-bit consumer usage bits");
        *usage |= GRALLOC_USAGE_PRIVATE_10BIT_VIDEO;
        if (mUseLinearColorFormat & REQUEST_LINEAR_COLOR_10_BIT) {
            *usage &= ~GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
            DEBUG_PRINT_INFO("Clear UBWC consumer usage bits as 10-bit linear color requested");
        }
    } else if (mUseLinearColorFormat & REQUEST_LINEAR_COLOR_8_BIT ||
            m_codec == OMX_VIDEO_CodingImageHEIC) {
        *usage &= ~GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
        DEBUG_PRINT_INFO("Clear UBWC consumer usage bits as 8-bit linear color requested");
    }

    if (venc_handle->is_flip_conv_needed(NULL))
        *usage = *usage | GRALLOC_USAGE_SW_READ_OFTEN;

    if (m_codec == OMX_VIDEO_CodingImageHEIC) {
        DEBUG_PRINT_INFO("Clear UBWC and set HEIF consumer usage bit");
        *usage &= ~GRALLOC_USAGE_PRIVATE_ALLOC_UBWC;
        *usage |= GRALLOC_USAGE_PRIVATE_HEIF_VIDEO;
    }

    DEBUG_PRINT_INFO("venc_get_consumer_usage 0x%x", *usage);
}

bool venc_dev::venc_set_config(void *configData, OMX_INDEXTYPE index)
{

    if (is_streamon_done(PORT_INDEX_IN)) {
        if (venc_store_dynamic_config(index, configData)) {
            DEBUG_PRINT_HIGH("dynamic config %#X successfully stored.", index);
            return true;
        }

        /* If failed, try to handle the dynamic config immediately */
        DEBUG_PRINT_ERROR("Store dynamic config %#X failed", index);
    }

    DEBUG_PRINT_LOW("Inside venc_set_config");

    switch ((int)index) {
        case OMX_IndexConfigVideoBitrate:
            {
                OMX_VIDEO_CONFIG_BITRATETYPE *bit_rate = (OMX_VIDEO_CONFIG_BITRATETYPE *)
                    configData;
                DEBUG_PRINT_LOW("venc_set_config: OMX_IndexConfigVideoBitrate");
                if (!venc_config_bitrate(bit_rate))
                    return false;

                break;
            }
        case OMX_IndexConfigVideoFramerate:
            {
                OMX_CONFIG_FRAMERATETYPE *frame_rate = (OMX_CONFIG_FRAMERATETYPE *)
                    configData;
                DEBUG_PRINT_LOW("venc_set_config: OMX_IndexConfigVideoFramerate");
                if (!venc_config_framerate(frame_rate))
                    return false;

                break;
            }
        case QOMX_IndexConfigVideoIntraperiod:
            {
                DEBUG_PRINT_LOW("venc_set_param:QOMX_IndexConfigVideoIntraperiod");
                QOMX_VIDEO_INTRAPERIODTYPE *intraperiod =
                    (QOMX_VIDEO_INTRAPERIODTYPE *)configData;

                if (set_nP_frames(intraperiod->nPFrames) == false ||
                    set_nB_frames(intraperiod->nBFrames) == false)
                    return false;

                break;
            }
        case OMX_IndexConfigVideoIntraVOPRefresh:
            {
                OMX_CONFIG_INTRAREFRESHVOPTYPE *intra_vop_refresh = (OMX_CONFIG_INTRAREFRESHVOPTYPE *)
                    configData;
                DEBUG_PRINT_LOW("venc_set_config: OMX_IndexConfigVideoIntraVOPRefresh");
                if (!venc_config_intravoprefresh(intra_vop_refresh))
                    return false;

                break;
            }
        case OMX_IndexConfigCommonMirror:
            {
                OMX_CONFIG_MIRRORTYPE *mirror = (OMX_CONFIG_MIRRORTYPE*) configData;
                DEBUG_PRINT_LOW("venc_set_param: OMX_IndexConfigCommonMirror");

                if (!venc_handle->m_no_vpss && venc_set_mirror(mirror->eMirror) == false) {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_IndexConfigCommonMirror failed");
                    return false;
                } else if(venc_handle->m_no_vpss) {
                    if ((venc_handle->m_nOperatingRate >> 16) <= 30) {
                        venc_handle->initFastCV();
                    } else {
                        DEBUG_PRINT_ERROR("ERROR: Flip not supported fps %u",
                                venc_handle->m_nOperatingRate >> 16);
                    }
                }

                break;
            }
        case OMX_IndexConfigCommonRotate:
            {
                OMX_CONFIG_ROTATIONTYPE *config_rotation =
                    reinterpret_cast<OMX_CONFIG_ROTATIONTYPE*>(configData);
                OMX_U32 nFrameWidth;
                if (!config_rotation) {
                   return false;
                }

                if (venc_handle->m_no_vpss) {
                    if (venc_prepare_c2d_rotation(config_rotation->nRotation) == false) {
                        DEBUG_PRINT_ERROR("ERROR: venc_prepare_c2d_rotation failed");
                        return false;
                    }
                } else {
                    if (venc_set_vpe_rotation(config_rotation->nRotation) == false) {
                        DEBUG_PRINT_ERROR("ERROR: Dimension Change for Rotation failed");
                        return false;
                    }
                }

                break;
            }
        case OMX_IndexConfigVideoAVCIntraPeriod:
            {
                OMX_VIDEO_CONFIG_AVCINTRAPERIOD *avc_iperiod = (OMX_VIDEO_CONFIG_AVCINTRAPERIOD*) configData;
                DEBUG_PRINT_LOW("venc_set_param: OMX_IndexConfigVideoAVCIntraPeriod");

                if (set_nP_frames(avc_iperiod->nPFrames) == false) {
                    DEBUG_PRINT_ERROR("ERROR: Setting intra period failed");
                    return false;
                }
                break;
            }
        case OMX_IndexConfigVideoVp8ReferenceFrame:
            {
                OMX_VIDEO_VP8REFERENCEFRAMETYPE* vp8refframe = (OMX_VIDEO_VP8REFERENCEFRAMETYPE*) configData;
                DEBUG_PRINT_LOW("venc_set_config: OMX_IndexConfigVideoVp8ReferenceFrame");
                if (!venc_config_vp8refframe(vp8refframe))
                    return false;

                break;
            }
        case OMX_QcomIndexConfigVideoLTRUse:
            {
                OMX_QCOM_VIDEO_CONFIG_LTRUSE_TYPE* pParam = (OMX_QCOM_VIDEO_CONFIG_LTRUSE_TYPE*)configData;
                DEBUG_PRINT_LOW("venc_set_config: OMX_QcomIndexConfigVideoLTRUse");
                if (!venc_config_useLTR(pParam))
                    return false;

                break;
            }
        case OMX_IndexParamVideoAndroidVp8Encoder:
           {
               DEBUG_PRINT_LOW("venc_set_param: OMX_IndexParamVideoAndroidVp8Encoder");
               OMX_VIDEO_PARAM_ANDROID_VP8ENCODERTYPE *vp8EncodeParams =
                   (OMX_VIDEO_PARAM_ANDROID_VP8ENCODERTYPE *)configData;

               if (vp8EncodeParams->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                   int pFrames = vp8EncodeParams->nKeyFrameInterval - 1;
                   if (set_nP_frames(pFrames) == false) {
                       DEBUG_PRINT_ERROR("ERROR: Request for setting intra period failed");
                       return false;
                   }

               } else {
                   DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexParamVideoAndroidVp8Encoder");
               }
               break;
           }
        case OMX_QcomIndexConfigVideoLTRMark:
            {
                OMX_QCOM_VIDEO_CONFIG_LTRMARK_TYPE* pParam = (OMX_QCOM_VIDEO_CONFIG_LTRMARK_TYPE*)configData;
                DEBUG_PRINT_LOW("venc_set_config: OMX_QcomIndexConfigVideoLTRMark");
                if (!venc_config_markLTR(pParam))
                    return false;

                break;
            }
        case OMX_IndexConfigAndroidVideoTemporalLayering:
            {
                OMX_VIDEO_CONFIG_ANDROID_TEMPORALLAYERINGTYPE *pParam =
                    (OMX_VIDEO_CONFIG_ANDROID_TEMPORALLAYERINGTYPE *) configData;

                // Update temporal_layers_config with input config
                if (pParam->nPLayerCountActual < OMX_VIDEO_ANDROID_MAXTEMPORALLAYERS) {
                    temporal_layers_config.nPLayers = pParam->nPLayerCountActual;
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_IndexConfigAndroidVideoTemporalLayering failed");
                    return false;
                }
                temporal_layers_config.ePattern = pParam->ePattern;
                temporal_layers_config.hier_mode = HIER_P;
                temporal_layers_config.nBLayers = 0;
                // Resetting to zero as we are sending all bitrate ratios to kernel
                memset(&temporal_layers_config.nTemporalLayerBitrateRatio, 0x0, sizeof(OMX_U32)*OMX_VIDEO_ANDROID_MAXTEMPORALLAYERS);
                for (OMX_U32 i = 0; i < temporal_layers_config.nPLayers; ++i) {
                    temporal_layers_config.nTemporalLayerBitrateRatio[i] = pParam->nBitrateRatios[i];
                }

                if (OMX_ErrorNone != venc_set_hierp_layer()) {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_IndexConfigAndroidVideoTemporalLayering failed in setting hierp layers");
                    return false;
                }
                if (OMX_ErrorNone != venc_set_bitrate_ratios()) {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_IndexConfigAndroidVideoTemporalLayering failed in setting bitrate ratios");
                    return false;
                }
                break;
            }
        case OMX_QcomIndexConfigBaseLayerId:
            {
                OMX_SKYPE_VIDEO_CONFIG_BASELAYERPID* pParam =
                    (OMX_SKYPE_VIDEO_CONFIG_BASELAYERPID*) configData;
                if (venc_set_baselayerid(pParam->nPID) == false) {
                    DEBUG_PRINT_ERROR("Failed to set OMX_QcomIndexConfigBaseLayerId failed");
                    return false;
                }
                break;
            }
        case OMX_QcomIndexConfigQp:
            {
                OMX_QCOM_VIDEO_CONFIG_QP* pParam =
                    (OMX_QCOM_VIDEO_CONFIG_QP*) configData;
                DEBUG_PRINT_LOW("Set_config: nQP %d", pParam->nQP);
                if (!venc_config_qp(pParam))
                    return false;

                break;
            }
        case OMX_IndexConfigPriority:
            {
                OMX_PARAM_U32TYPE *priority = (OMX_PARAM_U32TYPE *)configData;
                DEBUG_PRINT_LOW("Set_config: priority %d",priority->nU32);
                if (!venc_set_priority(priority->nU32)) {
                    DEBUG_PRINT_ERROR("Failed to set priority");
                    return false;
                }
                sess_priority.priority = priority->nU32;
                break;
            }
        case OMX_IndexConfigOperatingRate:
            {
                OMX_PARAM_U32TYPE *rate = (OMX_PARAM_U32TYPE *)configData;
                DEBUG_PRINT_LOW("Set_config: operating rate %u", rate->nU32);
                if (!venc_set_operatingrate(rate->nU32)) {
                    DEBUG_PRINT_ERROR("Failed to set operating rate");
                    return false;
                }
                break;
            }
        case OMX_IndexConfigAndroidIntraRefresh:
            {
                OMX_VIDEO_CONFIG_ANDROID_INTRAREFRESHTYPE *intra_refresh_period = (OMX_VIDEO_CONFIG_ANDROID_INTRAREFRESHTYPE *)configData;
                DEBUG_PRINT_LOW("OMX_IndexConfigAndroidIntraRefresh : num frames = %d", intra_refresh_period->nRefreshPeriod);

                if (intra_refresh_period->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    intra_refresh.framecount = intra_refresh_period->nRefreshPeriod;
                    intra_refresh.irmode     = OMX_VIDEO_IntraRefreshRandom;
                    intra_refresh.mbcount    = 0;
                    venc_set_intra_refresh();
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexConfigVideoIntraRefreshType");
                }
                break;
            }
        case OMX_QTIIndexConfigVideoBlurResolution:
        {
             OMX_QTI_VIDEO_CONFIG_BLURINFO *blur = (OMX_QTI_VIDEO_CONFIG_BLURINFO *)configData;
             if (blur->nPortIndex == (OMX_U32)PORT_INDEX_IN) {
                 DEBUG_PRINT_LOW("Set_config: blur resolution: %u", blur->nBlurInfo);
                 if(!venc_set_blur_resolution(blur)) {
                    DEBUG_PRINT_ERROR("Failed to set Blur Resolution");
                    return false;
                 }
             } else {
                  DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_QTIIndexConfigVideoBlurResolution");
                  return false;
             }
             break;
        }
        case OMX_QTIIndexConfigDescribeColorAspects:
            {
                DescribeColorAspectsParams *params = (DescribeColorAspectsParams *)configData;

                OMX_U32 color_space = MSM_VIDC_BT601_6_625;
                OMX_U32 full_range = 0;
                OMX_U32 matrix_coeffs = MSM_VIDC_MATRIX_601_6_625;
                OMX_U32 transfer_chars = MSM_VIDC_TRANSFER_601_6_625;

                switch((ColorAspects::Primaries)(params->sAspects.mPrimaries)) {
                    case ColorAspects::PrimariesBT709_5:
                        color_space = MSM_VIDC_BT709_5;
                        break;
                    case ColorAspects::PrimariesBT470_6M:
                        color_space = MSM_VIDC_BT470_6_M;
                        break;
                    case ColorAspects::PrimariesBT601_6_625:
                        color_space = MSM_VIDC_BT601_6_625;
                        break;
                    case ColorAspects::PrimariesBT601_6_525:
                        color_space = MSM_VIDC_BT601_6_525;
                        break;
                    case ColorAspects::PrimariesGenericFilm:
                        color_space = MSM_VIDC_GENERIC_FILM;
                        break;
                    case ColorAspects::PrimariesBT2020:
                        color_space = MSM_VIDC_BT2020;
                        break;
                    default:
                        color_space = MSM_VIDC_BT601_6_625;
                        //params->sAspects.mPrimaries = ColorAspects::PrimariesBT601_6_625;
                        break;
                }
                switch((ColorAspects::Range)params->sAspects.mRange) {
                    case ColorAspects::RangeFull:
                        full_range = 1;
                        break;
                    case ColorAspects::RangeLimited:
                        full_range = 0;
                        break;
                    default:
                        break;
                }
                switch((ColorAspects::Transfer)params->sAspects.mTransfer) {
                    case ColorAspects::TransferSMPTE170M:
                        transfer_chars = MSM_VIDC_TRANSFER_601_6_525;
                        break;
                    case ColorAspects::TransferUnspecified:
                        transfer_chars = MSM_VIDC_TRANSFER_UNSPECIFIED;
                        break;
                    case ColorAspects::TransferGamma22:
                        transfer_chars = MSM_VIDC_TRANSFER_BT_470_6_M;
                        break;
                    case ColorAspects::TransferGamma28:
                        transfer_chars = MSM_VIDC_TRANSFER_BT_470_6_BG;
                        break;
                    case ColorAspects::TransferSMPTE240M:
                        transfer_chars = MSM_VIDC_TRANSFER_SMPTE_240M;
                        break;
                    case ColorAspects::TransferLinear:
                        transfer_chars = MSM_VIDC_TRANSFER_LINEAR;
                        break;
                    case ColorAspects::TransferXvYCC:
                        transfer_chars = MSM_VIDC_TRANSFER_IEC_61966;
                        break;
                    case ColorAspects::TransferBT1361:
                        transfer_chars = MSM_VIDC_TRANSFER_BT_1361;
                        break;
                    case ColorAspects::TransferSRGB:
                        transfer_chars = MSM_VIDC_TRANSFER_SRGB;
                        break;
                    case ColorAspects::TransferST2084:
                        transfer_chars = MSM_VIDC_TRANSFER_SMPTE_ST2084;
                        break;
                    case ColorAspects::TransferHLG:
                        transfer_chars = MSM_VIDC_TRANSFER_HLG;
                        break;
                    default:
                        //params->sAspects.mTransfer = ColorAspects::TransferSMPTE170M;
                        transfer_chars = MSM_VIDC_TRANSFER_601_6_625;
                        break;
                }
                switch((ColorAspects::MatrixCoeffs)params->sAspects.mMatrixCoeffs) {
                    case ColorAspects::MatrixUnspecified:
                        matrix_coeffs = MSM_VIDC_MATRIX_UNSPECIFIED;
                        break;
                    case ColorAspects::MatrixBT709_5:
                        matrix_coeffs = MSM_VIDC_MATRIX_BT_709_5;
                        break;
                    case ColorAspects::MatrixBT470_6M:
                        matrix_coeffs = MSM_VIDC_MATRIX_FCC_47;
                        break;
                    case ColorAspects::MatrixBT601_6:
                        matrix_coeffs = MSM_VIDC_MATRIX_601_6_525;
                        break;
                    case ColorAspects::MatrixSMPTE240M:
                        matrix_coeffs = MSM_VIDC_MATRIX_SMPTE_240M;
                        break;
                    case ColorAspects::MatrixBT2020:
                        matrix_coeffs = MSM_VIDC_MATRIX_BT_2020;
                        break;
                    case ColorAspects::MatrixBT2020Constant:
                        matrix_coeffs = MSM_VIDC_MATRIX_BT_2020_CONST;
                        break;
                    default:
                        //params->sAspects.mMatrixCoeffs = ColorAspects::MatrixBT601_6;
                        matrix_coeffs = MSM_VIDC_MATRIX_601_6_625;
                        break;
                }
                if (!venc_set_colorspace(color_space, full_range,
                            transfer_chars, matrix_coeffs)) {

                    DEBUG_PRINT_ERROR("Failed to set operating rate");
                    return false;
                }
                break;
            }
        case OMX_QTIIndexConfigVideoRoiInfo:
        {
            if(!venc_set_roi_qp_info((OMX_QTI_VIDEO_CONFIG_ROIINFO *)configData)) {
                DEBUG_PRINT_ERROR("Failed to set ROI QP info");
                return false;
            }
            break;
        }
        case OMX_IndexConfigVideoNalSize:
        {
            if(!venc_set_nal_size((OMX_VIDEO_CONFIG_NALSIZE *)configData)) {
                DEBUG_PRINT_LOW("Failed to set Nal size info");
                return false;
            }
            break;
        }
        case OMX_QTIIndexConfigVideoRoiRectRegionInfo:
        {
            if(!venc_set_roi_region_qp_info((OMX_QTI_VIDEO_CONFIG_ROI_RECT_REGION_INFO *)configData)) {
                DEBUG_PRINT_LOW("Failed to set ROI Region QP info");
                return false;
            }
            break;
        }
        case OMX_QTIIndexConfigContentAdaptiveCoding:
           {
                if(!venc_set_bitrate_savings_mode(*(OMX_U32*) configData)) {
                    DEBUG_PRINT_LOW("Failed to set Bitrate Savings Mode");
                    return false;
                }
                break;
           }
        default:
            DEBUG_PRINT_ERROR("Unsupported config index = %u", index);
            break;
    }

    return true;
}

bool venc_dev::venc_store_dynamic_config(OMX_INDEXTYPE config_type, OMX_PTR config)
{
    struct dynamicConfig newConfig;
    memset(&newConfig, 0, sizeof(dynamicConfig));
    newConfig.type = config_type;

   switch ((int)config_type) {
        case OMX_IndexConfigVideoFramerate:
            memcpy(&newConfig.config_data.framerate, config, sizeof(OMX_CONFIG_FRAMERATETYPE));
            break;
        case OMX_IndexConfigVideoIntraVOPRefresh:
            memcpy(&newConfig.config_data.intravoprefresh, config, sizeof(OMX_CONFIG_INTRAREFRESHVOPTYPE));
            break;
        case OMX_IndexConfigVideoBitrate:
            memcpy(&newConfig.config_data.bitrate, config, sizeof(OMX_VIDEO_CONFIG_BITRATETYPE));
            break;
        case OMX_QcomIndexConfigVideoLTRUse:
            memcpy(&newConfig.config_data.useltr, config, sizeof(OMX_QCOM_VIDEO_CONFIG_LTRUSE_TYPE));
            break;
        case OMX_QcomIndexConfigVideoLTRMark:
            memcpy(&newConfig.config_data.markltr, config, sizeof(OMX_QCOM_VIDEO_CONFIG_LTRMARK_TYPE));
            break;
        case OMX_QcomIndexConfigQp:
            memcpy(&newConfig.config_data.configqp, config, sizeof(OMX_QCOM_VIDEO_CONFIG_QP));
            break;
        case QOMX_IndexConfigVideoIntraperiod:
            memcpy(&newConfig.config_data.intraperiod, config, sizeof(QOMX_VIDEO_INTRAPERIODTYPE));
            break;
        case OMX_IndexConfigVideoVp8ReferenceFrame:
            memcpy(&newConfig.config_data.vp8refframe, config, sizeof(OMX_VIDEO_VP8REFERENCEFRAMETYPE));
            break;
        case OMX_IndexConfigCommonMirror:
            memcpy(&newConfig.config_data.mirror, config, sizeof(OMX_CONFIG_MIRRORTYPE));
            break;
        default:
            DEBUG_PRINT_INFO("Unsupported dynamic config.");
            return false;
    }

    if(venc_handle->m_etb_count)
        newConfig.timestamp = venc_handle->m_etb_timestamp + 1;
    else
        newConfig.timestamp = 0;

    pthread_mutex_lock(&m_configlock);
    DEBUG_PRINT_LOW("list add dynamic config with type %d timestamp %lld us", config_type, newConfig.timestamp);
    m_configlist.push_back(newConfig);
    pthread_mutex_unlock(&m_configlock);
    return true;

}

bool venc_dev::venc_set_param(void *paramData, OMX_INDEXTYPE index)
{
    DEBUG_PRINT_LOW("venc_set_param index 0x%x", index);
    struct v4l2_format fmt;
    struct v4l2_requestbuffers bufreq;
    int ret;
    bool isCBR;

    switch ((int)index) {
        case OMX_IndexParamPortDefinition:
            {
                OMX_PARAM_PORTDEFINITIONTYPE *portDefn;
                portDefn = (OMX_PARAM_PORTDEFINITIONTYPE *) paramData;

                if (portDefn->nPortIndex == PORT_INDEX_IN) {
                    if (!venc_set_encode_framerate(portDefn->format.video.xFramerate)) {
                        return false;
                    }

                    unsigned long inputformat = venc_get_color_format(portDefn->format.video.eColorFormat);

                    unsigned long width  = m_bDimensionsNeedFlip ? portDefn->format.video.nFrameHeight :
                                                                   portDefn->format.video.nFrameWidth;
                    unsigned long height = m_bDimensionsNeedFlip ? portDefn->format.video.nFrameWidth :
                                                                   portDefn->format.video.nFrameHeight;

                    if (m_sVenc_cfg.input_height != height || m_sVenc_cfg.input_width != width ||
                        m_sInput_buff_property.actualcount != portDefn->nBufferCountActual ||
                        m_sVenc_cfg.inputformat != inputformat) {

                        DEBUG_PRINT_LOW("venc_set_param: OMX_IndexParamPortDefinition: port: %u, WxH %lux%lu --> %ux%u, count %lu --> %u, format %#lx --> %#lx",
                            portDefn->nPortIndex, m_sVenc_cfg.input_width, m_sVenc_cfg.input_height,
                            portDefn->format.video.nFrameWidth, portDefn->format.video.nFrameHeight,
                            m_sInput_buff_property.actualcount, portDefn->nBufferCountActual,
                            m_sVenc_cfg.inputformat, inputformat);

                        if (portDefn->nBufferCountActual < m_sInput_buff_property.mincount) {
                            DEBUG_PRINT_LOW("Actual count %u is less than driver mincount %lu on port %u",
                                portDefn->nBufferCountActual, m_sInput_buff_property.mincount, portDefn->nPortIndex);
                            return false;
                        }

                        m_sVenc_cfg.input_height = height;
                        m_sVenc_cfg.input_width = width;
                        m_sVenc_cfg.inputformat = inputformat;
                        m_sInput_buff_property.actualcount = portDefn->nBufferCountActual;

                        memset(&fmt, 0, sizeof(fmt));
                        fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
                        fmt.fmt.pix_mp.height = m_sVenc_cfg.input_height;
                        fmt.fmt.pix_mp.width = m_sVenc_cfg.input_width;
                        fmt.fmt.pix_mp.pixelformat = m_sVenc_cfg.inputformat;
                        fmt.fmt.pix_mp.colorspace = V4L2_COLORSPACE_470_SYSTEM_BG;
                        if (ioctl(m_nDriver_fd, VIDIOC_S_FMT, &fmt)) {
                            DEBUG_PRINT_ERROR("set format failed, type %d, wxh %dx%d, pixelformat %#x, colorspace %#x",
                                 fmt.type, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
                                 fmt.fmt.pix_mp.pixelformat, fmt.fmt.pix_mp.colorspace);
                            hw_overload = errno == EBUSY;
                            return false;
                        }
                        m_sInput_buff_property.datasize=fmt.fmt.pix_mp.plane_fmt[0].sizeimage;

                        bufreq.memory = V4L2_MEMORY_USERPTR;
                        bufreq.count = portDefn->nBufferCountActual;
                        bufreq.type=V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
                        if (ioctl(m_nDriver_fd,VIDIOC_REQBUFS, &bufreq)) {
                            DEBUG_PRINT_ERROR("reqbufs failed, type %d, count %d", bufreq.type, bufreq.count);
                            return false;
                        }

                        if (num_input_planes > 1) {
                            input_extradata_info.count = m_sInput_buff_property.actualcount;
                            venc_handle->m_client_in_extradata_info.set_extradata_info(input_extradata_info.buffer_size, input_extradata_info.count);
                        }

                        if (!downscalar_enabled) {
                            m_sVenc_cfg.dvs_height = height;
                            m_sVenc_cfg.dvs_width = width;
                        }
                        memset(&fmt, 0, sizeof(fmt));
                        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                        fmt.fmt.pix_mp.height = m_sVenc_cfg.dvs_height;
                        fmt.fmt.pix_mp.width = m_sVenc_cfg.dvs_width;
                        fmt.fmt.pix_mp.pixelformat = m_sVenc_cfg.codectype;

                        if (ioctl(m_nDriver_fd, VIDIOC_S_FMT, &fmt)) {
                            DEBUG_PRINT_ERROR("VIDIOC_S_FMT CAPTURE_MPLANE Failed");
                            hw_overload = errno == EBUSY;
                            return false;
                        }
                        m_sOutput_buff_property.datasize = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;

                    } else {
                        DEBUG_PRINT_LOW("venc_set_param: OMX_IndexParamPortDefinition: parameters not changed on port %d",
                            portDefn->nPortIndex);
                    }
                } else if (portDefn->nPortIndex == PORT_INDEX_OUT) {

                    unsigned long codectype = venc_get_codectype(portDefn->format.video.eCompressionFormat);

                    unsigned long width  = m_bDimensionsNeedFlip ? portDefn->format.video.nFrameHeight :
                                                                   portDefn->format.video.nFrameWidth;
                    unsigned long height = m_bDimensionsNeedFlip ? portDefn->format.video.nFrameWidth :
                                                                   portDefn->format.video.nFrameHeight;

                    //Don't worry about width/height if downscalar is enabled.
                    if (m_sVenc_cfg.dvs_height != height || m_sVenc_cfg.dvs_width != width ||
                        m_sOutput_buff_property.actualcount != portDefn->nBufferCountActual ||
                        m_sVenc_cfg.codectype != codectype) {

                        if (portDefn->nBufferCountActual < m_sOutput_buff_property.mincount) {
                            DEBUG_PRINT_LOW("Actual count %u is less than driver mincount %lu on port %u",
                                portDefn->nBufferCountActual, m_sOutput_buff_property.mincount, portDefn->nPortIndex);
                            return false;
                        }

                        //If downscalar is enabled. Correct width/height is populated no need to replace with port def width/height
                        if (!downscalar_enabled) {
                            DEBUG_PRINT_LOW("venc_set_param: OMX_IndexParamPortDefinition: port: %u, WxH %lux%lu --> %ux%u, count %lu --> %u, format %#lx --> %#lx",
                                            portDefn->nPortIndex, m_sVenc_cfg.dvs_width, m_sVenc_cfg.dvs_height,
                                            portDefn->format.video.nFrameWidth, portDefn->format.video.nFrameHeight,
                                            m_sOutput_buff_property.actualcount, portDefn->nBufferCountActual,
                                            m_sVenc_cfg.codectype, codectype);
                            m_sVenc_cfg.dvs_height = height;
                            m_sVenc_cfg.dvs_width = width;
                        }

                        m_sVenc_cfg.codectype = codectype;
                        m_sOutput_buff_property.actualcount = portDefn->nBufferCountActual;

                        memset(&fmt, 0, sizeof(fmt));
                        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                        fmt.fmt.pix_mp.height = m_sVenc_cfg.dvs_height;
                        fmt.fmt.pix_mp.width = m_sVenc_cfg.dvs_width;
                        fmt.fmt.pix_mp.pixelformat = m_sVenc_cfg.codectype;
                        if (ioctl(m_nDriver_fd, VIDIOC_S_FMT, &fmt)) {
                            DEBUG_PRINT_ERROR("set format failed, type %d, wxh %dx%d, pixelformat %#x",
                                 fmt.type, fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
                                 fmt.fmt.pix_mp.pixelformat);
                            hw_overload = errno == EBUSY;
                            return false;
                        }
                        m_sOutput_buff_property.datasize = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;

                        if (!venc_set_target_bitrate(portDefn->format.video.nBitrate)) {
                            return false;
                        }

                        bufreq.memory = V4L2_MEMORY_USERPTR;
                        bufreq.count = portDefn->nBufferCountActual;
                        bufreq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                        if (ioctl(m_nDriver_fd,VIDIOC_REQBUFS, &bufreq)) {
                            DEBUG_PRINT_ERROR("reqbufs failed, type %d, count %d", bufreq.type, bufreq.count);
                            return false;
                        }

                        if (num_output_planes > 1) {
                            output_extradata_info.count = m_sOutput_buff_property.actualcount;
                            venc_handle->m_client_out_extradata_info.set_extradata_info(output_extradata_info.buffer_size, output_extradata_info.count);
                        }
                    } else {
                        DEBUG_PRINT_LOW("venc_set_param: OMX_IndexParamPortDefinition: parameters not changed on port %d",
                            portDefn->nPortIndex);
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index (%d) for OMX_IndexParamPortDefinition", portDefn->nPortIndex);
                }
            }
            break;
        case OMX_IndexParamVideoPortFormat:
            {
                OMX_VIDEO_PARAM_PORTFORMATTYPE *portFmt;
                portFmt =(OMX_VIDEO_PARAM_PORTFORMATTYPE *)paramData;
                DEBUG_PRINT_LOW("venc_set_param: OMX_IndexParamVideoPortFormat");

                if (portFmt->nPortIndex == (OMX_U32) PORT_INDEX_IN) {
                    if (!venc_set_color_format(portFmt->eColorFormat)) {
                        return false;
                    }
                } else if (portFmt->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    if (!venc_set_encode_framerate(portFmt->xFramerate)) {
                        return false;
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexParamVideoPortFormat");
                }
                    break;
            }
        case OMX_IndexParamVideoBitrate:
            {
                OMX_VIDEO_PARAM_BITRATETYPE* pParam;
                pParam = (OMX_VIDEO_PARAM_BITRATETYPE*)paramData;
                DEBUG_PRINT_LOW("venc_set_param: OMX_IndexParamVideoBitrate");

                if (pParam->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    if (!venc_set_ratectrl_cfg(pParam->eControlRate)) {
                        DEBUG_PRINT_ERROR("ERROR: Rate Control setting failed");
                        return false;
                    }

                    if (!venc_set_target_bitrate(pParam->nTargetBitrate)) {
                        DEBUG_PRINT_ERROR("ERROR: Target Bit Rate setting failed");
                        return false;
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexParamVideoBitrate");
                }

                break;
            }
        case OMX_IndexParamVideoAvc:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_IndexParamVideoAvc");
                OMX_VIDEO_PARAM_AVCTYPE* pParam = (OMX_VIDEO_PARAM_AVCTYPE*)paramData;
                OMX_U32 bFrames = 0;

                if (pParam->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    DEBUG_PRINT_LOW("pParam->eProfile :%d ,pParam->eLevel %d",
                            pParam->eProfile,pParam->eLevel);

                    if (!venc_set_profile (pParam->eProfile)) {
                        DEBUG_PRINT_ERROR("ERROR: Unsuccessful in updating Profile %d",
                                pParam->eProfile);
                        return false;
                    }
                    if(!venc_set_level(pParam->eLevel)) {
                        DEBUG_PRINT_ERROR("ERROR: Unsuccessful in updating level");
                        return false;
                    }
                    if (set_nP_frames(pParam->nPFrames) == false ||
                        (pParam->nBFrames && set_nB_frames(pParam->nBFrames) == false)) {
                        DEBUG_PRINT_ERROR("ERROR: Request for setting intra period failed");
                        return false;
                    }
                    if (!venc_set_entropy_config (pParam->bEntropyCodingCABAC, pParam->nCabacInitIdc)) {
                        DEBUG_PRINT_ERROR("ERROR: Request for setting Entropy failed");
                        return false;
                    }
                    if (!venc_set_inloop_filter (pParam->eLoopFilterMode)) {
                        DEBUG_PRINT_ERROR("ERROR: Request for setting Inloop filter failed");
                        return false;
                    }

                    if (!venc_set_multislice_cfg(V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_MB, pParam->nSliceHeaderSpacing)) {
                        DEBUG_PRINT_ERROR("WARNING: Unsuccessful in updating slice_config");
                        return false;
                    }
                    if (!venc_h264_transform_8x8(pParam->bDirect8x8Inference)) {
                       DEBUG_PRINT_ERROR("WARNING: Request for setting Transform8x8 failed");
                       return false;
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexParamVideoAvc");
                }

                //TBD, lot of other variables to be updated, yet to decide
                break;
            }
        case (OMX_INDEXTYPE)OMX_IndexParamVideoVp8:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_IndexParamVideoVp8");
                OMX_VIDEO_PARAM_VP8TYPE* pParam = (OMX_VIDEO_PARAM_VP8TYPE*)paramData;

                //TODO: Set VP8 level/profile currently based on driver change
                if (!venc_set_profile (pParam->eProfile)) {
                    DEBUG_PRINT_ERROR("ERROR: Unsuccessful in updating Profile %d",
                            pParam->eProfile);
                    return false;
                }
                if(venc_set_vpx_error_resilience(pParam->bErrorResilientMode) == false) {
                    DEBUG_PRINT_ERROR("ERROR: Failed to set vpx error resilience");
                    return false;
                }
                break;
            }
            case (OMX_INDEXTYPE)OMX_IndexParamVideoHevc:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_IndexParamVideoHevc");
                OMX_VIDEO_PARAM_HEVCTYPE* pParam = (OMX_VIDEO_PARAM_HEVCTYPE*)paramData;

                if (!venc_set_profile (pParam->eProfile)) {
                    DEBUG_PRINT_ERROR("ERROR: Unsuccessful in updating Profile %d",
                                        pParam->eProfile);
                    return false;
                }
                if (!venc_set_inloop_filter(OMX_VIDEO_AVCLoopFilterEnable))
                    DEBUG_PRINT_HIGH("WARN: Request for setting Inloop filter failed for HEVC encoder");

                OMX_U32 fps = m_sVenc_cfg.fps_den ? m_sVenc_cfg.fps_num / m_sVenc_cfg.fps_den : 30;
                OMX_U32 nPFrames = pParam->nKeyFrameInterval > 0 ? pParam->nKeyFrameInterval - 1 : fps - 1;
                if (set_nP_frames(nPFrames) == false) {
                    DEBUG_PRINT_ERROR("ERROR: Request for setting intra period failed");
                    return false;
                }
                break;
            }
        case (OMX_INDEXTYPE)OMX_IndexParamVideoAndroidImageGrid:
            {
                DEBUG_PRINT_LOW("venc_set_param: OMX_IndexParamVideoAndroidImageGrid. Ignore!");
                break;
            }
        case OMX_IndexParamVideoIntraRefresh:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_IndexParamVideoIntraRefresh");
                OMX_VIDEO_PARAM_INTRAREFRESHTYPE *intra_refresh_param =
                    (OMX_VIDEO_PARAM_INTRAREFRESHTYPE *)paramData;

                if (intra_refresh_param->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    intra_refresh.irmode     = OMX_VIDEO_IntraRefreshCyclic;
                    intra_refresh.mbcount    = intra_refresh_param->nCirMBs;
                    intra_refresh.framecount = 0;
                    venc_set_intra_refresh();
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexParamVideoIntraRefresh");
                }

                break;
            }
        case OMX_IndexParamVideoErrorCorrection:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_IndexParamVideoErrorCorrection");
                OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE *error_resilience =
                    (OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE *)paramData;

                if (error_resilience->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    if (venc_set_error_resilience(error_resilience) == false) {
                        DEBUG_PRINT_ERROR("ERROR: Setting Error Correction failed");
                        return false;
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexParamVideoErrorCorrection");
                }

                break;
            }
         case OMX_QcomIndexParamVideoSliceSpacing:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_QcomIndexParamVideoSliceSpacing");
                QOMX_VIDEO_PARAM_SLICE_SPACING_TYPE *slice_spacing =
                    (QOMX_VIDEO_PARAM_SLICE_SPACING_TYPE*)paramData;

                if (slice_spacing->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    if (!venc_set_multislice_cfg(slice_spacing->eSliceMode, slice_spacing->nSliceSize)) {
                        DEBUG_PRINT_ERROR("ERROR: Setting Slice Spacing failed");
                        return false;
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_QcomIndexParamVideoSliceSpacing");
                }

                break;
            }
        case OMX_IndexParamVideoProfileLevelCurrent:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_IndexParamVideoProfileLevelCurrent");
                OMX_VIDEO_PARAM_PROFILELEVELTYPE *profile_level =
                    (OMX_VIDEO_PARAM_PROFILELEVELTYPE *)paramData;

                if (profile_level->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    if (!venc_set_profile(profile_level->eProfile)) {
                        DEBUG_PRINT_ERROR("WARNING: Unsuccessful in updating Profile");
                        return false;
                    }
                    if (!venc_set_level(profile_level->eLevel)) {
                        DEBUG_PRINT_ERROR("WARNING: Unsuccessful in updating level");
                        return false;
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexParamVideoProfileLevelCurrent");
                }

                break;
            }
        case OMX_IndexParamVideoQuantization:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_IndexParamVideoQuantization");
                OMX_VIDEO_PARAM_QUANTIZATIONTYPE *session_qp =
                    (OMX_VIDEO_PARAM_QUANTIZATIONTYPE *)paramData;
                if (session_qp->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    if (venc_set_qp(session_qp->nQpI,
                                session_qp->nQpP,
                                session_qp->nQpB,
                                ENABLE_I_QP | ENABLE_P_QP | ENABLE_B_QP) == false) {
                        DEBUG_PRINT_ERROR("ERROR: Setting Session QP failed");
                        return false;
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for OMX_IndexParamVideoQuantization");
                }

                break;
            }
        case QOMX_IndexParamVideoInitialQp:
            {
                DEBUG_PRINT_LOW("venc_set_param:QOMX_IndexParamVideoInitialQp");
                QOMX_EXTNINDEX_VIDEO_INITIALQP *initial_qp =
                    (QOMX_EXTNINDEX_VIDEO_INITIALQP *)paramData;
                if (initial_qp->nPortIndex == (OMX_U32) PORT_INDEX_OUT) {
                    if (venc_set_qp(initial_qp->nQpI,
                                initial_qp->nQpP,
                                initial_qp->nQpB,
                                initial_qp->bEnableInitQp) == false) {
                        DEBUG_PRINT_ERROR("ERROR: Setting Initial QP failed");
                        return false;
                    }
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Invalid Port Index for QOMX_IndexParamVideoInitialQp");
                }

                break;
            }
        case OMX_QcomIndexParamVideoIPBQPRange:
            {
                DEBUG_PRINT_LOW("venc_set_param:OMX_QcomIndexParamVideoIPBQPRange");
                OMX_QCOM_VIDEO_PARAM_IPB_QPRANGETYPE *session_qp_range =
                    (OMX_QCOM_VIDEO_PARAM_IPB_QPRANGETYPE *)paramData;
                if(session_qp_range->nPortIndex == (OMX_U32)PORT_INDEX_OUT) {
                    if ( venc_set_session_qp_range (session_qp_range) == false) {
                        DEBUG_PRINT_ERROR("ERROR: Setting QP range failed");
                        return false;
                    }
                }

                break;
            }
        case OMX_QcomIndexParamIndexExtraDataType:
            {
                DEBUG_PRINT_LOW("venc_set_param: OMX_QcomIndexParamIndexExtraDataType");
                QOMX_INDEXEXTRADATATYPE *pParam = (QOMX_INDEXEXTRADATATYPE *)paramData;

                if (pParam->nIndex == (OMX_INDEXTYPE)OMX_QTI_ExtraDataCategory_Enc_ROI &&
                        m_sVenc_cfg.codectype != V4L2_PIX_FMT_H264 &&
                        m_sVenc_cfg.codectype != V4L2_PIX_FMT_HEVC) {
                    DEBUG_PRINT_ERROR("OMX_QTI_ExtraDataCategory_Enc_ROI is not supported for %lu codec", m_sVenc_cfg.codectype);
                    return false;
                }

                if (venc_set_extradata(pParam->nIndex, pParam->bEnabled) == false) {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_QcomIndexParamIndexExtraDataType failed");
                    return false;
                }

                if (pParam->nIndex == (OMX_INDEXTYPE)OMX_QTI_ExtraDataCategory_Enc_ROI && pParam->bEnabled) {
                    m_roi_enabled = true;
                    struct v4l2_control control;
                    control.id = V4L2_CID_MPEG_VIDC_VIDEO_ROI_TYPE;
                    control.value = ROI_NONE;
                    if (ioctl(m_nDriver_fd, VIDIOC_G_CTRL, &control)) {
                        DEBUG_PRINT_ERROR("ERROR: failed to query supported ROI type");
                        m_roi_type = ROI_NONE;
                    } else {
                        auto type = static_cast<roi_type>(control.value);
                        if (type != ROI_2BIT && type != ROI_2BYTE) {
                            DEBUG_PRINT_LOW("invalid ROI type : %u", m_roi_type);
                            m_roi_type = ROI_NONE;
                        } else {
                            m_roi_type = type;
                            DEBUG_PRINT_LOW("queried ROI type : %u", m_roi_type);
                        }
                    }
                }

                break;
            }
        case OMX_QcomIndexParamSequenceHeaderWithIDR:
            {
                PrependSPSPPSToIDRFramesParams * pParam =
                    (PrependSPSPPSToIDRFramesParams *)paramData;

                DEBUG_PRINT_LOW("set inband sps/pps: %d", pParam->bEnable);
                if(venc_set_inband_video_header(pParam->bEnable) == false) {
                    DEBUG_PRINT_ERROR("ERROR: set inband sps/pps failed");
                    return false;
                }

                break;
            }
        case OMX_QcomIndexParamH264VUITimingInfo:
            {
                OMX_QCOM_VIDEO_PARAM_VUI_TIMING_INFO *pParam =
                        (OMX_QCOM_VIDEO_PARAM_VUI_TIMING_INFO *)paramData;
                DEBUG_PRINT_LOW("Set VUI timing info: %d", pParam->bEnable);
                if(venc_set_vui_timing_info(pParam->bEnable) == false) {
                    DEBUG_PRINT_ERROR("ERROR: Failed to set vui timing info to %d", pParam->bEnable);
                    return false;
                } else {
                    vui_timing_info.enabled = (unsigned int) pParam->bEnable;
                }
                break;
            }
        case OMX_QcomIndexParamVideoLTRCount:
            {
                DEBUG_PRINT_LOW("venc_set_param: OMX_QcomIndexParamVideoLTRCount");
                OMX_QCOM_VIDEO_PARAM_LTRCOUNT_TYPE* pParam =
                        (OMX_QCOM_VIDEO_PARAM_LTRCOUNT_TYPE*)paramData;
                if (venc_set_ltrcount(pParam->nCount) == false) {
                    DEBUG_PRINT_ERROR("ERROR: Enable LTR mode failed");
                    return false;
                }
                break;
            }
        case OMX_QcomIndexParamBatchSize:
            {
                OMX_PARAM_U32TYPE* pParam =
                    (OMX_PARAM_U32TYPE*)paramData;

                if (pParam->nPortIndex == PORT_INDEX_OUT) {
                    DEBUG_PRINT_ERROR("For the moment, client-driven batching not supported"
                            " on output port");
                    return false;
                }
                break;
            }
        case OMX_QcomIndexParamVencAspectRatio:
            {
                if (!venc_set_aspectratio(paramData)) {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_QcomIndexParamVencAspectRatio failed");
                    return false;
                }
                break;
            }
        case OMX_QTIIndexParamLowLatencyMode:
            {
                QOMX_EXTNINDEX_VIDEO_LOW_LATENCY_MODE *pParam =
                    (QOMX_EXTNINDEX_VIDEO_LOW_LATENCY_MODE*)paramData;

                if (!venc_set_lowlatency_mode(pParam->bEnableLowLatencyMode)) {
                     DEBUG_PRINT_ERROR("Setting low latency mode failed");
                     return false;
                }
                break;
            }
        case OMX_IndexParamAndroidVideoTemporalLayering:
            {
                OMX_VIDEO_PARAM_ANDROID_TEMPORALLAYERINGTYPE hierData;
                memcpy(&hierData, paramData, sizeof(hierData));

                // Update temporal_layers_config with input param
                if (hierData.nPLayerCountActual < OMX_VIDEO_ANDROID_MAXTEMPORALLAYERS) {
                    temporal_layers_config.nPLayers = hierData.nPLayerCountActual;
                } else {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_IndexParamAndroidVideoTemporalLayering failed");
                    return false;
                }
                temporal_layers_config.ePattern = hierData.ePattern;
                temporal_layers_config.hier_mode = HIER_P;
                temporal_layers_config.nMaxLayers = hierData.nLayerCountMax;
                temporal_layers_config.nMaxBLayers = hierData.nBLayerCountMax;
                temporal_layers_config.nBLayers = hierData.nBLayerCountActual;
                // Resetting to zero as we are sending all bitrate ratios to kernel
                memset(&temporal_layers_config.nTemporalLayerBitrateRatio, 0x0, sizeof(OMX_U32)*OMX_VIDEO_ANDROID_MAXTEMPORALLAYERS);
                for (OMX_U32 i = 0; i < temporal_layers_config.nPLayers; ++i) {
                    temporal_layers_config.nTemporalLayerBitrateRatio[i] = hierData.nBitrateRatios[i];
                }

                if (OMX_ErrorNone != venc_set_max_hierp_layer()) {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_IndexParamAndroidVideoTemporalLayering failed in setting max hierp layers");
                    return false;
                }
                if (OMX_ErrorNone != venc_set_hierp_layer()) {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_IndexParamAndroidVideoTemporalLayering failed in setting hierp layers");
                    return false;
                }
                if (OMX_ErrorNone != venc_set_bitrate_ratios()) {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_IndexParamAndroidVideoTemporalLayering failed in setting bitrate ratios");
                    return false;
                }
                break;
            }
        case OMX_QTIIndexParamEnableAVTimerTimestamps:
            {
                QOMX_ENABLETYPE *pParam = (QOMX_ENABLETYPE *)paramData;
                mUseAVTimerTimestamps = pParam->bEnable == OMX_TRUE;
                DEBUG_PRINT_INFO("AVTimer timestamps enabled");
                break;
            }
        case OMX_QcomIndexParamVideoDownScalar:
            {
                QOMX_INDEXDOWNSCALAR *pParam = (QOMX_INDEXDOWNSCALAR *)paramData;
                downscalar_enabled = pParam->bEnable;

                DEBUG_PRINT_INFO("Downscalar settings: Enabled : %d Width : %u Height %u",
                    pParam->bEnable, pParam->nOutputWidth, pParam->nOutputHeight);
                if (downscalar_enabled) {
                    m_sVenc_cfg.dvs_width = pParam->nOutputWidth;
                    m_sVenc_cfg.dvs_height = pParam->nOutputHeight;

                    memset(&fmt, 0, sizeof(fmt));
                    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                    fmt.fmt.pix_mp.width = pParam->nOutputWidth;
                    fmt.fmt.pix_mp.height = pParam->nOutputHeight;
                    fmt.fmt.pix_mp.pixelformat = m_sVenc_cfg.codectype;
                    if (ioctl(m_nDriver_fd, VIDIOC_S_FMT, &fmt)) {
                        DEBUG_PRINT_ERROR("Failed to set format on capture port");
                        return false;
                    }
                    m_sOutput_buff_property.datasize = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
                }
                break;
            }
        case OMX_QTIIndexParamColorSpaceConversion:
            {
                QOMX_ENABLETYPE *pParam = (QOMX_ENABLETYPE *)paramData;
                csc_enable = pParam->bEnable;
                DEBUG_PRINT_INFO("CSC settings: Enabled : %d ", pParam->bEnable);
                break;
            }
        case OMX_QTIIndexParamEnableLinearColorFormat:
            {
                QOMX_ENABLETYPE *pParam = (QOMX_ENABLETYPE *)paramData;
                mUseLinearColorFormat = pParam->bEnable ? REQUEST_LINEAR_COLOR_ALL : 0;
                DEBUG_PRINT_INFO("Linear Color Format Enabled : %d ", pParam->bEnable);
                break;
            }
        case OMX_QTIIndexParamNativeRecorder:
            {
                QOMX_ENABLETYPE *pParam = (QOMX_ENABLETYPE *)paramData;
                if (!set_native_recoder(pParam->bEnable)) {
                    DEBUG_PRINT_ERROR("ERROR: Setting OMX_QTIIndexParamNativeRecorder failed");
                    return false;
                }
                DEBUG_PRINT_INFO("Native recorder encode session %d", pParam->bEnable);
                break;
            }
        case OMX_QTIIndexParamVbvDelay:
            {
                OMX_EXTNINDEX_VIDEO_VBV_DELAY *pParam =
                    (OMX_EXTNINDEX_VIDEO_VBV_DELAY*)paramData;
                if (!venc_set_vbv_delay(pParam->nVbvDelay)) {
                     DEBUG_PRINT_ERROR("Setting OMX_QTIIndexParamVbvDelay failed");
                     return false;
                }
                break;
            }
        default:
            DEBUG_PRINT_ERROR("ERROR: Unsupported parameter in venc_set_param: %u",
                    index);
            break;
    }

    return true;
}

bool venc_dev::venc_set_inband_video_header(OMX_BOOL enable)
{
    struct v4l2_control control;

    control.id = V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR;
    control.value = V4L2_MPEG_MSM_VIDC_DISABLE;
    if(enable) {
        control.value = V4L2_MPEG_MSM_VIDC_ENABLE;
    }

    DEBUG_PRINT_HIGH("Set inband sps/pps: %d", enable);
    if(ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control) < 0) {
        DEBUG_PRINT_ERROR("Request for inband sps/pps failed");
        return false;
    }
    return true;
}

bool venc_dev::venc_set_extradata(OMX_U32 extra_data, OMX_BOOL enable)
{
    struct v4l2_control control;

    DEBUG_PRINT_HIGH("venc_set_extradata:: %x", (int) extra_data);

    if (enable == OMX_FALSE) {
        /* No easy way to turn off extradata to the driver
         * at the moment */
        return false;
    }

    control.id = V4L2_CID_MPEG_VIDC_VIDEO_EXTRADATA;
    switch (extra_data) {
        case OMX_QTI_ExtraDataCategory_Advanced:
            control.value = EXTRADATA_ADVANCED;
            break;
        case OMX_QTI_ExtraDataCategory_Enc_ROI:
            control.value = EXTRADATA_ENC_INPUT_ROI;
            break;
        default:
            DEBUG_PRINT_ERROR("Unrecognized extradata index 0x%x", (unsigned int)extra_data);
            return false;
    }

    if (ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control)) {
        DEBUG_PRINT_ERROR("ERROR: Request for setting extradata (%x) failed %d",
                (unsigned int)extra_data, errno);
        return false;
    }

    return true;
}

bool venc_dev::venc_set_session_qp_range(OMX_QCOM_VIDEO_PARAM_IPB_QPRANGETYPE* qp_range)
{
    int rc;
    struct v4l2_control control[2];

    control[0].id = V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP;
    control[0].value = qp_range->minIQP | (qp_range->minPQP << 8) | (qp_range->minBQP << 16);

    control[1].id = V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP;
    control[1].value = qp_range->maxIQP | (qp_range->maxPQP << 8) | (qp_range->maxBQP << 16);

    for(int i=0; i<2; i++) {
        if(ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control[i])) {
            DEBUG_PRINT_ERROR("Failed to set QP %s range", i==0?"MIN":"MAX");
            return false;
        }
    }

    session_ipb_qp_values.min_qp_packed = control[0].value;
    session_ipb_qp_values.max_qp_packed = control[1].value;

    return true;
}

bool venc_dev::venc_set_profile(OMX_U32 eProfile)
{
    int rc;
    struct v4l2_control control;

    DEBUG_PRINT_LOW("venc_set_profile:: eProfile = %u",
            (unsigned int)eProfile);

    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_H264) {
        control.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE;
    } else if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_VP8) {
        control.id = V4L2_CID_MPEG_VIDEO_VP8_PROFILE;
    } else if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC) {
        control.id = V4L2_CID_MPEG_VIDEO_HEVC_PROFILE;
    } else {
        DEBUG_PRINT_ERROR("Wrong CODEC");
        return false;
    }

    if (m_disable_hdr & ENC_HDR_DISABLE_FLAG) {
        if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC) {
            if (eProfile == OMX_VIDEO_HEVCProfileMain10 ||
                eProfile == OMX_VIDEO_HEVCProfileMain10HDR10 ||
                eProfile == OMX_VIDEO_HEVCProfileMain10HDR10Plus) {
                DEBUG_PRINT_ERROR("%s: HDR profile unsupported", __FUNCTION__);
                return false;
            }
        }
    }

    if (!profile_level_converter::convert_omx_profile_to_v4l2(m_sVenc_cfg.codectype, eProfile, &control.value)) {
        DEBUG_PRINT_ERROR("Cannot find v4l2 profile for OMX profile : %d Codec : %lu ",
                          eProfile, m_sVenc_cfg.codectype);
        return false;
    }

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }
    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    codec_profile.profile = control.value;

    if (venc_set_extradata_hdr10metadata(eProfile) == false) {
        DEBUG_PRINT_ERROR("Failed to set extradata HDR10PLUS_METADATA");
        return false;
    }

    return true;
}

bool venc_dev::venc_set_level(OMX_U32 eLevel)
{
    int rc;
    struct v4l2_control control;
    unsigned int tier = V4L2_MPEG_VIDEO_HEVC_TIER_HIGH;
    OMX_U32 omx_profile_level = eLevel;

    DEBUG_PRINT_LOW("venc_set_level:: eLevel = %u",
                    (unsigned int)eLevel);

    if (!eLevel) {
        DEBUG_PRINT_ERROR(" Unknown OMX level : %u" ,
                    (unsigned int)eLevel );
        return true;
	}
    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC) {
        if(!profile_level_converter::find_tier(m_sVenc_cfg.codectype, eLevel, &tier)) {
            DEBUG_PRINT_ERROR("Failed to find HEVC v4l2 level tier for OMX level : %d", eLevel);
            return true;
        }
        /* HEVC high tier profile levels are mapped to same V4L2 profile levels as main tier profile levels */
        if (tier == V4L2_MPEG_VIDEO_HEVC_TIER_HIGH)
            omx_profile_level = eLevel >> 1;
    }
	if (!profile_level_converter::convert_omx_level_to_v4l2(m_sVenc_cfg.codectype, omx_profile_level, &control.value)) {
        DEBUG_PRINT_ERROR("Failed to find v4l2 level for OMX level : %d" \
                        " Codec : %lu", eLevel, m_sVenc_cfg.codectype);
        return true;
	}

    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_H264) {
        control.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL;
    } else if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_VP8) {
        control.id = V4L2_CID_MPEG_VIDC_VIDEO_VP8_PROFILE_LEVEL;
    } else if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC) {
        control.id = V4L2_CID_MPEG_VIDEO_HEVC_LEVEL;
        profile_level.tier = tier;
    }
    else {
        DEBUG_PRINT_ERROR("Wrong CODEC");
        return false;
    }

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d",
                            control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d",
                        control.id, control.value);
        return false;
    }
    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d",
                        control.id, control.value);

    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_HEVC) {
        struct v4l2_control control_tier = {
            .id = V4L2_CID_MPEG_VIDEO_HEVC_TIER,
            .value = (signed int)tier
        };
        DEBUG_PRINT_LOW("Calling IOCTL set tier control for HEVC, id %#x, value %d",
                            control_tier.id, control_tier.value);

        rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control_tier);
        if (rc) {
            DEBUG_PRINT_ERROR("Failed to set tier control for HEVC, id %#x, value %d",
                                control_tier.id, control_tier.value);
        } else {
            profile_level.tier = control_tier.value;
        }
    }
    profile_level.level = control.value;
    return true;
}

bool venc_dev::venc_set_grid_enable()
{
    int rc;
    struct v4l2_control control;

    DEBUG_PRINT_LOW("venc_set_grid_enable");
    control.id = V4L2_CID_MPEG_VIDC_IMG_GRID_SIZE;
    control.value = 1; // TODO: DO we need to get this value from input argument?
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);
    mIsGridset = true;
    return true;
}

bool venc_dev::venc_set_entropy_config(OMX_BOOL enable, OMX_U32 i_cabac_level)
{
    int rc = 0;
    struct v4l2_control control;

    DEBUG_PRINT_LOW("venc_set_entropy_config: CABAC = %u level: %u", enable, (unsigned int)i_cabac_level);

    if (enable && (codec_profile.profile != V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE) &&
            (codec_profile.profile != V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE)) {
        control.value = V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC;
    } else if (!enable) {
        control.value =  V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC;
    } else {
        DEBUG_PRINT_ERROR("Invalid Entropy mode for Baseline Profile");
        return false;
    }

    control.id = V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);
    entropy.longentropysel = control.value;

    return true;
}

bool venc_dev::venc_set_multislice_cfg(OMX_U32 nSlicemode, OMX_U32 nSlicesize)
{
    int rc;
    int slice_id = 0;
    struct v4l2_control control;
    bool status = true;

    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_H263 || nSlicesize == 0) {
        nSlicemode = V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE;
        nSlicesize = 0;
    }

    if (nSlicemode == V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_MB) {
        if (!venc_validate_range(V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB, nSlicesize)) {
            DEBUG_PRINT_ERROR("Invalid settings, hardware doesn't support %u as slicesize", nSlicesize);
            return false;
        }
        slice_id = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB;

    } else if (nSlicemode == V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_BYTES) {
        if (!venc_validate_range(V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES, nSlicesize)) {
            DEBUG_PRINT_ERROR("Invalid settings, hardware doesn't support %u as slicesize", nSlicesize);
            return false;
        }
        slice_id = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_BYTES;

    } else if (nSlicesize) {
        DEBUG_PRINT_ERROR("Invalid settings, unexpected slicemode = %u and slice size = %u", nSlicemode, nSlicesize);
        return false;
    }

    control.id    = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE;
    control.value = nSlicemode;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    if (nSlicemode == V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE) {
        return status;
    }

    control.id    = slice_id;
    control.value = nSlicesize;

    DEBUG_PRINT_LOW("Calling SLICE_MB IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control");
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    multislice.mslice_mode = nSlicemode;
    multislice.mslice_size = nSlicesize;

    return status;
}

bool venc_dev::venc_set_error_resilience(OMX_VIDEO_PARAM_ERRORCORRECTIONTYPE* error_resilience)
{
    bool status = true;
    struct venc_headerextension hec_cfg;
    struct venc_multiclicecfg multislice_cfg;
    int rc;
    OMX_U32 resynchMarkerSpacingBytes = 0;
    struct v4l2_control control;

    memset(&control, 0, sizeof(control));

    if (m_sVenc_cfg.codectype == V4L2_PIX_FMT_MPEG4) {
        if (error_resilience->bEnableHEC) {
            hec_cfg.header_extension = 1;
        } else {
            hec_cfg.header_extension = 0;
        }

        hec.header_extension = error_resilience->bEnableHEC;
    }

    if (error_resilience->bEnableRVLC) {
        DEBUG_PRINT_ERROR("RVLC is not Supported");
        return false;
    }

    if (( m_sVenc_cfg.codectype != V4L2_PIX_FMT_H263) &&
            (error_resilience->bEnableDataPartitioning)) {
        DEBUG_PRINT_ERROR("DataPartioning are not Supported for MPEG4/H264");
        return false;
    }

    if (error_resilience->nResynchMarkerSpacing) {
        resynchMarkerSpacingBytes = error_resilience->nResynchMarkerSpacing;
        resynchMarkerSpacingBytes = ALIGN(resynchMarkerSpacingBytes, 8) >> 3;
    }

    status = venc_set_multislice_cfg(V4L2_MPEG_VIDEO_MULTI_SICE_MODE_MAX_BYTES, resynchMarkerSpacingBytes);

    return status;
}

bool venc_dev::venc_set_inloop_filter(OMX_VIDEO_AVCLOOPFILTERTYPE loopfilter)
{
    int rc;
    struct v4l2_control control;
    control.id=V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_MODE;
    control.value=0;

    if (loopfilter == OMX_VIDEO_AVCLoopFilterEnable) {
        control.value=V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_ENABLED;
    } else if (loopfilter == OMX_VIDEO_AVCLoopFilterDisable) {
        control.value=V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED;
    } else if (loopfilter == OMX_VIDEO_AVCLoopFilterDisableSliceBoundary) {
        control.value=V4L2_MPEG_VIDEO_H264_LOOP_FILTER_MODE_DISABLED_AT_SLICE_BOUNDARY;
    }

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    dbkfilter.db_mode=control.value;

    control.id=V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_ALPHA;
    control.value=0;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);
    control.id=V4L2_CID_MPEG_VIDEO_H264_LOOP_FILTER_BETA;
    control.value=0;
    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);


    dbkfilter.slicealpha_offset = dbkfilter.slicebeta_offset = 0;
    return true;
}

unsigned long venc_dev::venc_get_codectype(OMX_VIDEO_CODINGTYPE eCompressionFormat)
{
    unsigned long codectype = V4L2_PIX_FMT_H264;

    switch ((int)eCompressionFormat) {
    case OMX_VIDEO_CodingAVC:
        codectype = V4L2_PIX_FMT_H264;
        break;
    case OMX_VIDEO_CodingVP8:
        codectype = V4L2_PIX_FMT_VP8;
        break;
    case OMX_VIDEO_CodingVP9:
        codectype = V4L2_PIX_FMT_VP9;
        break;
    case OMX_VIDEO_CodingHEVC:
    case OMX_VIDEO_CodingImageHEIC:
        codectype = V4L2_PIX_FMT_HEVC;
        break;
    default:
        DEBUG_PRINT_ERROR("Unsupported eCompressionFormat %#x", eCompressionFormat);
        codectype = V4L2_PIX_FMT_H264;
        break;
    }

    return codectype;
}

bool venc_dev::venc_set_bitrate_savings_mode(OMX_U32 bitrateSavingEnable)
{
    struct v4l2_control control;
    int rc = 0;

    DEBUG_PRINT_LOW("Set bitrate savings %d", bitrateSavingEnable);
    control.id = V4L2_CID_MPEG_VIDC_VENC_BITRATE_SAVINGS;
    control.value = bitrateSavingEnable;
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);
    if (rc) {
        DEBUG_PRINT_HIGH("Non-Fatal: Request to set bitrate savings failed");
    }
    mBitrateSavingsEnable = bitrateSavingEnable;

    return true;
}

bool venc_dev::venc_set_ratectrl_cfg(OMX_VIDEO_CONTROLRATETYPE eControlRate)
{
    bool status = true;
    struct v4l2_control control;
    int rc = 0;

    control.id = V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE;
    control.value = !!((OMX_U32)eControlRate ^ (OMX_U32)OMX_Video_ControlRateDisable);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set RC_ENABLE");
        return false;
    }

    control.id = V4L2_CID_MPEG_VIDEO_BITRATE_MODE;
    int temp = eControlRate;
    switch ((OMX_U32)eControlRate) {
        case OMX_Video_ControlRateDisable:
            control.value = -1;
            break;
        case OMX_Video_ControlRateVariableSkipFrames:
            control.value = V4L2_MPEG_VIDEO_BITRATE_MODE_VBR;
            break;
        case OMX_Video_ControlRateVariable:
            control.value = V4L2_MPEG_VIDEO_BITRATE_MODE_VBR;
            break;
        case OMX_Video_ControlRateConstantSkipFrames:
            control.value = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR_VFR;
            break;
        case OMX_Video_ControlRateConstant:
            control.value = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR;
            break;
        case QOMX_Video_ControlRateMaxBitrate:
            control.value = V4L2_MPEG_VIDEO_BITRATE_MODE_MBR;
            break;
        case QOMX_Video_ControlRateMaxBitrateSkipFrames:
            control.value = V4L2_MPEG_VIDEO_BITRATE_MODE_MBR_VFR;
            break;
        case OMX_Video_ControlRateConstantQuality:
            control.value = V4L2_MPEG_VIDEO_BITRATE_MODE_CQ;
            break;
        default:
            status = false;
            break;
    }

    if (status && control.value != -1) {

        DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
        rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

        if (rc) {
            DEBUG_PRINT_ERROR("Failed to set control");
            return false;
        }

        DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

        rate_ctrl.rcmode = control.value;
    }

    venc_set_bitrate_savings_mode(mBitrateSavingsEnable);

    return status;
}

bool venc_dev::venc_set_aspectratio(void *nSar)
{
    int rc;
    struct v4l2_control control;
    struct v4l2_ext_control ctrl[2];
    struct v4l2_ext_controls controls;
    QOMX_EXTNINDEX_VIDEO_VENC_SAR *sar;

    sar = (QOMX_EXTNINDEX_VIDEO_VENC_SAR *) nSar;

    ctrl[0].id = V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_WIDTH;
    ctrl[0].value = sar->nSARWidth;
    ctrl[1].id = V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_HEIGHT;
    ctrl[1].value = sar->nSARHeight;

    controls.count = 2;
    controls.ctrl_class = V4L2_CTRL_CLASS_MPEG;
    controls.controls = ctrl;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%x val=%d, id=%x val=%d",
                    controls.controls[0].id, controls.controls[0].value,
                    controls.controls[1].id, controls.controls[1].value);

    rc = ioctl(m_nDriver_fd, VIDIOC_S_EXT_CTRLS, &controls);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set SAR %d", rc);
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%x val=%d, id=%x val=%d",
                    controls.controls[0].id, controls.controls[0].value,
                    controls.controls[1].id, controls.controls[1].value);
    return true;
}

bool venc_dev::venc_set_lowlatency_mode(OMX_BOOL enable)
{
    int rc = 0;
    struct v4l2_control control;

    control.id = V4L2_CID_MPEG_VIDC_VIDEO_LOWLATENCY_MODE;
    if (enable)
        control.value = V4L2_MPEG_MSM_VIDC_ENABLE;
    else
        control.value = V4L2_MPEG_MSM_VIDC_DISABLE;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%x, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set lowlatency control");
        return false;
    }
    low_latency_mode = enable;
    DEBUG_PRINT_LOW("Success IOCTL set control for id=%x, value=%d", control.id, control.value);

    return true;
}

bool venc_dev::venc_set_vui_timing_info(OMX_BOOL enable)
{
    struct v4l2_control control;
    int rc = 0;
    control.id = V4L2_CID_MPEG_VIDC_VIDEO_VUI_TIMING_INFO;

    if (enable)
        control.value = V4L2_MPEG_MSM_VIDC_ENABLE;
    else
        control.value = V4L2_MPEG_MSM_VIDC_DISABLE;

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%x, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set VUI timing info control");
        return false;
    }
    DEBUG_PRINT_LOW("Success IOCTL set control for id=%x, value=%d", control.id, control.value);
    return true;
}

bool venc_dev::venc_set_peak_bitrate(OMX_U32 nPeakBitrate)
{
    struct v4l2_control control;
    int rc = 0;
    control.id = V4L2_CID_MPEG_VIDEO_BITRATE_PEAK;
    control.value = nPeakBitrate;

    DEBUG_PRINT_LOW("venc_set_peak_bitrate: bitrate = %u", (unsigned int)nPeakBitrate);

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set peak bitrate control");
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);

    return true;
}

bool venc_dev::venc_set_vpx_error_resilience(OMX_BOOL enable)
{
    struct v4l2_control control;
    int rc = 0;
    control.id = V4L2_CID_MPEG_VIDC_VIDEO_VPX_ERROR_RESILIENCE;

    if (enable)
        control.value = 1;
    else
        control.value = 0;

    DEBUG_PRINT_LOW("venc_set_vpx_error_resilience: %d", control.value);

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);

    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);
    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set VPX Error Resilience");
        return false;
    }
    vpx_err_resilience.enable = 1;
    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);
    return true;
}

bool venc_dev::venc_set_vbv_delay(OMX_U32 nVbvDelay)
{
    int rc = 0;
    struct v4l2_control control;

    control.id = V4L2_CID_MPEG_VIDEO_VBV_DELAY;
    control.value = nVbvDelay;

    DEBUG_PRINT_LOW("venc_set_vbv_delay: vbvdelay = %u", (unsigned int)control.value);

    DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
    rc = ioctl(m_nDriver_fd, VIDIOC_S_CTRL, &control);

    if (rc) {
        DEBUG_PRINT_ERROR("Failed to set vbv delay");
        return false;
    }

    DEBUG_PRINT_LOW("Success IOCTL set control for id=%d, value=%d", control.id, control.value);
    return true;
}
