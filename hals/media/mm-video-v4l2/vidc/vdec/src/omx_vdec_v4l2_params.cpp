/*--------------------------------------------------------------------------
Copyright (c) 2010 - 2020, The Linux Foundation. All rights reserved.

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

/*============================================================================
                            O p e n M A X   w r a p p e r s
                             O p e n  M A X   C o r e

  This module contains the implementation of the OpenMAX core & component.

*//*========================================================================*/

//////////////////////////////////////////////////////////////////////////////
//                             Include Files
//////////////////////////////////////////////////////////////////////////////

#include "omx_vdec.h"

#define BUFFER_LOG_LOC "/data/vendor/media"

#ifdef _ANDROID_
extern "C" {
#include<utils/Log.h>
}
#endif//_ANDROID_

using namespace android;

/* ======================================================================
   FUNCTION
   omx_vdec::GetParameter

   DESCRIPTION
   OMX Get Parameter method implementation

   PARAMETERS
   <TBD>.

   RETURN VALUE
   Error None if successful.

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::get_parameter(OMX_IN OMX_HANDLETYPE     hComp,
        OMX_IN OMX_INDEXTYPE paramIndex,
        OMX_INOUT OMX_PTR     paramData)
{
    (void) hComp;
    OMX_ERRORTYPE eRet = OMX_ErrorNone;

    DEBUG_PRINT_LOW("get_parameter:");
    if (m_state == OMX_StateInvalid) {
        DEBUG_PRINT_ERROR("Get Param in Invalid State");
        return OMX_ErrorInvalidState;
    }
    if (paramData == NULL) {
        DEBUG_PRINT_LOW("Get Param in Invalid paramData");
        return OMX_ErrorBadParameter;
    }
    switch ((unsigned long)paramIndex) {
        case OMX_IndexParamPortDefinition: {
                               VALIDATE_OMX_PARAM_DATA(paramData, OMX_PARAM_PORTDEFINITIONTYPE);
                               OMX_PARAM_PORTDEFINITIONTYPE *portDefn =
                                   (OMX_PARAM_PORTDEFINITIONTYPE *) paramData;
                               DEBUG_PRINT_LOW("get_parameter: OMX_IndexParamPortDefinition");

                               OMX_COLOR_FORMATTYPE drv_color_format;
                               bool status = false;

                               if (!client_buffers.is_color_conversion_enabled()) {
                                   status = client_buffers.get_color_format(drv_color_format);
                               }

                               fix_drv_output_format();

                               if (status) {
                                 if (!client_buffers.is_color_conversion_enabled()) {
                                         client_buffers.set_client_buffers_disabled(true);
                                         client_buffers.set_color_format(drv_color_format);
                                 }
                              }

                               eRet = update_portdef(portDefn);
                               if (eRet == OMX_ErrorNone)
                                   m_port_def = *portDefn;
                               break;
                           }
        case OMX_IndexParamVideoInit: {
                              VALIDATE_OMX_PARAM_DATA(paramData, OMX_PORT_PARAM_TYPE);
                              OMX_PORT_PARAM_TYPE *portParamType =
                                  (OMX_PORT_PARAM_TYPE *) paramData;
                              DEBUG_PRINT_LOW("get_parameter: OMX_IndexParamVideoInit");

                              portParamType->nVersion.nVersion = OMX_SPEC_VERSION;
                              portParamType->nSize = sizeof(OMX_PORT_PARAM_TYPE);
                              portParamType->nPorts           = 2;
                              portParamType->nStartPortNumber = 0;
                              break;
                          }
        case OMX_IndexParamVideoPortFormat: {
                                VALIDATE_OMX_PARAM_DATA(paramData, OMX_VIDEO_PARAM_PORTFORMATTYPE);
                                OMX_VIDEO_PARAM_PORTFORMATTYPE *portFmt =
                                    (OMX_VIDEO_PARAM_PORTFORMATTYPE *)paramData;
                                DEBUG_PRINT_LOW("get_parameter: OMX_IndexParamVideoPortFormat");

                                portFmt->nVersion.nVersion = OMX_SPEC_VERSION;
                                portFmt->nSize             = sizeof(OMX_VIDEO_PARAM_PORTFORMATTYPE);

                                if (0 == portFmt->nPortIndex) {
                                    if (0 == portFmt->nIndex) {
                                        portFmt->eColorFormat =  OMX_COLOR_FormatUnused;
                                        portFmt->eCompressionFormat = eCompressionFormat;
                                    } else {
                                        DEBUG_PRINT_ERROR("get_parameter: OMX_IndexParamVideoPortFormat:"\
                                                " NoMore compression formats");
                                        eRet =  OMX_ErrorNoMore;
                                    }
                                } else if (1 == portFmt->nPortIndex) {
                                    portFmt->eCompressionFormat =  OMX_VIDEO_CodingUnused;

                                    // Distinguish non-surface mode from normal playback use-case based on
                                    // usage hinted via "OMX.google.android.index.useAndroidNativeBuffer2"
                                    // For non-android, use the default list
                                    // Also use default format-list if FLEXIBLE YUV is supported,
                                    // as the client negotiates the standard color-format if it needs to
                                    bool useNonSurfaceMode = false;
#if defined(_ANDROID_) && !defined(FLEXYUV_SUPPORTED) && !defined(USE_GBM)
                                    useNonSurfaceMode = (m_enable_android_native_buffers == OMX_FALSE);
#endif
                                    portFmt->eColorFormat = useNonSurfaceMode ?
                                        getPreferredColorFormatNonSurfaceMode(portFmt->nIndex) :
                                        getPreferredColorFormatDefaultMode(portFmt->nIndex);

                                    if (portFmt->eColorFormat == OMX_COLOR_FormatMax ) {
                                        eRet = OMX_ErrorNoMore;
                                        DEBUG_PRINT_LOW("get_parameter: OMX_IndexParamVideoPortFormat:"\
                                                " NoMore Color formats");
                                    }
                                    DEBUG_PRINT_HIGH("returning color-format: 0x%x", portFmt->eColorFormat);
                                } else {
                                    DEBUG_PRINT_ERROR("get_parameter: Bad port index %d",
                                            (int)portFmt->nPortIndex);
                                    eRet = OMX_ErrorBadPortIndex;
                                }
                                break;
                            }
                            /*Component should support this port definition*/
        case OMX_IndexParamAudioInit: {
                              VALIDATE_OMX_PARAM_DATA(paramData, OMX_PORT_PARAM_TYPE);
                              OMX_PORT_PARAM_TYPE *audioPortParamType =
                                  (OMX_PORT_PARAM_TYPE *) paramData;
                              DEBUG_PRINT_LOW("get_parameter: OMX_IndexParamAudioInit");
                              audioPortParamType->nVersion.nVersion = OMX_SPEC_VERSION;
                              audioPortParamType->nSize = sizeof(OMX_PORT_PARAM_TYPE);
                              audioPortParamType->nPorts           = 0;
                              audioPortParamType->nStartPortNumber = 0;
                              break;
                          }
                          /*Component should support this port definition*/
        case OMX_IndexParamImageInit: {
                              VALIDATE_OMX_PARAM_DATA(paramData, OMX_PORT_PARAM_TYPE);
                              OMX_PORT_PARAM_TYPE *imagePortParamType =
                                  (OMX_PORT_PARAM_TYPE *) paramData;
                              DEBUG_PRINT_LOW("get_parameter: OMX_IndexParamImageInit");
                              imagePortParamType->nVersion.nVersion = OMX_SPEC_VERSION;
                              imagePortParamType->nSize = sizeof(OMX_PORT_PARAM_TYPE);
                              imagePortParamType->nPorts           = 0;
                              imagePortParamType->nStartPortNumber = 0;
                              break;

                          }
                          /*Component should support this port definition*/
        case OMX_IndexParamOtherInit: {
                              DEBUG_PRINT_ERROR("get_parameter: OMX_IndexParamOtherInit %08x",
                                      paramIndex);
                              eRet =OMX_ErrorUnsupportedIndex;
                              break;
                          }
        case OMX_IndexParamStandardComponentRole: {
                                  VALIDATE_OMX_PARAM_DATA(paramData, OMX_PARAM_COMPONENTROLETYPE);
                                  OMX_PARAM_COMPONENTROLETYPE *comp_role;
                                  comp_role = (OMX_PARAM_COMPONENTROLETYPE *) paramData;
                                  comp_role->nVersion.nVersion = OMX_SPEC_VERSION;
                                  comp_role->nSize = sizeof(*comp_role);

                                  DEBUG_PRINT_LOW("Getparameter: OMX_IndexParamStandardComponentRole %d",
                                          paramIndex);
                                  strlcpy((char*)comp_role->cRole,(const char*)m_cRole,
                                          OMX_MAX_STRINGNAME_SIZE);
                                  break;
                              }
                              /* Added for parameter test */
        case OMX_IndexParamPriorityMgmt: {
                             VALIDATE_OMX_PARAM_DATA(paramData, OMX_PRIORITYMGMTTYPE);
                             OMX_PRIORITYMGMTTYPE *priorityMgmType =
                                 (OMX_PRIORITYMGMTTYPE *) paramData;
                             DEBUG_PRINT_LOW("get_parameter: OMX_IndexParamPriorityMgmt");
                             priorityMgmType->nVersion.nVersion = OMX_SPEC_VERSION;
                             priorityMgmType->nSize = sizeof(OMX_PRIORITYMGMTTYPE);

                             break;
                         }
                         /* Added for parameter test */
        case OMX_IndexParamCompBufferSupplier: {
                                   VALIDATE_OMX_PARAM_DATA(paramData, OMX_PARAM_BUFFERSUPPLIERTYPE);
                                   OMX_PARAM_BUFFERSUPPLIERTYPE *bufferSupplierType =
                                       (OMX_PARAM_BUFFERSUPPLIERTYPE*) paramData;
                                   DEBUG_PRINT_LOW("get_parameter: OMX_IndexParamCompBufferSupplier");

                                   bufferSupplierType->nSize = sizeof(OMX_PARAM_BUFFERSUPPLIERTYPE);
                                   bufferSupplierType->nVersion.nVersion = OMX_SPEC_VERSION;
                                   if (0 == bufferSupplierType->nPortIndex)
                                       bufferSupplierType->nPortIndex = OMX_BufferSupplyUnspecified;
                                   else if (1 == bufferSupplierType->nPortIndex)
                                       bufferSupplierType->nPortIndex = OMX_BufferSupplyUnspecified;
                                   else
                                       eRet = OMX_ErrorBadPortIndex;


                                   break;
                               }
        case OMX_IndexParamVideoAvc: {
                             DEBUG_PRINT_LOW("get_parameter: OMX_IndexParamVideoAvc %08x",
                                     paramIndex);
                             break;
                         }
        case (OMX_INDEXTYPE)QOMX_IndexParamVideoMvc: {
                             DEBUG_PRINT_LOW("get_parameter: QOMX_IndexParamVideoMvc %08x",
                                     paramIndex);
                             break;
                         }
        case OMX_IndexParamVideoMpeg2: {
                               DEBUG_PRINT_LOW("get_parameter: OMX_IndexParamVideoMpeg2 %08x",
                                       paramIndex);
                               break;
                           }
        case OMX_IndexParamVideoProfileLevelQuerySupported: {
                                        VALIDATE_OMX_PARAM_DATA(paramData, OMX_VIDEO_PARAM_PROFILELEVELTYPE);
                                        DEBUG_PRINT_LOW("get_parameter: OMX_IndexParamVideoProfileLevelQuerySupported %08x", paramIndex);
                                        OMX_VIDEO_PARAM_PROFILELEVELTYPE *profileLevelType =
                                            (OMX_VIDEO_PARAM_PROFILELEVELTYPE *)paramData;
                                        eRet = get_supported_profile_level(profileLevelType);
                                        break;
                                    }
#if defined (_ANDROID_HONEYCOMB_) || defined (_ANDROID_ICS_)
        case OMX_GoogleAndroidIndexGetAndroidNativeBufferUsage: {
                                        VALIDATE_OMX_PARAM_DATA(paramData, GetAndroidNativeBufferUsageParams);
                                        DEBUG_PRINT_LOW("get_parameter: OMX_GoogleAndroidIndexGetAndroidNativeBufferUsage");
                                        GetAndroidNativeBufferUsageParams* nativeBuffersUsage = (GetAndroidNativeBufferUsageParams *) paramData;
                                        if (nativeBuffersUsage->nPortIndex == OMX_CORE_OUTPUT_PORT_INDEX) {

                                            if (secure_mode) {
                                                nativeBuffersUsage->nUsage = (GRALLOC_USAGE_PRIVATE_MM_HEAP | GRALLOC_USAGE_PROTECTED |
                                                        GRALLOC_USAGE_PRIVATE_UNCACHED);
                                            } else {
                                                nativeBuffersUsage->nUsage = GRALLOC_USAGE_PRIVATE_UNCACHED;
                                            }
                                        } else {
                                            DEBUG_PRINT_HIGH("get_parameter: OMX_GoogleAndroidIndexGetAndroidNativeBufferUsage failed!");
                                            eRet = OMX_ErrorBadParameter;
                                        }
                                    }
                                    break;
#endif

#ifdef FLEXYUV_SUPPORTED
        case OMX_QcomIndexFlexibleYUVDescription: {
                DEBUG_PRINT_LOW("get_parameter: describeColorFormat");
                VALIDATE_OMX_PARAM_DATA(paramData, DescribeColorFormatParams);
                eRet = describeColorFormat(paramData);
                if (eRet == OMX_ErrorUnsupportedSetting) {
                    DEBUG_PRINT_LOW("The standard OMX linear formats are understood by client. Please ignore this  Unsupported Setting (0x80001019).");
                }
                break;
            }
#endif
        case OMX_IndexParamVideoProfileLevelCurrent: {
             VALIDATE_OMX_PARAM_DATA(paramData, OMX_VIDEO_PARAM_PROFILELEVELTYPE);
             OMX_VIDEO_PARAM_PROFILELEVELTYPE* pParam = (OMX_VIDEO_PARAM_PROFILELEVELTYPE*)paramData;
             struct v4l2_control profile_control, level_control;

             switch (drv_ctx.decoder_format) {
                 case VDEC_CODECTYPE_H264:
                     profile_control.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE;
                     level_control.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL;
                     break;
                 default:
                     DEBUG_PRINT_ERROR("get_param of OMX_IndexParamVideoProfileLevelCurrent only available for H264");
                     eRet = OMX_ErrorNotImplemented;
                     break;
             }

             if (!eRet && !ioctl(drv_ctx.video_driver_fd, VIDIOC_G_CTRL, &profile_control)) {
                switch ((enum v4l2_mpeg_video_h264_profile)profile_control.value) {
                    case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE:
                        pParam->eProfile = OMX_VIDEO_AVCProfileBaseline;
                        break;
                    case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE:
                        pParam->eProfile = OMX_VIDEO_AVCProfileConstrainedBaseline;
                        break;
                    case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_HIGH:
                        pParam->eProfile = OMX_VIDEO_AVCProfileConstrainedHigh;
                        break;
                    case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN:
                        pParam->eProfile = OMX_VIDEO_AVCProfileMain;
                        break;
                    case V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED:
                        pParam->eProfile = OMX_VIDEO_AVCProfileExtended;
                        break;
                    case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH:
                        pParam->eProfile = OMX_VIDEO_AVCProfileHigh;
                        break;
                    case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10:
                        pParam->eProfile = OMX_VIDEO_AVCProfileHigh10;
                        break;
                    case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422:
                        pParam->eProfile = OMX_VIDEO_AVCProfileHigh422;
                        break;
                    case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE:
                    case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10_INTRA:
                    case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422_INTRA:
                    case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_INTRA:
                    case V4L2_MPEG_VIDEO_H264_PROFILE_CAVLC_444_INTRA:
                    case V4L2_MPEG_VIDEO_H264_PROFILE_SCALABLE_BASELINE:
                    case V4L2_MPEG_VIDEO_H264_PROFILE_SCALABLE_HIGH:
                    case V4L2_MPEG_VIDEO_H264_PROFILE_SCALABLE_HIGH_INTRA:
                    case V4L2_MPEG_VIDEO_H264_PROFILE_STEREO_HIGH:
                    case V4L2_MPEG_VIDEO_H264_PROFILE_MULTIVIEW_HIGH:
                        eRet = OMX_ErrorUnsupportedIndex;
                        break;
                }
             } else {
                 eRet = OMX_ErrorUnsupportedIndex;
             }


             if (!eRet && !ioctl(drv_ctx.video_driver_fd, VIDIOC_G_CTRL, &level_control)) {
                switch ((enum v4l2_mpeg_video_h264_level)level_control.value) {
                    case V4L2_MPEG_VIDEO_H264_LEVEL_1_0:
                        pParam->eLevel = OMX_VIDEO_AVCLevel1;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_1B:
                        pParam->eLevel = OMX_VIDEO_AVCLevel1b;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_1_1:
                        pParam->eLevel = OMX_VIDEO_AVCLevel11;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_1_2:
                        pParam->eLevel = OMX_VIDEO_AVCLevel12;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_1_3:
                        pParam->eLevel = OMX_VIDEO_AVCLevel13;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_2_0:
                        pParam->eLevel = OMX_VIDEO_AVCLevel2;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_2_1:
                        pParam->eLevel = OMX_VIDEO_AVCLevel21;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_2_2:
                        pParam->eLevel = OMX_VIDEO_AVCLevel22;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_3_0:
                        pParam->eLevel = OMX_VIDEO_AVCLevel3;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_3_1:
                        pParam->eLevel = OMX_VIDEO_AVCLevel31;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_3_2:
                        pParam->eLevel = OMX_VIDEO_AVCLevel32;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_4_0:
                        pParam->eLevel = OMX_VIDEO_AVCLevel4;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_4_1:
                        pParam->eLevel = OMX_VIDEO_AVCLevel41;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_4_2:
                        pParam->eLevel = OMX_VIDEO_AVCLevel42;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_5_0:
                        pParam->eLevel = OMX_VIDEO_AVCLevel5;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_5_1:
                        pParam->eLevel = OMX_VIDEO_AVCLevel51;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_5_2:
                        pParam->eLevel = OMX_VIDEO_AVCLevel52;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_6_0:
                        pParam->eLevel = OMX_VIDEO_AVCLevel6;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_6_1:
                        pParam->eLevel = OMX_VIDEO_AVCLevel61;
                        break;
                    case V4L2_MPEG_VIDEO_H264_LEVEL_6_2:
                        pParam->eLevel = OMX_VIDEO_AVCLevel62;
                        break;
                    default:
                        eRet = OMX_ErrorUnsupportedIndex;
                        break;
                }
             } else {
                 eRet = OMX_ErrorUnsupportedIndex;
             }

             break;

         }
        case OMX_QTIIndexParamVideoClientExtradata:
        {
            VALIDATE_OMX_PARAM_DATA(paramData, QOMX_EXTRADATA_ENABLE);
            DEBUG_PRINT_LOW("get_parameter: OMX_QTIIndexParamVideoClientExtradata");
            QOMX_EXTRADATA_ENABLE *pParam =
                (QOMX_EXTRADATA_ENABLE *)paramData;
            if (pParam->nPortIndex == OMX_CORE_OUTPUT_EXTRADATA_INDEX) {
                pParam->bEnable = m_client_extradata ? OMX_TRUE : OMX_FALSE;
                eRet = OMX_ErrorNone;
            } else {
                eRet = OMX_ErrorUnsupportedIndex;
            }
            break;
        }
        default: {
                 DEBUG_PRINT_ERROR("get_parameter: unknown param %08x", paramIndex);
                 eRet =OMX_ErrorUnsupportedIndex;
             }

    }

    DEBUG_PRINT_LOW("get_parameter returning WxH(%d x %d) SxSH(%d x %d)",
            drv_ctx.video_resolution.frame_width,
            drv_ctx.video_resolution.frame_height,
            drv_ctx.video_resolution.stride,
            drv_ctx.video_resolution.scan_lines);

    return eRet;
}

/* ======================================================================
   FUNCTION
   omx_vdec::Setparameter

   DESCRIPTION
   OMX Set Parameter method implementation.

   PARAMETERS
   <TBD>.

   RETURN VALUE
   OMX Error None if successful.

   ========================================================================== */
OMX_ERRORTYPE  omx_vdec::set_parameter(OMX_IN OMX_HANDLETYPE     hComp,
        OMX_IN OMX_INDEXTYPE paramIndex,
        OMX_IN OMX_PTR        paramData)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    int ret=0;
    struct v4l2_format fmt;
#ifdef _ANDROID_
    char property_value[PROPERTY_VALUE_MAX] = {0};
#endif
    if (m_state == OMX_StateInvalid) {
        DEBUG_PRINT_ERROR("Set Param in Invalid State");
        return OMX_ErrorInvalidState;
    }
    if (paramData == NULL) {
        DEBUG_PRINT_ERROR("Get Param in Invalid paramData");
        return OMX_ErrorBadParameter;
    }
    if ((m_state != OMX_StateLoaded) &&
            BITMASK_ABSENT(&m_flags,OMX_COMPONENT_OUTPUT_ENABLE_PENDING) &&
            (m_out_bEnabled == OMX_TRUE) &&
            BITMASK_ABSENT(&m_flags, OMX_COMPONENT_INPUT_ENABLE_PENDING) &&
            (m_inp_bEnabled == OMX_TRUE)) {
        DEBUG_PRINT_ERROR("Set Param in Invalid State");
        return OMX_ErrorIncorrectStateOperation;
    }
    switch ((unsigned long)paramIndex) {
        case OMX_IndexParamPortDefinition: {
                               VALIDATE_OMX_PARAM_DATA(paramData, OMX_PARAM_PORTDEFINITIONTYPE);
                               OMX_PARAM_PORTDEFINITIONTYPE *portDefn;
                               portDefn = (OMX_PARAM_PORTDEFINITIONTYPE *) paramData;
                               //TODO: Check if any allocate buffer/use buffer/useNativeBuffer has
                               //been called.
                               DEBUG_PRINT_LOW(
                                       "set_parameter: OMX_IndexParamPortDefinition: dir %d port %d wxh %dx%d count: min %d actual %d size %d",
                                       (int)portDefn->eDir, (int)portDefn->nPortIndex,
                                       (int)portDefn->format.video.nFrameWidth,
                                       (int)portDefn->format.video.nFrameHeight,
                                       (int)portDefn->nBufferCountMin,
                                       (int)portDefn->nBufferCountActual,
                                       (int)portDefn->nBufferSize);

                               if (portDefn->nBufferCountActual > MAX_NUM_INPUT_OUTPUT_BUFFERS) {
                                   DEBUG_PRINT_ERROR("ERROR: Buffers requested exceeds max limit %d",
                                                          portDefn->nBufferCountActual);
                                   eRet = OMX_ErrorBadParameter;
                                   break;
                               }
                               if (OMX_CORE_OUTPUT_EXTRADATA_INDEX == portDefn->nPortIndex) {
                                   if (portDefn->nBufferCountActual < MIN_NUM_INPUT_OUTPUT_EXTRADATA_BUFFERS ||
                                        portDefn->nBufferSize != m_client_out_extradata_info.getSize()) {
                                        DEBUG_PRINT_ERROR("ERROR: Bad parameeters request for extradata limit %d size - %d",
                                                          portDefn->nBufferCountActual, portDefn->nBufferSize);
                                        eRet = OMX_ErrorBadParameter;
                                        break;
                                   }
                                    m_client_out_extradata_info.set_extradata_info(portDefn->nBufferSize,
                                            portDefn->nBufferCountActual);
                                    break;
                               }

                               if (OMX_DirOutput == portDefn->eDir) {
                                   DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamPortDefinition OP port");
                                   bool port_format_changed = false;
                                   m_display_id = portDefn->format.video.pNativeWindow;
                                   unsigned int buffer_size;

                                   fix_drv_output_format();

                                   if (portDefn->nBufferCountActual > MAX_NUM_INPUT_OUTPUT_BUFFERS) {
                                       DEBUG_PRINT_ERROR("Requested o/p buf count (%u) exceeds limit (%u)",
                                               portDefn->nBufferCountActual, MAX_NUM_INPUT_OUTPUT_BUFFERS);
                                       eRet = OMX_ErrorBadParameter;
                                   } else if (!client_buffers.get_buffer_req(buffer_size)) {
                                       DEBUG_PRINT_ERROR("Error in getting buffer requirements");
                                       eRet = OMX_ErrorBadParameter;
                                   } else if (!port_format_changed) {

                                       // Buffer count can change only when port is unallocated
                                       if (m_out_mem_ptr &&
                                                (portDefn->nBufferCountActual != drv_ctx.op_buf.actualcount ||
                                                portDefn->nBufferSize != drv_ctx.op_buf.buffer_size)) {

                                           DEBUG_PRINT_ERROR("Cannot change o/p buffer count since all buffers are not freed yet !");
                                           eRet = OMX_ErrorInvalidState;
                                           break;
                                       }

                                       // route updating of buffer requirements via c2d proxy.
                                       // Based on whether c2d is enabled, requirements will be handed
                                       // to the vidc driver appropriately
                                       eRet = client_buffers.set_buffer_req(portDefn->nBufferSize,
                                                portDefn->nBufferCountActual);
                                       if (eRet == OMX_ErrorNone) {
                                           m_port_def = *portDefn;
                                       } else {
                                           DEBUG_PRINT_ERROR("ERROR: OP Requirements(#%d: %u) Requested(#%u: %u)",
                                                   drv_ctx.op_buf.mincount, (unsigned int)buffer_size,
                                                   (unsigned int)portDefn->nBufferCountActual, (unsigned int)portDefn->nBufferSize);
                                           eRet = OMX_ErrorBadParameter;
                                       }
                                   }
                               } else if (OMX_DirInput == portDefn->eDir) {
                                   DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamPortDefinition IP port");
                                   bool port_format_changed = false;
                                   if ((portDefn->format.video.xFramerate >> 16) > 0 &&
                                           (portDefn->format.video.xFramerate >> 16) <= MAX_SUPPORTED_FPS) {
                                       // Frame rate only should be set if this is a "known value" or to
                                       // activate ts prediction logic (arbitrary mode only) sending input
                                       // timestamps with max value (LLONG_MAX).
                                       m_fps_received = portDefn->format.video.xFramerate;
                                       DEBUG_PRINT_HIGH("set_parameter: frame rate set by omx client : %u",
                                               (unsigned int)portDefn->format.video.xFramerate >> 16);
                                       Q16ToFraction(portDefn->format.video.xFramerate, drv_ctx.frame_rate.fps_numerator,
                                               drv_ctx.frame_rate.fps_denominator);
                                       if (!drv_ctx.frame_rate.fps_numerator) {
                                           DEBUG_PRINT_ERROR("Numerator is zero setting to 30");
                                           drv_ctx.frame_rate.fps_numerator = 30;
                                       }
                                       if (drv_ctx.frame_rate.fps_denominator)
                                           drv_ctx.frame_rate.fps_numerator = (int)
                                               drv_ctx.frame_rate.fps_numerator / drv_ctx.frame_rate.fps_denominator;
                                       drv_ctx.frame_rate.fps_denominator = 1;
                                       frm_int = drv_ctx.frame_rate.fps_denominator * 1e6 /
                                           drv_ctx.frame_rate.fps_numerator;
                                       DEBUG_PRINT_LOW("set_parameter: frm_int(%u) fps(%.2f)",
                                               (unsigned int)frm_int, drv_ctx.frame_rate.fps_numerator /
                                               (float)drv_ctx.frame_rate.fps_denominator);

                                       struct v4l2_control control;

                                       control.id =  V4L2_CID_MPEG_VIDC_VIDEO_FRAME_RATE;
                                       control.value = drv_ctx.frame_rate.fps_numerator / drv_ctx.frame_rate.fps_denominator;
                                       control.value <<= 16;
                                       control.value |= (0x0000FFFF & (drv_ctx.frame_rate.fps_numerator % drv_ctx.frame_rate.fps_denominator));
                                       DEBUG_PRINT_LOW("Calling IOCTL set control for id=%d, val=%d", control.id, control.value);
                                       ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_S_CTRL, &control);
                                       if (ret) {
                                           DEBUG_PRINT_ERROR("Failed to set control, id %#x, value %d", control.id, control.value);
                                           return OMX_ErrorHardware;
                                       }
                                   }

                                   if (drv_ctx.video_resolution.frame_height !=
                                           portDefn->format.video.nFrameHeight ||
                                           drv_ctx.video_resolution.frame_width  !=
                                           portDefn->format.video.nFrameWidth) {
                                       DEBUG_PRINT_LOW("SetParam IP: WxH(%u x %u)",
                                               (unsigned int)portDefn->format.video.nFrameWidth,
                                               (unsigned int)portDefn->format.video.nFrameHeight);
                                       port_format_changed = true;
                                       OMX_U32 frameWidth = portDefn->format.video.nFrameWidth;
                                       OMX_U32 frameHeight = portDefn->format.video.nFrameHeight;
                                       if (frameHeight != 0x0 && frameWidth != 0x0) {
                                           m_extradata_misr.output_crop_rect.nLeft = 0;
                                           m_extradata_misr.output_crop_rect.nTop = 0;
                                           m_extradata_misr.output_crop_rect.nWidth = frameWidth;
                                           m_extradata_misr.output_crop_rect.nHeight = frameHeight;

                                           update_resolution(frameWidth, frameHeight,
                                                   frameWidth, frameHeight);

                                           memset(&fmt, 0x0, sizeof(struct v4l2_format));
                                           fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
                                           fmt.fmt.pix_mp.height = drv_ctx.video_resolution.frame_height;
                                           fmt.fmt.pix_mp.width = drv_ctx.video_resolution.frame_width;
                                           fmt.fmt.pix_mp.pixelformat = output_capability;
                                           DEBUG_PRINT_LOW("DS Disabled : height = %d , width = %d",
                                                           fmt.fmt.pix_mp.height,fmt.fmt.pix_mp.width);
                                           ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_S_FMT, &fmt);
                                           fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                                           fmt.fmt.pix_mp.pixelformat = capture_capability;
                                           ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_S_FMT, &fmt);

                                           if (ret) {
                                               DEBUG_PRINT_ERROR("Set Resolution failed");
                                               eRet = errno == EBUSY ? OMX_ErrorInsufficientResources : OMX_ErrorUnsupportedSetting;
                                           } else {
                                               eRet = get_buffer_req(&drv_ctx.op_buf);
                                           }
                                           if (eRet)
                                               break;
                                       }
                                   }
                                   if (m_custom_buffersize.input_buffersize
                                        && (portDefn->nBufferSize > m_custom_buffersize.input_buffersize)) {
                                       DEBUG_PRINT_ERROR("ERROR: Custom buffer size set by client: %d, trying to set: %d",
                                               m_custom_buffersize.input_buffersize, portDefn->nBufferSize);
                                       eRet = OMX_ErrorBadParameter;
                                       break;
                                   }
                                   if (portDefn->nBufferCountActual > MAX_NUM_INPUT_OUTPUT_BUFFERS) {
                                       DEBUG_PRINT_ERROR("Requested i/p buf count (%u) exceeds limit (%u)",
                                               portDefn->nBufferCountActual, MAX_NUM_INPUT_OUTPUT_BUFFERS);
                                       eRet = OMX_ErrorBadParameter;
                                       break;
                                   }
                                   // Buffer count can change only when port is unallocated
                                   if (m_inp_mem_ptr &&
                                            (portDefn->nBufferCountActual != drv_ctx.ip_buf.actualcount ||
                                            portDefn->nBufferSize != drv_ctx.ip_buf.buffer_size)) {
                                       DEBUG_PRINT_ERROR("Cannot change i/p buffer count since all buffers are not freed yet !");
                                       eRet = OMX_ErrorInvalidState;
                                       break;
                                   }

                                   if (portDefn->nBufferCountActual >= drv_ctx.ip_buf.mincount
                                           || portDefn->nBufferSize != drv_ctx.ip_buf.buffer_size) {
                                       port_format_changed = true;
                                       vdec_allocatorproperty *buffer_prop = &drv_ctx.ip_buf;
                                       drv_ctx.ip_buf.actualcount = portDefn->nBufferCountActual;
                                       drv_ctx.ip_buf.buffer_size = (portDefn->nBufferSize + buffer_prop->alignment - 1) &
                                           (~(buffer_prop->alignment - 1));
                                       eRet = set_buffer_req(buffer_prop);
                                   }
                                   if (false == port_format_changed) {
                                       DEBUG_PRINT_ERROR("ERROR: IP Requirements(#%d: %u) Requested(#%u: %u)",
                                               drv_ctx.ip_buf.mincount, (unsigned int)drv_ctx.ip_buf.buffer_size,
                                               (unsigned int)portDefn->nBufferCountActual, (unsigned int)portDefn->nBufferSize);
                                       eRet = OMX_ErrorBadParameter;
                                   }
                               } else if (portDefn->eDir ==  OMX_DirMax) {
                                   DEBUG_PRINT_ERROR(" Set_parameter: Bad Port idx %d",
                                           (int)portDefn->nPortIndex);
                                   eRet = OMX_ErrorBadPortIndex;
                               }
                           }
                           break;
        case OMX_IndexParamVideoPortFormat: {
                                VALIDATE_OMX_PARAM_DATA(paramData, OMX_VIDEO_PARAM_PORTFORMATTYPE);
                                OMX_VIDEO_PARAM_PORTFORMATTYPE *portFmt =
                                    (OMX_VIDEO_PARAM_PORTFORMATTYPE *)paramData;
                                int ret=0;
                                struct v4l2_format fmt;
                                DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamVideoPortFormat 0x%x, port: %u",
                                        portFmt->eColorFormat, (unsigned int)portFmt->nPortIndex);

                                memset(&fmt, 0x0, sizeof(struct v4l2_format));
                                if (1 == portFmt->nPortIndex) {
                                    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                                    ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_G_FMT, &fmt);
                                    if (ret < 0) {
                                        DEBUG_PRINT_ERROR("%s: Failed to get format on capture mplane", __func__);
                                        return OMX_ErrorBadParameter;
                                    }
                                    enum vdec_output_format op_format;
                                    if (portFmt->eColorFormat == (OMX_COLOR_FORMATTYPE)
                                                     QOMX_COLOR_FORMATYUV420PackedSemiPlanar32m) {
                                        op_format = (enum vdec_output_format)VDEC_YUV_FORMAT_NV12;
                                        fmt.fmt.pix_mp.pixelformat = capture_capability = V4L2_PIX_FMT_NV12;
                                        //check if the required color format is a supported flexible format
                                        is_flexible_format = check_supported_flexible_formats(portFmt->eColorFormat);
                                    } else if (portFmt->eColorFormat == (OMX_COLOR_FORMATTYPE)
                                                   QOMX_COLOR_FORMATYUV420PackedSemiPlanar32mCompressed ||
                                               portFmt->eColorFormat == OMX_COLOR_FormatYUV420Planar ||
                                               portFmt->eColorFormat == OMX_COLOR_FormatYUV420SemiPlanar ||
                                               portFmt->eColorFormat == OMX_COLOR_Format16bitRGB565) {
                                        if (!m_disable_ubwc_mode) {
                                            op_format = (enum vdec_output_format)VDEC_YUV_FORMAT_NV12_UBWC;
                                            fmt.fmt.pix_mp.pixelformat = capture_capability = V4L2_PIX_FMT_NV12_UBWC;
                                        } else {
                                            op_format = (enum vdec_output_format)VDEC_YUV_FORMAT_NV12;
                                            fmt.fmt.pix_mp.pixelformat = capture_capability = V4L2_PIX_FMT_NV12;
                                        }
                                        //check if the required color format is a supported flexible format
                                        is_flexible_format = check_supported_flexible_formats(portFmt->eColorFormat);
                                    } else {
                                        eRet = OMX_ErrorBadParameter;
                                    }

                                    if (eRet == OMX_ErrorNone) {
                                        drv_ctx.output_format = op_format;
                                        ret = ioctl(drv_ctx.video_driver_fd, VIDIOC_S_FMT, &fmt);
                                        if (ret) {
                                            DEBUG_PRINT_ERROR("Set output format failed");
                                            eRet = OMX_ErrorUnsupportedSetting;
                                            /*TODO: How to handle this case */
                                        } else {
                                            eRet = get_buffer_req(&drv_ctx.op_buf);
                                        }
                                    }
                                    if (eRet == OMX_ErrorNone) {
                                        if (!client_buffers.set_color_format(portFmt->eColorFormat)) {
                                            DEBUG_PRINT_ERROR("Set color format failed");
                                            eRet = OMX_ErrorBadParameter;
                                        }
                                    }
                                }
                            }
                            break;
        case OMX_QTIIndexParamVideoClientExtradata: {
                                  VALIDATE_OMX_PARAM_DATA(paramData, QOMX_EXTRADATA_ENABLE);
                                  QOMX_EXTRADATA_ENABLE *pParam = (QOMX_EXTRADATA_ENABLE *)paramData;
                                  DEBUG_PRINT_LOW("set_parameter: OMX_QTIIndexParamVideoClientExtradata %d", pParam->bEnable);

                                  if (m_state != OMX_StateLoaded) {
                                      DEBUG_PRINT_ERROR("Set Parameter called in Invalid state");
                                      return OMX_ErrorIncorrectStateOperation;
                                  }

                                  if (pParam->nPortIndex == OMX_CORE_OUTPUT_EXTRADATA_INDEX) {
                                      m_client_out_extradata_info.enable_client_extradata(pParam->bEnable);
                                  } else {
                                      DEBUG_PRINT_ERROR("Incorrect portIndex - %d", pParam->nPortIndex);
                                      eRet = OMX_ErrorUnsupportedIndex;
                                  }
                                  break;
                              }
        case OMX_IndexParamStandardComponentRole: {
                                  VALIDATE_OMX_PARAM_DATA(paramData, OMX_PARAM_COMPONENTROLETYPE);
                                  OMX_PARAM_COMPONENTROLETYPE *comp_role;
                                  comp_role = (OMX_PARAM_COMPONENTROLETYPE *) paramData;
                                  DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamStandardComponentRole %s",
                                          comp_role->cRole);

                                  if ((m_state == OMX_StateLoaded)&&
                                          !BITMASK_PRESENT(&m_flags, OMX_COMPONENT_IDLE_PENDING)) {
                                      DEBUG_PRINT_LOW("Set Parameter called in valid state");
                                  } else {
                                      DEBUG_PRINT_ERROR("Set Parameter called in Invalid State");
                                      return OMX_ErrorIncorrectStateOperation;
                                  }

                                  if (!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.avc", OMX_MAX_STRINGNAME_SIZE)) {
                                      if (!strncmp((char*)comp_role->cRole, "video_decoder.avc", OMX_MAX_STRINGNAME_SIZE)) {
                                          strlcpy((char*)m_cRole, "video_decoder.avc", OMX_MAX_STRINGNAME_SIZE);
                                      } else {
                                          DEBUG_PRINT_ERROR("Setparameter: unknown Index %s", comp_role->cRole);
                                          eRet =OMX_ErrorUnsupportedSetting;
                                      }
                                  } else if (!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.mpeg2", OMX_MAX_STRINGNAME_SIZE)) {
                                      if (!strncmp((const char*)comp_role->cRole, "video_decoder.mpeg2", OMX_MAX_STRINGNAME_SIZE)) {
                                          strlcpy((char*)m_cRole, "video_decoder.mpeg2", OMX_MAX_STRINGNAME_SIZE);
                                      } else {
                                          DEBUG_PRINT_ERROR("Setparameter: unknown Index %s", comp_role->cRole);
                                          eRet = OMX_ErrorUnsupportedSetting;
                                      }
                                  } else if (!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.vp8", OMX_MAX_STRINGNAME_SIZE)) {
                                      if (!strncmp((const char*)comp_role->cRole, "video_decoder.vp8", OMX_MAX_STRINGNAME_SIZE) ||
                                              !strncmp((const char*)comp_role->cRole, "video_decoder.vpx", OMX_MAX_STRINGNAME_SIZE)) {
                                          strlcpy((char*)m_cRole, "video_decoder.vp8", OMX_MAX_STRINGNAME_SIZE);
                                      } else {
                                          DEBUG_PRINT_ERROR("Setparameter: unknown Index %s", comp_role->cRole);
                                          eRet = OMX_ErrorUnsupportedSetting;
                                      }
                                  } else if (!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.vp9", OMX_MAX_STRINGNAME_SIZE)) {
                                      if (!strncmp((const char*)comp_role->cRole, "video_decoder.vp9", OMX_MAX_STRINGNAME_SIZE) ||
                                              !strncmp((const char*)comp_role->cRole, "video_decoder.vpx", OMX_MAX_STRINGNAME_SIZE)) {
                                          strlcpy((char*)m_cRole, "video_decoder.vp9", OMX_MAX_STRINGNAME_SIZE);
                                      } else {
                                          DEBUG_PRINT_ERROR("Setparameter: unknown Index %s", comp_role->cRole);
                                          eRet = OMX_ErrorUnsupportedSetting;
                                      }
                                  } else if (!strncmp(drv_ctx.kind, "OMX.qcom.video.decoder.hevc", OMX_MAX_STRINGNAME_SIZE)) {
                                      if (!strncmp((const char*)comp_role->cRole, "video_decoder.hevc", OMX_MAX_STRINGNAME_SIZE)) {
                                          strlcpy((char*)m_cRole, "video_decoder.hevc", OMX_MAX_STRINGNAME_SIZE);
                                      } else {
                                          DEBUG_PRINT_ERROR("Setparameter: unknown Index %s", comp_role->cRole);
                                          eRet = OMX_ErrorUnsupportedSetting;
                                      }
                                  } else {
                                      DEBUG_PRINT_ERROR("Setparameter: unknown param %s", drv_ctx.kind);
                                      eRet = OMX_ErrorInvalidComponentName;
                                  }
                                  break;
                              }

        case OMX_IndexParamPriorityMgmt: {
                             VALIDATE_OMX_PARAM_DATA(paramData, OMX_PRIORITYMGMTTYPE);
                             if (m_state != OMX_StateLoaded) {
                                 DEBUG_PRINT_ERROR("Set Parameter called in Invalid State");
                                 return OMX_ErrorIncorrectStateOperation;
                             }
                             OMX_PRIORITYMGMTTYPE *priorityMgmtype = (OMX_PRIORITYMGMTTYPE*) paramData;
                             DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamPriorityMgmt %u",
                                     (unsigned int)priorityMgmtype->nGroupID);

                             DEBUG_PRINT_LOW("set_parameter: priorityMgmtype %u",
                                     (unsigned int)priorityMgmtype->nGroupPriority);

                             m_priority_mgm.nGroupID = priorityMgmtype->nGroupID;
                             m_priority_mgm.nGroupPriority = priorityMgmtype->nGroupPriority;

                             break;
                         }

        case OMX_IndexParamCompBufferSupplier: {
                                   VALIDATE_OMX_PARAM_DATA(paramData, OMX_PARAM_BUFFERSUPPLIERTYPE);
                                   OMX_PARAM_BUFFERSUPPLIERTYPE *bufferSupplierType = (OMX_PARAM_BUFFERSUPPLIERTYPE*) paramData;
                                   DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamCompBufferSupplier %d",
                                           bufferSupplierType->eBufferSupplier);
                                   if (bufferSupplierType->nPortIndex == 0 || bufferSupplierType->nPortIndex ==1)
                                       m_buffer_supplier.eBufferSupplier = bufferSupplierType->eBufferSupplier;

                                   else

                                       eRet = OMX_ErrorBadPortIndex;

                                   break;

                               }
        case OMX_IndexParamVideoAvc: {
                             DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamVideoAvc %d",
                                     paramIndex);
                             break;
                         }
        case (OMX_INDEXTYPE)QOMX_IndexParamVideoMvc: {
                            DEBUG_PRINT_LOW("set_parameter: QOMX_IndexParamVideoMvc %d",
                                     paramIndex);
                             break;
                        }
        case OMX_IndexParamVideoMpeg2: {
                               DEBUG_PRINT_LOW("set_parameter: OMX_IndexParamVideoMpeg2 %d",
                                       paramIndex);
                               break;
                           }
        case OMX_QTIIndexParamLowLatencyMode: {
                               struct v4l2_control control;
                               int rc = 0;
                               QOMX_EXTNINDEX_VIDEO_LOW_LATENCY_MODE* pParam =
                                   (QOMX_EXTNINDEX_VIDEO_LOW_LATENCY_MODE*)paramData;
                                control.id = V4L2_CID_MPEG_VIDC_VIDEO_LOWLATENCY_MODE;
                                if (pParam->bEnableLowLatencyMode)
                                    control.value = V4L2_MPEG_MSM_VIDC_ENABLE;
                                else
                                    control.value = V4L2_MPEG_MSM_VIDC_DISABLE;

                                rc = ioctl(drv_ctx.video_driver_fd, VIDIOC_S_CTRL, &control);
                                if (rc) {
                                    DEBUG_PRINT_ERROR("Set low latency failed");
                                    eRet = OMX_ErrorUnsupportedSetting;
                                } else {
                                    m_sParamLowLatency.bEnableLowLatencyMode = pParam->bEnableLowLatencyMode;
                                }
                               break;
                           }
        case OMX_QcomIndexParamVideoDecoderPictureOrder: {
                                     VALIDATE_OMX_PARAM_DATA(paramData, QOMX_VIDEO_DECODER_PICTURE_ORDER);
                                     QOMX_VIDEO_DECODER_PICTURE_ORDER *pictureOrder =
                                         (QOMX_VIDEO_DECODER_PICTURE_ORDER *)paramData;
                                     struct v4l2_control control;
                                     int pic_order,rc=0;
                                     DEBUG_PRINT_HIGH("set_parameter: OMX_QcomIndexParamVideoDecoderPictureOrder %d",
                                             pictureOrder->eOutputPictureOrder);
                                     if (pictureOrder->eOutputPictureOrder == QOMX_VIDEO_DISPLAY_ORDER) {
                                         pic_order = V4L2_MPEG_MSM_VIDC_DISABLE;
                                     } else if (pictureOrder->eOutputPictureOrder == QOMX_VIDEO_DECODE_ORDER) {
                                         pic_order = V4L2_MPEG_MSM_VIDC_ENABLE;
                                         time_stamp_dts.set_timestamp_reorder_mode(false);
                                     } else
                                         eRet = OMX_ErrorBadParameter;
                                     if (eRet == OMX_ErrorNone) {
                                         control.id = V4L2_CID_MPEG_VIDC_VIDEO_DECODE_ORDER;
                                         control.value = pic_order;
                                         rc = ioctl(drv_ctx.video_driver_fd, VIDIOC_S_CTRL, &control);
                                         if (rc) {
                                             DEBUG_PRINT_ERROR("Set picture order failed");
                                             eRet = OMX_ErrorUnsupportedSetting;
                                         }
                                     }
                                     m_decode_order_mode =
                                            pictureOrder->eOutputPictureOrder == QOMX_VIDEO_DECODE_ORDER;
                                     break;
                                 }
        case OMX_QcomIndexParamVideoSyncFrameDecodingMode: {
                                       DEBUG_PRINT_HIGH("set_parameter: OMX_QcomIndexParamVideoSyncFrameDecodingMode");
                                       DEBUG_PRINT_HIGH("set idr only decoding for thumbnail mode");
                                       struct v4l2_control control;
                                       int rc;
                                       drv_ctx.idr_only_decoding = 1;
                                       control.id = V4L2_CID_MPEG_VIDC_VIDEO_DECODE_ORDER;
                                       control.value = V4L2_MPEG_MSM_VIDC_ENABLE;
                                       rc = ioctl(drv_ctx.video_driver_fd, VIDIOC_S_CTRL, &control);
                                       if (rc) {
                                           DEBUG_PRINT_ERROR("Set picture order failed");
                                           eRet = OMX_ErrorUnsupportedSetting;
                                       } else {
                                           control.id = V4L2_CID_MPEG_VIDC_VIDEO_SYNC_FRAME_DECODE;
                                           control.value = V4L2_MPEG_MSM_VIDC_ENABLE;
                                           rc = ioctl(drv_ctx.video_driver_fd, VIDIOC_S_CTRL, &control);
                                           if (rc) {
                                               DEBUG_PRINT_ERROR("Sync frame setting failed");
                                               eRet = OMX_ErrorUnsupportedSetting;
                                           }
                                           /*Setting sync frame decoding on driver might change buffer
                                            * requirements so update them here*/
                                           if (get_buffer_req(&drv_ctx.ip_buf)) {
                                               DEBUG_PRINT_ERROR("Sync frame setting failed: falied to get buffer i/p requirements");
                                               eRet = OMX_ErrorUnsupportedSetting;
                                           }
                                           if (get_buffer_req(&drv_ctx.op_buf)) {
                                               DEBUG_PRINT_ERROR("Sync frame setting failed: falied to get buffer o/p requirements");
                                               eRet = OMX_ErrorUnsupportedSetting;
                                           }
                                       }
                                   }
                                   break;
        case OMX_QcomIndexParamIndexExtraDataType: {
            VALIDATE_OMX_PARAM_DATA(paramData, QOMX_INDEXEXTRADATATYPE);
            QOMX_INDEXEXTRADATATYPE *extradataIndexType = (QOMX_INDEXEXTRADATATYPE *) paramData;
            DEBUG_PRINT_LOW("set_parameter: OMX_QcomIndexParamIndexExtraDataType %d", extradataIndexType->nIndex);
            if (extradataIndexType->nIndex == (OMX_INDEXTYPE)OMX_QTI_ExtraDataCategory_Basic) {
                m_client_extradata |= EXTRADATA_DEFAULT;
            } else if (extradataIndexType->nIndex == (OMX_INDEXTYPE)OMX_QTI_ExtraDataCategory_Advanced) {
                m_client_extradata |= EXTRADATA_ADVANCED;
            }
            if (m_client_extradata) {
                eRet = enable_extradata(m_client_extradata);
            }
            break;
        }
#if defined (_ANDROID_HONEYCOMB_) || defined (_ANDROID_ICS_)
                                  /* Need to allow following two set_parameters even in Idle
                                   * state. This is ANDROID architecture which is not in sync
                                   * with openmax standard. */
        case OMX_GoogleAndroidIndexEnableAndroidNativeBuffers: {
                                           VALIDATE_OMX_PARAM_DATA(paramData, EnableAndroidNativeBuffersParams);
                                           EnableAndroidNativeBuffersParams* enableNativeBuffers = (EnableAndroidNativeBuffersParams *) paramData;
                                           if (enableNativeBuffers->nPortIndex != OMX_CORE_OUTPUT_PORT_INDEX) {
                                                DEBUG_PRINT_ERROR("Enable/Disable android-native-buffers allowed only on output port!");
                                                eRet = OMX_ErrorUnsupportedSetting;
                                                break;
                                           } else if (m_out_mem_ptr) {
                                                DEBUG_PRINT_ERROR("Enable/Disable android-native-buffers is not allowed since Output port is not free !");
                                                eRet = OMX_ErrorInvalidState;
                                                break;
                                           }
                                           if (enableNativeBuffers) {
                                               m_enable_android_native_buffers = enableNativeBuffers->enable;
                                           }
#if !defined(FLEXYUV_SUPPORTED)
                                           if (m_enable_android_native_buffers) {
                                               // Use the most-preferred-native-color-format as surface-mode is hinted here
                                               if(!client_buffers.set_color_format(getPreferredColorFormatDefaultMode(0))) {
                                                   DEBUG_PRINT_ERROR("Failed to set native color format!");
                                                   eRet = OMX_ErrorUnsupportedSetting;
                                               }
                                           }
#endif
                                       }
                                       break;
        case OMX_GoogleAndroidIndexUseAndroidNativeBuffer: {
                                       VALIDATE_OMX_PARAM_DATA(paramData, UseAndroidNativeBufferParams);
                                       eRet = use_android_native_buffer(hComp, paramData);
                                   }
                                   break;
#if ALLOCATE_OUTPUT_NATIVEHANDLE
        case OMX_GoogleAndroidIndexAllocateNativeHandle: {

                AllocateNativeHandleParams* allocateNativeHandleParams = (AllocateNativeHandleParams *) paramData;
                VALIDATE_OMX_PARAM_DATA(paramData, AllocateNativeHandleParams);

                if (allocateNativeHandleParams->nPortIndex != OMX_CORE_INPUT_PORT_INDEX) {
                    DEBUG_PRINT_LOW("Enable/Disable allocate-native-handle allowed only on input port!. Please ignore this Unsupported Setting (0x80001019).");
                    eRet = OMX_ErrorUnsupportedSetting;
                    break;
                } else if (m_inp_mem_ptr) {
                    DEBUG_PRINT_ERROR("Enable/Disable allocate-native-handle is not allowed since Input port is not free !");
                    eRet = OMX_ErrorInvalidState;
                    break;
                }

                if (allocateNativeHandleParams != NULL) {
                    allocate_native_handle = allocateNativeHandleParams->enable;
                }
            }
            break;
#endif //ALLOCATE_OUTPUT_NATIVEHANDLE
#endif
        case OMX_QcomIndexParamEnableTimeStampReorder: {
                                       VALIDATE_OMX_PARAM_DATA(paramData, QOMX_INDEXTIMESTAMPREORDER);
                                       QOMX_INDEXTIMESTAMPREORDER *reorder = (QOMX_INDEXTIMESTAMPREORDER *)paramData;
                                       if (drv_ctx.picture_order == (vdec_output_order)QOMX_VIDEO_DISPLAY_ORDER) {
                                           if (reorder->bEnable == OMX_TRUE) {
                                               frm_int =0;
                                               time_stamp_dts.set_timestamp_reorder_mode(true);
                                           } else
                                               time_stamp_dts.set_timestamp_reorder_mode(false);
                                       } else {
                                           time_stamp_dts.set_timestamp_reorder_mode(false);
                                           if (reorder->bEnable == OMX_TRUE) {
                                               eRet = OMX_ErrorUnsupportedSetting;
                                           }
                                       }
                                   }
                                   break;
        case OMX_IndexParamVideoProfileLevelCurrent: {
            VALIDATE_OMX_PARAM_DATA(paramData, OMX_VIDEO_PARAM_PROFILELEVELTYPE);
            OMX_VIDEO_PARAM_PROFILELEVELTYPE *pParam = (OMX_VIDEO_PARAM_PROFILELEVELTYPE*)paramData;

            DEBUG_PRINT_LOW("set_parameter: Client set profile is: %d", pParam->eProfile);
            DEBUG_PRINT_LOW("set_parameter: Client set level is: %d", pParam->eLevel);
            clientSet_profile_level.eProfile = pParam->eProfile;
            clientSet_profile_level.eLevel = pParam->eLevel;
            break;
        }
        case OMX_QTIIndexParamVideoDecoderOutputFrameRate:
        {
            VALIDATE_OMX_PARAM_DATA(paramData, QOMX_VIDEO_OUTPUT_FRAME_RATE);
            DEBUG_PRINT_LOW("set_parameter: OMX_QTIIndexParamVideoDecoderOutputFrameRate");
            QOMX_VIDEO_OUTPUT_FRAME_RATE *pParam = (QOMX_VIDEO_OUTPUT_FRAME_RATE*)paramData;
            DEBUG_PRINT_LOW("set_parameter: decoder output-frame-rate %d", pParam->fps);
            m_dec_hfr_fps=pParam->fps;
            if (m_dec_hfr_fps > m_dec_output_rate)
                m_dec_hfr_fps = m_dec_output_rate;

            DEBUG_PRINT_HIGH("output-frame-rate value = %d", m_dec_hfr_fps);
            break;
        }
        case OMX_QcomIndexParamVideoMetaBufferMode:
        {
            VALIDATE_OMX_PARAM_DATA(paramData, StoreMetaDataInBuffersParams);
            StoreMetaDataInBuffersParams *metabuffer =
                (StoreMetaDataInBuffersParams *)paramData;
            if (!metabuffer) {
                DEBUG_PRINT_ERROR("Invalid param: %p", metabuffer);
                eRet = OMX_ErrorBadParameter;
                break;
            }
            if (m_disable_dynamic_buf_mode) {
                DEBUG_PRINT_HIGH("Dynamic buffer mode is disabled");
                eRet = OMX_ErrorUnsupportedSetting;
                break;
            }
            if (metabuffer->nPortIndex == OMX_CORE_OUTPUT_PORT_INDEX) {

                if (m_out_mem_ptr) {
                    DEBUG_PRINT_ERROR("Enable/Disable dynamic-buffer-mode is not allowed since Output port is not free !");
                    eRet = OMX_ErrorInvalidState;
                    break;
                }

                dynamic_buf_mode = metabuffer->bStoreMetaData;
                DEBUG_PRINT_HIGH("%s buffer mode",
                    (metabuffer->bStoreMetaData == true)? "Enabled dynamic" : "Disabled dynamic");

            } else {
                DEBUG_PRINT_ERROR(
                   "OMX_QcomIndexParamVideoMetaBufferMode not supported for port: %u",
                   (unsigned int)metabuffer->nPortIndex);
                eRet = OMX_ErrorUnsupportedSetting;
            }
            break;
        }
        case OMX_QcomIndexParamVideoCustomBufferSize:
        {
            VALIDATE_OMX_PARAM_DATA(paramData, QOMX_VIDEO_CUSTOM_BUFFERSIZE);
            DEBUG_PRINT_LOW("set_parameter: OMX_QcomIndexParamVideoCustomBufferSize");
            QOMX_VIDEO_CUSTOM_BUFFERSIZE* pParam = (QOMX_VIDEO_CUSTOM_BUFFERSIZE*)paramData;
            if (pParam->nPortIndex == OMX_CORE_INPUT_PORT_INDEX) {
                struct v4l2_control control;
                control.id = V4L2_CID_MPEG_VIDC_VIDEO_BUFFER_SIZE_LIMIT;
                control.value = pParam->nBufferSize;
                if (ioctl(drv_ctx.video_driver_fd, VIDIOC_S_CTRL, &control)) {
                    DEBUG_PRINT_ERROR("Failed to set input buffer size");
                    eRet = OMX_ErrorUnsupportedSetting;
                } else {
                    eRet = get_buffer_req(&drv_ctx.ip_buf);
                    if (eRet == OMX_ErrorNone) {
                        m_custom_buffersize.input_buffersize = drv_ctx.ip_buf.buffer_size;
                        DEBUG_PRINT_HIGH("Successfully set custom input buffer size = %d",
                            m_custom_buffersize.input_buffersize);
                    } else {
                        DEBUG_PRINT_ERROR("Failed to get buffer requirement");
                    }
                }
            } else {
                DEBUG_PRINT_ERROR("ERROR: Custom buffer size in not supported on output port");
                eRet = OMX_ErrorBadParameter;
            }
            break;
        }
        case OMX_QTIIndexParamPassInputBufferFd:
        {
            VALIDATE_OMX_PARAM_DATA(paramData, QOMX_ENABLETYPE);
            m_input_pass_buffer_fd = ((QOMX_ENABLETYPE *)paramData)->bEnable;
            if (m_input_pass_buffer_fd)
                DEBUG_PRINT_LOW("Enable passing input buffer FD");
            break;
        }
        case OMX_QTIIndexParamForceUnCompressedForOPB:
        {
            DEBUG_PRINT_LOW("set_parameter: OMX_QTIIndexParamForceUnCompressedForOPB");
            OMX_QTI_VIDEO_PARAM_FORCE_UNCOMPRESSED_FOR_OPB_TYPE *pParam =
                (OMX_QTI_VIDEO_PARAM_FORCE_UNCOMPRESSED_FOR_OPB_TYPE *)paramData;
            if (!paramData) {
                DEBUG_PRINT_ERROR("set_parameter: OMX_QTIIndexParamForceUnCompressedForOPB paramData is NULL");
                eRet = OMX_ErrorBadParameter;
                break;
            }
            m_disable_ubwc_mode = pParam->bEnable;
            DEBUG_PRINT_LOW("set_parameter: UBWC %s for OPB", pParam->bEnable ? "disabled" : "enabled");
            break;
        }
        default: {
                 DEBUG_PRINT_ERROR("Setparameter: unknown param %d", paramIndex);
                 eRet = OMX_ErrorUnsupportedIndex;
             }
    }
    if (eRet != OMX_ErrorNone)
        DEBUG_PRINT_ERROR("set_parameter: Error: 0x%x, setting param 0x%x", eRet, paramIndex);
    return eRet;
}

