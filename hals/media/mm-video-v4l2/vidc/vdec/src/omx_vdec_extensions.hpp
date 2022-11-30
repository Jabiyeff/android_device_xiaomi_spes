/*--------------------------------------------------------------------------
Copyright (c) 2017, 2019, The Linux Foundation. All rights reserved.

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

void omx_vdec::init_vendor_extensions (VendorExtensionStore &store) {

    //TODO: add extensions based on Codec, m_platform and/or other capability queries

    ADD_EXTENSION("qti-ext-dec-picture-order", OMX_QcomIndexParamVideoDecoderPictureOrder, OMX_DirOutput)
    ADD_PARAM_END("enable", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-dec-low-latency", OMX_QTIIndexParamLowLatencyMode, OMX_DirOutput)
    ADD_PARAM_END("enable", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-extradata-enable", OMX_QcomIndexParamIndexExtraDataType, OMX_DirOutput)
    ADD_PARAM_END("types", OMX_AndroidVendorValueString)

    ADD_EXTENSION("qti-ext-dec-caps-vt-driver-version", OMX_QTIIndexParamCapabilitiesVTDriverVersion, OMX_DirOutput)
    ADD_PARAM_END("number", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-dec-output-frame-rate", OMX_QTIIndexParamVideoDecoderOutputFrameRate, OMX_DirOutput)
    ADD_PARAM_END("value", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-dec-thumbnail-mode", OMX_QcomIndexParamVideoSyncFrameDecodingMode, OMX_DirOutput)
    ADD_PARAM_END("value", OMX_AndroidVendorValueInt32)
}


OMX_ERRORTYPE omx_vdec::get_vendor_extension_config(
                OMX_CONFIG_ANDROID_VENDOR_EXTENSIONTYPE *ext) {
    if (ext->nIndex >= mVendorExtensionStore.size()) {
        return OMX_ErrorNoMore;
    }

    const VendorExtension& vExt = mVendorExtensionStore[ext->nIndex];
    DEBUG_PRINT_LOW("VendorExt: getConfig: index=%u (%s)", ext->nIndex, vExt.name());

    vExt.copyInfoTo(ext);
    if (ext->nParamSizeUsed < vExt.paramCount()) {
        // this happens during initial getConfig to query only extension-name and param-count
        return OMX_ErrorNone;
    }

    // We now have sufficient params allocated in extension data passed.
    // Following code is to set the extension-specific data

    bool setStatus = true;

    switch ((OMX_U32)vExt.extensionIndex()) {
        case OMX_QcomIndexParamVideoDecoderPictureOrder:
        {
            setStatus &= vExt.setParamInt32(ext, "enable", m_decode_order_mode);
            break;
        }
        case OMX_QTIIndexParamLowLatencyMode:
        {
            setStatus &= vExt.setParamInt32(ext, "enable", m_sParamLowLatency.bEnableLowLatencyMode);
            break;
        }
        case OMX_QcomIndexParamIndexExtraDataType:
        {
            char exType[OMX_MAX_STRINGVALUE_SIZE + 1];
            memset (exType, 0, (sizeof(char)*OMX_MAX_STRINGVALUE_SIZE));
            strlcat(exType, "basic", OMX_MAX_STRINGVALUE_SIZE);

            if (m_client_extradata & EXTRADATA_ADVANCED) {
                strlcat(exType, "|advanced", OMX_MAX_STRINGVALUE_SIZE);
            }

            setStatus &= vExt.setParamString(ext, "types", exType);
            DEBUG_PRINT_LOW("VendorExt: getparam: Extradata %s", exType);
            break;
        }
        case OMX_QTIIndexParamCapabilitiesVTDriverVersion:
        {
            setStatus &= vExt.setParamInt32(ext, "number", VT_DRIVER_VERSION);
            break;
        }
        case OMX_QTIIndexParamVideoDecoderOutputFrameRate:
        {
            setStatus &= vExt.setParamInt32(ext, "value", m_dec_hfr_fps);
            break;
        }
        case OMX_QcomIndexParamVideoSyncFrameDecodingMode:
        {
            setStatus &= vExt.setParamInt32(ext, "value", drv_ctx.idr_only_decoding);
            break;
        }
        default:
        {
            return OMX_ErrorNotImplemented;
        }
    }
    return setStatus ? OMX_ErrorNone : OMX_ErrorUndefined;
}

OMX_ERRORTYPE omx_vdec::set_vendor_extension_config(
                OMX_CONFIG_ANDROID_VENDOR_EXTENSIONTYPE *ext) {

    DEBUG_PRINT_LOW("set_vendor_extension_config");
    if (ext->nIndex >= mVendorExtensionStore.size()) {
        DEBUG_PRINT_ERROR("unrecognized vendor extension index (%u) max(%u)",
                ext->nIndex, mVendorExtensionStore.size());
        return OMX_ErrorBadParameter;
    }

    const VendorExtension& vExt = mVendorExtensionStore[ext->nIndex];
    DEBUG_PRINT_LOW("VendorExt: setConfig: index=%u (%s)", ext->nIndex, vExt.name());

    OMX_ERRORTYPE err = OMX_ErrorNone;
    err = vExt.isConfigValid(ext);
    if (err != OMX_ErrorNone) {
        return err;
    }

    // mark this as set, regardless of set_config succeeding/failing.
    // App will know by inconsistent values in output-format
    vExt.set();

    bool valueSet = false;
    switch ((OMX_U32)vExt.extensionIndex()) {
        case OMX_QcomIndexParamVideoDecoderPictureOrder:
        {
            OMX_S32 pic_order_enable = 0;
            valueSet |= vExt.readParamInt32(ext, "enable", &pic_order_enable);
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: OMX_QcomIndexParamVideoDecoderPictureOrder : %d",
                    pic_order_enable);

            QOMX_VIDEO_DECODER_PICTURE_ORDER decParam;
            OMX_INIT_STRUCT(&decParam, QOMX_VIDEO_DECODER_PICTURE_ORDER);
            decParam.eOutputPictureOrder =
                    pic_order_enable ? QOMX_VIDEO_DECODE_ORDER : QOMX_VIDEO_DISPLAY_ORDER;
            decParam.nPortIndex = OMX_DirOutput;

            err = set_parameter(
                    NULL, (OMX_INDEXTYPE)OMX_QcomIndexParamVideoDecoderPictureOrder, &decParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_QcomIndexParamVideoDecoderPictureOrder failed !");
            }
            break;
        }
        case OMX_QTIIndexParamLowLatencyMode:
        {
            QOMX_EXTNINDEX_VIDEO_LOW_LATENCY_MODE lowLatency;
            memcpy(&lowLatency, &m_sParamLowLatency, sizeof(QOMX_EXTNINDEX_VIDEO_LOW_LATENCY_MODE));
            valueSet |= vExt.readParamInt32(ext, "enable", (OMX_S32 *)&(lowLatency.bEnableLowLatencyMode));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_param: low latency mode =%u", lowLatency.bEnableLowLatencyMode);
            err = set_parameter(
                    NULL, (OMX_INDEXTYPE)OMX_QTIIndexParamLowLatencyMode, &lowLatency);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_param: OMX_QTIIndexParamLowLatencyMode failed !");
            }

            break;
        }
        case  OMX_QcomIndexParamIndexExtraDataType:
        {
            QOMX_INDEXEXTRADATATYPE extraDataParam;
            char exType[OMX_MAX_STRINGVALUE_SIZE];
            OMX_INIT_STRUCT(&extraDataParam, QOMX_INDEXEXTRADATATYPE);
            valueSet |= vExt.readParamString(ext, "types", exType);
            if (!valueSet) {
                break;
            }
            char *rest = exType;
            char *token = NULL;
            while ((token = strtok_r(rest, "|", &rest))) {
                extraDataParam.nPortIndex = OMX_CORE_OUTPUT_PORT_INDEX;
                extraDataParam.bEnabled = OMX_TRUE;
                if (!strcmp(token, "basic")) {
                    extraDataParam.nIndex = (OMX_INDEXTYPE)OMX_QTI_ExtraDataCategory_Basic;
                } else if (!strcmp(token, "advanced")) {
                    extraDataParam.nIndex = (OMX_INDEXTYPE)OMX_QTI_ExtraDataCategory_Advanced;
                } else {
                    DEBUG_PRINT_HIGH("extradata %s not supported", token);
                    continue;
                }
                DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: extradata: enable for index = %x",
                        extraDataParam.nIndex);
                err = set_parameter(
                    NULL, (OMX_INDEXTYPE)OMX_QcomIndexParamIndexExtraDataType, &extraDataParam);
                if (err != OMX_ErrorNone) {
                    DEBUG_PRINT_ERROR("set_config: OMX_QcomIndexParamIndexExtraDataType failed !");
                }
            }
            break;
        }
        case OMX_QTIIndexParamVideoDecoderOutputFrameRate:
        {
            QOMX_VIDEO_OUTPUT_FRAME_RATE decOutputFrameRateParam;
            OMX_INIT_STRUCT(&decOutputFrameRateParam, QOMX_VIDEO_OUTPUT_FRAME_RATE);
            valueSet |= vExt.readParamInt32(ext, "value", (OMX_S32 *)&decOutputFrameRateParam.fps);
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_param: decoder output-frame-rate value =%d", decOutputFrameRateParam.fps);
            err = set_parameter(
                    NULL, (OMX_INDEXTYPE)OMX_QTIIndexParamVideoDecoderOutputFrameRate, &decOutputFrameRateParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_param: OMX_QTIIndexParamVideoDecoderOutputFrameRate failed !");
            }
            break;
        }
        case OMX_QcomIndexParamVideoSyncFrameDecodingMode:
        {
            OMX_U32 thumbnail_mode = 0;
            valueSet |= vExt.readParamInt32(ext, "value", (OMX_S32 *)&thumbnail_mode);
            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: OMX_QcomIndexParamVideoSyncFrameDecodingMode : %d",
                    thumbnail_mode);
            if (!valueSet || !thumbnail_mode)
                break;
            err = set_parameter(
                    NULL, (OMX_INDEXTYPE)OMX_QcomIndexParamVideoSyncFrameDecodingMode, &thumbnail_mode);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_param: OMX_QcomIndexParamVideoSyncFrameDecodingMode failed !");
            }
            break;
        }
        case OMX_QTIIndexParamCapabilitiesVTDriverVersion:
        {
            break;
        }
        default:
        {
            return OMX_ErrorNotImplemented;
        }
    }

    return err;
}
