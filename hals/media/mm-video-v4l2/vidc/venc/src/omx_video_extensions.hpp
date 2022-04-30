/*--------------------------------------------------------------------------
Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.

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

void omx_video::init_vendor_extensions(VendorExtensionStore &store) {

    //TODO: add extensions based on Codec, m_platform and/or other capability queries

    ADD_EXTENSION("qti-ext-enc-preprocess-rotate", OMX_IndexConfigCommonRotate, OMX_DirOutput)
    ADD_PARAM_END("angle", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-preprocess-mirror", OMX_IndexConfigCommonMirror, OMX_DirOutput)
    ADD_PARAM_END("flip", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-avc-intra-period", QOMX_IndexConfigVideoIntraperiod, OMX_DirOutput)
    ADD_PARAM    ("n-pframes",    OMX_AndroidVendorValueInt32)
    ADD_PARAM_END("n-idr-period", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-intra-period", QOMX_IndexConfigVideoIntraperiod, OMX_DirOutput)
    ADD_PARAM    ("n-pframes",    OMX_AndroidVendorValueInt32)
    ADD_PARAM    ("n-bframes",    OMX_AndroidVendorValueInt32)
    ADD_PARAM_END("n-idr-period", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-error-correction", OMX_QcomIndexParamVideoSliceSpacing, OMX_DirOutput)
    ADD_PARAM_END("resync-marker-spacing-bits", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-slice", OMX_QcomIndexParamVideoSliceSpacing, OMX_DirOutput)
    ADD_PARAM_END("spacing", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-custom-profile-level", OMX_IndexParamVideoProfileLevelCurrent, OMX_DirOutput)
    ADD_PARAM    ("profile", OMX_AndroidVendorValueInt32)
    ADD_PARAM_END("level",   OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-timestamp-source-avtimer", OMX_QTIIndexParamEnableAVTimerTimestamps, OMX_DirInput)
    ADD_PARAM_END("enable", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-frame-qp", OMX_QcomIndexConfigQp, OMX_DirOutput)
    ADD_PARAM_END("value", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-down-scalar", OMX_QcomIndexParamVideoDownScalar, OMX_DirOutput)
    ADD_PARAM    ("output-width", OMX_AndroidVendorValueInt32)
    ADD_PARAM_END("output-height", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-app-input-control", OMX_QcomIndexParamVencControlInputQueue, OMX_DirInput)
    ADD_PARAM_END("enable", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-input-trigger", OMX_IndexConfigTimePosition, OMX_DirInput)
    ADD_PARAM_END("timestamp", OMX_AndroidVendorValueInt64)

    ADD_EXTENSION("qti-ext-enc-low-latency", OMX_QTIIndexParamLowLatencyMode, OMX_DirInput)
    ADD_PARAM_END("enable", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-base-layer-pid", OMX_QcomIndexConfigBaseLayerId, OMX_DirInput)
    ADD_PARAM_END("value", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-ltr-count", OMX_QcomIndexParamVideoLTRCount, OMX_DirOutput)
    ADD_PARAM_END("num-ltr-frames", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-ltr", OMX_QcomIndexConfigVideoLTRUse, OMX_DirInput)
    ADD_PARAM_END("use-frame", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-ltr", OMX_QcomIndexConfigVideoLTRMark, OMX_DirInput)
    ADD_PARAM_END("mark-frame", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-dynamic-frame-rate", OMX_IndexConfigVideoFramerate, OMX_DirOutput)
    ADD_PARAM_END("frame-rate", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-extradata-enable", OMX_QcomIndexParamIndexExtraDataType, OMX_DirOutput)
    ADD_PARAM_END("types", OMX_AndroidVendorValueString)

    ADD_EXTENSION("qti-ext-enc-caps-vt-driver-version", OMX_QTIIndexParamCapabilitiesVTDriverVersion, OMX_DirOutput)
    ADD_PARAM_END("number", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-caps-preprocess", OMX_QTIIndexParamCapabilitiesMaxDownScaleRatio, OMX_DirOutput)
    ADD_PARAM_END("max-downscale-factor", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-caps-preprocess", OMX_QTIIndexParamCapabilitiesRotationSupport, OMX_DirOutput)
    ADD_PARAM_END("rotation", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-caps-ltr", OMX_QTIIndexParamCapabilitiesMaxLTR, OMX_DirOutput)
    ADD_PARAM_END("max-count", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-caps-temporal-layers", OMX_QTIIndexParamCapabilitiesMaxTemporalLayers, OMX_DirInput)
    ADD_PARAM    ("max-p-count", OMX_AndroidVendorValueInt32)
    ADD_PARAM_END("max-b-count", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-colorspace-conversion", OMX_QTIIndexParamColorSpaceConversion, OMX_DirInput)
    ADD_PARAM_END("enable", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-initial-qp", QOMX_IndexParamVideoInitialQp, OMX_DirOutput)
    ADD_PARAM    ("qp-i", OMX_AndroidVendorValueInt32)
    ADD_PARAM    ("qp-i-enable", OMX_AndroidVendorValueInt32)
    ADD_PARAM    ("qp-p", OMX_AndroidVendorValueInt32)
    ADD_PARAM    ("qp-p-enable", OMX_AndroidVendorValueInt32)
    ADD_PARAM    ("qp-b", OMX_AndroidVendorValueInt32)
    ADD_PARAM_END("qp-b-enable", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-blurinfo", OMX_QTIIndexConfigVideoBlurResolution, OMX_DirInput)
    ADD_PARAM_END("info", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-qp-range", OMX_QcomIndexParamVideoIPBQPRange, OMX_DirOutput)
    ADD_PARAM    ("qp-i-min", OMX_AndroidVendorValueInt32)
    ADD_PARAM    ("qp-i-max", OMX_AndroidVendorValueInt32)
    ADD_PARAM    ("qp-p-min", OMX_AndroidVendorValueInt32)
    ADD_PARAM    ("qp-p-max", OMX_AndroidVendorValueInt32)
    ADD_PARAM    ("qp-b-min", OMX_AndroidVendorValueInt32)
    ADD_PARAM_END("qp-b-max", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-bitrate-mode", OMX_IndexParamVideoBitrate, OMX_DirOutput)
    ADD_PARAM_END("value", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-linear-color-format", OMX_QTIIndexParamEnableLinearColorFormat, OMX_DirInput)
    ADD_PARAM_END("value", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-vbvdelay", OMX_QTIIndexParamVbvDelay, OMX_DirInput)
    ADD_PARAM_END("value", OMX_AndroidVendorValueInt32)
    ADD_EXTENSION("qti-ext-enc-roiinfo", OMX_QTIIndexConfigVideoRoiRectRegionInfo, OMX_DirInput)
    ADD_PARAM    ("timestamp", OMX_AndroidVendorValueInt64)
    ADD_PARAM    ("type", OMX_AndroidVendorValueString)
    ADD_PARAM    ("rect-payload", OMX_AndroidVendorValueString)
    ADD_PARAM_END("rect-payload-ext", OMX_AndroidVendorValueString)

    ADD_EXTENSION("qti-ext-enc-roiinfo-rect-mode", OMX_QTIIndexConfigVideoRoiRectRegionInfo, OMX_DirOutput)
    ADD_PARAM_END("enable", OMX_AndroidVendorValueInt32)

    ADD_EXTENSION("qti-ext-enc-content-adaptive-mode", OMX_QTIIndexConfigContentAdaptiveCoding, OMX_DirInput)
    ADD_PARAM_END("value", OMX_AndroidVendorValueInt32)
}

OMX_ERRORTYPE omx_video::get_vendor_extension_config(
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
        case OMX_IndexConfigCommonRotate:
        {
            setStatus &= vExt.setParamInt32(ext, "angle", m_sConfigFrameRotation.nRotation);
            break;
        }
        case OMX_IndexConfigCommonMirror:
        {
            setStatus &= vExt.setParamInt32(ext, "flip", m_sConfigFrameMirror.eMirror);
            break;
        }
        case QOMX_IndexConfigVideoIntraperiod:
        {
            if (vExt.isConfigKey(ext, "qti-ext-enc-avc-intra-period")) {
                setStatus &= vExt.setParamInt32(ext, "n-pframes", m_sIntraperiod.nPFrames);
                setStatus &= vExt.setParamInt32(ext, "n-idr-period", m_sIntraperiod.nIDRPeriod);
            } else if (vExt.isConfigKey(ext, "qti-ext-enc-intra-period")) {
                setStatus &= vExt.setParamInt32(ext, "n-pframes", m_sIntraperiod.nPFrames);
                setStatus &= vExt.setParamInt32(ext, "n-bframes",  m_sIntraperiod.nBFrames);
                setStatus &= vExt.setParamInt32(ext, "n-idr-period", m_sIntraperiod.nIDRPeriod);
            }
            break;
        }
        case OMX_QcomIndexParamVideoSliceSpacing:
        {
            if (vExt.isConfigKey(ext, "qti-ext-enc-error-correction")) {
                setStatus &= vExt.setParamInt32(ext,
                        "resync-marker-spacing-bits", m_sSliceSpacing.nSliceSize);
            } else if (vExt.isConfigKey(ext, "qti-ext-enc-slice")) {
                setStatus &= vExt.setParamInt32(ext, "spacing", m_sSliceSpacing.nSliceSize);
            }
            break;
        }
        case OMX_IndexParamVideoProfileLevelCurrent:
        {
            setStatus &= vExt.setParamInt32(ext, "profile", m_sParamProfileLevel.eProfile);
            setStatus &= vExt.setParamInt32(ext, "level", m_sParamProfileLevel.eLevel);

            break;
        }
        case OMX_QTIIndexParamEnableAVTimerTimestamps:
        {
            setStatus &= vExt.setParamInt32(ext, "enable", m_sParamAVTimerTimestampMode.bEnable);
            break;
        }
        case OMX_QcomIndexConfigQp:
        {
            setStatus &= vExt.setParamInt32(ext, "value", m_sConfigQP.nQP);
            break;
        }
        case OMX_QcomIndexParamVideoDownScalar:
        {
            setStatus &= vExt.setParamInt32(ext, "output-width", m_sParamDownScalar.nOutputWidth);
            setStatus &= vExt.setParamInt32(ext, "output-height", m_sParamDownScalar.nOutputHeight);
            break;
        }
        case OMX_QcomIndexParamVencControlInputQueue:
        {
            setStatus &= vExt.setParamInt32(ext, "enable", m_sParamControlInputQueue.bEnable);
            break;
        }
        case OMX_IndexConfigTimePosition:
        {
            setStatus &= vExt.setParamInt64(ext, "timestamp", m_sConfigInputTrigTS.nTimestamp);
            break;
        }
        case OMX_QTIIndexParamLowLatencyMode:
        {
            setStatus &= vExt.setParamInt32(ext, "enable", m_sParamLowLatency.bEnableLowLatencyMode);
            break;
        }
        case OMX_QcomIndexConfigBaseLayerId:
        {
            setStatus &= vExt.setParamInt32(ext, "value", m_sBaseLayerID.nPID);
            break;
        }
        case OMX_QcomIndexParamVideoLTRCount:
        {
            setStatus &= vExt.setParamInt32(ext, "num-ltr-frames", m_sParamLTRCount.nCount);
            break;
        }
        case OMX_QcomIndexConfigVideoLTRUse:
        {
            setStatus &= vExt.setParamInt32(ext, "use-frame", m_sConfigLTRUse.nID);
            break;
        }
        case OMX_QcomIndexConfigVideoLTRMark:
        {
            setStatus &= vExt.setParamInt32(ext, "mark-frame", m_sConfigLTRMark.nID);
            break;
        }
        case OMX_IndexConfigVideoFramerate:
        {
            setStatus &= vExt.setParamInt32(ext, "frame-rate", m_sConfigFramerate.xEncodeFramerate);
            break;
        }
        case  OMX_QcomIndexParamIndexExtraDataType:
        {
            char exType[OMX_MAX_STRINGVALUE_SIZE+1];
            memset (exType,0, (sizeof(char)*OMX_MAX_STRINGVALUE_SIZE));
            if ((OMX_BOOL)(m_sExtraData & EXTRADATA_ADVANCED)) {
                strlcat(exType, "advanced", OMX_MAX_STRINGVALUE_SIZE);
            }
            if ((OMX_BOOL)(m_sExtraData & EXTRADATA_ENC_INPUT_ROI)) {
                if (exType[0]!=0) {
                    strlcat(exType,"|", OMX_MAX_STRINGVALUE_SIZE);
                }
                strlcat(exType, "roiinfo", OMX_MAX_STRINGVALUE_SIZE);
            }
            setStatus &= vExt.setParamString(ext, "types", exType);
            DEBUG_PRINT_LOW("VendorExt: getparam: Extradata %s",exType);
            break;
        }
        case OMX_QTIIndexParamCapabilitiesVTDriverVersion:
        {
            setStatus &= vExt.setParamInt32(ext, "number", VT_DRIVER_VERSION);
            break;
        }
        case OMX_QTIIndexParamCapabilitiesMaxLTR:
        {
            setStatus &= vExt.setParamInt32(ext, "max-count", 3);
            break;
        }
        case OMX_QTIIndexParamCapabilitiesMaxDownScaleRatio:
        {
            setStatus &= vExt.setParamInt32(ext, "max-downscale-factor", 8);
            break;
        }
        case OMX_QTIIndexParamCapabilitiesRotationSupport:
        {
            setStatus &= vExt.setParamInt32(ext, "rotation",1);
            break;
        }
        case OMX_QTIIndexParamCapabilitiesMaxTemporalLayers:
        {
            OMX_U32 nPLayerCountMax , nBLayerCountMax;
            OMX_VIDEO_ANDROID_TEMPORALLAYERINGPATTERNTYPE SupportedPattern;
            if (!dev_get_temporal_layer_caps(&nPLayerCountMax,
                        &nBLayerCountMax, &SupportedPattern)) {
                DEBUG_PRINT_ERROR("Failed to get temporal layer capabilities");
                break;
            }
            setStatus &= vExt.setParamInt32(ext, "max-p-count",nPLayerCountMax);
            setStatus &= vExt.setParamInt32(ext, "max-b-count",nBLayerCountMax);
            break;
        }
        case OMX_QTIIndexParamColorSpaceConversion:
        {
            setStatus &= vExt.setParamInt32(ext, "enable", m_sParamColorSpaceConversion.bEnable);
            break;
        }
        case QOMX_IndexParamVideoInitialQp:
        {
            setStatus &= vExt.setParamInt32(ext, "qp-i", m_sSessionQuantization.nQpI);
            setStatus &= vExt.setParamInt32(ext, "qp-p", m_sSessionQuantization.nQpP);
            setStatus &= vExt.setParamInt32(ext, "qp-b", m_sSessionQuantization.nQpB);
            setStatus &= vExt.setParamInt32(ext, "qp-i-enable", (m_QPSet & 0x1));
            setStatus &= vExt.setParamInt32(ext, "qp-p-enable", ((m_QPSet >> 0x1) & 0x1));
            setStatus &= vExt.setParamInt32(ext, "qp-b-enable", ((m_QPSet >> 0x2) & 0x1));
            break;
        }
        case OMX_QTIIndexParamVideoEnableBlur:
        case OMX_QTIIndexConfigVideoBlurResolution:
        case OMX_QTIIndexConfigContentAdaptiveCoding:
        {
            break;
        }
        case OMX_QcomIndexParamVideoIPBQPRange:
        {
            setStatus &= vExt.setParamInt32(ext, "qp-i-min", m_sSessionQPRange.minIQP);
            setStatus &= vExt.setParamInt32(ext, "qp-i-max", m_sSessionQPRange.maxIQP);
            setStatus &= vExt.setParamInt32(ext, "qp-p-min", m_sSessionQPRange.minPQP);
            setStatus &= vExt.setParamInt32(ext, "qp-p-max", m_sSessionQPRange.maxPQP);
            setStatus &= vExt.setParamInt32(ext, "qp-b-min", m_sSessionQPRange.minBQP);
            setStatus &= vExt.setParamInt32(ext, "qp-b-max", m_sSessionQPRange.maxBQP);
            break;
        }
        case OMX_IndexParamVideoBitrate:
        {
            setStatus &= vExt.setParamInt32(ext, "value", m_sParamBitrate.eControlRate);
            break;
        }
        case OMX_QTIIndexParamEnableLinearColorFormat:
        {
            setStatus &= vExt.setParamInt32(ext, "value", m_sParamLinearColorFormat.bEnable);
            break;
        }
        case OMX_QTIIndexParamVbvDelay:
        {
            setStatus &= vExt.setParamInt32(ext, "value", m_sParamVbvDelay.nVbvDelay);
            break;
        }
        case OMX_QTIIndexConfigVideoRoiRectRegionInfo:
        {
            if (vExt.paramCount() == 1) {
                setStatus &= vExt.setParamInt32(ext, "enable", 1);
            } else {
                setStatus &= vExt.setParamInt64(ext, "timestamp", 0);
                setStatus &= vExt.setParamString(ext, "type", "rect");
                setStatus &= vExt.setParamString(ext, "rect-payload", "");
                setStatus &= vExt.setParamString(ext, "rect-payload-ext", "");
            }
            break;
        }
        default:
        {
            return OMX_ErrorNotImplemented;
        }
    }
    return setStatus ? OMX_ErrorNone : OMX_ErrorUndefined;
}

OMX_ERRORTYPE omx_video::set_vendor_extension_config(
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
        case OMX_IndexConfigCommonRotate:
        {
            OMX_CONFIG_ROTATIONTYPE rotationParam;
            memcpy(&rotationParam, &m_sConfigFrameRotation, sizeof(OMX_CONFIG_ROTATIONTYPE));
            valueSet |= vExt.readParamInt32(ext, "angle", &rotationParam.nRotation);
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: OMX_IndexConfigCommonRotate : %d",
                    rotationParam.nRotation);

            err = set_config(
                    NULL, OMX_IndexConfigCommonRotate, &rotationParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_IndexConfigCommonRotate failed !");
            }
            break;
        }
        case OMX_IndexConfigCommonMirror:
        {
            OMX_CONFIG_MIRRORTYPE mirrorParam;
            memcpy(&mirrorParam, &m_sConfigFrameMirror, sizeof(OMX_CONFIG_MIRRORTYPE));
            valueSet |= vExt.readParamInt32(ext, "flip", (OMX_S32*)&mirrorParam.eMirror);
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: OMX_IndexConfigCommonMirror : %d",
                             mirrorParam.eMirror);

            err = set_config(
                    NULL, OMX_IndexConfigCommonMirror, &mirrorParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_IndexConfigCommonMirror failed !");
            }
            break;
        }
        case QOMX_IndexConfigVideoIntraperiod:
        {
            QOMX_VIDEO_INTRAPERIODTYPE intraPeriodConfig;
            memcpy(&intraPeriodConfig, &m_sIntraperiod, sizeof(QOMX_VIDEO_INTRAPERIODTYPE));

            if (vExt.isConfigKey(ext, "qti-ext-enc-avc-intra-period")) {
                valueSet |= vExt.readParamInt32(ext, "n-pframes", (OMX_S32 *)&(intraPeriodConfig.nPFrames));
                valueSet |= vExt.readParamInt32(ext, "n-idr-period", (OMX_S32 *)&(intraPeriodConfig.nIDRPeriod));

                DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: AVC-intra-period : nP=%d, nIDR=%d",
                    intraPeriodConfig.nPFrames, intraPeriodConfig.nIDRPeriod);
            } else if (vExt.isConfigKey(ext, "qti-ext-enc-intra-period")) {
                valueSet |= vExt.readParamInt32(ext, "n-bframes", (OMX_S32 *)&(intraPeriodConfig.nBFrames));
                valueSet |= vExt.readParamInt32(ext, "n-pframes", (OMX_S32 *)&(intraPeriodConfig.nPFrames));
                valueSet |= vExt.readParamInt32(ext, "n-idr-period", (OMX_S32 *)&(intraPeriodConfig.nIDRPeriod));

                DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: intra-period : nIDR=%d, nP=%d, nB=%d",
                intraPeriodConfig.nIDRPeriod, intraPeriodConfig.nPFrames, intraPeriodConfig.nBFrames);
            }
            if (!valueSet) {
                break;
            }

            err = set_config(
                    NULL, (OMX_INDEXTYPE)QOMX_IndexConfigVideoIntraperiod, &intraPeriodConfig);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: QOMX_IndexConfigVideoIntraperiod failed !");
            }
            break;
        }
        case OMX_QcomIndexParamVideoSliceSpacing:
        {
            QOMX_VIDEO_PARAM_SLICE_SPACING_TYPE sliceSpacing;
            memcpy(&sliceSpacing, &m_sSliceSpacing, sizeof(QOMX_VIDEO_PARAM_SLICE_SPACING_TYPE));

            if (vExt.isConfigKey(ext, "qti-ext-enc-error-correction")) {
                sliceSpacing.eSliceMode = QOMX_SLICEMODE_BYTE_COUNT;
                valueSet |= vExt.readParamInt32(ext,
                    "resync-marker-spacing-bits", (OMX_S32 *)&(sliceSpacing.nSliceSize));
                sliceSpacing.nSliceSize = ALIGN(sliceSpacing.nSliceSize, 8) >> 3;
            } else if (vExt.isConfigKey(ext, "qti-ext-enc-slice")) {
                sliceSpacing.eSliceMode = QOMX_SLICEMODE_MB_COUNT;
                valueSet |= vExt.readParamInt32(ext,
                    "spacing", (OMX_S32 *)&(sliceSpacing.nSliceSize));
            } else {
              DEBUG_PRINT_ERROR("VENDOR-EXT: set_config: Slice Spacing : Incorrect Mode !");
              break;
            }

            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: slice spacing : mode %d size %d",
                    sliceSpacing.eSliceMode, sliceSpacing.nSliceSize);

            err = set_parameter(
                    NULL, (OMX_INDEXTYPE)OMX_QcomIndexParamVideoSliceSpacing, &sliceSpacing);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_QcomIndexParamVideoSliceSpacing failed !");
            }
            break;
        }
        case OMX_IndexParamVideoProfileLevelCurrent:
        {
            OMX_VIDEO_PARAM_PROFILELEVELTYPE profileParam;
            memcpy(&profileParam, &m_sParamProfileLevel, sizeof(OMX_VIDEO_PARAM_PROFILELEVELTYPE));
            valueSet |= vExt.readParamInt32(ext, "profile", (OMX_S32 *)&(profileParam.eProfile));
            valueSet |= vExt.readParamInt32(ext, "level", (OMX_S32 *)&(profileParam.eLevel));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: custom-profile/level : profile=%u level=%u",
                    (OMX_U32)profileParam.eProfile, (OMX_U32)profileParam.eLevel);

            err = set_parameter(
                    NULL, OMX_IndexParamVideoProfileLevelCurrent, &profileParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_IndexParamVideoProfileLevelCurrent failed !");
            }

            break;
        }
        case OMX_QTIIndexParamEnableAVTimerTimestamps:
        {
            QOMX_ENABLETYPE avTimerEnableParam;
            memcpy(&avTimerEnableParam, &m_sParamAVTimerTimestampMode, sizeof(QOMX_ENABLETYPE));
            valueSet |= vExt.readParamInt32(ext, "enable", (OMX_S32 *)&(avTimerEnableParam.bEnable));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: AV-timer timestamp mode enable=%u", avTimerEnableParam.bEnable);

            err = set_parameter(
                    NULL, (OMX_INDEXTYPE)OMX_QTIIndexParamEnableAVTimerTimestamps, &avTimerEnableParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_param: OMX_QTIIndexParamEnableAVTimerTimestamps failed !");
            }

            break;
        }
        case OMX_QcomIndexConfigQp:
        {
            OMX_QCOM_VIDEO_CONFIG_QP qpConfig;
            memcpy(&qpConfig, &m_sConfigQP, sizeof(OMX_QCOM_VIDEO_CONFIG_QP));
            valueSet |= vExt.readParamInt32(ext, "value", (OMX_S32 *)&(qpConfig.nQP));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: nQP =%u", qpConfig.nQP);

            err = set_config(
                    NULL, (OMX_INDEXTYPE)OMX_QcomIndexConfigQp, &qpConfig);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_QcomIndexConfigQp failed !");
            }

            break;
        }
        case OMX_QcomIndexParamVideoDownScalar:
        {
            QOMX_INDEXDOWNSCALAR downScalarParam;
            memcpy(&downScalarParam, &m_sParamDownScalar, sizeof(QOMX_INDEXDOWNSCALAR));
            downScalarParam.bEnable = OMX_TRUE;
            valueSet |= vExt.readParamInt32(ext, "output-width", (OMX_S32 *)&(downScalarParam.nOutputWidth));
            valueSet |= vExt.readParamInt32(ext, "output-height", (OMX_S32 *)&(downScalarParam.nOutputHeight));
            if(!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: Downscalar Enable = %u Output Width = %u Output Height = %u",
                             downScalarParam.bEnable, downScalarParam.nOutputWidth, downScalarParam.nOutputHeight);

            err = set_parameter(NULL, (OMX_INDEXTYPE)OMX_QcomIndexParamVideoDownScalar, &downScalarParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_param: OMX_QcomIndexParamVideoDownScalar failed !");
            }
            break;
        }
        case OMX_QcomIndexParamVencControlInputQueue:
        {
            QOMX_ENABLETYPE controlInputQueueParam;
            memcpy(&controlInputQueueParam, &m_sParamControlInputQueue, sizeof(QOMX_ENABLETYPE));
            valueSet |= vExt.readParamInt32(ext, "enable", (OMX_S32 *)&(controlInputQueueParam.bEnable));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_param: control input queue enable=%u", controlInputQueueParam.bEnable);

            err = set_parameter(
                    NULL, (OMX_INDEXTYPE)OMX_QcomIndexParamVencControlInputQueue, &controlInputQueueParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_param: OMX_QcomIndexParamVencControlInputQueue failed !");
            }

            break;
        }
        case OMX_IndexConfigTimePosition:
        {
            OMX_TIME_CONFIG_TIMESTAMPTYPE triggerTimeStamp;
            memcpy(&triggerTimeStamp, &m_sConfigInputTrigTS, sizeof(OMX_TIME_CONFIG_TIMESTAMPTYPE));
            valueSet |= vExt.readParamInt64(ext, "timestamp", (OMX_S64 *)&(triggerTimeStamp.nTimestamp));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: trigger timestamp =%lld", triggerTimeStamp.nTimestamp);

            err = set_config(
                    NULL, (OMX_INDEXTYPE)OMX_IndexConfigTimePosition, &triggerTimeStamp);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_IndexConfigTimePosition failed !");
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
        case OMX_QcomIndexConfigBaseLayerId:
        {
            OMX_SKYPE_VIDEO_CONFIG_BASELAYERPID baselayerPID;
            memcpy(&baselayerPID, &m_sBaseLayerID, sizeof(OMX_SKYPE_VIDEO_CONFIG_BASELAYERPID));
            valueSet |= vExt.readParamInt32(ext, "value", (OMX_S32 *)&(baselayerPID.nPID));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: base layer pid =%u", baselayerPID.nPID);

            err = set_config(
                    NULL, (OMX_INDEXTYPE)OMX_QcomIndexConfigBaseLayerId, &baselayerPID);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_QcomIndexConfigBaseLayerId failed !");
            }

            break;
        }
        case OMX_QcomIndexParamVideoLTRCount:
        {
           QOMX_VIDEO_PARAM_LTRCOUNT_TYPE ltrCount;
           memcpy(&ltrCount, &m_sParamLTRCount, sizeof(QOMX_VIDEO_PARAM_LTRCOUNT_TYPE));
           valueSet |= vExt.readParamInt32(ext, "num-ltr-frames", (OMX_S32 *)&(ltrCount.nCount));
           if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_parameter: ltr count  =%u", ltrCount.nCount);

            err = set_parameter(
                    NULL, (OMX_INDEXTYPE)QOMX_IndexParamVideoLTRCount, &ltrCount);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_parameter: OMX_QcomIndexParamVideoLTRCount failed !");
            }

            break;
        }
        case OMX_QcomIndexConfigVideoLTRUse:
        {
           QOMX_VIDEO_CONFIG_LTRUSE_TYPE ltrUse;
           memcpy(&ltrUse, &m_sConfigLTRUse, sizeof(QOMX_VIDEO_CONFIG_LTRUSE_TYPE));
           valueSet |= vExt.readParamInt32(ext, "use-frame", (OMX_S32 *)&(ltrUse.nID));
           if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: ltr use  =%u", ltrUse.nID);

            err = set_config(
                    NULL, (OMX_INDEXTYPE)QOMX_IndexConfigVideoLTRUse, &ltrUse);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_QcomIndexConfigVideoLTRUse failed !");
            }

            break;
        }
        case OMX_QcomIndexConfigVideoLTRMark:
        {
           QOMX_VIDEO_CONFIG_LTRMARK_TYPE ltrMark;
           memcpy(&ltrMark, &m_sConfigLTRMark, sizeof(QOMX_VIDEO_CONFIG_LTRMARK_TYPE));
           valueSet |= vExt.readParamInt32(ext, "mark-frame", (OMX_S32 *)&(ltrMark.nID));
           if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: ltr mark  =%u", ltrMark.nID);

            err = set_config(
                    NULL, (OMX_INDEXTYPE)QOMX_IndexConfigVideoLTRMark, &ltrMark);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_QcomIndexConfigVideoLTRMark failed !");
            }

            break;
        }
        case OMX_IndexConfigVideoFramerate:
        {
            OMX_CONFIG_FRAMERATETYPE rateParam;
            memcpy(&rateParam, &m_sConfigFramerate, sizeof(OMX_CONFIG_FRAMERATETYPE));
            valueSet |= vExt.readParamInt32(ext, "frame-rate", (OMX_S32 *)&rateParam.xEncodeFramerate);
            if (!valueSet) {
                break;
            }
            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: OMX_IndexConfigVideoFramerate : %d",
                    rateParam.xEncodeFramerate);

            err = set_config(
                    NULL, OMX_IndexConfigVideoFramerate, &rateParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_IndexConfigVideoFramerate failed !");
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
                extraDataParam.bEnabled = OMX_TRUE;
                if (!strcmp(token, "advanced")) {
                    extraDataParam.nIndex = (OMX_INDEXTYPE)OMX_QTI_ExtraDataCategory_Advanced;
                    extraDataParam.nPortIndex = (OMX_U32)PORT_INDEX_OUT;
                } else if (!strcmp(token, "roiinfo")) {
                    extraDataParam.nIndex = (OMX_INDEXTYPE)OMX_QTI_ExtraDataCategory_Enc_ROI;
                    extraDataParam.nPortIndex = (OMX_U32)PORT_INDEX_IN;
                } else {
                    DEBUG_PRINT_HIGH("extradata %s not supported ", token);
                    continue;
                }
                DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: extradata: enable for index = %d",
                                  extraDataParam.nIndex);
                err = set_parameter(
                       NULL, (OMX_INDEXTYPE)OMX_QcomIndexParamIndexExtraDataType, &extraDataParam);
                if (err != OMX_ErrorNone) {
                    DEBUG_PRINT_ERROR("set_config: OMX_QcomIndexParamIndexExtraDataType failed !");
                }
            }
            break;
        }
        case OMX_QTIIndexParamColorSpaceConversion:
        {
            QOMX_ENABLETYPE colorspaceConversionParam;
            memcpy(&colorspaceConversionParam, &m_sParamColorSpaceConversion, sizeof(QOMX_ENABLETYPE));
            valueSet |= vExt.readParamInt32(ext, "enable", (OMX_S32 *)&(colorspaceConversionParam.bEnable));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_param: color space conversion enable=%u", colorspaceConversionParam.bEnable);

            err = set_parameter(
                   NULL, (OMX_INDEXTYPE)OMX_QTIIndexParamColorSpaceConversion, &colorspaceConversionParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_param: OMX_QTIIndexParamColorSpaceConversion failed !");
            }

            break;
        }
        case OMX_QTIIndexParamCapabilitiesVTDriverVersion:
        case OMX_QTIIndexParamCapabilitiesMaxLTR:
        case OMX_QTIIndexParamCapabilitiesMaxDownScaleRatio:
        case OMX_QTIIndexParamCapabilitiesRotationSupport:
        case OMX_QTIIndexParamCapabilitiesMaxTemporalLayers:
        {
            break;
        }
        case QOMX_IndexParamVideoInitialQp:
        {
            QOMX_EXTNINDEX_VIDEO_INITIALQP initQP;
            OMX_INIT_STRUCT(&initQP, QOMX_EXTNINDEX_VIDEO_INITIALQP);
            initQP.nPortIndex = (OMX_U32)PORT_INDEX_OUT;
            OMX_S32 enableIQp = (m_QPSet & 0x1);
            OMX_S32 enablePQp = ((m_QPSet >> 0x1) & 0x1);
            OMX_S32 enableBQp = ((m_QPSet >> 0x2) & 0x1);
            initQP.nQpI = m_sSessionQuantization.nQpI;
            initQP.nQpP = m_sSessionQuantization.nQpP;
            initQP.nQpB = m_sSessionQuantization.nQpB;
            initQP.bEnableInitQp = m_QPSet;
            valueSet |= vExt.readParamInt32(ext, "qp-i", (OMX_S32 *)&(initQP.nQpI));
            valueSet |= vExt.readParamInt32(ext, "qp-b", (OMX_S32 *)&(initQP.nQpB));
            valueSet |= vExt.readParamInt32(ext, "qp-p", (OMX_S32 *)&(initQP.nQpP));
            valueSet |= vExt.readParamInt32(ext, "qp-i-enable", (OMX_S32 *)&enableIQp);
            valueSet |= vExt.readParamInt32(ext, "qp-p-enable", (OMX_S32 *)&enablePQp);
            valueSet |= vExt.readParamInt32(ext, "qp-b-enable", (OMX_S32 *)&enableBQp);
            initQP.bEnableInitQp = (((enableIQp == 0x1) << 0x0 ) | ((enablePQp == 0x1) << 0x1 ) | ((enableBQp == 0x1) << 0x2 ));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_param: QOMX_IndexParamVideoInitialQp I/B/P value: %d %d %d",
                initQP.nQpI,initQP.nQpB,initQP.nQpP);

            err = set_parameter(
                    NULL, (OMX_INDEXTYPE)QOMX_IndexParamVideoInitialQp, &initQP);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_param: QOMX_IndexParamVideoInitialQp failed !");
            }
            break;
        }
        case OMX_QTIIndexConfigVideoBlurResolution:
        {
            OMX_QTI_VIDEO_CONFIG_BLURINFO blurInfo;

            memcpy(&blurInfo, &m_blurInfo, sizeof(OMX_QTI_VIDEO_CONFIG_BLURINFO));

            valueSet |= vExt.readParamInt32(ext, "info", (OMX_S32 *)&(blurInfo.nBlurInfo));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: OMX_QTIIndexConfigVideoBlurResolution : %u",
                             blurInfo.nBlurInfo);

            err = set_config(
                    NULL, (OMX_INDEXTYPE)OMX_QTIIndexConfigVideoBlurResolution, &blurInfo);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_QTIIndexConfigVideoBlurResolution failed !");
            }
            break;
        }
        case OMX_QcomIndexParamVideoIPBQPRange:
        {
            OMX_QCOM_VIDEO_PARAM_IPB_QPRANGETYPE qpRange;
            memcpy(&qpRange, &m_sSessionQPRange, sizeof(OMX_QCOM_VIDEO_PARAM_IPB_QPRANGETYPE));
            valueSet |= vExt.readParamInt32(ext, "qp-i-min", (OMX_S32 *)&(qpRange.minIQP));
            valueSet |= vExt.readParamInt32(ext, "qp-i-max", (OMX_S32 *)&(qpRange.maxIQP));
            valueSet |= vExt.readParamInt32(ext, "qp-p-min", (OMX_S32 *)&(qpRange.minPQP));
            valueSet |= vExt.readParamInt32(ext, "qp-p-max", (OMX_S32 *)&(qpRange.maxPQP));
            valueSet |= vExt.readParamInt32(ext, "qp-b-min", (OMX_S32 *)&(qpRange.minBQP));
            valueSet |= vExt.readParamInt32(ext, "qp-b-max", (OMX_S32 *)&(qpRange.maxBQP));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_param: OMX_QcomIndexParamVideoIPBQPRange for min/max value for I/P/B: %d-%d / %d-%d / %d-%d"
                ,qpRange.minIQP,qpRange.maxIQP,qpRange.minPQP,qpRange.maxPQP,qpRange.minBQP,qpRange.maxBQP);

            err = set_parameter(
                    NULL, (OMX_INDEXTYPE)OMX_QcomIndexParamVideoIPBQPRange, &qpRange);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_param: OMX_QcomIndexParamVideoIPBQPRange failed !");
            }
            break;
        }
        case OMX_IndexParamVideoBitrate:
        {
            OMX_VIDEO_PARAM_BITRATETYPE bitRateParam;
            memcpy(&bitRateParam, &m_sParamBitrate, sizeof(OMX_VIDEO_PARAM_BITRATETYPE));
            valueSet |= vExt.readParamInt32(ext, "value", (OMX_S32 *)&(bitRateParam.eControlRate));
            if (!valueSet) {
                break;
            }
            DEBUG_PRINT_HIGH("VENDOR-EXT: set_param: ControlRate = %d",
                                bitRateParam.eControlRate);

            err = set_parameter(
                   NULL, (OMX_INDEXTYPE)OMX_IndexParamVideoBitrate, &bitRateParam);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_param: OMX_IndexParamVideoBitrate failed !");
            }

            break;
        }
        case OMX_QTIIndexParamEnableLinearColorFormat:
        {
            QOMX_ENABLETYPE linearColor;
            memcpy(&linearColor, &m_sParamLinearColorFormat, sizeof(QOMX_ENABLETYPE));
            valueSet |= vExt.readParamInt32(ext, "value", (OMX_S32 *)&(linearColor.bEnable));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_param: OMX_QTIIndexParamEnableLinearColorFormat : %d",
                                linearColor.bEnable);

            err = set_parameter(
                   NULL, (OMX_INDEXTYPE)OMX_QTIIndexParamEnableLinearColorFormat, &linearColor);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_param: OMX_QTIIndexParamEnableLinearColorFormat failed !");
            }
            break;
        }
        case OMX_QTIIndexParamVbvDelay:
        {
            OMX_EXTNINDEX_VIDEO_VBV_DELAY vbvDelay;
            memcpy(&vbvDelay, &m_sParamVbvDelay, sizeof(OMX_EXTNINDEX_VIDEO_VBV_DELAY));
            valueSet |= vExt.readParamInt32(ext, "value", (OMX_S32 *)&(vbvDelay.nVbvDelay));
            if (!valueSet) {
                break;
            }

            DEBUG_PRINT_HIGH("VENDOR-EXT: set_param: OMX_QTIIndexParamVbvDelay : %d",
                                vbvDelay.nVbvDelay);

            err = set_parameter(
                   NULL, (OMX_INDEXTYPE)OMX_QTIIndexParamVbvDelay, &vbvDelay);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_param: OMX_QTIIndexParamVbvDelay failed !");
            }
            break;
        }
        case OMX_QTIIndexConfigVideoRoiRectRegionInfo:
        {
            if (vExt.paramCount() == 1) {
                break;
            }
            uint32_t format = m_sOutPortDef.format.video.eCompressionFormat;
            if (format != OMX_VIDEO_CodingHEVC && format != OMX_VIDEO_CodingAVC) {
                DEBUG_PRINT_ERROR("ROI-Ext: only support avc or hevc");
                break;
            }
            char type[OMX_MAX_STRINGVALUE_SIZE];
            valueSet = vExt.readParamString(ext, "type", type);
            if (!valueSet || strncmp(type, "rect", 4)) {
                DEBUG_PRINT_ERROR("ROI-Ext: type is invalid");
                break;
            }
            int64_t timestamp;
            valueSet = vExt.readParamInt64(ext, "timestamp", (OMX_S64 *)&timestamp);
            if (!valueSet) {
                DEBUG_PRINT_ERROR("ROI-Ext: timestamp is invalid");
                break;
            }
            char rawData[OMX_MAX_STRINGVALUE_SIZE * 2] = {0};
            valueSet = vExt.readParamString(ext, "rect-payload", rawData);
            if (!valueSet) {
                DEBUG_PRINT_ERROR("ROI-Ext: payload is invalid");
                break;
            }
            uint32_t size = strlen(rawData);
            if (size == 0 || size > (OMX_MAX_STRINGVALUE_SIZE -1)) {
                DEBUG_PRINT_ERROR("ROI-Ext: payload size is invalid : %u", size);
                break;
            }
            // "payload-ext" is optional, if it's set by the client, joint this string
            // with the "payload" string and seperate them with ';'
            if (vExt.readParamString(ext, "rect-payload-ext", rawData + size + 1)) {
                rawData[size] = ';';
                size = strlen(rawData);
            }
            uint32_t width = m_sOutPortDef.format.video.nFrameWidth;
            uint32_t height = m_sOutPortDef.format.video.nFrameHeight;
            DEBUG_PRINT_LOW("ROI-Ext: clip(%ux%u), ts(%lld), payload(%u):%s",
                    width, height, (long long)timestamp, size, rawData);

            OMX_QTI_VIDEO_CONFIG_ROI_RECT_REGION_INFO roiInfo;
            OMX_INIT_STRUCT(&roiInfo, OMX_QTI_VIDEO_CONFIG_ROI_RECT_REGION_INFO);
            roiInfo.nTimeStamp = timestamp;
            roiInfo.nPortIndex = PORT_INDEX_IN;

            // The pattern to parse the String: "top,left-bottom,right=deltaQp;"
            // each rectangle region string is seperated by ";"
            // For example, the rectangle region is: top: 128, left:96, bottom:336, right:272
            // and delta QP is -9, the string is "128,96-336,272=-9;".
            uint32_t index = 0;
            int top, left, bottom, right, deltaQp;
            char *save = rawData;
            char *rect = strtok_r(rawData, ";", &save);
            const char* patternString = "%d,%d-%d,%d=%d";
            while (rect != NULL) {
                int tags = sscanf(rect, patternString, &top, &left, &bottom, &right, &deltaQp);
                if (tags == 5) {
                    // sanity check
                    if (top > bottom || left > right || top < 0 || left < 0
                            || bottom >= (int32_t)height || right >= (int32_t)width
                            || index == MAX_RECT_ROI_NUM) {
                        DEBUG_PRINT_ERROR("ROI-Ext: invalid roi info with deltaQp");
                    } else {
                        // delta QP range is (-32, 31)
                        if (deltaQp < -31) {
                            deltaQp = -31;
                        }
                        if (deltaQp > 30) {
                            deltaQp = 30;
                        }
                        roiInfo.nRegions[index].nLeft = (uint32_t)left;
                        roiInfo.nRegions[index].nTop = (uint32_t)top;
                        roiInfo.nRegions[index].nRight = (uint32_t)right;
                        roiInfo.nRegions[index].nBottom = (uint32_t)bottom;
                        roiInfo.nRegions[index].nDeltaQP = (int8_t)deltaQp;
                        DEBUG_PRINT_LOW("ROI-Ext: region(%u): t%d-l%d-b%d-r%d: qp:%d",
                                index, top, left, bottom, right, deltaQp);
                        index++;
                    }
                } else {
                    DEBUG_PRINT_LOW("error rect : %s", rect);
                }
                rect = strtok_r(NULL, ";", &save);
            }
            if (index > 0) {
                roiInfo.nRegionNum = index;
                err = set_config(NULL, (OMX_INDEXTYPE)OMX_QTIIndexConfigVideoRoiRectRegionInfo,
                        &roiInfo);
                if (err != OMX_ErrorNone) {
                    DEBUG_PRINT_ERROR("ROI-Ext: failed to set roi region info");
                } else {
                    DEBUG_PRINT_LOW("ROI-Ext: set %u number of roi region info", index);
                }
            } else {
                DEBUG_PRINT_LOW("ROI-Ext: none valid roi region info");
            }
            break;
        }
        case OMX_QTIIndexConfigContentAdaptiveCoding:
        {
            OMX_U32 bitrateSavingsMode;
            valueSet |= vExt.readParamInt32(ext, "value", (OMX_S32 *)&(bitrateSavingsMode));
            if (!valueSet) {
                break;
            }
            DEBUG_PRINT_HIGH("VENDOR-EXT: set_config: OMX_QTIIndexConfigContentAdaptiveCoding : %u",
                             bitrateSavingsMode);

            err = set_config(
                    NULL, (OMX_INDEXTYPE)OMX_QTIIndexConfigContentAdaptiveCoding, &bitrateSavingsMode);
            if (err != OMX_ErrorNone) {
                DEBUG_PRINT_ERROR("set_config: OMX_QTIIndexConfigContentAdaptiveCoding failed !");
            }
            break;
        }
        default:
        {
            return OMX_ErrorNotImplemented;
        }
    }
    return err;
}
