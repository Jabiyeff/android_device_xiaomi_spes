/*--------------------------------------------------------------------------
Copyright (c) 2010-2018,2020, The Linux Foundation. All rights reserved.

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

#include "vidc_debug.h"

void print_debug_color_aspects(ColorAspects *a, const char *prefix)
{
  DEBUG_PRINT_HIGH("%s : Color aspects : Primaries = %d(%s) Range = %d(%s) Tx = %d(%s) Matrix = %d(%s)",
                   prefix, a->mPrimaries, asString(a->mPrimaries), a->mRange, asString(a->mRange),
                   a->mTransfer, asString(a->mTransfer), a->mMatrixCoeffs, asString(a->mMatrixCoeffs));

}

void print_debug_hdr_color_info(HDRStaticInfo *hdr_info, const char *prefix)
{
    if (!hdr_info->mID) {
        DEBUG_PRINT_LOW("%s : HDRstaticinfo MDC: mR.x = %d mR.y = %d", prefix,
                         hdr_info->sType1.mR.x, hdr_info->sType1.mR.y);
        DEBUG_PRINT_LOW("%s : HDRstaticinfo MDC: mG.x = %d mG.y = %d", prefix,
                         hdr_info->sType1.mG.x, hdr_info->sType1.mG.y);
        DEBUG_PRINT_LOW("%s : HDRstaticinfo MDC: mB.x = %d mB.y = %d", prefix,
                         hdr_info->sType1.mB.x, hdr_info->sType1.mB.y);
        DEBUG_PRINT_LOW("%s : HDRstaticinfo MDC: mW.x = %d mW.y = %d", prefix,
                         hdr_info->sType1.mW.x, hdr_info->sType1.mW.y);
        DEBUG_PRINT_LOW("%s : HDRstaticinfo MDC: maxDispLum = %d minDispLum = %d", prefix,
                         hdr_info->sType1.mMaxDisplayLuminance, hdr_info->sType1.mMinDisplayLuminance);
        DEBUG_PRINT_LOW("%s : HDRstaticinfo CLL: CLL = %d FLL = %d", prefix,
                        hdr_info->sType1.mMaxContentLightLevel, hdr_info->sType1.mMaxFrameAverageLightLevel);
    }

}

void print_debug_hdr_color_info_mdata(ColorMetaData* color_mdata)
{
    DEBUG_PRINT_LOW("setMetaData COLOR_METADATA : color_primaries = %u, range = %u, transfer = %u, matrix = %u",
                    color_mdata->colorPrimaries, color_mdata->range,
                    color_mdata->transfer, color_mdata->matrixCoefficients);

    for(uint8_t i = 0; i < 3; i++) {
        for(uint8_t j = 0; j < 2; j++) {
            DEBUG_PRINT_LOW("setMetadata COLOR_METADATA : rgbPrimaries[%d][%d] = %d", i, j, color_mdata->masteringDisplayInfo.primaries.rgbPrimaries[i][j]);
        }
    }

    DEBUG_PRINT_LOW("setMetadata COLOR_METADATA : whitepoint[0] = %d whitepoint[1] = %d",
                    color_mdata->masteringDisplayInfo.primaries.whitePoint[0],
                    color_mdata->masteringDisplayInfo.primaries.whitePoint[1]);

    DEBUG_PRINT_LOW("setMetadata COLOR_METADATA : maxDispLum = %d minDispLum = %d",
                    color_mdata->masteringDisplayInfo.maxDisplayLuminance,
                    color_mdata->masteringDisplayInfo.minDisplayLuminance);

    DEBUG_PRINT_LOW("setMetadata COLOR_METADATA : maxCLL = %d maxFLL = %d",
                    color_mdata->contentLightLevel.maxContentLightLevel,
                    color_mdata->contentLightLevel.minPicAverageLightLevel);
}

void print_debug_hdr10plus_metadata(ColorMetaData& color_mdata)
{
    DEBUG_PRINT_LOW("HDR10+ valid data length: %d", color_mdata.dynamicMetaDataLen);
    for (uint32_t i = 0 ; i < color_mdata.dynamicMetaDataLen && i+3 < HDR_DYNAMIC_META_DATA_SZ; i=i+4) {
        DEBUG_PRINT_LOW("HDR10+ mdata: %02X %02X %02X %02X", color_mdata.dynamicMetaDataPayload[i],
                        color_mdata.dynamicMetaDataPayload[i+1],
                        color_mdata.dynamicMetaDataPayload[i+2],
                        color_mdata.dynamicMetaDataPayload[i+3]);
    }

}

