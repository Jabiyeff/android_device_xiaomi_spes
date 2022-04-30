/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __PLATFORM_CONFIG_H__
#define __PLATFORM_CONFIG_H__

//////////////////////////////////////////////////////////////////////////////
//                             Include Files
//////////////////////////////////////////////////////////////////////////////
#include <string>
#include <map>

#ifdef ENABLE_CONFIGSTORE
#include <vendor/qti/hardware/capabilityconfigstore/1.0/ICapabilityConfigStore.h>
#endif

#ifdef __cplusplus
    extern "C" {
#endif

namespace Platform {

typedef std::map<std::string, std::string> ConfigMap;

#ifdef ENABLE_CONFIGSTORE
using namespace android;
using vendor::qti::hardware::capabilityconfigstore::V1_0::ICapabilityConfigStore;
#endif

typedef enum {
    OK = 0,
    FAIL = 1,
} ConfigError_t;

typedef enum {
    vidc_dec_log_in = 0,
    vidc_dec_log_out,
    vidc_dec_hfr_fps,
    vidc_enc_log_in,
    vidc_enc_log_out,
    vidc_dec_sec_prefetch_size_internal,
    vidc_dec_sec_prefetch_size_output,
    vidc_dec_conceal_color_8bit,
    vidc_dec_conceal_color_10bit,
    vidc_enc_csc_custom_matrix,
    vidc_perf_control_enable,
    vidc_enc_linear_color_format,
    vidc_enc_bitrate_savings_enable,
    vidc_enc_auto_blur_disable,
    vidc_dec_thumbnail_yuv_output,
    vidc_no_vpss,
    vidc_disable_hdr,
    vidc_enc_quality_boost_enable,
    vidc_dec_output_rate,
} Config_t;

struct configStr {
    Config_t config;
    const char * name;
};

static const struct configStr configStrMap[] = {
    {vidc_dec_log_in, "vidc_dec_log_in"},
    {vidc_dec_log_out, "vidc_dec_log_out"},
    {vidc_dec_hfr_fps, "vidc_dec_hfr_fps"},
    {vidc_enc_log_in, "vidc_enc_log_in"},
    {vidc_enc_log_out, "vidc_enc_log_out"},
    {vidc_dec_sec_prefetch_size_internal,"vidc_dec_sec_prefetch_size_internal"},
    {vidc_dec_sec_prefetch_size_output,"vidc_dec_sec_prefetch_size_output"},
    {vidc_dec_conceal_color_8bit, "vidc_dec_conceal_color_8bit"},
    {vidc_dec_conceal_color_10bit, "vidc_dec_conceal_color_10bit"},
    {vidc_enc_csc_custom_matrix, "vidc_enc_csc_custom_matrix"},
    {vidc_perf_control_enable, "vidc_perf_control_enable"},
    {vidc_enc_linear_color_format, "vidc_enc_linear_color_format"},
    {vidc_enc_bitrate_savings_enable, "vidc_enc_bitrate_savings_enable"},
    {vidc_enc_auto_blur_disable, "vidc_enc_auto_blur_disable"},
    {vidc_dec_thumbnail_yuv_output, "vidc_dec_thumbnail_yuv_output"},
    {vidc_no_vpss, "vidc_no_vpss"},
    {vidc_disable_hdr, "vidc_disable_hdr"},
    {vidc_enc_quality_boost_enable, "vidc_enc_quality_boost_enable"},
    {vidc_dec_output_rate, "vidc_dec_output_rate"},
};

class Config {
    private:
        Config();
        Config& operator=(Config const&) {
            return *mInstance;
        }
        static Config* getInstance();

        ConfigMap mConfigMap;
        static Config* mInstance;

#ifdef ENABLE_CONFIGSTORE
        android::sp<ICapabilityConfigStore> mConfigStore;
#endif

    public:
        static ConfigError_t getInt32(Config_t config, int32_t *value,
                const int32_t defaultValue);

        static bool isConfigStoreEnabled();
        static ConfigError_t getConfigStoreBool(const char *area,
                const char *config, bool &value, const bool defaultValue);
};

}
#ifdef __cplusplus
}
#endif
#endif // __PLATFORM_CONFIG_H__
