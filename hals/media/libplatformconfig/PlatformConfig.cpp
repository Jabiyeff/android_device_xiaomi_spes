/*
 * Copyright (c) 2017 - 2019, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "PlatformConfig"

#include <errno.h>
#include <utils/Log.h>
#include <sys/mman.h>
#include "PlatformConfig.h"
#include "ConfigParser.h"

#ifdef ENABLE_CONFIGSTORE
#include <vendor/qti/hardware/capabilityconfigstore/1.0/types.h>
#include <vendor/qti/hardware/capabilityconfigstore/1.0/ICapabilityConfigStore.h>
#endif

namespace Platform {

#define PLAT_CONFIG_FILE "/vendor/etc/system_properties.xml"

Config* Config::mInstance;

Config::Config() {
    Platform::ConfigParser::initAndParse(PLAT_CONFIG_FILE, mConfigMap);
}

Config* Config::getInstance() {
    VIDC_PLAT_LOGH("%s: Enter", __func__);
    if (!mInstance) {
        mInstance = new Config();
    }
    return mInstance;
}

ConfigError_t Config::getInt32(Config_t config, int32_t *value,
        const int32_t defaultValue) {
     Config *conf = getInstance();
    if (conf == nullptr) {
        *value = defaultValue;
        return FAIL;
    }
    if (conf->mConfigMap.find(configStrMap[config].name) == conf->mConfigMap.end()) {
        VIDC_PLAT_LOGH("%s: Returning default", __func__);
        *value = defaultValue;
        return FAIL;
    }
    *value = (int32_t) atoi(conf->mConfigMap[configStrMap[config].name].c_str());
    VIDC_PLAT_LOGH("%s Config name: %s value: %d",
            __func__, configStrMap[config].name, *value);
    return OK;
}

bool Config::isConfigStoreEnabled()
{
#ifdef ENABLE_CONFIGSTORE
    return true;
#else
    return false;
#endif
}

ConfigError_t Config::getConfigStoreBool(const char *area, const char *config,
        bool &value, const bool defaultValue)
{
#ifdef ENABLE_CONFIGSTORE
    using vendor::qti::hardware::capabilityconfigstore::V1_0::Result;
    using vendor::qti::hardware::capabilityconfigstore::V1_0::CommandResult;
    if (area == nullptr || config == nullptr) {
        return FAIL;
    }

    value = defaultValue;

    Config *conf = getInstance();
    if (conf == nullptr) {
        return FAIL;
    }

    // load on demand
    if (conf->mConfigStore == nullptr) {
        conf->mConfigStore = ICapabilityConfigStore::tryGetService();
    }

    if (conf->mConfigStore == nullptr) {
        VIDC_PLAT_LOGH("%s: unable to get configstore service", __func__);
        return FAIL;
    }

    CommandResult result;
    result.result_type = Result::NOT_FOUND;
    conf->mConfigStore->getConfig(area, config, [&](const CommandResult &res) {
        result = res;
    });

    if (result.result_type != Result::SUCCESS) {
        VIDC_PLAT_LOGH("%s: %s/%s not found(%u)", __func__, area, config,
                       result.result_type);
        return FAIL;
    }

    if (strncasecmp("true", result.value.c_str(), result.value.size()) == 0) {
        value = true;
    } else {
        value = false;
    }
    VIDC_PLAT_LOGH("%s: %s/%s=%s, returning %s ", __func__, area, config,
            result.value.c_str(), value ? "true" : "false");
    return OK;
#else
    (void)(area);
    (void)(config);
    value = defaultValue;
    return OK;
#endif
}

}
