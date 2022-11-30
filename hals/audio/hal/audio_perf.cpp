/*
 * Copyright (C) 2017 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "audio_hw_primary"

#include <cinttypes>

#include <utils/Log.h>
#include <utils/Mutex.h>

#include <android/hardware/power/1.2/IPower.h>
#include <aidl/android/hardware/power/Boost.h>
#include <aidl/android/hardware/power/IPower.h>
#include <aidl/android/hardware/power/Mode.h>
#include <android/binder_manager.h>

#include "audio_perf.h"

// Protect gPowerHal_1_2_ and gPowerHal_Aidl_
static android::sp<android::hardware::power::V1_2::IPower> gPowerHal_1_2_;
static std::shared_ptr<aidl::android::hardware::power::IPower> gPowerHal_Aidl_;
static std::mutex gPowerHalMutex;
static constexpr int kDefaultBoostDurationMs = 2000;
static constexpr int kBoostOff = -1;

static const std::string kInstance =
        std::string(aidl::android::hardware::power::IPower::descriptor) + "/default";

enum hal_version {
    NONE,
    HIDL_1_2,
    AIDL,
};

// Connnect PowerHAL
static hal_version connectPowerHalLocked() {
    static bool gPowerHalHidlExists = true;
    static bool gPowerHalAidlExists = true;

    if (!gPowerHalHidlExists && !gPowerHalAidlExists) {
        return NONE;
    }

    if (gPowerHalHidlExists) {
        // (re)connect if handle is null
        if (!gPowerHal_1_2_) {
            gPowerHal_1_2_ =
                    android::hardware::power::V1_2::IPower::getService();
        }
        if (gPowerHal_1_2_) {
            ALOGV("Successfully connected to Power Hal Hidl service.");
            return HIDL_1_2;
        } else {
            // no more try on this handle
            gPowerHalHidlExists = false;
        }
    }

    if (gPowerHalAidlExists) {
        // (re)connect if handle is null
        if (!gPowerHal_Aidl_) {
            ndk::SpAIBinder pwBinder = ndk::SpAIBinder(
                AServiceManager_getService(kInstance.c_str()));
            gPowerHal_Aidl_ = aidl::android::hardware::power::IPower::fromBinder(pwBinder);
        }
        if (gPowerHal_Aidl_) {
            ALOGV("Successfully connected to Power Hal Aidl service.");
            return AIDL;
        } else {
            // no more try on this handle
            gPowerHalAidlExists = false;
        }
    }

    return NONE;
}

bool audio_streaming_hint_start() {
    std::lock_guard<std::mutex> lock(gPowerHalMutex);
    switch(connectPowerHalLocked()) {
        case NONE:
            return false;
        case HIDL_1_2:
            {
                auto ret = gPowerHal_1_2_->powerHintAsync_1_2(
                    android::hardware::power::V1_2::PowerHint::AUDIO_STREAMING,
                    1);
                if (!ret.isOk()) {
                    ALOGE("powerHint failed, error: %s",
                          ret.description().c_str());
                    gPowerHal_1_2_ = nullptr;
                    return false;
                }
                return true;
            }
        case AIDL:
            {
                auto ret = gPowerHal_Aidl_->setBoost(
                    aidl::android::hardware::power::Boost::AUDIO_LAUNCH,
                    kDefaultBoostDurationMs);
                if (!ret.isOk()) {
                    std::string err = ret.getDescription();
                    ALOGE("Failed to set power hint. Error: %s", err.c_str());
                    gPowerHal_Aidl_ = nullptr;
                    return false;
                }
                return true;
            }
        default:
            ALOGE("Unknown HAL state");
            return false;
    }
}

bool audio_streaming_hint_end() {
    std::lock_guard<std::mutex> lock(gPowerHalMutex);
    switch(connectPowerHalLocked()) {
        case NONE:
            return false;
        case HIDL_1_2:
            {
                auto ret = gPowerHal_1_2_->powerHintAsync_1_2(
                    android::hardware::power::V1_2::PowerHint::AUDIO_STREAMING,
                    0);
                if (!ret.isOk()) {
                    ALOGE("powerHint failed, error: %s",
                          ret.description().c_str());
                    gPowerHal_1_2_ = nullptr;
                    return false;
                }
                return true;
            }
        case AIDL:
            {
                auto ret = gPowerHal_Aidl_->setBoost(
                    aidl::android::hardware::power::Boost::AUDIO_LAUNCH,
                    kBoostOff);
                if (!ret.isOk()) {
                    std::string err = ret.getDescription();
                    ALOGE("Failed to set power hint. Error: %s", err.c_str());
                    gPowerHal_Aidl_ = nullptr;
                    return false;
                }
                return true;
            }
        default:
            ALOGE("Unknown HAL state");
            return false;
    }
}

bool audio_low_latency_hint_start() {
    std::lock_guard<std::mutex> lock(gPowerHalMutex);
    switch(connectPowerHalLocked()) {
        case NONE:
            return false;
        case HIDL_1_2:
            {
                auto ret = gPowerHal_1_2_->powerHintAsync_1_2(
                    android::hardware::power::V1_2::PowerHint::AUDIO_LOW_LATENCY,
                    1);
                if (!ret.isOk()) {
                    ALOGE("powerHint failed, error: %s",
                          ret.description().c_str());
                    gPowerHal_1_2_ = nullptr;
                    return false;
                }
                return true;
            }
        case AIDL:
            {
                auto ret = gPowerHal_Aidl_->setMode(
                    aidl::android::hardware::power::Mode::AUDIO_STREAMING_LOW_LATENCY,
                    true);
                if (!ret.isOk()) {
                    std::string err = ret.getDescription();
                    ALOGE("Failed to set power hint. Error: %s", err.c_str());
                    gPowerHal_Aidl_ = nullptr;
                    return false;
                }
                return true;
            }
        default:
            ALOGE("Unknown HAL state");
            return false;
    }
}

bool audio_low_latency_hint_end() {
    std::lock_guard<std::mutex> lock(gPowerHalMutex);
    switch(connectPowerHalLocked()) {
        case NONE:
            return false;
        case HIDL_1_2:
            {
                auto ret = gPowerHal_1_2_->powerHintAsync_1_2(
                    android::hardware::power::V1_2::PowerHint::AUDIO_LOW_LATENCY,
                    0);
                if (!ret.isOk()) {
                    ALOGE("powerHint failed, error: %s",
                          ret.description().c_str());
                    gPowerHal_1_2_ = nullptr;
                    return false;
                }
                return true;
            }
        case AIDL:
            {
                auto ret = gPowerHal_Aidl_->setMode(
                    aidl::android::hardware::power::Mode::AUDIO_STREAMING_LOW_LATENCY,
                    false);
                if (!ret.isOk()) {
                    std::string err = ret.getDescription();
                    ALOGE("Failed to set power hint. Error: %s", err.c_str());
                    gPowerHal_Aidl_ = nullptr;
                    return false;
                }
                return true;
            }
        default:
            ALOGE("Unknown HAL state");
            return false;
    }
}
