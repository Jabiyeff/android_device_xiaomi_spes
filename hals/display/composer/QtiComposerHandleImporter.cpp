/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 *
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

#include <log/log.h>

#include "QtiComposerHandleImporter.h"

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace composer {
namespace V3_0 {

using MapperV2Error = android::hardware::graphics::mapper::V2_0::Error;
using MapperV3Error = android::hardware::graphics::mapper::V3_0::Error;

ComposerHandleImporter::ComposerHandleImporter() : mInitialized(false) {}

void ComposerHandleImporter::initialize() {
    // allow only one client
  if (mInitialized) {
    return;
  }

  mMapper_V3 = IMapperV3::getService();
  if (mMapper_V3 == nullptr) {
    mMapper_V2 = IMapperV2::getService();
    if (mMapper_V2 == nullptr) {
      ALOGE("%s: cannnot acccess graphics mapper HAL!", __FUNCTION__);
      return;
    }
  }

  mInitialized = true;
  return;
}

void ComposerHandleImporter::cleanup() {
  if (mMapper_V3 != nullptr) {
    mMapper_V3.clear();
  } else {
    mMapper_V2.clear();
  }
  mInitialized = false;
}

// In IComposer, any buffer_handle_t is owned by the caller and we need to
// make a clone for hwcomposer2.  We also need to translate empty handle
// to nullptr.  This function does that, in-place.
bool ComposerHandleImporter::importBuffer(buffer_handle_t& handle) {
  if (!handle) {
    return true;
  }

  if (!handle->numFds && !handle->numInts) {
    handle = nullptr;
    return true;
  }

  Mutex::Autolock lock(mLock);
  if (!mInitialized) {
    initialize();
  }

  if (mMapper_V3 == nullptr && mMapper_V2 == nullptr) {
    ALOGE("%s: mMapper is null!", __FUNCTION__);
    return false;
  }

  if (mMapper_V3 != nullptr) {
    MapperV3Error error;
    buffer_handle_t importedHandle;

    auto ret = mMapper_V3->importBuffer(
        hidl_handle(handle),
        [&](const auto &tmpError, const auto &tmpBufferHandle) {
          error = tmpError;
          importedHandle = static_cast<buffer_handle_t>(tmpBufferHandle);
        });

    if (!ret.isOk()) {
      ALOGE("%s: mapper importBuffer failed: %s", __FUNCTION__, ret.description().c_str());
      return false;
    }

    if (error != MapperV3Error::NONE) {
      return false;
    }

    handle = importedHandle;
  } else {
    MapperV2Error error;
    buffer_handle_t importedHandle;

    auto ret = mMapper_V2->importBuffer(
        hidl_handle(handle),
        [&](const auto &tmpError, const auto &tmpBufferHandle) {
          error = tmpError;
          importedHandle = static_cast<buffer_handle_t>(tmpBufferHandle);
        });

    if (!ret.isOk()) {
      ALOGE("%s: mapper importBuffer failed: %s", __FUNCTION__, ret.description().c_str());
      return false;
    }

    if (error != MapperV2Error::NONE) {
      return false;
    }

    handle = importedHandle;
  }

  return true;
}

void ComposerHandleImporter::freeBuffer(buffer_handle_t handle) {
  if (!handle) {
    return;
  }

  Mutex::Autolock lock(mLock);

  if (mMapper_V3 == nullptr && mMapper_V2 == nullptr) {
    ALOGE("%s: mMapper is null!", __FUNCTION__);
    return;
  }

  if (mMapper_V3 != nullptr) {
    auto ret = mMapper_V3->freeBuffer(const_cast<native_handle_t *>(handle));
    if (!ret.isOk()) {
      ALOGE("%s: mapper freeBuffer failed: %s", __FUNCTION__, ret.description().c_str());
    }
  } else {
    auto ret = mMapper_V2->freeBuffer(const_cast<native_handle_t *>(handle));
    if (!ret.isOk()) {
      ALOGE("%s: mapper freeBuffer failed: %s", __FUNCTION__, ret.description().c_str()); }
  }
}

}  // namespace V3_0
}  // namespace composer
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
