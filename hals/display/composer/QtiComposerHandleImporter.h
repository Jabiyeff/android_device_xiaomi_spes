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

#ifndef __QTICOMPOSERHANDLEIMPORTER_H__
#define __QTICOMPOSERHANDLEIMPORTER_H__

#include <android/hardware/graphics/mapper/2.0/IMapper.h>
#include <android/hardware/graphics/mapper/3.0/IMapper.h>
#include <utils/Mutex.h>

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace composer {
namespace V3_0 {

using IMapperV2 = ::android::hardware::graphics::mapper::V2_0::IMapper;
using IMapperV3 = ::android::hardware::graphics::mapper::V3_0::IMapper;
using ::android::sp;
using ::android::Mutex;
using ::android::hardware::hidl_handle;

class ComposerHandleImporter {
 public:
  ComposerHandleImporter();

  // In IComposer, any buffer_handle_t is owned by the caller and we need to
  // make a clone for hwcomposer2.  We also need to translate empty handle
  // to nullptr.  This function does that, in-place.
  bool importBuffer(buffer_handle_t& handle);
  void freeBuffer(buffer_handle_t handle);
  void initialize();
  void cleanup();

 private:
  Mutex mLock;
  bool mInitialized = false;
  sp<IMapperV2> mMapper_V2;
  sp<IMapperV3> mMapper_V3;
};

}  // namespace V3_0
}  // namespace composer
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor

#endif  // __QTICOMPOSERHANDLEIMPORTER_H__
