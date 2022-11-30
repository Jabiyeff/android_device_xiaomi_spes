/*
 * Copyright (c) 2018-2020 The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *    * Neither the name of The Linux Foundation. nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
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
#include <hidl/LegacySupport.h>

#include "QtiAllocator.h"

using android::hardware::configureRpcThreadpool;
using android::hardware::joinRpcThreadpool;
using IQtiAllocator3 = vendor::qti::hardware::display::allocator::V3_0::IQtiAllocator;
using IQtiAllocator4 = vendor::qti::hardware::display::allocator::V4_0::IQtiAllocator;

int main(int, char **) {
  android::sp<IQtiAllocator3> service3 =
      new vendor::qti::hardware::display::allocator::V3_0::implementation::QtiAllocator();

  configureRpcThreadpool(4, true /*callerWillJoin*/);
  android::hardware::setMinSchedulerPolicy(service3, SCHED_NORMAL, -20);
  if (service3->registerAsService() != android::OK) {
    ALOGE("Cannot register QTI Allocator 3 service");
    return -EINVAL;
  }
  ALOGI("Initialized qti-allocator 3");

#ifdef TARGET_USES_GRALLOC4
  android::sp<IQtiAllocator4> service4 =
      new vendor::qti::hardware::display::allocator::V4_0::implementation::QtiAllocator();
  android::hardware::setMinSchedulerPolicy(service4, SCHED_NORMAL, -20);
  if (service4->registerAsService() != android::OK) {
    ALOGE("Cannot register QTI Allocator 4 service");
    return -EINVAL;
  }
  ALOGI("Initialized qti-allocator 4");
#endif

  joinRpcThreadpool();

  return 0;
}
