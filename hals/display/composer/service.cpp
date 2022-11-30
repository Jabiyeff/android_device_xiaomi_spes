/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
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
#include "QtiComposer.h"

using android::ProcessState;
using android::sp;
using android::hardware::configureRpcThreadpool;
using android::hardware::joinRpcThreadpool;
using android::hardware::graphics::composer::V2_3::IComposer;
using vendor::qti::hardware::display::composer::V3_0::IQtiComposer;
using vendor::qti::hardware::display::composer::V3_0::implementation::QtiComposer;

int main(int, char **) {
  // TODO(user): double-check for SCHED_FIFO logic
  // the conventional HAL might start binder services
  ProcessState::initWithDriver("/dev/vndbinder");
  sp<ProcessState> ps(ProcessState::self());
  ps->setThreadPoolMaxThreadCount(4);
  ps->startThreadPool();

  // same as SF main thread
  struct sched_param param = {0};
  param.sched_priority = 2;
  if (sched_setscheduler(0, SCHED_FIFO | SCHED_RESET_ON_FORK, &param) != 0) {
    ALOGE("Couldn't set SCHED_FIFO: %d", errno);
  }

  sp<IQtiComposer> composer = QtiComposer::initialize();
  if (composer == nullptr) {
    ALOGE("Cannot initialize composer");
    return -EINVAL;
  }

  configureRpcThreadpool(4, true /*callerWillJoin*/);
  if (composer->registerAsService() != android::OK) {
    ALOGE("Cannot register QTI composer service");
    return -EINVAL;
  }

  ALOGI("Initialized qti-composer");
  joinRpcThreadpool();
  return 0;
}
