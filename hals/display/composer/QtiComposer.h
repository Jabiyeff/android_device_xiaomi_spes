/*
* Copyright (c) 2019, The Linux Foundation. All rights reserved.
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


#ifndef __QTICOMPOSER_H__
#define __QTICOMPOSER_H__

#include <QtiComposerClient.h>

// TODO(user): recheck on this header inclusion
#include <hardware/hwcomposer2.h>
#include <log/log.h>
#include <vendor/qti/hardware/display/composer/3.0/IQtiComposer.h>

#include <unordered_set>

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace composer {
namespace V3_0 {
namespace implementation {

using ::vendor::qti::hardware::display::composer::V3_0::IQtiComposer;

class QtiComposer : public IQtiComposer {
 public:
  QtiComposer();
  virtual ~QtiComposer();
  // Methods from ::android::hardware::graphics::composer::V2_1::IComposer follow.
  Return<void> getCapabilities(getCapabilities_cb _hidl_cb) override;
  Return<void> dumpDebugInfo(dumpDebugInfo_cb _hidl_cb) override;
  Return<void> createClient(createClient_cb _hidl_cb) override;

  // Methods from ::android::hardware::graphics::composer::V2_3::IComposer follow.
  Return<void> createClient_2_3(createClient_2_3_cb _hidl_cb) override;

  // Methods from ::android::hardware::graphics::composer::V2_4::IComposer follow.
  Return<void> createClient_2_4(createClient_2_4_cb _hidl_cb) override;

  // Methods from ::android::hidl::base::V1_0::IBase follow.

  static QtiComposer *initialize();

 private:
    HWCSession *hwc_session_ = nullptr;
};

extern "C" IQtiComposer* HIDL_FETCH_IQtiComposer(const char* name);

}  // namespace implementation
}  // namespace V3_0
}  // namespace composer
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor

#endif  // __QTICOMPOSER_H__
