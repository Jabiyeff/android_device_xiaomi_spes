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

#include <vector>
#include "QtiComposer.h"

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace composer {
namespace V3_0 {
namespace implementation {

QtiComposerClient* QtiComposerClient::qti_composerclient_instance_ = nullptr;

QtiComposer::QtiComposer() {
  hwc_session_ = HWCSession::GetInstance();
}

QtiComposer::~QtiComposer() {
  hwc_session_->Deinit();
}

// Methods from ::android::hardware::graphics::composer::V2_1::IComposer follow.
Return<void> QtiComposer::getCapabilities(getCapabilities_cb _hidl_cb) {
  const std::array<IComposer::Capability, 3> all_caps = {{
    IComposer::Capability::SIDEBAND_STREAM,
    IComposer::Capability::SKIP_CLIENT_COLOR_TRANSFORM,
    IComposer::Capability::PRESENT_FENCE_IS_NOT_RELIABLE,
  }};

  uint32_t count = 0;
  hwc_session_->GetCapabilities(&count, nullptr);

  std::vector<int32_t> composer_caps(count);
  hwc_session_->GetCapabilities(&count, composer_caps.data());
  composer_caps.resize(count);

  std::unordered_set<hwc2_capability_t> Capabilities;
  Capabilities.reserve(count);
  for (auto cap : composer_caps) {
    Capabilities.insert(static_cast<hwc2_capability_t>(cap));
  }

  std::vector<IComposer::Capability> caps;
  for (auto cap : all_caps) {
    if (Capabilities.count(static_cast<hwc2_capability_t>(cap)) > 0) {
     caps.push_back(cap);
    }
  }

  hidl_vec<IComposer::Capability> caps_reply;
  caps_reply.setToExternal(caps.data(), caps.size());

  _hidl_cb(caps_reply);
  return Void();
}

Return<void> QtiComposer::dumpDebugInfo(dumpDebugInfo_cb _hidl_cb) {
  uint32_t len;
  hwc_session_->Dump(&len, nullptr);

  std::vector<char> buf(len + 1);
  hwc_session_->Dump(&len, buf.data());

  buf.resize(len + 1);
  buf[len] = '\0';
  hidl_string buf_reply;
  buf_reply.setToExternal(buf.data(), len);

  _hidl_cb(buf_reply);
  return Void();
}

Return<void> QtiComposer::createClient(createClient_cb _hidl_cb) {
  // TODO(user): Implement combinedly w.r.t createClient_2_3
  sp<QtiComposerClient> composer_client = QtiComposerClient::CreateQtiComposerClientInstance();
  if (!composer_client) {
    _hidl_cb(Error::NO_RESOURCES, nullptr);
    return Void();
  }

  _hidl_cb(Error::NONE, composer_client);
  return Void();
}


// Methods from ::android::hardware::graphics::composer::V2_3::IComposer follow.
Return<void> QtiComposer::createClient_2_3(createClient_2_3_cb _hidl_cb) {
  sp<QtiComposerClient> composer_client = QtiComposerClient::CreateQtiComposerClientInstance();
  if (!composer_client) {
    _hidl_cb(Error::NO_RESOURCES, nullptr);
    return Void();
  }

  _hidl_cb(Error::NONE, composer_client);
  return Void();
}

// Methods from ::android::hardware::graphics::composer::V2_4::IComposer follow.
Return<void> QtiComposer::createClient_2_4(createClient_2_4_cb _hidl_cb) {
  sp<QtiComposerClient> composer_client = QtiComposerClient::CreateQtiComposerClientInstance();
  if (!composer_client) {
    _hidl_cb(composer_V2_4::Error::NO_RESOURCES, nullptr);
    return Void();
  }

  _hidl_cb(composer_V2_4::Error::NONE, composer_client);
  return Void();
}

QtiComposer *QtiComposer::initialize() {
  auto error = HWCSession::GetInstance()->Init();
  if (error) {
    ALOGE("failed to get hwcomposer instance");
    return nullptr;
  }

  return new QtiComposer();
}

// Methods from ::android::hidl::base::V1_0::IBase follow.

IQtiComposer* HIDL_FETCH_IQtiComposer(const char* /* name */) {
  return new QtiComposer();
}

}  // namespace implementation
}  // namespace V3_0
}  // namespace composer
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
