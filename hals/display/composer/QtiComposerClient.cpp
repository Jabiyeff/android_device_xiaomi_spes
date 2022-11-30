/*
 * Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
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

#include <vector>
#include <string>

#include "QtiComposerClient.h"

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace composer {
namespace V3_0 {
namespace implementation {

ComposerHandleImporter mHandleImporter;

BufferCacheEntry::BufferCacheEntry() : mHandle(nullptr) {}

BufferCacheEntry::BufferCacheEntry(BufferCacheEntry&& other) {
  mHandle = other.mHandle;
  other.mHandle = nullptr;
}

BufferCacheEntry& BufferCacheEntry::operator=(buffer_handle_t handle) {
  clear();
  mHandle = handle;
  return *this;
}

BufferCacheEntry::~BufferCacheEntry() {
  clear();
}

void BufferCacheEntry::clear() {
  if (mHandle) {
    mHandleImporter.freeBuffer(mHandle);
  }
}

QtiComposerClient::QtiComposerClient() : mWriter(kWriterInitialSize), mReader(*this) {
  hwc_session_ = HWCSession::GetInstance();
  mHandleImporter.initialize();
}

QtiComposerClient::~QtiComposerClient() {
  // We want to call hwc2_close here (and move hwc2_open to the
  // constructor), with the assumption that hwc2_close would
  //
  //  - clean up all resources owned by the client
  //  - make sure all displays are blank (since there is no layer)
  //
  // But since SF used to crash at this point, different hwcomposer2
  // implementations behave differently on hwc2_close.  Our only portable
  // choice really is to abort().  But that is not an option anymore
  // because we might also have VTS or VR as clients that can come and go.
  //
  // Below we manually clean all resources (layers and virtual
  // displays), and perform a presentDisplay afterwards.
  ALOGW("destroying composer client");

  enableCallback(false);

  // no need to grab the mutex as any in-flight hwbinder call would have
  // kept the client alive
  for (const auto& dpy : mDisplayData) {
    ALOGW("destroying client resources for display %" PRIu64, dpy.first);

    for (const auto& ly : dpy.second.Layers) {
      hwc_session_->DestroyLayer(dpy.first, ly.first);
    }

    if (dpy.second.IsVirtual) {
      destroyVirtualDisplay(dpy.first);
    } else {
      ALOGW("performing a final presentDisplay");

      std::vector<Layer> changedLayers;
      std::vector<IComposerClient::Composition> compositionTypes;
      uint32_t displayRequestMask = 0;
      std::vector<Layer> requestedLayers;
      std::vector<uint32_t> requestMasks;
      IComposerClient::ClientTargetProperty clientTargetProperty;
      mReader.validateDisplay(dpy.first, changedLayers, compositionTypes, displayRequestMask,
                              requestedLayers, requestMasks, clientTargetProperty);

      hwc_session_->AcceptDisplayChanges(dpy.first);

      shared_ptr<Fence> presentFence = nullptr;
      std::vector<Layer> releasedLayers;
      std::vector<shared_ptr<Fence>> releaseFences;
      mReader.presentDisplay(dpy.first, &presentFence, releasedLayers, releaseFences);
    }
  }

  mDisplayData.clear();

  mHandleImporter.cleanup();

  ALOGW("removed composer client");
}

void QtiComposerClient::onHotplug(hwc2_callback_data_t callbackData, hwc2_display_t display,
                                    int32_t connected) {
  auto client = reinterpret_cast<QtiComposerClient*>(callbackData);
  auto connect = static_cast<composer_V2_4::IComposerCallback::Connection>(connected);
  if (connect == composer_V2_4::IComposerCallback::Connection::CONNECTED) {
    std::lock_guard<std::mutex> lock_d(client->mDisplayDataMutex);
    client->mDisplayData.emplace(display, DisplayData(false));
  }

  auto ret = client->callback_->onHotplug(display, connect);
  ALOGW_IF(!ret.isOk(), "failed to send onHotplug: %s. SF likely unavailable.",
           ret.description().c_str());

  if (connect == composer_V2_4::IComposerCallback::Connection::DISCONNECTED) {
    // Trigger refresh to make sure disconnect event received/updated properly by SurfaceFlinger.
    client->hwc_session_->Refresh(HWC_DISPLAY_PRIMARY);
    // Wait for sufficient time to ensure sufficient resources are available to process connection.
    uint32_t vsync_period;
    client->hwc_session_->GetVsyncPeriod(HWC_DISPLAY_PRIMARY, &vsync_period);
    usleep(vsync_period * 2 / 1000);

    // Wait for the input command message queue to process before destroying the local display data.
    std::lock_guard<std::mutex> lock(client->mCommandMutex);
    std::lock_guard<std::mutex> lock_d(client->mDisplayDataMutex);
    client->mDisplayData.erase(display);
  }
}

void QtiComposerClient::onRefresh(hwc2_callback_data_t callbackData, hwc2_display_t display) {
  auto client = reinterpret_cast<QtiComposerClient*>(callbackData);
  auto ret = client->callback_->onRefresh(display);
  ALOGW_IF(!ret.isOk(), "failed to send onRefresh: %s. SF likely unavailable.",
           ret.description().c_str());
}

void QtiComposerClient::onVsync(hwc2_callback_data_t callbackData, hwc2_display_t display,
                                int64_t timestamp) {
  auto client = reinterpret_cast<QtiComposerClient*>(callbackData);
  auto ret = client->callback_->onVsync(display, timestamp);
  ALOGW_IF(!ret.isOk(), "failed to send onVsync: %s. SF likely unavailable.",
           ret.description().c_str());
}

void QtiComposerClient::onVsync_2_4(hwc2_callback_data_t callbackData, hwc2_display_t display,
                                    int64_t timestamp, VsyncPeriodNanos vsyncPeriodNanos) {
  auto client = reinterpret_cast<QtiComposerClient*>(callbackData);
  auto ret = client->callback24_->onVsync_2_4(display, timestamp, vsyncPeriodNanos);
  ALOGW_IF(!ret.isOk(), "failed to send onVsync_2_4: %s. SF likely unavailable.",
           ret.description().c_str());
}

void QtiComposerClient::onVsyncPeriodTimingChanged(hwc2_callback_data_t callbackData,
      hwc2_display_t display, hwc_vsync_period_change_timeline_t *updatedTimeline) {
   VsyncPeriodChangeTimeline timeline =
                                   {updatedTimeline->newVsyncAppliedTimeNanos,
                                    static_cast<bool>(updatedTimeline->refreshRequired),
                                    updatedTimeline->refreshTimeNanos};

  auto client = reinterpret_cast<QtiComposerClient*>(callbackData);
  auto ret = client->callback24_->onVsyncPeriodTimingChanged(display, timeline);
  ALOGW_IF(!ret.isOk(), "failed to send onVsyncPeriodTimingChanged: %s. SF likely unavailable.",
          ret.description().c_str());
}

void QtiComposerClient::onSeamlessPossible(hwc2_callback_data_t callbackData,
                                           hwc2_display_t display) {
  auto client = reinterpret_cast<QtiComposerClient*>(callbackData);
  auto ret = client->callback24_->onSeamlessPossible(display);
  ALOGW_IF(!ret.isOk(), "failed to send onSeamlessPossible: %s. SF likely unavailable.",
           ret.description().c_str());
}

// convert fenceFd to or from hidl_handle
// Handle would still own original fence. Hence create a Fence object on duped fd.
Error QtiComposerClient::getFence(const hidl_handle& fenceHandle, shared_ptr<Fence>* outFence,
                                  const string& name) {
  auto handle = fenceHandle.getNativeHandle();
  if (handle && handle->numFds > 1) {
    ALOGE("invalid fence handle with %d fds", handle->numFds);
    return Error::BAD_PARAMETER;
  }

  int fenceFd = (handle && handle->numFds == 1) ? handle->data[0] : -1;
  *outFence = Fence::Create(dup(fenceFd), name);

  return Error::NONE;
}

// Handle would own fence hereafter. Hence provide a dupped fd.
hidl_handle QtiComposerClient::getFenceHandle(const shared_ptr<Fence>& fence,
                                              char* handleStorage) {
  native_handle_t* handle = nullptr;
  if (fence) {
    handle = native_handle_init(handleStorage, 1, 0);
    if (handle) {
      handle->data[0] = Fence::Dup(fence);
    }
  }

  return hidl_handle(handle);
}

Error QtiComposerClient::getDisplayReadbackBuffer(Display display,
                                                  const native_handle_t* rawHandle,
                                                  const native_handle_t** outHandle) {
  // TODO(user): revisit for caching and freeBuffer in success case.
  if (!mHandleImporter.importBuffer(rawHandle)) {
    ALOGE("%s: importBuffer failed: ", __FUNCTION__);
    return Error::NO_RESOURCES;
  }

  std::lock_guard<std::mutex> lock(mDisplayDataMutex);
  auto iter = mDisplayData.find(display);
  if (iter == mDisplayData.end()) {
    mHandleImporter.freeBuffer(rawHandle);
    return Error::BAD_DISPLAY;
  }

  *outHandle = rawHandle;
  return Error::NONE;
}

void QtiComposerClient::getCapabilities() {
  uint32_t count = 0;
  hwc_session_->GetCapabilities(&count, nullptr);

  std::vector<int32_t> composer_caps(count);
  hwc_session_->GetCapabilities(&count, composer_caps.data());
  composer_caps.resize(count);

  mCapabilities.reserve(count);
  for (auto cap : composer_caps) {
    mCapabilities.insert(static_cast<hwc2_capability_t>(cap));
  }
}

void QtiComposerClient::enableCallback(bool enable) {
  if (enable) {
    hwc_session_->RegisterCallback(HWC2_CALLBACK_HOTPLUG, this,
                                   reinterpret_cast<hwc2_function_pointer_t>(onHotplug));
    hwc_session_->RegisterCallback(HWC2_CALLBACK_REFRESH, this,
                                   reinterpret_cast<hwc2_function_pointer_t>(onRefresh));
    if (!mUseCallback24_) {
      hwc_session_->RegisterCallback(HWC2_CALLBACK_VSYNC, this,
                                     reinterpret_cast<hwc2_function_pointer_t>(onVsync));
    } else {
      hwc_session_->RegisterCallback(HWC2_CALLBACK_VSYNC_2_4, this,
                                     reinterpret_cast<hwc2_function_pointer_t>(onVsync_2_4));
      hwc_session_->RegisterCallback(HWC2_CALLBACK_VSYNC_PERIOD_TIMING_CHANGED, this,
                             reinterpret_cast<hwc2_function_pointer_t>(onVsyncPeriodTimingChanged));
      hwc_session_->RegisterCallback(HWC2_CALLBACK_SEAMLESS_POSSIBLE, this,
                                     reinterpret_cast<hwc2_function_pointer_t>(onSeamlessPossible));
    }
  } else {
    hwc_session_->RegisterCallback(HWC2_CALLBACK_HOTPLUG, this, nullptr);
    hwc_session_->RegisterCallback(HWC2_CALLBACK_REFRESH, this, nullptr);
    if (!mUseCallback24_) {
      hwc_session_->RegisterCallback(HWC2_CALLBACK_VSYNC, this, nullptr);
    } else {
      hwc_session_->RegisterCallback(HWC2_CALLBACK_VSYNC_2_4, this, nullptr);
      hwc_session_->RegisterCallback(HWC2_CALLBACK_VSYNC_PERIOD_TIMING_CHANGED, this, nullptr);
      hwc_session_->RegisterCallback(HWC2_CALLBACK_SEAMLESS_POSSIBLE, this, nullptr);
    }
  }
}

// Methods from ::android::hardware::graphics::composer::V2_1::IComposerClient follow.
Return<void> QtiComposerClient::registerCallback(
                                            const sp<composer_V2_1::IComposerCallback>& callback) {
  callback_ = callback;
  enableCallback(callback != nullptr);
  return Void();
}

Return<uint32_t> QtiComposerClient::getMaxVirtualDisplayCount() {
  return hwc_session_->GetMaxVirtualDisplayCount();
}

Return<void> QtiComposerClient::createVirtualDisplay(uint32_t width, uint32_t height,
                                                     common_V1_0::PixelFormat formatHint,
                                                     uint32_t outputBufferSlotCount,
                                                     createVirtualDisplay_cb _hidl_cb) {
  // TODO(user): Implement combinedly w.r.t createVirtualDisplay_2_2
  int32_t format = static_cast<int32_t>(formatHint);
  uint64_t display;
  auto error = hwc_session_->CreateVirtualDisplay(width, height, &format, &display);

  if (static_cast<Error>(error) == Error::NONE) {
    std::lock_guard<std::mutex> lock(mDisplayDataMutex);

    auto dpy = mDisplayData.emplace(static_cast<Display>(display), DisplayData(true)).first;
    dpy->second.OutputBuffers.resize(outputBufferSlotCount);
  }

  _hidl_cb(static_cast<Error>(error), display, static_cast<common_V1_0::PixelFormat>(format));
  return Void();
}

Return<composer_V2_1::Error> QtiComposerClient::destroyVirtualDisplay(uint64_t display) {
  auto error = hwc_session_->DestroyVirtualDisplay(display);
  if (static_cast<Error>(error) == Error::NONE) {
    std::lock_guard<std::mutex> lock(mDisplayDataMutex);

    mDisplayData.erase(display);
  }

  return static_cast<Error>(error);
}

Return<void> QtiComposerClient::createLayer(uint64_t display, uint32_t bufferSlotCount,
                                            createLayer_cb _hidl_cb) {
  composer_V2_1::Layer layer = 0;
  auto error = hwc_session_->CreateLayer(display, &layer);
  Error err = static_cast<Error>(error);
  if (err == Error::NONE) {
    std::lock_guard<std::mutex> lock(mDisplayDataMutex);
    auto dpy = mDisplayData.find(display);
    // The display entry may have already been removed by onHotplug.
    if (dpy != mDisplayData.end()) {
      auto ly = dpy->second.Layers.emplace(layer, LayerBuffers()).first;
      ly->second.Buffers.resize(bufferSlotCount);
    } else {
      err = Error::BAD_DISPLAY;
      // Note: We do not destroy the layer on this error as the hotplug
      // disconnect invalidates the display id. The implementation should
      // ensure all layers for the display are destroyed.
    }
  }

  _hidl_cb(err, layer);
  return Void();
}

Return<Error> QtiComposerClient::destroyLayer(uint64_t display, uint64_t layer) {
  auto error = hwc_session_->DestroyLayer(display, layer);
  Error err = static_cast<Error>(error);
  if (err == Error::NONE) {
    std::lock_guard<std::mutex> lock(mDisplayDataMutex);

    auto dpy = mDisplayData.find(display);
    // The display entry may have already been removed by onHotplug.
    if (dpy != mDisplayData.end()) {
      dpy->second.Layers.erase(layer);
    }
  }

  return static_cast<Error>(error);
}

Return<void> QtiComposerClient::getActiveConfig(uint64_t display, getActiveConfig_cb _hidl_cb) {
  uint32_t config = 0;
  auto error = hwc_session_->GetActiveConfig(display, &config);

  _hidl_cb(static_cast<Error>(error), config);

  return Void();
}

Return<Error> QtiComposerClient::getClientTargetSupport(uint64_t display, uint32_t width,
                                                        uint32_t height,
                                                        common_V1_0::PixelFormat format,
                                                        common_V1_0::Dataspace dataspace) {
  auto error = hwc_session_->GetClientTargetSupport(display, width, height,
                                                    static_cast<int32_t>(format),
                                                    static_cast<int32_t>(dataspace));

  return static_cast<Error>(error);
}

Return<void> QtiComposerClient::getColorModes(uint64_t display, getColorModes_cb _hidl_cb) {
  // TODO(user): Implement combinedly w.r.t getColorModes_2_3
  hidl_vec<common_V1_0::ColorMode> modes;
  uint32_t count = 0;

  auto error = hwc_session_->GetColorModes(display, &count, nullptr);
  if (error != HWC2_ERROR_NONE) {
    _hidl_cb(static_cast<Error>(error), modes);
    return Void();
  }

  modes.resize(count);
  error = hwc_session_->GetColorModes(display, &count,
              reinterpret_cast<std::underlying_type<common_V1_0::ColorMode>::type*>(modes.data()));

  _hidl_cb(static_cast<Error>(error), modes);
  return Void();
}

Return<void> QtiComposerClient::getDisplayAttribute(uint64_t display, uint32_t config,
                                               composer_V2_1::IComposerClient::Attribute attribute,
                                               getDisplayAttribute_cb _hidl_cb) {
  int32_t value = 0;
  auto error = hwc_session_->GetDisplayAttribute(
      display, config, static_cast<composer_V2_4::IComposerClient::Attribute>(attribute), &value);

  _hidl_cb(static_cast<Error>(error), value);
  return Void();
}

Return<void> QtiComposerClient::getDisplayConfigs(uint64_t display,
                                                  getDisplayConfigs_cb _hidl_cb) {
  hidl_vec<uint32_t> configs;
  uint32_t count = 0;

  auto error = hwc_session_->GetDisplayConfigs(display, &count, nullptr);
  if (error != HWC2_ERROR_NONE) {
    _hidl_cb(static_cast<Error>(error), configs);
    return Void();
  }

  configs.resize(count);
  error = hwc_session_->GetDisplayConfigs(display, &count, configs.data());

  _hidl_cb(static_cast<Error>(error), configs);
  return Void();
}

Return<void> QtiComposerClient::getDisplayName(uint64_t display, getDisplayName_cb _hidl_cb) {
  uint32_t count = 0;
  hidl_string name_reply;
  std::vector<char> name;

  auto error = hwc_session_->GetDisplayName(display, &count, nullptr);
  if (error != HWC2_ERROR_NONE) {
    _hidl_cb(static_cast<Error>(error), name_reply);
    return Void();
  }

  name.resize(count + 1);
  error = hwc_session_->GetDisplayName(display, &count, name.data());
  if (error != HWC2_ERROR_NONE) {
    _hidl_cb(static_cast<Error>(error), name_reply);
    return Void();
  }

  name.resize(count + 1);
  name[count] = '\0';
  name_reply.setToExternal(name.data(), count);

  _hidl_cb(static_cast<Error>(error), name_reply);
  return Void();
}

Return<void> QtiComposerClient::getDisplayType(uint64_t display, getDisplayType_cb _hidl_cb) {
  int32_t hwc_type;
  auto error = hwc_session_->GetDisplayType(display, &hwc_type);

  _hidl_cb(static_cast<Error>(error), static_cast<IComposerClient::DisplayType>(hwc_type));
  return Void();
}

Return<void> QtiComposerClient::getDozeSupport(uint64_t display, getDozeSupport_cb _hidl_cb) {
  int32_t hwc_support = 0;
  auto error = hwc_session_->GetDozeSupport(display, &hwc_support);

  _hidl_cb(static_cast<Error>(error), hwc_support);
  return Void();
}

Return<void> QtiComposerClient::getHdrCapabilities(uint64_t display,
                                                   getHdrCapabilities_cb _hidl_cb) {
  // TODO(user): Implement combinedly w.r.t getHdrCapabilities_2_3
  uint32_t count = 0;
  hidl_vec<common_V1_0::Hdr> types;
  float max_lumi = 0.0f;
  float max_avg_lumi = 0.0f;
  float min_lumi = 0.0f;

  auto error = hwc_session_->GetHdrCapabilities(display, &count, nullptr, &max_lumi,
                                                &max_avg_lumi, &min_lumi);
  if (error != HWC2_ERROR_NONE) {
    _hidl_cb(static_cast<Error>(error), types, max_lumi, max_avg_lumi, min_lumi);
    return Void();
  }

  types.resize(count);
  error = hwc_session_->GetHdrCapabilities(display, &count,
           reinterpret_cast<std::underlying_type<common_V1_2::Hdr>::type*>(types.data()),
           &max_lumi, &max_avg_lumi, &min_lumi);

  _hidl_cb(static_cast<Error>(error), types, max_lumi, max_avg_lumi, min_lumi);
  return Void();
}

Return<Error> QtiComposerClient::setClientTargetSlotCount(uint64_t display,
                                                          uint32_t clientTargetSlotCount) {
  std::lock_guard<std::mutex> lock(mDisplayDataMutex);

  auto dpy = mDisplayData.find(display);
  if (dpy == mDisplayData.end()) {
    return Error::BAD_DISPLAY;
  }
  dpy->second.ClientTargets.resize(clientTargetSlotCount);

  return Error::NONE;
}

Return<Error> QtiComposerClient::setActiveConfig(uint64_t display, uint32_t config) {
  auto error = hwc_session_->SetActiveConfig(display, config);

  return static_cast<Error>(error);
}

Return<Error> QtiComposerClient::setColorMode(uint64_t display, common_V1_0::ColorMode mode) {
  auto error = hwc_session_->SetColorMode(display, static_cast<int32_t>(mode));

  return static_cast<Error>(error);
}

Return<Error> QtiComposerClient::setPowerMode(uint64_t display,
                                              composer_V2_1::IComposerClient::PowerMode mode) {
  // TODO(user): Implement combinedly w.r.t setPowerMode_2_2
  auto error = hwc_session_->SetPowerMode(display, static_cast<int32_t>(mode));

  return static_cast<Error>(error);
}

Return<Error> QtiComposerClient::setVsyncEnabled(uint64_t display,
                                                 composer_V2_1::IComposerClient::Vsync enabled) {
  auto error = hwc_session_->SetVsyncEnabled(display, static_cast<int32_t>(enabled));

  return static_cast<Error>(error);
}

Return<Error> QtiComposerClient::setInputCommandQueue(
                                                    const MQDescriptorSync<uint32_t>& descriptor) {
  std::lock_guard<std::mutex> lock(mCommandMutex);
  return mReader.setMQDescriptor(descriptor) ? Error::NONE : Error::NO_RESOURCES;
}

Return<void> QtiComposerClient::getOutputCommandQueue(getOutputCommandQueue_cb _hidl_cb) {
  // no locking as we require this function to be called inside
  // executeCommands_cb

  auto outDescriptor = mWriter.getMQDescriptor();
  if (outDescriptor) {
    _hidl_cb(Error::NONE, *outDescriptor);
  } else {
    _hidl_cb(Error::NO_RESOURCES, MQDescriptorSync<uint32_t>());
  }

  return Void();
}

Return<void> QtiComposerClient::executeCommands(uint32_t inLength,
                                                const hidl_vec<hidl_handle>& inHandles,
                                                executeCommands_cb _hidl_cb) {
  std::lock_guard<std::mutex> lock(mCommandMutex);

  bool outChanged = false;
  uint32_t outLength = 0;
  hidl_vec<hidl_handle> outHandles;

  if (!mReader.readQueue(inLength, inHandles)) {
    _hidl_cb(Error::BAD_PARAMETER, outChanged, outLength, outHandles);
    return Void();
  }

  Error err = mReader.parse();
  if (err == Error::NONE &&
      !mWriter.writeQueue(outChanged, outLength, outHandles)) {
    err = Error::NO_RESOURCES;
  }

  _hidl_cb(Error::NONE, outChanged, outLength, outHandles);

  mReader.reset();
  mWriter.reset();

  return Void();
}


// Methods from ::android::hardware::graphics::composer::V2_2::IComposerClient follow.
Return<void> QtiComposerClient::getPerFrameMetadataKeys(uint64_t display,
                                                        getPerFrameMetadataKeys_cb _hidl_cb) {
  // TODO(user): Implement combinedly w.r.t getPerFrameMetadataKeys_2_3
  std::vector<PerFrameMetadataKey_V2> keys;
  uint32_t count = 0;

  auto error = hwc_session_->GetPerFrameMetadataKeys(display, &count, nullptr);
  if (error != HWC2_ERROR_NONE) {
    _hidl_cb(static_cast<Error>(error), keys);
    return Void();
  }

  keys.resize(count);
  error = hwc_session_->GetPerFrameMetadataKeys(display, &count,
               reinterpret_cast<std::underlying_type<PerFrameMetadataKey_V2>::type*>(keys.data()));

  _hidl_cb(static_cast<Error>(error), keys);
  return Void();
}

Return<void> QtiComposerClient::getReadbackBufferAttributes(uint64_t display,
                                                         getReadbackBufferAttributes_cb _hidl_cb) {
  // TODO(user): Implement combinedly w.r.t getReadbackBufferAttributes_2_3
  int32_t format = 0;
  int32_t dataspace = 0;

  auto error = hwc_session_->GetReadbackBufferAttributes(display, &format, &dataspace);

  if (error != HWC2_ERROR_NONE) {
    format = 0;
    dataspace = 0;
  }

  _hidl_cb(static_cast<Error>(error), static_cast<common_V1_1::PixelFormat>(format),
           static_cast<common_V1_1::Dataspace>(dataspace));
  return Void();
}

Return<void> QtiComposerClient::getReadbackBufferFence(uint64_t display,
                                                       getReadbackBufferFence_cb _hidl_cb) {
  shared_ptr<Fence> fence = nullptr;
  auto error = hwc_session_->GetReadbackBufferFence(display, &fence);
  if (static_cast<Error>(error) != Error::NONE) {
    _hidl_cb(static_cast<Error>(error), nullptr);
    return Void();
  }

  NATIVE_HANDLE_DECLARE_STORAGE(fenceStorage, 1, 0);

  _hidl_cb(static_cast<Error>(error), getFenceHandle(fence, fenceStorage));
  return Void();
}

Return<Error> QtiComposerClient::setReadbackBuffer(uint64_t display, const hidl_handle& buffer,
                                                   const hidl_handle& releaseFence) {
  shared_ptr<Fence> fence = nullptr;
  Error error = getFence(releaseFence, &fence, "read_back");
  if (error != Error::NONE) {
    return error;
  }

  const native_handle_t* readbackBuffer;
  getDisplayReadbackBuffer(display, buffer.getNativeHandle(), &readbackBuffer);
  if (error != Error::NONE) {
    return error;
  }

  auto err = hwc_session_->SetReadbackBuffer(display, readbackBuffer, fence);
  return static_cast<Error>(err);
}

Return<void> QtiComposerClient::createVirtualDisplay_2_2(uint32_t width, uint32_t height,
                                                         common_V1_1::PixelFormat formatHint,
                                                         uint32_t outputBufferSlotCount,
                                                         createVirtualDisplay_2_2_cb _hidl_cb) {
  int32_t format = static_cast<int32_t>(formatHint);
  uint64_t display;
  auto error = hwc_session_->CreateVirtualDisplay(width, height, &format, &display);

  if (static_cast<Error>(error) == Error::NONE) {
    std::lock_guard<std::mutex> lock(mDisplayDataMutex);

    auto dpy = mDisplayData.emplace(static_cast<Display>(display), DisplayData(true)).first;
    dpy->second.OutputBuffers.resize(outputBufferSlotCount);
  }

  _hidl_cb(static_cast<Error>(error), display, static_cast<common_V1_1::PixelFormat>(format));
  return Void();
}

Return<Error> QtiComposerClient::getClientTargetSupport_2_2(uint64_t display, uint32_t width,
                                                            uint32_t height,
                                                            common_V1_1::PixelFormat format,
                                                            common_V1_1::Dataspace dataspace) {
  auto error = hwc_session_->GetClientTargetSupport(display, width, height,
                                                    static_cast<int32_t>(format),
                                                    static_cast<int32_t>(dataspace));

  return static_cast<Error>(error);
}

Return<Error> QtiComposerClient::setPowerMode_2_2(uint64_t display,
                                                  composer_V2_2::IComposerClient::PowerMode mode) {
  if (mode == IComposerClient::PowerMode::ON_SUSPEND) {
    return Error::UNSUPPORTED;
  }
  auto error = hwc_session_->SetPowerMode(display, static_cast<int32_t>(mode));

  return static_cast<Error>(error);
}

Return<void> QtiComposerClient::getColorModes_2_2(uint64_t display,
                                                  getColorModes_2_2_cb _hidl_cb) {
  // TODO(user): Implement combinedly w.r.t getColorModes_2_3
  hidl_vec<common_V1_1::ColorMode> modes;
  uint32_t count = 0;

  auto error = hwc_session_->GetColorModes(display, &count, nullptr);
  if (error != HWC2_ERROR_NONE) {
    _hidl_cb(static_cast<Error>(error), modes);
    return Void();
  }

  modes.resize(count);
  error = hwc_session_->GetColorModes(display, &count,
              reinterpret_cast<std::underlying_type<common_V1_1::ColorMode>::type*>(modes.data()));

  _hidl_cb(static_cast<Error>(error), modes);
  return Void();
}

Return<void> QtiComposerClient::getRenderIntents(uint64_t display, common_V1_1::ColorMode mode,
                                                 getRenderIntents_cb _hidl_cb) {
  // TODO(user): Implement combinedly w.r.t getRenderIntents_2_3
  uint32_t count = 0;
  std::vector<RenderIntent> intents;

  auto error = hwc_session_->GetRenderIntents(display, int32_t(mode), &count, nullptr);
  if (error != HWC2_ERROR_NONE) {
    _hidl_cb(static_cast<Error>(error), intents);
    return Void();
  }

  intents.resize(count);
  error = hwc_session_->GetRenderIntents(display, int32_t(mode), &count,
  reinterpret_cast<std::underlying_type<RenderIntent>::type*>(intents.data()));

  _hidl_cb(static_cast<Error>(error), intents);
  return Void();
}

Return<Error> QtiComposerClient::setColorMode_2_2(uint64_t display, common_V1_1::ColorMode mode,
                                                  common_V1_1::RenderIntent intent) {
  auto error = hwc_session_->SetColorModeWithRenderIntent(display, static_cast<int32_t>(mode),
                                                          static_cast<int32_t>(intent));

  return static_cast<Error>(error);
}

Return<void> QtiComposerClient::getDataspaceSaturationMatrix(common_V1_1::Dataspace dataspace,
                                                        getDataspaceSaturationMatrix_cb _hidl_cb) {
  if (dataspace != common_V1_1::Dataspace::SRGB_LINEAR) {
    _hidl_cb(Error::BAD_PARAMETER, std::array<float, 16>{0.0f}.data());
    return Void();
  }

  std::array<float, 16> matrix;
  int32_t error = HWC2_ERROR_UNSUPPORTED;
  error = hwc_session_->GetDataspaceSaturationMatrix(static_cast<int32_t>(dataspace),
                                                     matrix.data());
  if (error != HWC2_ERROR_NONE) {
    matrix = {
      1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
    };
  }
  _hidl_cb(Error::NONE, matrix.data());
  return Void();
}

Return<void> QtiComposerClient::executeCommands_2_2(uint32_t inLength,
                                                    const hidl_vec<hidl_handle>& inHandles,
                                                    executeCommands_2_2_cb _hidl_cb) {
  std::lock_guard<std::mutex> lock(mCommandMutex);

  bool outChanged = false;
  uint32_t outLength = 0;
  hidl_vec<hidl_handle> outHandles;

  if (!mReader.readQueue(inLength, inHandles)) {
    _hidl_cb(Error::BAD_PARAMETER, outChanged, outLength, outHandles);
    return Void();
  }

  Error err = mReader.parse();
  if (err == Error::NONE &&
      !mWriter.writeQueue(outChanged, outLength, outHandles)) {
      err = Error::NO_RESOURCES;
  }

  _hidl_cb(Error::NONE, outChanged, outLength, outHandles);

  mReader.reset();
  mWriter.reset();

  return Void();
}


// Methods from ::android::hardware::graphics::composer::V2_3::IComposerClient follow.
Return<void> QtiComposerClient::getDisplayIdentificationData(uint64_t display,
                                                        getDisplayIdentificationData_cb _hidl_cb) {
  uint8_t port = 0;
  uint32_t size = 0;
  std::vector<uint8_t> data(size);

  auto error = hwc_session_->GetDisplayIdentificationData(display, &port, &size, nullptr);
  if (error != HWC2_ERROR_NONE) {
    _hidl_cb(static_cast<Error>(error), port, data);
    return Void();
  }

  data.resize(size);
  error = hwc_session_->GetDisplayIdentificationData(display, &port, &size, data.data());

  _hidl_cb(static_cast<Error>(error), port, data);
  return Void();
}

Return<void> QtiComposerClient::getReadbackBufferAttributes_2_3(uint64_t display,
                                                     getReadbackBufferAttributes_2_3_cb _hidl_cb) {
  int32_t format = 0;
  int32_t dataspace = 0;

  auto error = hwc_session_->GetReadbackBufferAttributes(display, &format, &dataspace);

  if (error != HWC2_ERROR_NONE) {
    format = 0;
    dataspace = 0;
  }

  _hidl_cb(static_cast<Error>(error), static_cast<common_V1_2::PixelFormat>(format),
           static_cast<common_V1_2::Dataspace>(dataspace));
  return Void();
}

Return<Error> QtiComposerClient::getClientTargetSupport_2_3(uint64_t display, uint32_t width,
                                                            uint32_t height,
                                                            common_V1_2::PixelFormat format,
                                                            common_V1_2::Dataspace dataspace) {
  auto error = hwc_session_->GetClientTargetSupport(display, width, height,
                                                    static_cast<int32_t>(format),
                                                    static_cast<int32_t>(dataspace));

  return static_cast<Error>(error);
}

Return<void> QtiComposerClient::getDisplayedContentSamplingAttributes(uint64_t display,
                                               getDisplayedContentSamplingAttributes_cb _hidl_cb) {
  // getDisplayedContentSamplingAttributes is not supported
  int constexpr invalid = -1;
  auto error = Error::UNSUPPORTED;
  common_V1_2::PixelFormat format = static_cast<common_V1_2::PixelFormat>(invalid);
  common_V1_2::Dataspace dataspace = static_cast<common_V1_2::Dataspace>(invalid);
  hidl_bitfield<IComposerClient::FormatColorComponent> componentMask =
    static_cast<hidl_bitfield<IComposerClient::FormatColorComponent>>(invalid);

  _hidl_cb(error, format, dataspace, componentMask);
  return Void();
}

Return<Error> QtiComposerClient::setDisplayedContentSamplingEnabled(uint64_t display,
                                   composer_V2_3::IComposerClient::DisplayedContentSampling enable,
                                   hidl_bitfield<FormatColorComponent> componentMask,
                                   uint64_t maxFrames) {
  // setDisplayedContentSamplingEnabled is not supported
  return Error::UNSUPPORTED;
}

Return<void> QtiComposerClient::getDisplayedContentSample(uint64_t display, uint64_t maxFrames,
                                                          uint64_t timestamp,
                                                          getDisplayedContentSample_cb _hidl_cb) {
  // getDisplayedContentSample is not supported
  auto error = Error::UNSUPPORTED;
  uint64_t frameCount = 0;
  hidl_vec<uint64_t> sampleComponent0 = 0;
  hidl_vec<uint64_t> sampleComponent1 = 0;
  hidl_vec<uint64_t> sampleComponent2 = 0;
  hidl_vec<uint64_t> sampleComponent3 = 0;

  _hidl_cb(error, frameCount, sampleComponent0, sampleComponent1, sampleComponent2,
           sampleComponent3);
  return Void();
}

Return<void> QtiComposerClient::executeCommands_2_3(uint32_t inLength,
                                                    const hidl_vec<hidl_handle>& inHandles,
                                                    executeCommands_2_3_cb _hidl_cb) {
  // TODO(user): Implement combinedly w.r.t executeCommands_2_2
  std::lock_guard<std::mutex> lock(mCommandMutex);

  bool outChanged = false;
  uint32_t outLength = 0;
  hidl_vec<hidl_handle> outHandles;

  if (!mReader.readQueue(inLength, inHandles)) {
    _hidl_cb(Error::BAD_PARAMETER, outChanged, outLength, outHandles);
    return Void();
  }

  Error err = mReader.parse();
  if (err == Error::NONE &&
      !mWriter.writeQueue(outChanged, outLength, outHandles)) {
      err = Error::NO_RESOURCES;
  }

  _hidl_cb(Error::NONE, outChanged, outLength, outHandles);

  mReader.reset();
  mWriter.reset();

  return Void();
}

Return<void> QtiComposerClient::getRenderIntents_2_3(uint64_t display, common_V1_2::ColorMode mode,
                                                     getRenderIntents_2_3_cb _hidl_cb) {
  uint32_t count = 0;
  std::vector<RenderIntent> intents;

  auto error = hwc_session_->GetRenderIntents(display, int32_t(mode), &count, nullptr);
  if (error != HWC2_ERROR_NONE) {
    _hidl_cb(static_cast<Error>(error), intents);
    return Void();
  }

  intents.resize(count);
  error = hwc_session_->GetRenderIntents(display, int32_t(mode), &count,
  reinterpret_cast<std::underlying_type<RenderIntent>::type*>(intents.data()));

  _hidl_cb(static_cast<Error>(error), intents);
  return Void();
}

Return<void> QtiComposerClient::getColorModes_2_3(uint64_t display,
                                                  getColorModes_2_3_cb _hidl_cb) {
  hidl_vec<common_V1_2::ColorMode> modes;
  uint32_t count = 0;

  auto error = hwc_session_->GetColorModes(display, &count, nullptr);
  if (error != HWC2_ERROR_NONE) {
    _hidl_cb(static_cast<Error>(error), modes);
    return Void();
  }

  modes.resize(count);
  error = hwc_session_->GetColorModes(display, &count,
              reinterpret_cast<std::underlying_type<common_V1_2::ColorMode>::type*>(modes.data()));

  _hidl_cb(static_cast<Error>(error), modes);
  return Void();
}

Return<Error> QtiComposerClient::setColorMode_2_3(uint64_t display, common_V1_2::ColorMode mode,
                                                  common_V1_1::RenderIntent intent) {
  auto error = hwc_session_->SetColorModeWithRenderIntent(display, static_cast<int32_t>(mode),
                                                          static_cast<int32_t>(intent));

  return static_cast<Error>(error);
}

Return<void> QtiComposerClient::getDisplayCapabilities(uint64_t display,
                                                       getDisplayCapabilities_cb _hidl_cb) {
  // We only care about passing VTS for older composer versions
  // Not returning any capabilities that are optional

  hidl_vec<DisplayCapability_V2_3> capabilities;

  uint32_t count = 0;
  auto error = hwc_session_->GetDisplayCapabilities2_3(display, &count, nullptr);
  if (error != HWC2_ERROR_NONE) {
    _hidl_cb(static_cast<Error>(error), capabilities);
    return Void();
  }

  capabilities.resize(count);
  error = hwc_session_->GetDisplayCapabilities2_3(display, &count,
                 reinterpret_cast<std::underlying_type<DisplayCapability_V2_3>::type*>(
                 capabilities.data()));
   if (error != HWC2_ERROR_NONE) {
     capabilities = hidl_vec<DisplayCapability_V2_3>();
     _hidl_cb(static_cast<Error>(error), capabilities);
     return Void();
   }

   _hidl_cb(static_cast<Error>(error), capabilities);
   return Void();
}

Return<void> QtiComposerClient::getPerFrameMetadataKeys_2_3(uint64_t display,
                                                         getPerFrameMetadataKeys_2_3_cb _hidl_cb) {
  std::vector<PerFrameMetadataKey> keys;
  uint32_t count = 0;

  auto error = hwc_session_->GetPerFrameMetadataKeys(display, &count, nullptr);
  if (error != HWC2_ERROR_NONE) {
    _hidl_cb(static_cast<Error>(error), keys);
    return Void();
  }

  keys.resize(count);
  error = hwc_session_->GetPerFrameMetadataKeys(display, &count,
                  reinterpret_cast<std::underlying_type<PerFrameMetadataKey>::type*>(keys.data()));

  _hidl_cb(static_cast<Error>(error), keys);
  return Void();
}

Return<void> QtiComposerClient::getHdrCapabilities_2_3(uint64_t display,
                                                       getHdrCapabilities_2_3_cb _hidl_cb) {
  uint32_t count = 0;
  hidl_vec<common_V1_2::Hdr> types;
  float max_lumi = 0.0f;
  float max_avg_lumi = 0.0f;
  float min_lumi = 0.0f;

  auto error = hwc_session_->GetHdrCapabilities(display, &count, nullptr, &max_lumi,
                                                &max_avg_lumi, &min_lumi);
  if (error != HWC2_ERROR_NONE) {
    _hidl_cb(static_cast<Error>(error), types, max_lumi, max_avg_lumi, min_lumi);
    return Void();
  }

  types.resize(count);
  error = hwc_session_->GetHdrCapabilities(display, &count,
           reinterpret_cast<std::underlying_type<common_V1_2::Hdr>::type*>(types.data()),
           &max_lumi, &max_avg_lumi, &min_lumi);

  _hidl_cb(static_cast<Error>(error), types, max_lumi, max_avg_lumi, min_lumi);
  return Void();
}

Return<void> QtiComposerClient::getDisplayBrightnessSupport(uint64_t display,
                                                         getDisplayBrightnessSupport_cb _hidl_cb) {
  bool support = false;
  auto error = hwc_session_->GetDisplayBrightnessSupport(display, &support);

  _hidl_cb(static_cast<Error>(error), support);
  return Void();
}

Return<Error> QtiComposerClient::setDisplayBrightness(uint64_t display, float brightness) {
  if (std::isnan(brightness) || brightness > 1.0f || (brightness < 0.0f && brightness != -1.0f)) {
    return Error::BAD_PARAMETER;
  }

  auto error = hwc_session_->SetDisplayBrightness(display, brightness);
  return static_cast<Error>(error);
}

// Methods from ::android::hardware::graphics::composer::V2_4::IComposerClient follow.
Return<void> QtiComposerClient::registerCallback_2_4(
    const sp<composer_V2_4::IComposerCallback> &callback) {
  callback_ = sp<composer_V2_1::IComposerCallback>(callback.get());
  callback24_ = callback;
  mUseCallback24_ = true;
  enableCallback(callback != nullptr);
  return Void();
}
Return<void> QtiComposerClient::getDisplayCapabilities_2_4(uint64_t display,
                                                           getDisplayCapabilities_2_4_cb _hidl_cb) {
  hidl_vec<composer_V2_4::IComposerClient::DisplayCapability> capabilities;
  auto error = hwc_session_->GetDisplayCapabilities(display, &capabilities);
  _hidl_cb(static_cast<composer_V2_4::Error>(error), capabilities);
  return Void();
}

Return<void> QtiComposerClient::getDisplayConnectionType(uint64_t display,
                                                         getDisplayConnectionType_cb _hidl_cb) {
  HwcDisplayConnectionType type;
  auto error = hwc_session_->GetDisplayConnectionType(display, &type);
  _hidl_cb(static_cast<composer_V2_4::Error>(error), type);
  return Void();
}

Return<void> QtiComposerClient::getDisplayAttribute_2_4(
    uint64_t display, uint32_t config, composer_V2_4::IComposerClient::Attribute attribute,
    getDisplayAttribute_2_4_cb _hidl_cb) {
  int32_t value = 0;
  auto error = hwc_session_->GetDisplayAttribute(display, config, attribute, &value);
  _hidl_cb(static_cast<composer_V2_4::Error>(error), value);
  return Void();
}

Return<void> QtiComposerClient::getDisplayVsyncPeriod(uint64_t display,
                                                      getDisplayVsyncPeriod_cb _hidl_cb) {
  VsyncPeriodNanos vsync_period;
  auto error = hwc_session_->GetDisplayVsyncPeriod(display, &vsync_period);
  _hidl_cb(static_cast<composer_V2_4::Error>(error), vsync_period);
  return Void();
}

Return<void> QtiComposerClient::setActiveConfigWithConstraints(
    uint64_t display, uint32_t config,
    const VsyncPeriodChangeConstraints &vsyncPeriodChangeConstraints,
    setActiveConfigWithConstraints_cb _hidl_cb) {
  VsyncPeriodChangeTimeline timeline;
  timeline.newVsyncAppliedTimeNanos = systemTime();
  timeline.refreshRequired = false;
  timeline.refreshTimeNanos = 0;

  auto error = hwc_session_->SetActiveConfigWithConstraints(
      display, config, &vsyncPeriodChangeConstraints, &timeline);
  _hidl_cb(static_cast<composer_V2_4::Error>(error), timeline);
  return Void();
}

Return<composer_V2_4::Error> QtiComposerClient::setAutoLowLatencyMode(uint64_t display, bool on) {
  if (mDisplayData.find(display) == mDisplayData.end()) {
    return composer_V2_4::Error::BAD_DISPLAY;
  }
  return composer_V2_4::Error::UNSUPPORTED;
}

Return<void> QtiComposerClient::getSupportedContentTypes(uint64_t display,
                                                         getSupportedContentTypes_cb _hidl_cb) {
  hidl_vec<composer_V2_4::IComposerClient::ContentType> types = {};
  if (mDisplayData.find(display) == mDisplayData.end()) {
    _hidl_cb(composer_V2_4::Error::BAD_DISPLAY, types);
    return Void();
  }
  _hidl_cb(composer_V2_4::Error::NONE, types);
  return Void();
}

Return<composer_V2_4::Error> QtiComposerClient::setContentType(
    uint64_t display, composer_V2_4::IComposerClient::ContentType type) {
  if (mDisplayData.find(display) == mDisplayData.end()) {
    return composer_V2_4::Error::BAD_DISPLAY;
  }
  if (type == composer_V2_4::IComposerClient::ContentType::NONE) {
    return composer_V2_4::Error::NONE;
  }
  return composer_V2_4::Error::UNSUPPORTED;
}

Return<void> QtiComposerClient::getLayerGenericMetadataKeys(
    getLayerGenericMetadataKeys_cb _hidl_cb) {
  hidl_vec<composer_V2_4::IComposerClient::LayerGenericMetadataKey> keys = {};
  _hidl_cb(composer_V2_4::Error::NONE, keys);
  return Void();
}

QtiComposerClient::CommandReader::CommandReader(QtiComposerClient& client)
  : mClient(client), mWriter(client.mWriter) {
}

bool QtiComposerClient::CommandReader::parseCommonCmd(
    IComposerClient::Command command, uint16_t length) {
  bool parsed = false;

  switch (command) {
  // Commands from ::android::hardware::graphics::composer::V2_1::IComposerClient follow.
  case IComposerClient::Command::SELECT_DISPLAY:
    parsed = parseSelectDisplay(length);
    // Displays will not be removed while processing the command queue.
    if (parsed && mClient.mDisplayData.find(mDisplay) == mClient.mDisplayData.end()) {
      ALOGW("Command::SELECT_DISPLAY: Display %" PRId64 "not found. Dropping commands.", mDisplay);
      mDisplay = sdm::HWCCallbacks::kNumDisplays;
    }
    break;
  case IComposerClient::Command::SELECT_LAYER:
    parsed = parseSelectLayer(length);
    break;
  case IComposerClient::Command::SET_COLOR_TRANSFORM:
    parsed = parseSetColorTransform(length);
    break;
  case IComposerClient::Command::SET_CLIENT_TARGET:
    parsed = parseSetClientTarget(length);
    break;
  case IComposerClient::Command::SET_OUTPUT_BUFFER:
    parsed = parseSetOutputBuffer(length);
    break;
  case IComposerClient::Command::VALIDATE_DISPLAY:
    parsed = parseValidateDisplay(length);
    break;
  case IComposerClient::Command::ACCEPT_DISPLAY_CHANGES:
    parsed = parseAcceptDisplayChanges(length);
    break;
  case IComposerClient::Command::PRESENT_DISPLAY:
    parsed = parsePresentDisplay(length);
    break;
  case IComposerClient::Command::PRESENT_OR_VALIDATE_DISPLAY:
    parsed = parsePresentOrValidateDisplay(length);
    break;
  case IComposerClient::Command::SET_LAYER_CURSOR_POSITION:
    parsed = parseSetLayerCursorPosition(length);
    break;
  case IComposerClient::Command::SET_LAYER_BUFFER:
    parsed = parseSetLayerBuffer(length);
    break;
  case IComposerClient::Command::SET_LAYER_SURFACE_DAMAGE:
    parsed = parseSetLayerSurfaceDamage(length);
    break;
  case IComposerClient::Command::SET_LAYER_BLEND_MODE:
    parsed = parseSetLayerBlendMode(length);
    break;
  case IComposerClient::Command::SET_LAYER_COLOR:
    parsed = parseSetLayerColor(length);
    break;
  case IComposerClient::Command::SET_LAYER_COMPOSITION_TYPE:
    parsed = parseSetLayerCompositionType(length);
    break;
  case IComposerClient::Command::SET_LAYER_DATASPACE:
    parsed = parseSetLayerDataspace(length);
    break;
  case IComposerClient::Command::SET_LAYER_DISPLAY_FRAME:
    parsed = parseSetLayerDisplayFrame(length);
    break;
  case IComposerClient::Command::SET_LAYER_PLANE_ALPHA:
    parsed = parseSetLayerPlaneAlpha(length);
    break;
  case IComposerClient::Command::SET_LAYER_SIDEBAND_STREAM:
    parsed = parseSetLayerSidebandStream(length);
    break;
  case IComposerClient::Command::SET_LAYER_SOURCE_CROP:
    parsed = parseSetLayerSourceCrop(length);
    break;
  case IComposerClient::Command::SET_LAYER_TRANSFORM:
    parsed = parseSetLayerTransform(length);
    break;
  case IComposerClient::Command::SET_LAYER_VISIBLE_REGION:
    parsed = parseSetLayerVisibleRegion(length);
    break;
  case IComposerClient::Command::SET_LAYER_Z_ORDER:
    parsed = parseSetLayerZOrder(length);
    break;
  // Commands from ::android::hardware::graphics::composer::V2_2::IComposerClient follow.
  case IComposerClient::Command::SET_LAYER_PER_FRAME_METADATA:
    parsed = parseSetLayerPerFrameMetadata(length);
    break;
  case IComposerClient::Command::SET_LAYER_FLOAT_COLOR:
    parsed = parseSetLayerFloatColor(length);
    break;
  // Commands from ::android::hardware::graphics::composer::V2_3::IComposerClient follow.
  case IComposerClient::Command::SET_LAYER_COLOR_TRANSFORM:
    parsed = parseSetLayerColorTransform(length);
    break;
  case IComposerClient::Command::SET_LAYER_PER_FRAME_METADATA_BLOBS:
    parsed = parseSetLayerPerFrameMetadataBlobs(length);
    break;
  default:
    parsed = false;
    break;
  }

  return parsed;
}

Error QtiComposerClient::CommandReader::parse() {
  IQtiComposerClient::Command qticommand;
  uint16_t length;

  while (!isEmpty()) {
    if (!beginCommand(qticommand, length)) {
      break;
    }

    bool parsed = false;
    switch (qticommand) {
      case IQtiComposerClient::Command::SET_LAYER_TYPE:
        parsed = parseSetLayerType(length);
        break;
      case IQtiComposerClient::Command::SET_DISPLAY_ELAPSE_TIME:
        parsed = parseSetDisplayElapseTime(length);
        break;
      default:
        parsed = parseCommonCmd(static_cast<IComposerClient::Command>(qticommand), length);
        break;
    }

    endCommand();

    if (!parsed) {
      ALOGE("failed to parse command 0x%x, length %" PRIu16,
          qticommand, length);
      break;
    }
  }

  return (isEmpty()) ? Error::NONE : Error::BAD_PARAMETER;
}

bool QtiComposerClient::CommandReader::parseSelectDisplay(uint16_t length) {
  if (length != CommandWriter::kSelectDisplayLength) {
    return false;
  }

  mDisplay = read64();
  mWriter.selectDisplay(mDisplay);

  return true;
}

bool QtiComposerClient::CommandReader::parseSelectLayer(uint16_t length) {
  if (length != CommandWriter::kSelectLayerLength) {
    return false;
  }

  mLayer = read64();

  return true;
}

bool QtiComposerClient::CommandReader::parseSetColorTransform(uint16_t length) {
  if (length != CommandWriter::kSetColorTransformLength) {
    return false;
  }

  float matrix[16];
  for (int i = 0; i < 16; i++) {
    matrix[i] = readFloat();
  }
  auto transform = readSigned();

  auto err = mClient.hwc_session_->SetColorTransform(mDisplay, matrix, transform);
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetClientTarget(uint16_t length) {
  // 4 parameters followed by N rectangles
  if ((length - 4) % 4 != 0) {
    return false;
  }

  bool useCache = false;
  auto slot = read();
  auto clientTarget = readHandle(useCache);
  shared_ptr<Fence> fence = nullptr;
  readFence(&fence, "fbt");
  auto dataspace = readSigned();
  auto damage = readRegion((length - 4) / 4);
  hwc_region region = {damage.size(), damage.data()};
  auto err = lookupBuffer(BufferCache::CLIENT_TARGETS, slot, useCache, clientTarget, &clientTarget);
  if (err == Error::NONE) {
    auto error = mClient.hwc_session_->SetClientTarget(mDisplay, clientTarget, fence,
        dataspace, region);
    err = static_cast<Error>(error);
    auto updateBufErr = updateBuffer(BufferCache::CLIENT_TARGETS, slot,
        useCache, clientTarget);
    if (err == Error::NONE) {
      err = updateBufErr;
    }
  }
  if (err != Error::NONE) {
    mWriter.setError(getCommandLoc(), err);
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetOutputBuffer(uint16_t length) {
  if (length != CommandWriter::kSetOutputBufferLength) {
    return false;
  }

  bool useCache;
  auto slot = read();
  auto outputBuffer = readHandle(useCache);
  shared_ptr<Fence> fence = nullptr;
  readFence(&fence, "outbuf");
  auto err = lookupBuffer(BufferCache::OUTPUT_BUFFERS, slot, useCache, outputBuffer, &outputBuffer);
  if (err == Error::NONE) {
    auto error = mClient.hwc_session_->SetOutputBuffer(mDisplay, outputBuffer, fence);
    err = static_cast<Error>(error);
    auto updateBufErr = updateBuffer(BufferCache::OUTPUT_BUFFERS, slot, useCache, outputBuffer);
    if (err == Error::NONE) {
      err = updateBufErr;
    }
  }

  if (err != Error::NONE) {
    mWriter.setError(getCommandLoc(), err);
  }

  return true;
}

Error QtiComposerClient::CommandReader::validateDisplay(Display display,
                                       std::vector<Layer>& changedLayers,
                                       std::vector<IComposerClient::Composition>& compositionTypes,
                                       uint32_t& displayRequestMask,
                                       std::vector<Layer>& requestedLayers,
                                       std::vector<uint32_t>& requestMasks,
                                       IComposerClient::ClientTargetProperty& clientTargetProperty) {
  uint32_t types_count = 0;
  uint32_t reqs_count = 0;

  auto err = mClient.hwc_session_->ValidateDisplay(mDisplay, &types_count, &reqs_count);
  if (err != HWC2_ERROR_NONE && err != HWC2_ERROR_HAS_CHANGES) {
    return static_cast<Error>(err);
  }

  err = mClient.hwc_session_->GetChangedCompositionTypes(mDisplay, &types_count, nullptr, nullptr);
  if (err != HWC2_ERROR_NONE) {
    return static_cast<Error>(err);
  }

  changedLayers.resize(types_count);
  compositionTypes.resize(types_count);
  err = mClient.hwc_session_->GetChangedCompositionTypes(mDisplay, &types_count,
                        changedLayers.data(),
                        reinterpret_cast<std::underlying_type<IComposerClient::Composition>::type*>(
                        compositionTypes.data()));

  if (err != HWC2_ERROR_NONE) {
    changedLayers.clear();
    compositionTypes.clear();
    return static_cast<Error>(err);
  }

  int32_t display_reqs = 0;
  err = mClient.hwc_session_->GetDisplayRequests(mDisplay, &display_reqs, &reqs_count, nullptr,
                                                 nullptr);
  if (err != HWC2_ERROR_NONE) {
    changedLayers.clear();
    compositionTypes.clear();
    return static_cast<Error>(err);
  }

  requestedLayers.resize(reqs_count);
  requestMasks.resize(reqs_count);
  err = mClient.hwc_session_->GetDisplayRequests(mDisplay, &display_reqs, &reqs_count,
                                                 requestedLayers.data(),
                                                 reinterpret_cast<int32_t*>(requestMasks.data()));
  if (err != HWC2_ERROR_NONE) {
    changedLayers.clear();
    compositionTypes.clear();

    requestedLayers.clear();
    requestMasks.clear();
    return static_cast<Error>(err);
  }

  displayRequestMask = display_reqs;

  err = mClient.hwc_session_->GetClientTargetProperty(mDisplay, &clientTargetProperty);
  if (err != HWC2_ERROR_NONE) {
    // todo: reset to default values
    return static_cast<Error>(err);
  }

  return static_cast<Error>(err);
}

bool QtiComposerClient::CommandReader::parseValidateDisplay(uint16_t length) {
  if (length != CommandWriter::kValidateDisplayLength) {
    return false;
  }

  std::vector<Layer> changedLayers;
  std::vector<IComposerClient::Composition> compositionTypes;
  uint32_t displayRequestMask;
  std::vector<Layer> requestedLayers;
  std::vector<uint32_t> requestMasks;
  IComposerClient::ClientTargetProperty clientTargetProperty;

  auto err = validateDisplay(mDisplay, changedLayers, compositionTypes, displayRequestMask,
                             requestedLayers, requestMasks, clientTargetProperty);

  if (static_cast<Error>(err) == Error::NONE) {
    mWriter.setChangedCompositionTypes(changedLayers, compositionTypes);
    mWriter.setDisplayRequests(displayRequestMask, requestedLayers, requestMasks);
    if (mClient.mUseCallback24_) {
      mWriter.setClientTargetProperty(clientTargetProperty);
    }
  } else {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseAcceptDisplayChanges(uint16_t length) {
  if (length != CommandWriter::kAcceptDisplayChangesLength) {
    return false;
  }

  auto err = mClient.hwc_session_->AcceptDisplayChanges(mDisplay);
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

Error QtiComposerClient::CommandReader::presentDisplay(Display display,
                                                  shared_ptr<Fence>* presentFence,
                                                  std::vector<Layer>& layers,
                                                  std::vector<shared_ptr<Fence>>& releaseFences) {
  int32_t err = mClient.hwc_session_->PresentDisplay(display, presentFence);
  if (err != HWC2_ERROR_NONE) {
    return static_cast<Error>(err);
  }

  uint32_t count = 0;
  err = mClient.hwc_session_->GetReleaseFences(display, &count, nullptr, nullptr);
  if (err != HWC2_ERROR_NONE) {
    ALOGW("failed to get release fences");
    return Error::NONE;
  }

  layers.resize(count);
  releaseFences.resize(count);
  err = mClient.hwc_session_->GetReleaseFences(display, &count, layers.data(), &releaseFences);
  if (err != HWC2_ERROR_NONE) {
    ALOGW("failed to get release fences");
    layers.clear();
    releaseFences.clear();
    return Error::NONE;
  }

  return static_cast<Error>(err);
}

bool QtiComposerClient::CommandReader::parsePresentDisplay(uint16_t length) {
  if (length != CommandWriter::kPresentDisplayLength) {
    return false;
  }

  shared_ptr<Fence> presentFence = nullptr;
  std::vector<Layer> layers;
  std::vector<shared_ptr<Fence>> fences;

  auto err = presentDisplay(mDisplay, &presentFence, layers, fences);
  if (err == Error::NONE) {
    mWriter.setPresentFence(presentFence);
    mWriter.setReleaseFences(layers, fences);
  } else {
    mWriter.setError(getCommandLoc(), err);
  }

  return true;
}

bool QtiComposerClient::CommandReader::parsePresentOrValidateDisplay(uint16_t length) {
  if (length != CommandWriter::kPresentOrValidateDisplayLength) {
     return false;
  }

  // First try to Present as is.
  mClient.getCapabilities();
  if (mClient.hasCapability(HWC2_CAPABILITY_SKIP_VALIDATE)) {
    shared_ptr<Fence> presentFence = nullptr;
    std::vector<Layer> layers;
    std::vector<shared_ptr<Fence>> fences;
    auto err = presentDisplay(mDisplay, &presentFence, layers, fences);
    if (err == Error::NONE) {
      mWriter.setPresentOrValidateResult(1);
      mWriter.setPresentFence(presentFence);
      mWriter.setReleaseFences(layers, fences);
      return true;
    }
  }

  // Present has failed. We need to fallback to validate
  std::vector<Layer> changedLayers;
  std::vector<IComposerClient::Composition> compositionTypes;
  uint32_t displayRequestMask = 0x0;
  std::vector<Layer> requestedLayers;
  std::vector<uint32_t> requestMasks;
  IComposerClient::ClientTargetProperty clientTargetProperty;

  auto err = validateDisplay(mDisplay, changedLayers, compositionTypes, displayRequestMask,
                             requestedLayers, requestMasks, clientTargetProperty);
  // mResources->setDisplayMustValidateState(mDisplay, false);
  if (err == Error::NONE) {
    mWriter.setPresentOrValidateResult(0);
    mWriter.setChangedCompositionTypes(changedLayers, compositionTypes);
    mWriter.setDisplayRequests(displayRequestMask, requestedLayers, requestMasks);
    if (mClient.mUseCallback24_) {
      mWriter.setClientTargetProperty(clientTargetProperty);
    }
  } else {
    mWriter.setError(getCommandLoc(), err);
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerCursorPosition(uint16_t length) {
  if (length != CommandWriter::kSetLayerCursorPositionLength) {
    return false;
  }

  auto err = mClient.hwc_session_->SetCursorPosition(mDisplay, mLayer, readSigned(), readSigned());
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerBuffer(uint16_t length) {
  if (length != CommandWriter::kSetLayerBufferLength) {
    return false;
  }

  bool useCache;
  auto slot = read();
  auto buffer = readHandle(useCache);
  shared_ptr<Fence> fence = nullptr;
  readFence(&fence, "layer");
  auto error = lookupBuffer(BufferCache::LAYER_BUFFERS, slot, useCache, buffer, &buffer);
  if (error == Error::NONE) {
    auto err = mClient.hwc_session_->SetLayerBuffer(mDisplay, mLayer, buffer, fence);
    error = static_cast<Error>(err);
    auto updateBufErr = updateBuffer(BufferCache::LAYER_BUFFERS, slot, useCache, buffer);
    if (static_cast<Error>(error) == Error::NONE) {
      error = updateBufErr;
    }
  }
  if (static_cast<Error>(error) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(error));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerSurfaceDamage(uint16_t length) {
  // N rectangles
  if (length % 4 != 0) {
    return false;
  }

  auto damage = readRegion(length / 4);
  hwc_region region = {damage.size(), damage.data()};
  auto err = mClient.hwc_session_->SetLayerSurfaceDamage(mDisplay, mLayer, region);
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerBlendMode(uint16_t length) {
  if (length != CommandWriter::kSetLayerBlendModeLength) {
    return false;
  }

  auto err = mClient.hwc_session_->SetLayerBlendMode(mDisplay, mLayer, readSigned());
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerColor(uint16_t length) {
  if (length != CommandWriter::kSetLayerColorLength) {
    return false;
  }
  auto color = readColor();
  hwc_color_t hwc_color{color.r, color.g, color.b, color.a};
  auto err = mClient.hwc_session_->SetLayerColor(mDisplay, mLayer, hwc_color);
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerCompositionType(uint16_t length) {
  if (length != CommandWriter::kSetLayerCompositionTypeLength) {
    return false;
  }

  auto err = mClient.hwc_session_->SetLayerCompositionType(mDisplay, mLayer, readSigned());
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerDataspace(uint16_t length) {
  if (length != CommandWriter::kSetLayerDataspaceLength) {
    return false;
  }

  auto err = mClient.hwc_session_->SetLayerDataspace(mDisplay, mLayer, readSigned());
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerDisplayFrame(uint16_t length) {
  if (length != CommandWriter::kSetLayerDisplayFrameLength) {
    return false;
  }

  auto err = mClient.hwc_session_->SetLayerDisplayFrame(mDisplay, mLayer, readRect());
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerPlaneAlpha(uint16_t length) {
  if (length != CommandWriter::kSetLayerPlaneAlphaLength) {
    return false;
  }

  auto err = mClient.hwc_session_->SetLayerPlaneAlpha(mDisplay, mLayer, readFloat());
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerSidebandStream(uint16_t length) {
  if (length != CommandWriter::kSetLayerSidebandStreamLength) {
    return false;
  }

  // Sideband stream is not supported
  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerSourceCrop(uint16_t length) {
  if (length != CommandWriter::kSetLayerSourceCropLength) {
    return false;
  }

  auto err = mClient.hwc_session_->SetLayerSourceCrop(mDisplay, mLayer, readFRect());
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerTransform(uint16_t length) {
  if (length != CommandWriter::kSetLayerTransformLength) {
    return false;
  }

  auto err = mClient.hwc_session_->SetLayerTransform(mDisplay, mLayer, readSigned());
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerVisibleRegion(uint16_t length) {
  // N rectangles
  if (length % 4 != 0) {
    return false;
  }

  auto region = readRegion(length / 4);
  hwc_region visibleRegion = {region.size(), region.data()};
  auto err = mClient.hwc_session_->SetLayerVisibleRegion(mDisplay, mLayer, visibleRegion);
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerZOrder(uint16_t length) {
  if (length != CommandWriter::kSetLayerZOrderLength) {
    return false;
  }

  auto err = mClient.hwc_session_->SetLayerZOrder(mDisplay, mLayer, read());
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerType(uint16_t length) {
  if (length != CommandWriter::kSetLayerTypeLength) {
    return false;
  }

  auto err = mClient.hwc_session_->SetLayerType(mDisplay, mLayer,
            static_cast<IQtiComposerClient::LayerType>(read()));
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerPerFrameMetadata(uint16_t length) {
  // (key, value) pairs
  if (length % 2 != 0) {
    return false;
  }

  std::vector<IComposerClient::PerFrameMetadata> metadata;
  metadata.reserve(length / 2);
  while (length > 0) {
    metadata.emplace_back(IComposerClient::PerFrameMetadata{
                          static_cast<IComposerClient::PerFrameMetadataKey>(readSigned()),
                          readFloat()});
    length -= 2;
  }

  std::vector<int32_t> keys;
  std::vector<float> values;
  keys.reserve(metadata.size());
  values.reserve(metadata.size());
  for (const auto& m : metadata) {
    keys.push_back(static_cast<int32_t>(m.key));
    values.push_back(m.value);
  }

  auto err = mClient.hwc_session_->SetLayerPerFrameMetadata(mDisplay, mLayer, metadata.size(),
                                                            keys.data(), values.data());
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerFloatColor(uint16_t length) {
  if (length != CommandWriter::kSetLayerFloatColorLength) {
    return false;
  }

  // setLayerFloatColor is not supported
  auto err = Error::UNSUPPORTED;
  mWriter.setError(getCommandLoc(), static_cast<Error>(err));

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerColorTransform(uint16_t length) {
  if (length != CommandWriter::kSetLayerColorTransformLength) {
    return false;
  }

  float matrix[16];
  for (int i = 0; i < 16; i++) {
    matrix[i] = readFloat();
  }

  auto error = mClient.hwc_session_->SetLayerColorTransform(mDisplay, mLayer, matrix);
  if (static_cast<Error>(error) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(error));
  }

  return true;
}

bool QtiComposerClient::CommandReader::parseSetLayerPerFrameMetadataBlobs(uint16_t length) {
  // must have at least one metadata blob
  // of at least size 1 in queue (i.e {/*numBlobs=*/1, key, size, blob})
  if (length < 4) {
    return false;
  }

  uint32_t numBlobs = read();
  length--;
  std::vector<IComposerClient::PerFrameMetadataBlob> metadata;

  for (size_t i = 0; i < numBlobs; i++) {
    IComposerClient::PerFrameMetadataKey key =
      static_cast<IComposerClient::PerFrameMetadataKey>(readSigned());
    uint32_t blobSize = read();
    length -= 2;

    if (length * sizeof(uint32_t) < blobSize) {
      return false;
    }

    metadata.push_back({key, std::vector<uint8_t>()});
    IComposerClient::PerFrameMetadataBlob& metadataBlob = metadata.back();
    metadataBlob.blob.resize(blobSize);
    readBlob(blobSize, metadataBlob.blob.data());
  }

  std::vector<int32_t> keys;
  std::vector<uint32_t> sizes_of_metablob_;
  std::vector<uint8_t> blob_of_data_;
  keys.reserve(metadata.size());
  sizes_of_metablob_.reserve(metadata.size());
  for (const auto& m : metadata) {
    keys.push_back(static_cast<int32_t>(m.key));
    sizes_of_metablob_.push_back(m.blob.size());
    for (size_t i = 0; i < m.blob.size(); i++) {
      blob_of_data_.push_back(m.blob[i]);
    }
  }
  auto err = mClient.hwc_session_->SetLayerPerFrameMetadataBlobs(mDisplay, mLayer, metadata.size(),
                                                                 keys.data(),
                                                                 sizes_of_metablob_.data(),
                                                                 blob_of_data_.data());
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }
  return true;
}

bool QtiComposerClient::CommandReader::parseSetDisplayElapseTime(uint16_t length) {
  if (length < CommandWriter::kSetDisplayElapseTime) {
    return false;
  }
  uint64_t time = read64();

  auto err = mClient.hwc_session_->SetDisplayElapseTime(mDisplay, time);
  if (static_cast<Error>(err) != Error::NONE) {
    mWriter.setError(getCommandLoc(), static_cast<Error>(err));
  }

  return true;
}

hwc_rect_t QtiComposerClient::CommandReader::readRect() {
  return hwc_rect_t{
    readSigned(),
    readSigned(),
    readSigned(),
    readSigned(),
  };
}

std::vector<hwc_rect_t> QtiComposerClient::CommandReader::readRegion(size_t count) {
  std::vector<hwc_rect_t> region;
  region.reserve(count);
  while (count > 0) {
    region.emplace_back(readRect());
    count--;
  }

  return region;
}

hwc_frect_t QtiComposerClient::CommandReader::readFRect() {
  return hwc_frect_t{
    readFloat(),
    readFloat(),
    readFloat(),
    readFloat(),
  };
}

Error QtiComposerClient::CommandReader::lookupBufferCacheEntryLocked(BufferCache cache,
                                                                     uint32_t slot,
                                                                     BufferCacheEntry** outEntry) {
  auto dpy = mClient.mDisplayData.find(mDisplay);
  if (dpy == mClient.mDisplayData.end()) {
    return Error::BAD_DISPLAY;
  }

  BufferCacheEntry* entry = nullptr;
  switch (cache) {
  case BufferCache::CLIENT_TARGETS:
    if (slot < dpy->second.ClientTargets.size()) {
      entry = &dpy->second.ClientTargets[slot];
    }
    break;
  case BufferCache::OUTPUT_BUFFERS:
    if (slot < dpy->second.OutputBuffers.size()) {
      entry = &dpy->second.OutputBuffers[slot];
    }
    break;
  case BufferCache::LAYER_BUFFERS:
    {
      auto ly = dpy->second.Layers.find(mLayer);
      if (ly == dpy->second.Layers.end()) {
        return Error::BAD_LAYER;
      }
      if (slot < ly->second.Buffers.size()) {
        entry = &ly->second.Buffers[slot];
      }
    }
    break;
  case BufferCache::LAYER_SIDEBAND_STREAMS:
    {
      auto ly = dpy->second.Layers.find(mLayer);
      if (ly == dpy->second.Layers.end()) {
        return Error::BAD_LAYER;
      }
      if (slot == 0) {
        entry = &ly->second.SidebandStream;
      }
    }
    break;
  default:
    break;
  }

  if (!entry) {
    ALOGW("invalid buffer slot %" PRIu32, slot);
    return Error::BAD_PARAMETER;
  }

  *outEntry = entry;

  return Error::NONE;
}

Error QtiComposerClient::CommandReader::lookupBuffer(BufferCache cache, uint32_t slot,
                                                     bool useCache, buffer_handle_t handle,
                                                     buffer_handle_t* outHandle) {
  if (useCache) {
    std::lock_guard<std::mutex> lock(mClient.mDisplayDataMutex);

    BufferCacheEntry* entry;
    Error error = lookupBufferCacheEntryLocked(cache, slot, &entry);
    if (error != Error::NONE) {
      return error;
    }

    // input handle is ignored
    *outHandle = entry->getHandle();
  } else if (cache == BufferCache::LAYER_SIDEBAND_STREAMS) {
    if (handle) {
      *outHandle = native_handle_clone(handle);
      if (*outHandle == nullptr) {
        return Error::NO_RESOURCES;
      }
    }
  } else {
    if (!mHandleImporter.importBuffer(handle)) {
      return Error::NO_RESOURCES;
    }

    *outHandle = handle;
  }

  return Error::NONE;
}

Error QtiComposerClient::CommandReader::updateBuffer(BufferCache cache, uint32_t slot,
                                                     bool useCache, buffer_handle_t handle) {
  // handle was looked up from cache
  if (useCache) {
    return Error::NONE;
  }

  std::lock_guard<std::mutex> lock(mClient.mDisplayDataMutex);

  BufferCacheEntry* entry = nullptr;
  Error error = lookupBufferCacheEntryLocked(cache, slot, &entry);
  if (error != Error::NONE) {
    return error;
  }

  *entry = handle;
  return Error::NONE;
}
// Methods from ::android::hidl::base::V1_0::IBase follow.

IQtiComposerClient* HIDL_FETCH_IQtiComposerClient(const char* /* name */) {
  return QtiComposerClient::CreateQtiComposerClientInstance();
}

}  // namespace implementation
}  // namespace V3_0
}  // namespace composer
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor
