/*
 * Copyright (c) 2014-2021, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 *
 * Copyright 2015 The Android Open Source Project
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

/*
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <QService.h>
#include <binder/Parcel.h>
#include <core/buffer_allocator.h>
#include <cutils/properties.h>
#include <display_config.h>
#include <hardware_legacy/uevent.h>
#include <private/color_params.h>
#include <qd_utils.h>
#include <sync/sync.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <utils/String16.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <QService.h>
#include <utils/utils.h>
#include <algorithm>
#include <utility>
#include <bitset>
#include <iterator>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hwc_buffer_allocator.h"
#include "hwc_session.h"
#include "hwc_debugger.h"

#define __CLASS__ "HWCSession"

#define HWC_UEVENT_SWITCH_HDMI "change@/devices/virtual/switch/hdmi"
#define HWC_UEVENT_DRM_EXT_HOTPLUG "mdss_mdp/drm/card"

using HwcAttribute = composer_V2_4::IComposerClient::Attribute;

namespace sdm {

static HWCUEvent g_hwc_uevent_;
Locker HWCSession::locker_[HWCCallbacks::kNumDisplays];
bool HWCSession::pending_power_mode_[HWCCallbacks::kNumDisplays];
Locker HWCSession::power_state_[HWCCallbacks::kNumDisplays];
Locker HWCSession::hdr_locker_[HWCCallbacks::kNumDisplays];
Locker HWCSession::display_config_locker_;
Locker HWCSession::system_locker_;
static const int kSolidFillDelay = 100 * 1000;
int HWCSession::null_display_mode_ = 0;
static const uint32_t kBrightnessScaleMax = 100;
static const uint32_t kSvBlScaleMax = 65535;

// Map the known color modes to dataspace.
int32_t GetDataspaceFromColorMode(ColorMode mode) {
  switch (mode) {
    case ColorMode::SRGB:
    case ColorMode::NATIVE:
      return HAL_DATASPACE_V0_SRGB;
    case ColorMode::DCI_P3:
      return HAL_DATASPACE_DCI_P3;
    case ColorMode::DISPLAY_P3:
      return HAL_DATASPACE_DISPLAY_P3;
    case ColorMode::BT2100_PQ:
      return HAL_DATASPACE_BT2020_PQ;
    case ColorMode::BT2100_HLG:
      return HAL_DATASPACE_BT2020_HLG;
    case ColorMode::DISPLAY_BT2020:
      return HAL_DATASPACE_DISPLAY_BT2020;
    default:
      return HAL_DATASPACE_UNKNOWN;
  }
}

void HWCUEvent::UEventThread(HWCUEvent *hwc_uevent) {
  const char *uevent_thread_name = "HWC_UeventThread";

  prctl(PR_SET_NAME, uevent_thread_name, 0, 0, 0);
  setpriority(PRIO_PROCESS, 0, HAL_PRIORITY_URGENT_DISPLAY);

  int status = uevent_init();
  if (!status) {
    std::unique_lock<std::mutex> caller_lock(hwc_uevent->mutex_);
    hwc_uevent->caller_cv_.notify_one();
    DLOGE("Failed to init uevent with err %d", status);
    return;
  }

  {
    // Signal caller thread that worker thread is ready to listen to events.
    std::unique_lock<std::mutex> caller_lock(hwc_uevent->mutex_);
    hwc_uevent->init_done_ = true;
    hwc_uevent->caller_cv_.notify_one();
  }

  while (1) {
    char uevent_data[PAGE_SIZE] = {};

    // keep last 2 zeros to ensure double 0 termination
    int length = uevent_next_event(uevent_data, INT32(sizeof(uevent_data)) - 2);

    // scope of lock to this block only, so that caller is free to set event handler to nullptr;
    {
      std::lock_guard<std::mutex> guard(hwc_uevent->mutex_);
      if (hwc_uevent->uevent_listener_) {
        hwc_uevent->uevent_listener_->UEventHandler(uevent_data, length);
      } else {
        DLOGW("UEvent dropped. No uevent listener.");
      }
    }
  }
}

HWCUEvent::HWCUEvent() {
  std::unique_lock<std::mutex> caller_lock(mutex_);
  std::thread thread(HWCUEvent::UEventThread, this);
  thread.detach();
  caller_cv_.wait(caller_lock);
}

void HWCUEvent::Register(HWCUEventListener *uevent_listener) {
  DLOGI("Set uevent listener = %p", uevent_listener);

  std::lock_guard<std::mutex> obj(mutex_);
  uevent_listener_ = uevent_listener;
}

HWCSession::HWCSession() : cwb_(this) {}

HWCSession *HWCSession::GetInstance() {
  // executed only once for the very first call.
  // GetInstance called multiple times from Composer and ComposerClient
  static HWCSession *hwc_session = new HWCSession();
  return hwc_session;
}

int HWCSession::Init() {
  SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  int status = -EINVAL;
  const char *qservice_name = "display.qservice";

  if (!g_hwc_uevent_.InitDone()) {
    return status;
  }


  // Start QService and connect to it.
  qService::QService::init();
  android::sp<qService::IQService> iqservice = android::interface_cast<qService::IQService>(
      android::defaultServiceManager()->getService(android::String16(qservice_name)));

  if (iqservice.get()) {
    iqservice->connect(android::sp<qClient::IQClient>(this));
    qservice_ = reinterpret_cast<qService::QService *>(iqservice.get());
  } else {
    DLOGE("Failed to acquire %s", qservice_name);
    return -EINVAL;
  }

  HWCDebugHandler::Get()->GetProperty(ENABLE_NULL_DISPLAY_PROP, &null_display_mode_);
  HWCDebugHandler::Get()->GetProperty(DISABLE_HOTPLUG_BWCHECK, &disable_hotplug_bwcheck_);
  HWCDebugHandler::Get()->GetProperty(DISABLE_MASK_LAYER_HINT, &disable_mask_layer_hint_);

  if (!null_display_mode_) {
    g_hwc_uevent_.Register(this);
  }

  int value = 0;  // Default value when property is not present.
  Debug::Get()->GetProperty(ENABLE_ASYNC_POWERMODE, &value);
  async_powermode_ = (value == 1);
  DLOGI("builtin_powermode_override: %d", async_powermode_);

  value = 0;
  Debug::Get()->GetProperty(ENABLE_ASYNC_VDS_CREATION, &value);
  async_vds_creation_ = (value == 1);
  DLOGI("async_vds_creation: %d", async_vds_creation_);

  InitSupportedDisplaySlots();
  // Create primary display here. Remaining builtin displays will be created after client has set
  // display indexes which may happen sometime before callback is registered.
  status = CreatePrimaryDisplay();
  if (status) {
    Deinit();
    return status;
  }

  is_composer_up_ = true;
  StartServices();

  PostInit();

  return 0;
}

void HWCSession::PostInit() {
  if (null_display_mode_) {
    return;
  }

  // Start services which need IDisplayConfig to be up.
  // This avoids deadlock between composer and its clients.
  auto hwc_display = hwc_display_[HWC_DISPLAY_PRIMARY];
  hwc_display->PostInit();
}

int HWCSession::Deinit() {
  // Destroy all connected displays
  DestroyDisplay(&map_info_primary_);

  for (auto &map_info : map_info_builtin_) {
    DestroyDisplay(&map_info);
  }

  for (auto &map_info : map_info_pluggable_) {
    DestroyDisplay(&map_info);
  }

  for (auto &map_info : map_info_virtual_) {
    DestroyDisplay(&map_info);
  }

  if (color_mgr_) {
    color_mgr_->DestroyColorManager();
  }

  if (!null_display_mode_) {
    g_hwc_uevent_.Register(nullptr);

    DisplayError error = CoreInterface::DestroyCore();
    if (error != kErrorNone) {
      DLOGE("Display core de-initialization failed. Error = %d", error);
    }
  }

  return 0;
}

void HWCSession::InitSupportedDisplaySlots() {
  // Default slots:
  //    Primary = 0, External = 1
  //    Additional external displays 2,3,...max_pluggable_count.
  //    Additional builtin displays max_pluggable_count + 1, max_pluggable_count + 2,...
  //    Last slots for virtual displays.
  // Virtual display id is only for SF <--> HWC communication.
  // It need not align with hwccomposer_defs

  map_info_primary_.client_id = qdutils::DISPLAY_PRIMARY;

  if (null_display_mode_) {
    InitSupportedNullDisplaySlots();
    return;
  }

  DisplayError error = CoreInterface::CreateCore(&buffer_allocator_, nullptr,
                                                 &socket_handler_, &core_intf_);
  if (error != kErrorNone) {
    DLOGE("Failed to create CoreInterface");
    return;
  }

  HWDisplayInterfaceInfo hw_disp_info = {};
  error = core_intf_->GetFirstDisplayInterfaceType(&hw_disp_info);
  if (error != kErrorNone) {
    CoreInterface::DestroyCore();
    DLOGE("Primary display type not recognized. Error = %d", error);
    return;
  }

  int max_builtin = 0;
  int max_pluggable = 0;
  int max_virtual = 0;

  error = core_intf_->GetMaxDisplaysSupported(kBuiltIn, &max_builtin);
  if (error != kErrorNone) {
    CoreInterface::DestroyCore();
    DLOGE("Could not find maximum built-in displays supported. Error = %d", error);
    return;
  }

  error = core_intf_->GetMaxDisplaysSupported(kPluggable, &max_pluggable);
  if (error != kErrorNone) {
    CoreInterface::DestroyCore();
    DLOGE("Could not find maximum pluggable displays supported. Error = %d", error);
    return;
  }

  error = core_intf_->GetMaxDisplaysSupported(kVirtual, &max_virtual);
  if (error != kErrorNone) {
    CoreInterface::DestroyCore();
    DLOGE("Could not find maximum virtual displays supported. Error = %d", error);
    return;
  }

  if (max_virtual == 0) {
    // Check if WB using GPU is supported.
    max_virtual += virtual_display_factory_.IsGPUColorConvertSupported() ? 1 : 0;
  }

  if (kPluggable == hw_disp_info.type) {
    // If primary is a pluggable display, we have already used one pluggable display interface.
    max_pluggable--;
  } else {
    max_builtin--;
  }

  // Init slots in accordance to h/w capability.
  uint32_t disp_count = UINT32(std::min(max_pluggable, HWCCallbacks::kNumPluggable));
  hwc2_display_t base_id = qdutils::DISPLAY_EXTERNAL;
  map_info_pluggable_.resize(disp_count);
  for (auto &map_info : map_info_pluggable_) {
    map_info.client_id = base_id++;
  }

  disp_count = UINT32(std::min(max_builtin, HWCCallbacks::kNumBuiltIn));
  map_info_builtin_.resize(disp_count);
  for (auto &map_info : map_info_builtin_) {
    map_info.client_id = base_id++;
  }

  disp_count = UINT32(std::min(max_virtual, HWCCallbacks::kNumVirtual));
  map_info_virtual_.resize(disp_count);
  for (auto &map_info : map_info_virtual_) {
    map_info.client_id = base_id++;
  }

  // resize HDR supported map to total number of displays.
  is_hdr_display_.resize(UINT32(base_id));

  if (!async_powermode_) {
    return;
  }

  int start_index = HWCCallbacks::kNumRealDisplays;
  std::vector<DisplayMapInfo> map_info = {map_info_primary_};
  std::copy(map_info_builtin_.begin(), map_info_builtin_.end(), std::back_inserter(map_info));
  std::copy(map_info_pluggable_.begin(), map_info_pluggable_.end(), std::back_inserter(map_info));
  for (auto &map : map_info) {
    DLOGI("Display Pairs: map.client_id: %d, start_index: %d", INT32(map.client_id),
          INT32(start_index));
    map_hwc_display_.insert(std::make_pair(map.client_id, start_index++));
  }
}

void HWCSession::InitSupportedNullDisplaySlots() {
  if (!null_display_mode_) {
    DLOGE("Should only be invoked during null display");
    return;
  }

  map_info_primary_.client_id = 0;
  // Resize HDR supported map to total number of displays
  is_hdr_display_.resize(1);

  if (!async_powermode_) {
    return;
  }

  DLOGI("Display Pairs: map.client_id: %d, start_index: %d", INT32(map_info_primary_.client_id),
                                                             HWCCallbacks::kNumRealDisplays);
  map_hwc_display_.insert(std::make_pair(map_info_primary_.client_id,
                                         HWCCallbacks::kNumRealDisplays));
}

int HWCSession::GetDisplayIndex(int dpy) {
  DisplayMapInfo *map_info = nullptr;
  switch (dpy) {
    case qdutils::DISPLAY_PRIMARY:
      map_info = &map_info_primary_;
      break;
    case qdutils::DISPLAY_EXTERNAL:
      map_info = map_info_pluggable_.size() ? &map_info_pluggable_[0] : nullptr;
      break;
    case qdutils::DISPLAY_EXTERNAL_2:
      map_info = (map_info_pluggable_.size() > 1) ? &map_info_pluggable_[1] : nullptr;
      break;
    case qdutils::DISPLAY_VIRTUAL:
      map_info = map_info_virtual_.size() ? &map_info_virtual_[0] : nullptr;
      break;
    case qdutils::DISPLAY_BUILTIN_2:
      map_info = map_info_builtin_.size() ? &map_info_builtin_[0] : nullptr;
      break;
    default:
      DLOGW("Unknown display %d.", dpy);
      break;
  }

  if (!map_info) {
    DLOGE("Display index not found for display %d.", dpy);
    return -1;
  }

  return INT(map_info->client_id);
}

void HWCSession::GetCapabilities(uint32_t *outCount, int32_t *outCapabilities) {
  if (!outCount) {
    return;
  }

  int value = 0;
  bool disable_skip_validate = false;
  if (Debug::Get()->GetProperty(DISABLE_SKIP_VALIDATE_PROP, &value) == kErrorNone) {
    disable_skip_validate = (value == 1);
  }
  uint32_t count = 1 + (disable_skip_validate ? 0 : 1);

  if (outCapabilities != nullptr && (*outCount >= count)) {
    outCapabilities[0] = HWC2_CAPABILITY_SKIP_CLIENT_COLOR_TRANSFORM;
    if (!disable_skip_validate) {
      outCapabilities[1] = HWC2_CAPABILITY_SKIP_VALIDATE;
    }
  }
  *outCount = count;
}

template <typename PFN, typename T>
static hwc2_function_pointer_t AsFP(T function) {
  static_assert(std::is_same<PFN, T>::value, "Incompatible function pointer");
  return reinterpret_cast<hwc2_function_pointer_t>(function);
}

// HWC2 functions returned in GetFunction
// Defined in the same order as in the HWC2 header

int32_t HWCSession::AcceptDisplayChanges(hwc2_display_t display) {
  return CallDisplayFunction(display, &HWCDisplay::AcceptDisplayChanges);
}

int32_t HWCSession::CreateLayer(hwc2_display_t display,
                                hwc2_layer_t *out_layer_id) {
  if (!out_layer_id) {
    return  HWC2_ERROR_BAD_PARAMETER;
  }

  return CallDisplayFunction(display, &HWCDisplay::CreateLayer, out_layer_id);
}

int32_t HWCSession::CreateVirtualDisplay(uint32_t width, uint32_t height, int32_t *format,
                                         hwc2_display_t *out_display_id) {
  // TODO(user): Handle concurrency with HDMI

  if (!out_display_id || !width || !height || !format) {
    return  HWC2_ERROR_BAD_PARAMETER;
  }

  if (null_display_mode_) {
    return 0;
  }

  if (async_vds_creation_ && virtual_id_ != HWCCallbacks::kNumDisplays) {
    *out_display_id = virtual_id_;
    return HWC2_ERROR_NONE;
  }

  auto status = CreateVirtualDisplayObj(width, height, format, out_display_id);
  if (status == HWC2::Error::None) {
    DLOGI("Created virtual display id:%" PRIu64 ", res: %dx%d", *out_display_id, width, height);
  } else {
    DLOGE("Failed to create virtual display: %s", to_string(status).c_str());
  }
  return INT32(status);
}

int32_t HWCSession::DestroyLayer(hwc2_display_t display, hwc2_layer_t layer) {
  return CallDisplayFunction(display, &HWCDisplay::DestroyLayer, layer);
}

int32_t HWCSession::DestroyVirtualDisplay(hwc2_display_t display) {
  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();

  if (active_builtin_disp_id < HWCCallbacks::kNumDisplays) {
    Locker::ScopeLock lock_a(locker_[active_builtin_disp_id]);
    std::bitset<kSecureMax> secure_sessions = 0;
    hwc_display_[active_builtin_disp_id]->GetActiveSecureSession(&secure_sessions);
    if (secure_sessions.any()) {
      DLOGW("Secure session is active, defer destruction of virtual display id:%" PRIu64, display);
      destroy_virtual_disp_pending_ = true;
      return HWC2_ERROR_NONE;
    }
  }

  for (auto &map_info : map_info_virtual_) {
    if (map_info.client_id == display) {
      DLOGI("Destroying virtual display id:%" PRIu64, display);
      DestroyDisplay(&map_info);
      break;
    }
  }
  virtual_id_ = HWCCallbacks::kNumDisplays;

  return HWC2_ERROR_NONE;
}

int32_t HWCSession::GetVirtualDisplayId() {
  HWDisplaysInfo hw_displays_info = {};
  core_intf_->GetDisplaysStatus(&hw_displays_info);
  for (auto &iter : hw_displays_info) {
    auto &info = iter.second;
    if (info.display_type != kVirtual) {
      continue;
    }

    return info.display_id;
  }

  return -1;
}

void HWCSession::Dump(uint32_t *out_size, char *out_buffer) {
  if (!out_size) {
    return;
  }

  const size_t max_dump_size = 16384;  // 16 kB

  if (out_buffer == nullptr) {
    *out_size = max_dump_size;
  } else {
    std::ostringstream os;
    for (int id = 0; id < HWCCallbacks::kNumRealDisplays; id++) {
      SCOPE_LOCK(locker_[id]);
      if (hwc_display_[id]) {
        hwc_display_[id]->Dump(&os);
      }
    }
    Fence::Dump(&os);

    std::string s = os.str();
    auto copied = s.copy(out_buffer, std::min(s.size(), max_dump_size), 0);
    *out_size = UINT32(copied);
  }
}

uint32_t HWCSession::GetMaxVirtualDisplayCount() {
  return map_info_virtual_.size();
}

int32_t HWCSession::GetActiveConfig(hwc2_display_t display, hwc2_config_t *out_config) {
  return CallDisplayFunction(display, &HWCDisplay::GetActiveConfig, out_config);
}

int32_t HWCSession::GetChangedCompositionTypes(hwc2_display_t display, uint32_t *out_num_elements,
                                               hwc2_layer_t *out_layers, int32_t *out_types) {
  // null_ptr check only for out_num_elements, as out_layers and out_types can be null.
  if (!out_num_elements) {
    return  HWC2_ERROR_BAD_PARAMETER;
  }
  return CallDisplayFunction(display, &HWCDisplay::GetChangedCompositionTypes, out_num_elements,
                             out_layers, out_types);
}

int32_t HWCSession::GetClientTargetSupport(hwc2_display_t display, uint32_t width, uint32_t height,
                                           int32_t format, int32_t dataspace) {
  return CallDisplayFunction(display, &HWCDisplay::GetClientTargetSupport, width, height, format,
                             dataspace);
}

int32_t HWCSession::GetColorModes(hwc2_display_t display, uint32_t *out_num_modes,
                                  int32_t /*ColorMode*/ *int_out_modes) {
  auto out_modes = reinterpret_cast<ColorMode *>(int_out_modes);
  if (out_num_modes == nullptr) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  return CallDisplayFunction(display, &HWCDisplay::GetColorModes, out_num_modes, out_modes);
}

int32_t HWCSession::GetRenderIntents(hwc2_display_t display, int32_t /*ColorMode*/ int_mode,
                                     uint32_t *out_num_intents,
                                     int32_t /*RenderIntent*/ *int_out_intents) {
  auto mode = static_cast<ColorMode>(int_mode);
  auto out_intents = reinterpret_cast<RenderIntent *>(int_out_intents);
  if (out_num_intents == nullptr) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (mode < ColorMode::NATIVE || mode > ColorMode::DISPLAY_BT2020) {
    DLOGE("Invalid ColorMode: %d", mode);
    return HWC2_ERROR_BAD_PARAMETER;
  }
  return CallDisplayFunction(display, &HWCDisplay::GetRenderIntents, mode, out_num_intents,
                             out_intents);
}

int32_t HWCSession::GetDataspaceSaturationMatrix(int32_t /*Dataspace*/ int_dataspace,
                                                 float *out_matrix) {
  auto dataspace = static_cast<Dataspace>(int_dataspace);
  if (out_matrix == nullptr || dataspace != Dataspace::SRGB_LINEAR) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  // We only have the matrix for sRGB
  float saturation_matrix[kDataspaceSaturationMatrixCount] = { 1.0, 0.0, 0.0, 0.0, \
                                                               0.0, 1.0, 0.0, 0.0, \
                                                               0.0, 0.0, 1.0, 0.0, \
                                                               0.0, 0.0, 0.0, 1.0 };

  for (int32_t i = 0; i < kDataspaceSaturationMatrixCount; i += 4) {
    DLOGD("%f %f %f %f", saturation_matrix[i], saturation_matrix[i + 1], saturation_matrix[i + 2],
          saturation_matrix[i + 3]);
  }
  for (uint32_t i = 0; i < kDataspaceSaturationMatrixCount; i++) {
    out_matrix[i] = saturation_matrix[i];
  }
  return HWC2_ERROR_NONE;
}

int32_t HWCSession::GetPerFrameMetadataKeys(hwc2_display_t display, uint32_t *out_num_keys,
                                            int32_t *int_out_keys) {
  auto out_keys = reinterpret_cast<PerFrameMetadataKey *>(int_out_keys);
  return CallDisplayFunction(display, &HWCDisplay::GetPerFrameMetadataKeys, out_num_keys,
                             out_keys);
}

int32_t HWCSession::SetLayerPerFrameMetadata(hwc2_display_t display, hwc2_layer_t layer,
                                             uint32_t num_elements, const int32_t *int_keys,
                                             const float *metadata) {
  auto keys = reinterpret_cast<const PerFrameMetadataKey *>(int_keys);
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerPerFrameMetadata, num_elements,
                           keys, metadata);
}

int32_t HWCSession:: SetLayerPerFrameMetadataBlobs(hwc2_display_t display,
                                                   hwc2_layer_t layer, uint32_t num_elements,
                                                   const int32_t *int_keys, const uint32_t *sizes,
                                                   const uint8_t *metadata) {
  auto keys = reinterpret_cast<const PerFrameMetadataKey *>(int_keys);
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerPerFrameMetadataBlobs,
                           num_elements, keys, sizes, metadata);
}

int32_t HWCSession::SetDisplayedContentSamplingEnabled(hwc2_display_t display, int32_t enabled,
                                                       uint8_t component_mask,
                                                       uint64_t max_frames) {
  static constexpr int32_t validComponentMask = HWC2_FORMAT_COMPONENT_0 | HWC2_FORMAT_COMPONENT_1 |
                                                HWC2_FORMAT_COMPONENT_2 | HWC2_FORMAT_COMPONENT_3;
  if (component_mask & ~validComponentMask)
    return HWC2_ERROR_BAD_PARAMETER;
  return CallDisplayFunction(display, &HWCDisplay::SetDisplayedContentSamplingEnabled, enabled,
                             component_mask, max_frames);
}

int32_t HWCSession::GetDisplayedContentSamplingAttributes(hwc2_display_t display, int32_t *format,
                                                          int32_t *dataspace,
                                                          uint8_t *supported_components) {
  return CallDisplayFunction(display, &HWCDisplay::GetDisplayedContentSamplingAttributes, format,
                             dataspace, supported_components);
}

int32_t HWCSession::GetDisplayedContentSample(hwc2_display_t display, uint64_t max_frames,
                                              uint64_t timestamp, uint64_t *numFrames,
                                              int32_t samples_size[NUM_HISTOGRAM_COLOR_COMPONENTS],
                                              uint64_t *samples[NUM_HISTOGRAM_COLOR_COMPONENTS]) {
  return CallDisplayFunction(display, &HWCDisplay::GetDisplayedContentSample, max_frames, timestamp,
                             numFrames, samples_size, samples);
}

int32_t HWCSession::GetDisplayAttribute(hwc2_display_t display, hwc2_config_t config,
                                        HwcAttribute attribute, int32_t *out_value) {
  if (out_value == nullptr) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  return CallDisplayFunction(display, &HWCDisplay::GetDisplayAttribute, config, attribute,
                             out_value);
}

int32_t HWCSession::GetDisplayConfigs(hwc2_display_t display, uint32_t *out_num_configs,
                                      hwc2_config_t *out_configs) {
  return CallDisplayFunction(display, &HWCDisplay::GetDisplayConfigs, out_num_configs,
                             out_configs);
}

int32_t HWCSession::GetDisplayName(hwc2_display_t display, uint32_t *out_size, char *out_name) {
  return CallDisplayFunction(display, &HWCDisplay::GetDisplayName, out_size, out_name);
}

int32_t HWCSession::GetDisplayRequests(hwc2_display_t display, int32_t *out_display_requests,
                                       uint32_t *out_num_elements, hwc2_layer_t *out_layers,
                                       int32_t *out_layer_requests) {
  return CallDisplayFunction(display, &HWCDisplay::GetDisplayRequests, out_display_requests,
                             out_num_elements, out_layers, out_layer_requests);
}

int32_t HWCSession::GetDisplayType(hwc2_display_t display, int32_t *out_type) {
  return CallDisplayFunction(display, &HWCDisplay::GetDisplayType, out_type);
}


int32_t HWCSession::GetHdrCapabilities(hwc2_display_t display, uint32_t* out_num_types,
                                       int32_t* out_types, float* out_max_luminance,
                                       float* out_max_average_luminance,
                                       float* out_min_luminance) {
  return CallDisplayFunction(display, &HWCDisplay::GetHdrCapabilities, out_num_types, out_types,
                             out_max_luminance, out_max_average_luminance, out_min_luminance);
}


int32_t HWCSession::GetReleaseFences(hwc2_display_t display, uint32_t *out_num_elements,
                                     hwc2_layer_t *out_layers,
                                     std::vector<shared_ptr<Fence>> *out_fences) {
  return CallDisplayFunction(display, &HWCDisplay::GetReleaseFences, out_num_elements, out_layers,
                             out_fences);
}

void HWCSession::PerformQsyncCallback(hwc2_display_t display) {
  std::shared_ptr<DisplayConfig::ConfigCallback> callback = qsync_callback_.lock();
  if (!callback) {
    return;
  }

  bool qsync_enabled = 0;
  int32_t refresh_rate = 0, qsync_refresh_rate = 0;
  if (hwc_display_[display]->IsQsyncCallbackNeeded(&qsync_enabled,
      &refresh_rate, &qsync_refresh_rate)) {
    callback->NotifyQsyncChange(qsync_enabled, refresh_rate, qsync_refresh_rate);
  }
}

void HWCSession::PerformIdleStatusCallback(hwc2_display_t display) {
  std::shared_ptr<DisplayConfig::ConfigCallback> callback = idle_callback_.lock();
  if (!callback) {
    return;
  }

  if (hwc_display_[display]->IsDisplayIdle()) {
    DTRACE_SCOPED();
    callback->NotifyIdleStatus(true);
  }
}

int32_t HWCSession::PresentDisplay(hwc2_display_t display, shared_ptr<Fence> *out_retire_fence) {
  auto status = HWC2::Error::BadDisplay;
  DTRACE_SCOPED();

  SCOPE_LOCK(system_locker_);
  if (display >= HWCCallbacks::kNumDisplays) {
    DLOGW("Invalid Display : display = %" PRIu64, display);
    return HWC2_ERROR_BAD_DISPLAY;
  }

  HandleSecureSession();


  hwc2_display_t target_display = display;

  {
    SCOPE_LOCK(power_state_[display]);
    if (power_state_transition_[display]) {
      // Route all interactions with client to dummy display.
      target_display = map_hwc_display_.find(display)->second;
    }
  }

  {
    SEQUENCE_EXIT_SCOPE_LOCK(locker_[target_display]);
    if (!hwc_display_[target_display]) {
      DLOGW("Removed Display : display = %" PRIu64, target_display);
      return HWC2_ERROR_BAD_DISPLAY;
    }

    if (out_retire_fence == nullptr) {
      return HWC2_ERROR_BAD_PARAMETER;
    }

    if (pending_power_mode_[display]) {
      status = HWC2::Error::None;
    } else {
      hwc_display_[target_display]->ProcessActiveConfigChange();
      status = PresentDisplayInternal(target_display);
      if (status == HWC2::Error::None) {
        // Check if hwc's refresh trigger is getting exercised.
        if (callbacks_.NeedsRefresh(display)) {
          hwc_display_[target_display]->SetPendingRefresh();
          callbacks_.ResetRefresh(display);
        }
        status = hwc_display_[target_display]->Present(out_retire_fence);
        if (status == HWC2::Error::None) {
          PerformQsyncCallback(target_display);
          PerformIdleStatusCallback(target_display);
        }
      }
    }
  }

  if (status != HWC2::Error::None && status != HWC2::Error::NotValidated) {
    SEQUENCE_CANCEL_SCOPE_LOCK(locker_[target_display]);
  }

  HandlePendingPowerMode(display, *out_retire_fence);
  HandlePendingHotplug(display, *out_retire_fence);
  HandlePendingRefresh();
  if (status != HWC2::Error::NotValidated) {
    cwb_.PresentDisplayDone(display);
  }
  display_ready_.set(UINT32(display));
  {
    std::unique_lock<std::mutex> caller_lock(hotplug_mutex_);
    hotplug_cv_.notify_one();
  }

  return INT32(status);
}

void HWCSession::HandlePendingRefresh() {
  if (pending_refresh_.none()) {
    return;
  }

  for (size_t i = 0; i < pending_refresh_.size(); i++) {
    if (pending_refresh_.test(i)) {
      callbacks_.Refresh(i);
      break;
    }
  }

  pending_refresh_.reset();
}

void HWCSession::RegisterCallback(int32_t descriptor, hwc2_callback_data_t callback_data,
                                  hwc2_function_pointer_t pointer) {
  auto desc = static_cast<HWC2::Callback>(descriptor);

  // Detect if client died and now is back
  bool already_connected = false;
  vector<hwc2_display_t> pending_hotplugs;
  if (descriptor == HWC2_CALLBACK_HOTPLUG && pointer) {
    already_connected = callbacks_.IsClientConnected();
    if (already_connected) {
      for (auto& map_info : map_info_builtin_) {
        SCOPE_LOCK(locker_[map_info.client_id]);
        if (hwc_display_[map_info.client_id]) {
          pending_hotplugs.push_back(static_cast<hwc2_display_t>(map_info.client_id));
        }
      }
      for (auto& map_info : map_info_pluggable_) {
        SCOPE_LOCK(locker_[map_info.client_id]);
        if (hwc_display_[map_info.client_id]) {
          pending_hotplugs.push_back(static_cast<hwc2_display_t>(map_info.client_id));
        }
      }
    }
  }

  auto error = callbacks_.Register(desc, callback_data, pointer);
  if (error != HWC2::Error::None) {
    return;
  }

  DLOGI("%s callback: %s", pointer ? "Registering" : "Deregistering", to_string(desc).c_str());
  if (descriptor == HWC2_CALLBACK_HOTPLUG && pointer) {
    if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
      DLOGI("Hotplugging primary...");
      callbacks_.Hotplug(HWC_DISPLAY_PRIMARY, HWC2::Connection::Connected);
    }
    // Create displays since they should now have their final display indices set.
    DLOGI("Handling built-in displays...");
    if (HandleBuiltInDisplays()) {
      DLOGW("Failed handling built-in displays.");
    }
    DLOGI("Handling pluggable displays...");
    int32_t err = HandlePluggableDisplays(false);
    if (err) {
      DLOGW("All displays could not be created. Error %d '%s'. Hotplug handling %s.", err,
            strerror(abs(err)), pending_hotplug_event_ == kHotPlugEvent ? "deferred" :
            "dropped");
    }

    // If previously registered, call hotplug for all connected displays to refresh
    if (already_connected) {
      std::vector<hwc2_display_t> updated_pending_hotplugs;
      for (auto client_id : pending_hotplugs) {
        SCOPE_LOCK(locker_[client_id]);
        // check if the display is unregistered
        if (hwc_display_[client_id]) {
          updated_pending_hotplugs.push_back(client_id);
        }
      }
      for (auto client_id : updated_pending_hotplugs) {
        DLOGI("Re-hotplug display connected: client id = %d", UINT32(client_id));
        callbacks_.Hotplug(client_id, HWC2::Connection::Connected);
      }
    }
  }

  if (descriptor == HWC2_CALLBACK_HOTPLUG) {
    client_connected_ = !!pointer;
    // Notfify all displays.
    NotifyClientStatus(client_connected_);
  }

  // On SF stop, disable the idle time.
  if (!pointer && is_idle_time_up_ && hwc_display_[HWC_DISPLAY_PRIMARY]) { // De-registering…
    DLOGI("disable idle time");
    hwc_display_[HWC_DISPLAY_PRIMARY]->SetIdleTimeoutMs(0,0);
    is_idle_time_up_ = false;
  }

  need_invalidate_ = false;
}

int32_t HWCSession::SetActiveConfig(hwc2_display_t display, hwc2_config_t config) {
  return CallDisplayFunction(display, &HWCDisplay::SetActiveConfig, config);
}

int32_t HWCSession::SetClientTarget(hwc2_display_t display, buffer_handle_t target,
                                    const shared_ptr<Fence> acquire_fence, int32_t dataspace,
                                    hwc_region_t damage) {
  return CallDisplayFunction(display, &HWCDisplay::SetClientTarget, target, acquire_fence,
                             dataspace, damage);
}

int32_t HWCSession::SetColorMode(hwc2_display_t display, int32_t /*ColorMode*/ int_mode) {
  auto mode = static_cast<ColorMode>(int_mode);
  if (mode < ColorMode::NATIVE || mode > ColorMode::DISPLAY_BT2020) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  return CallDisplayFunction(display, &HWCDisplay::SetColorMode, mode);
}

int32_t HWCSession::SetColorModeWithRenderIntent(hwc2_display_t display,
                                                 int32_t /*ColorMode*/ int_mode,
                                                 int32_t /*RenderIntent*/ int_render_intent) {
  auto mode = static_cast<ColorMode>(int_mode);
  if (mode < ColorMode::NATIVE || mode > ColorMode::DISPLAY_BT2020) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if ((int_render_intent < 0) || (int_render_intent > MAX_EXTENDED_RENDER_INTENT)) {
    DLOGE("Invalid RenderIntent: %d", int_render_intent);
    return HWC2_ERROR_BAD_PARAMETER;
  }

  auto render_intent = static_cast<RenderIntent>(int_render_intent);
  return CallDisplayFunction(display, &HWCDisplay::SetColorModeWithRenderIntent, mode,
                             render_intent);
}

int32_t HWCSession::SetColorTransform(hwc2_display_t display, const float *matrix,
                                      int32_t /*android_color_transform_t*/ hint) {
  if (!matrix || hint < HAL_COLOR_TRANSFORM_IDENTITY ||
       hint > HAL_COLOR_TRANSFORM_CORRECT_TRITANOPIA) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  android_color_transform_t transform_hint = static_cast<android_color_transform_t>(hint);
  return CallDisplayFunction(display, &HWCDisplay::SetColorTransform, matrix, transform_hint);
}

int32_t HWCSession::SetCursorPosition(hwc2_display_t display, hwc2_layer_t layer, int32_t x,
                                      int32_t y) {
  auto status = INT32(HWC2::Error::None);
  status = CallDisplayFunction(display, &HWCDisplay::SetCursorPosition, layer, x, y);
  if (status == INT32(HWC2::Error::None)) {
    // Update cursor position
    CallLayerFunction(display, layer, &HWCLayer::SetCursorPosition, x, y);
  }
  return status;
}

int32_t HWCSession::SetLayerBlendMode(hwc2_display_t display, hwc2_layer_t layer,
                                      int32_t int_mode) {
  if (int_mode < HWC2_BLEND_MODE_INVALID || int_mode > HWC2_BLEND_MODE_COVERAGE) {
    return HWC2_ERROR_BAD_PARAMETER;
  }
  auto mode = static_cast<HWC2::BlendMode>(int_mode);
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerBlendMode, mode);
}

int32_t HWCSession::SetLayerBuffer(hwc2_display_t display, hwc2_layer_t layer,
                                   buffer_handle_t buffer,
                                   const shared_ptr<Fence> &acquire_fence) {
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerBuffer, buffer, acquire_fence);
}

int32_t HWCSession::SetLayerColor(hwc2_display_t display, hwc2_layer_t layer, hwc_color_t color) {
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerColor, color);
}

int32_t HWCSession::SetLayerCompositionType(hwc2_display_t display, hwc2_layer_t layer,
                                            int32_t int_type) {
  auto type = static_cast<HWC2::Composition>(int_type);
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerCompositionType, type);
}

int32_t HWCSession::SetLayerDataspace(hwc2_display_t display, hwc2_layer_t layer,
                                      int32_t dataspace) {
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerDataspace, dataspace);
}

int32_t HWCSession::SetLayerDisplayFrame(hwc2_display_t display, hwc2_layer_t layer,
                                         hwc_rect_t frame) {
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerDisplayFrame, frame);
}

int32_t HWCSession::SetLayerPlaneAlpha(hwc2_display_t display, hwc2_layer_t layer, float alpha) {
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerPlaneAlpha, alpha);
}

int32_t HWCSession::SetLayerSourceCrop(hwc2_display_t display, hwc2_layer_t layer,
                                       hwc_frect_t crop) {
  return HWCSession::CallLayerFunction(display, layer, &HWCLayer::SetLayerSourceCrop, crop);
}

int32_t HWCSession::SetLayerSurfaceDamage(hwc2_display_t display, hwc2_layer_t layer,
                                          hwc_region_t damage) {
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerSurfaceDamage, damage);
}

int32_t HWCSession::SetLayerTransform(hwc2_display_t display, hwc2_layer_t layer,
                                      int32_t int_transform) {
  auto transform = static_cast<HWC2::Transform>(int_transform);
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerTransform, transform);
}

int32_t HWCSession::SetLayerVisibleRegion(hwc2_display_t display, hwc2_layer_t layer,
                                          hwc_region_t visible) {
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerVisibleRegion, visible);
}

int32_t HWCSession::SetLayerZOrder(hwc2_display_t display, hwc2_layer_t layer, uint32_t z) {
  return CallDisplayFunction(display, &HWCDisplay::SetLayerZOrder, layer, z);
}

int32_t HWCSession::SetLayerType(hwc2_display_t display, hwc2_layer_t layer,
                                 IQtiComposerClient::LayerType type) {
  return CallDisplayFunction(display, &HWCDisplay::SetLayerType, layer, type);
}

int32_t HWCSession::SetLayerColorTransform(hwc2_display_t display, hwc2_layer_t layer,
                                           const float *matrix) {
  return CallLayerFunction(display, layer, &HWCLayer::SetLayerColorTransform, matrix);
}

int32_t HWCSession::SetDisplayElapseTime(hwc2_display_t display, uint64_t time) {
  return CallDisplayFunction(display, &HWCDisplay::SetDisplayElapseTime, time);
}

int32_t HWCSession::SetOutputBuffer(hwc2_display_t display, buffer_handle_t buffer,
                                    const shared_ptr<Fence> &release_fence) {
  if (INT32(display) != GetDisplayIndex(qdutils::DISPLAY_VIRTUAL)) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  SCOPE_LOCK(locker_[display]);
  if (hwc_display_[display]) {
    auto vds = reinterpret_cast<HWCDisplayVirtual *>(hwc_display_[display]);
    auto status = vds->SetOutputBuffer(buffer, release_fence);
    return INT32(status);
  } else {
    return HWC2_ERROR_BAD_DISPLAY;
  }
}

int32_t HWCSession::SetPowerMode(hwc2_display_t display, int32_t int_mode) {
  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  //  validate device and also avoid undefined behavior in cast to HWC2::PowerMode
  if (int_mode < HWC2_POWER_MODE_OFF || int_mode > HWC2_POWER_MODE_DOZE_SUSPEND) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  auto mode = static_cast<HWC2::PowerMode>(int_mode);

  // When secure session going on primary, if power request comes on second built-in, cache it and
  // process once secure session ends.
  // Allow power off transition during secure session.
  bool is_builtin = (hwc_display_[display]->GetDisplayClass() == DISPLAY_CLASS_BUILTIN);
  bool is_power_off = (hwc_display_[display]->GetCurrentPowerMode() == HWC2::PowerMode::Off);
  if (secure_session_active_ && is_builtin && is_power_off) {
    if (GetActiveBuiltinDisplay() != HWCCallbacks::kNumDisplays) {
      DLOGI("Secure session in progress, defer power state change");
      hwc_display_[display]->SetPendingPowerMode(mode);
      return HWC2_ERROR_NONE;
    }
  }

  if (pending_power_mode_[display]) {
    DLOGW("Set power mode is not allowed during secure display session");
    return HWC2_ERROR_UNSUPPORTED;
  }

  //  all displays support on/off. Check for doze modes
  int support = 0;
  auto status = GetDozeSupport(display, &support);
  if (status != HWC2_ERROR_NONE) {
    DLOGE("Failed to get doze support Error = %d", status);
    return INT32(status);
  }

  if (!support && (mode == HWC2::PowerMode::Doze || mode == HWC2::PowerMode::DozeSuspend)) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  // async_powermode supported for power on and off
  bool override_mode = async_powermode_ && display_ready_.test(UINT32(display)) &&
                       async_power_mode_triggered_;
  HWC2::PowerMode last_power_mode = hwc_display_[display]->GetCurrentPowerMode();

  if (last_power_mode == mode) {
    return HWC2_ERROR_NONE;
  }

  // 1. For power transition cases other than Off->On or On->Off, async power mode
  // will not be used. Hence, set override_mode to false for them.
  // 2. When SF requests Doze mode transition on panels where Doze mode is not supported
  // (like video mode), HWComposer.cpp will override the request to "On". Handle such cases
  // in main thread path.
  if (!((last_power_mode == HWC2::PowerMode::Off && mode == HWC2::PowerMode::On) ||
     (last_power_mode == HWC2::PowerMode::On && mode == HWC2::PowerMode::Off)) ||
     (last_power_mode == HWC2::PowerMode::Off && mode == HWC2::PowerMode::On)) {
    override_mode = false;
  }

  if (!override_mode) {
    auto error = CallDisplayFunction(display, &HWCDisplay::SetPowerMode, mode,
                                     false /* teardown */);
    if (INT32(error) != HWC2_ERROR_NONE) {
      return INT32(error);
    }
  } else {
    Locker::ScopeLock lock_disp(locker_[display]);
    // Update hwc state for now. Actual poweron will handled through DisplayConfig.
    hwc_display_[display]->UpdatePowerMode(mode);
  }
  // Reset idle pc ref count on suspend, as we enable idle pc during suspend.
  if (mode == HWC2::PowerMode::Off) {
    idle_pc_ref_cnt_ = 0;
  }


  UpdateThrottlingRate();

  if (mode == HWC2::PowerMode::Doze) {
    // Trigger one more refresh for PP features to take effect.
    pending_refresh_.set(UINT32(display));
  }

  return HWC2_ERROR_NONE;
}

int32_t HWCSession::SetVsyncEnabled(hwc2_display_t display, int32_t int_enabled) {
  //  avoid undefined behavior in cast to HWC2::Vsync
  if (int_enabled < HWC2_VSYNC_INVALID || int_enabled > HWC2_VSYNC_DISABLE) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  auto enabled = static_cast<HWC2::Vsync>(int_enabled);

  if (int_enabled == HWC2_VSYNC_ENABLE) {
    callbacks_.UpdateVsyncSource(display);
  }

  return CallDisplayFunction(display, &HWCDisplay::SetVsyncEnabled, enabled);
}

int32_t HWCSession::GetDozeSupport(hwc2_display_t display, int32_t *out_support) {
  if (!out_support) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays || (hwc_display_[display] == nullptr)) {
    // display may come as -1  from VTS test case
    DLOGE("Invalid Display %d ", UINT32(display));
    return HWC2_ERROR_BAD_DISPLAY;
  }

  *out_support = 0;

  SCOPE_LOCK(locker_[display]);
  if (!hwc_display_[display]) {
    DLOGE("Display %d is not created yet.", INT32(display));
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (hwc_display_[display]->GetDisplayClass() != DISPLAY_CLASS_BUILTIN) {
    return HWC2_ERROR_NONE;
  }

  *out_support = hwc_display_[display]->HasSmartPanelConfig() ? 1 : 0;

  return HWC2_ERROR_NONE;
}

bool HWCSession::isSmartPanelConfig(uint32_t disp_id, uint32_t config_id) {
  if (disp_id != qdutils::DISPLAY_PRIMARY) {
    return false;
  }

  SCOPE_LOCK(locker_[disp_id]);
  if (!hwc_display_[disp_id]) {
    DLOGE("Display %d is not created yet.", disp_id);
    return false;
  }

  return hwc_display_[disp_id]->IsSmartPanelConfig(config_id);
}

int32_t HWCSession::ValidateDisplay(hwc2_display_t display, uint32_t *out_num_types,
                                    uint32_t *out_num_requests) {
  //  out_num_types and out_num_requests will be non-NULL

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  hwc2_display_t target_display = display;

  {
    SCOPE_LOCK(power_state_[display]);
    if (power_state_transition_[display]) {
      // Route all interactions with client to dummy display.
      target_display = map_hwc_display_.find(display)->second;
    }
  }
  DTRACE_SCOPED();
  // TODO(user): Handle secure session, handle QDCM solid fill
  auto status = HWC2::Error::BadDisplay;
  HandleSecureSession();
  {
    SEQUENCE_ENTRY_SCOPE_LOCK(locker_[target_display]);
    if (pending_power_mode_[display]) {
      status = HWC2::Error::None;
    } else if (hwc_display_[target_display]) {
      hwc_display_[target_display]->ProcessActiveConfigChange();
      hwc_display_[target_display]->SetFastPathComposition(false);
      status = ValidateDisplayInternal(target_display, out_num_types, out_num_requests);
    }
  }

  // Sequence locking currently begins on Validate, so cancel the sequence lock on failures
  if (status != HWC2::Error::None && status != HWC2::Error::HasChanges) {
    SEQUENCE_CANCEL_SCOPE_LOCK(locker_[target_display]);
  }

  if (display != target_display) {
    // Validate done on a dummy display. Assume present is complete.
    SEQUENCE_EXIT_SCOPE_LOCK(locker_[target_display]);
  }

  return INT32(status);
}

HWC2::Error HWCSession::CreateVirtualDisplayObj(uint32_t width, uint32_t height, int32_t *format,
                                                hwc2_display_t *out_display_id) {
  hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
  hwc2_display_t client_id = HWCCallbacks::kNumDisplays;
  if (active_builtin_disp_id < HWCCallbacks::kNumDisplays) {
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[active_builtin_disp_id]);
    std::bitset<kSecureMax> secure_sessions = 0;
    if (hwc_display_[active_builtin_disp_id]) {
      hwc_display_[active_builtin_disp_id]->GetActiveSecureSession(&secure_sessions);
    }
    if (secure_sessions.any()) {
      DLOGE("Secure session is active, cannot create virtual display.");
      return HWC2::Error::Unsupported;
    } else if (IsPluggableDisplayConnected()) {
      DLOGE("External session is active, cannot create virtual display.");
      return HWC2::Error::Unsupported;
    }
  }

  if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
    DisplayError error = hwc_display_[HWC_DISPLAY_PRIMARY]->TeardownConcurrentWriteback();
    if (error) {
      return HWC2::Error::NoResources;
    }
  }

  // Lock confined to this scope
  int status = -EINVAL;
  for (auto &map_info : map_info_virtual_) {
    client_id = map_info.client_id;
    {
      SCOPE_LOCK(locker_[client_id]);
      auto &hwc_display = hwc_display_[client_id];
      if (hwc_display) {
        continue;
      }

      int32_t display_id = GetVirtualDisplayId();
      status = virtual_display_factory_.Create(core_intf_, &buffer_allocator_, &callbacks_,
                                               client_id, display_id, width, height,
                                               format, set_min_lum_, set_max_lum_, &hwc_display);
      // TODO(user): validate width and height support
      if (status) {
        return HWC2::Error::NoResources;
      }

      {
        SCOPE_LOCK(hdr_locker_[client_id]);
        is_hdr_display_[UINT32(client_id)] = HasHDRSupport(hwc_display);
      }

      DLOGI("Created virtual display id:%" PRIu64 " with res: %dx%d", client_id, width, height);

      *out_display_id = client_id;
      map_info.disp_type = kVirtual;
      map_info.sdm_id = display_id;
      break;
    }
  }

  // Active builtin display needs revalidation
  if (!async_vds_creation_ && active_builtin_disp_id < HWCCallbacks::kNumRealDisplays) {
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[active_builtin_disp_id]);
    hwc_display_[active_builtin_disp_id]->ResetValidation();
  }

  return HWC2::Error::None;
}

bool HWCSession::IsPluggableDisplayConnected() {
  for (auto &map_info : map_info_pluggable_) {
    if (hwc_display_[map_info.client_id]) {
      return true;
    }
  }
  return false;
}

// Qclient methods
android::status_t HWCSession::notifyCallback(uint32_t command, const android::Parcel *input_parcel,
                                             android::Parcel *output_parcel) {
  android::status_t status = -EINVAL;

  switch (command) {
    case qService::IQService::DYNAMIC_DEBUG:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = 0;
      DynamicDebug(input_parcel);
      break;

    case qService::IQService::SCREEN_REFRESH:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = RefreshScreen(input_parcel);
      break;

    case qService::IQService::SET_IDLE_TIMEOUT:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetIdleTimeout(UINT32(input_parcel->readInt32()));
      break;

    case qService::IQService::SET_FRAME_DUMP_CONFIG:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetFrameDumpConfig(input_parcel);
      break;

    case qService::IQService::SET_MAX_PIPES_PER_MIXER:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetMaxMixerStages(input_parcel);
      break;

    case qService::IQService::SET_DISPLAY_MODE:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetDisplayMode(input_parcel);
      break;

    case qService::IQService::SET_SECONDARY_DISPLAY_STATUS: {
        if (!input_parcel || !output_parcel) {
          DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
          break;
        }
        int disp_id = INT(input_parcel->readInt32());
        HWCDisplay::DisplayStatus disp_status =
              static_cast<HWCDisplay::DisplayStatus>(input_parcel->readInt32());
        status = SetDisplayStatus(disp_id, disp_status);
        output_parcel->writeInt32(status);
      }
      break;

    case qService::IQService::CONFIGURE_DYN_REFRESH_RATE:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = ConfigureRefreshRate(input_parcel);
      break;

    case qService::IQService::TOGGLE_SCREEN_UPDATES: {
        if (!input_parcel || !output_parcel) {
          DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
          break;
        }
        int32_t input = input_parcel->readInt32();
        status = ToggleScreenUpdate(input == 1);
        output_parcel->writeInt32(status);
      }
      break;

    case qService::IQService::QDCM_SVC_CMDS:
      if (!input_parcel || !output_parcel) {
        DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
        break;
      }
      status = QdcmCMDHandler(input_parcel, output_parcel);
      break;

    case qService::IQService::MIN_HDCP_ENCRYPTION_LEVEL_CHANGED: {
        if (!input_parcel || !output_parcel) {
          DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
          break;
        }
        int disp_id = input_parcel->readInt32();
        uint32_t min_enc_level = UINT32(input_parcel->readInt32());
        status = MinHdcpEncryptionLevelChanged(disp_id, min_enc_level);
        output_parcel->writeInt32(status);
      }
      break;

    case qService::IQService::CONTROL_PARTIAL_UPDATE: {
        if (!input_parcel || !output_parcel) {
          DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
          break;
        }
        int disp_id = input_parcel->readInt32();
        uint32_t enable = UINT32(input_parcel->readInt32());
        status = ControlPartialUpdate(disp_id, enable == 1);
        output_parcel->writeInt32(status);
      }
      break;

    case qService::IQService::SET_ACTIVE_CONFIG: {
        if (!input_parcel) {
          DLOGE("QService command = %d: input_parcel needed.", command);
          break;
        }
        uint32_t config = UINT32(input_parcel->readInt32());
        int disp_id = input_parcel->readInt32();
        status = SetActiveConfigIndex(disp_id, config);
      }
      break;

    case qService::IQService::GET_ACTIVE_CONFIG: {
        if (!input_parcel || !output_parcel) {
          DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
          break;
        }
        int disp_id = input_parcel->readInt32();
        uint32_t config = 0;
        status = GetActiveConfigIndex(disp_id, &config);
        output_parcel->writeInt32(INT(config));
      }
      break;

    case qService::IQService::GET_CONFIG_COUNT: {
        if (!input_parcel || !output_parcel) {
          DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
          break;
        }
        int disp_id = input_parcel->readInt32();
        uint32_t count = 0;
        status = GetConfigCount(disp_id, &count);
        output_parcel->writeInt32(INT(count));
      }
      break;

    case qService::IQService::GET_DISPLAY_ATTRIBUTES_FOR_CONFIG:
      if (!input_parcel || !output_parcel) {
        DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
        break;
      }
      status = GetDisplayAttributesForConfig(input_parcel, output_parcel);
      break;

    case qService::IQService::GET_PANEL_BRIGHTNESS: {
        if (!output_parcel) {
          DLOGE("QService command = %d: output_parcel needed.", command);
          break;
        }

        uint32_t display = input_parcel->readUint32();
        uint32_t max_brightness_level = 0;
        status = getDisplayMaxBrightness(display, &max_brightness_level);
        if (status || !max_brightness_level) {
          output_parcel->writeInt32(max_brightness_level);
          DLOGE("Failed to get max brightness %u,  status %d", max_brightness_level, status);
          break;
        }
        DLOGV("Panel Max brightness is %u", max_brightness_level);

        float brightness_precent = -1.0f;
        status = getDisplayBrightness(display, &brightness_precent);
        if (brightness_precent == -1.0f) {
          output_parcel->writeInt32(0);
        } else {
          output_parcel->writeInt32(INT32(brightness_precent*(max_brightness_level - 1) + 1));
        }
      }
      break;

    case qService::IQService::SET_PANEL_BRIGHTNESS: {
        if (!input_parcel || !output_parcel) {
          DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
          break;
        }

        uint32_t max_brightness_level = 0;
        uint32_t display = HWC_DISPLAY_PRIMARY;
        status = getDisplayMaxBrightness(display, &max_brightness_level);
        if (status || max_brightness_level <= 1) {
          output_parcel->writeInt32(max_brightness_level);
          DLOGE("Failed to get max brightness %u, status %d", max_brightness_level, status);
          break;
        }
        DLOGV("Panel Max brightness is %u", max_brightness_level);

        int level = input_parcel->readInt32();
        if (level == 0) {
          status = SetDisplayBrightness(display, -1.0f);
        } else {
          status = SetDisplayBrightness(display,
                    (level - 1)/(static_cast<float>(max_brightness_level - 1)));
        }
        output_parcel->writeInt32(status);
      }
      break;

    case qService::IQService::GET_DISPLAY_VISIBLE_REGION:
      if (!input_parcel || !output_parcel) {
        DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
        break;
      }
      status = GetVisibleDisplayRect(input_parcel, output_parcel);
      break;

    case qService::IQService::SET_CAMERA_STATUS: {
        if (!input_parcel) {
          DLOGE("QService command = %d: input_parcel needed.", command);
          break;
        }
        uint32_t camera_status = UINT32(input_parcel->readInt32());
        status = SetCameraLaunchStatus(camera_status);
      }
      break;

    case qService::IQService::GET_BW_TRANSACTION_STATUS: {
        if (!output_parcel) {
          DLOGE("QService command = %d: output_parcel needed.", command);
          break;
        }
        bool state = true;
        status = DisplayBWTransactionPending(&state);
        output_parcel->writeInt32(state);
      }
      break;

    case qService::IQService::SET_LAYER_MIXER_RESOLUTION:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetMixerResolution(input_parcel);
      break;

    case qService::IQService::SET_COLOR_MODE:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetColorModeOverride(input_parcel);
      break;

    case qService::IQService::SET_COLOR_MODE_WITH_RENDER_INTENT:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetColorModeWithRenderIntentOverride(input_parcel);
      break;

    case qService::IQService::SET_COLOR_MODE_BY_ID:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetColorModeById(input_parcel);
      break;

    case qService::IQService::GET_COMPOSER_STATUS:
      if (!output_parcel) {
        DLOGE("QService command = %d: output_parcel needed.", command);
        break;
      }
      status = 0;
      output_parcel->writeInt32(getComposerStatus());
      break;

    case qService::IQService::SET_QSYNC_MODE:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetQSyncMode(input_parcel);
      break;

    case qService::IQService::SET_COLOR_SAMPLING_ENABLED:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = setColorSamplingEnabled(input_parcel);
      break;

    case qService::IQService::SET_IDLE_PC:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetIdlePC(input_parcel);
      break;

    case qService::IQService::SET_DPPS_AD4_ROI_CONFIG:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetAd4RoiConfig(input_parcel);
      break;

    case qService::IQService::SET_DSI_CLK:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetDsiClk(input_parcel);
      break;

    case qService::IQService::GET_DSI_CLK:
      if (!input_parcel || !output_parcel) {
        DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
        break;
      }
      status = GetDsiClk(input_parcel, output_parcel);
      break;

    case qService::IQService::GET_SUPPORTED_DSI_CLK:
      if (!input_parcel || !output_parcel) {
        DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
        break;
      }
      status = GetSupportedDsiClk(input_parcel, output_parcel);
      break;

    case qService::IQService::SET_PANEL_LUMINANCE:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetPanelLuminanceAttributes(input_parcel);
      break;

    case qService::IQService::SET_COLOR_MODE_FROM_CLIENT:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetColorModeFromClient(input_parcel);
      break;

    case qService::IQService::SET_FRAME_TRIGGER_MODE:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetFrameTriggerMode(input_parcel);
      break;

    case qService::IQService::SET_BRIGHTNESS_SCALE:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetDisplayBrightnessScale(input_parcel);
      break;

    case qService::IQService::SET_STAND_BY_MODE:
      if (!input_parcel) {
        DLOGE("QService command = %d: input_parcel needed.", command);
        break;
      }
      status = SetStandByMode(input_parcel);
      break;

    case qService::IQService::GET_PANEL_RESOLUTION:
      if (!input_parcel || !output_parcel) {
        DLOGE("QService command = %d: input_parcel and output_parcel needed.", command);
        break;
      }
      status = GetPanelResolution(input_parcel, output_parcel);
      break;

    case qService::IQService::DELAY_FIRST_COMMIT:
      status = DelayFirstCommit();
      break;

    default:
      DLOGW("QService command = %d is not supported.", command);
      break;
  }

  return status;
}

android::status_t HWCSession::getComposerStatus() {
  return is_composer_up_;
}

android::status_t HWCSession::GetDisplayAttributesForConfig(const android::Parcel *input_parcel,
                                                            android::Parcel *output_parcel) {
  int config = input_parcel->readInt32();
  int dpy = input_parcel->readInt32();
  int error = android::BAD_VALUE;
  DisplayConfigVariableInfo display_attributes;

  int disp_idx = GetDisplayIndex(dpy);
  if (disp_idx == -1 || config < 0) {
    DLOGE("Invalid display = %d, or config = %d", dpy, config);
    return android::BAD_VALUE;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
  if (hwc_display_[disp_idx]) {
    error = hwc_display_[disp_idx]->GetDisplayAttributesForConfig(config, &display_attributes);
    if (error == 0) {
      output_parcel->writeInt32(INT(display_attributes.vsync_period_ns));
      output_parcel->writeInt32(INT(display_attributes.x_pixels));
      output_parcel->writeInt32(INT(display_attributes.y_pixels));
      output_parcel->writeFloat(display_attributes.x_dpi);
      output_parcel->writeFloat(display_attributes.y_dpi);
      output_parcel->writeInt32(0);  // Panel type, unsupported.
    }
  }

  return error;
}

android::status_t HWCSession::setColorSamplingEnabled(const android::Parcel *input_parcel) {
  int dpy = input_parcel->readInt32();
  int enabled_cmd = input_parcel->readInt32();
  if (dpy < HWC_DISPLAY_PRIMARY || dpy >= HWC_NUM_DISPLAY_TYPES || enabled_cmd < 0 ||
      enabled_cmd > 1) {
    return android::BAD_VALUE;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[dpy]);
  if (!hwc_display_[dpy]) {
    DLOGW("No display id %i active to enable histogram event", dpy);
    return android::BAD_VALUE;
  }

  auto error = hwc_display_[dpy]->SetDisplayedContentSamplingEnabledVndService(enabled_cmd);
  return (error == HWC2::Error::None) ? android::OK : android::BAD_VALUE;
}

android::status_t HWCSession::ConfigureRefreshRate(const android::Parcel *input_parcel) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  uint32_t operation = UINT32(input_parcel->readInt32());
  HWCDisplay *hwc_display = hwc_display_[HWC_DISPLAY_PRIMARY];

  if (!hwc_display) {
    DLOGW("Display = %d is not connected.", HWC_DISPLAY_PRIMARY);
    return -ENODEV;
  }

  switch (operation) {
    case qdutils::DISABLE_METADATA_DYN_REFRESH_RATE:
      return hwc_display->Perform(HWCDisplayBuiltIn::SET_METADATA_DYN_REFRESH_RATE, false);

    case qdutils::ENABLE_METADATA_DYN_REFRESH_RATE:
      return hwc_display->Perform(HWCDisplayBuiltIn::SET_METADATA_DYN_REFRESH_RATE, true);

    case qdutils::SET_BINDER_DYN_REFRESH_RATE: {
      uint32_t refresh_rate = UINT32(input_parcel->readInt32());
      return hwc_display->Perform(HWCDisplayBuiltIn::SET_BINDER_DYN_REFRESH_RATE, refresh_rate);
    }

    default:
      DLOGW("Invalid operation %d", operation);
      return -EINVAL;
  }

  return 0;
}

android::status_t HWCSession::SetDisplayMode(const android::Parcel *input_parcel) {
  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  if (!hwc_display_[HWC_DISPLAY_PRIMARY]) {
    DLOGW("Display = %d is not connected.", HWC_DISPLAY_PRIMARY);
    return -ENODEV;
  }

  uint32_t mode = UINT32(input_parcel->readInt32());
  return hwc_display_[HWC_DISPLAY_PRIMARY]->Perform(HWCDisplayBuiltIn::SET_DISPLAY_MODE, mode);
}

android::status_t HWCSession::SetMaxMixerStages(const android::Parcel *input_parcel) {
  DisplayError error = kErrorNone;
  std::bitset<32> bit_mask_display_type = UINT32(input_parcel->readInt32());
  uint32_t max_mixer_stages = UINT32(input_parcel->readInt32());
  android::status_t status = 0;

  for (uint32_t i = 0; i < 32 && bit_mask_display_type[i]; i++) {
    int disp_idx = GetDisplayIndex(INT(i));
    if (disp_idx == -1) {
      continue;
    }
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
    auto &hwc_display = hwc_display_[disp_idx];
    if (!hwc_display) {
      DLOGW("Display = %d is not connected.", disp_idx);
      status = (status)? status : -ENODEV;  // Return higher priority error.
      continue;
    }

    error = hwc_display->SetMaxMixerStages(max_mixer_stages);
    if (error != kErrorNone) {
      status = -EINVAL;
    }
  }

  return status;
}

android::status_t HWCSession::SetFrameDumpConfig(const android::Parcel *input_parcel) {
  uint32_t frame_dump_count = UINT32(input_parcel->readInt32());
  std::bitset<32> bit_mask_display_type = UINT32(input_parcel->readInt32());
  uint32_t bit_mask_layer_type = UINT32(input_parcel->readInt32());
  int32_t output_format = HAL_PIXEL_FORMAT_RGB_888;
  bool post_processed = true;

  // Output buffer dump is not supported, if External or Virtual display is present.
  bool output_buffer_dump = bit_mask_layer_type & (1 << OUTPUT_LAYER_DUMP);
  if (output_buffer_dump) {
    int external_dpy_index = GetDisplayIndex(qdutils::DISPLAY_EXTERNAL);
    int virtual_dpy_index = GetDisplayIndex(qdutils::DISPLAY_VIRTUAL);
    if (((external_dpy_index != -1) && hwc_display_[external_dpy_index]) ||
        ((virtual_dpy_index != -1) && hwc_display_[virtual_dpy_index])) {
      DLOGW("Output buffer dump is not supported with External or Virtual display!");
      return -EINVAL;
    }
  }

  // Read optional user preferences: output_format and post_processed.
  if (input_parcel->dataPosition() != input_parcel->dataSize()) {
    // HAL Pixel Format for output buffer
    output_format = input_parcel->readInt32();
  }
  if (input_parcel->dataPosition() != input_parcel->dataSize()) {
    // Option to dump Layer Mixer output (0) or DSPP output (1)
    post_processed = (input_parcel->readInt32() != 0);
  }

  android::status_t status = 0;

  for (uint32_t i = 0; i < bit_mask_display_type.size(); i++) {
    if (!bit_mask_display_type[i]) {
      continue;
    }
    int disp_idx = GetDisplayIndex(INT(i));
    if (disp_idx == -1) {
      continue;
    }
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
    auto &hwc_display = hwc_display_[disp_idx];
    if (!hwc_display) {
      DLOGW("Display = %d is not connected.", disp_idx);
      status = (status)? status : -ENODEV;  // Return higher priority error.
      continue;
    }

    HWC2::Error error = hwc_display->SetFrameDumpConfig(frame_dump_count, bit_mask_layer_type,
                                                        output_format, post_processed);
    if (error != HWC2::Error::None) {
      status = (HWC2::Error::NoResources == error) ? -ENOMEM : -EINVAL;
    }
  }

  return status;
}

android::status_t HWCSession::SetMixerResolution(const android::Parcel *input_parcel) {
  DisplayError error = kErrorNone;
  uint32_t dpy = UINT32(input_parcel->readInt32());

  if (dpy != HWC_DISPLAY_PRIMARY) {
    DLOGW("Resolution change not supported for this display = %d", dpy);
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);
  if (!hwc_display_[HWC_DISPLAY_PRIMARY]) {
    DLOGW("Primary display is not initialized");
    return -ENODEV;
  }

  uint32_t width = UINT32(input_parcel->readInt32());
  uint32_t height = UINT32(input_parcel->readInt32());

  error = hwc_display_[HWC_DISPLAY_PRIMARY]->SetMixerResolution(width, height);
  if (error != kErrorNone) {
    return -EINVAL;
  }

  return 0;
}

android::status_t HWCSession::SetColorModeOverride(const android::Parcel *input_parcel) {
  int display = static_cast<int>(input_parcel->readInt32());
  auto mode = static_cast<ColorMode>(input_parcel->readInt32());

  int disp_idx = GetDisplayIndex(display);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", display);
    return -EINVAL;
  }

  if (mode < ColorMode::NATIVE || mode > ColorMode::DISPLAY_BT2020) {
    DLOGE("Invalid ColorMode: %d", mode);
    return HWC2_ERROR_BAD_PARAMETER;
  }
  auto err = CallDisplayFunction(static_cast<hwc2_display_t>(disp_idx), &HWCDisplay::SetColorMode,
                                 mode);
  if (err != HWC2_ERROR_NONE)
    return -EINVAL;

  return 0;
}

android::status_t HWCSession::SetAd4RoiConfig(const android::Parcel *input_parcel) {
  auto display_id = static_cast<uint32_t>(input_parcel->readInt32());
  auto h_s = static_cast<uint32_t>(input_parcel->readInt32());
  auto h_e = static_cast<uint32_t>(input_parcel->readInt32());
  auto v_s = static_cast<uint32_t>(input_parcel->readInt32());
  auto v_e = static_cast<uint32_t>(input_parcel->readInt32());
  auto f_in = static_cast<uint32_t>(input_parcel->readInt32());
  auto f_out = static_cast<uint32_t>(input_parcel->readInt32());

  return static_cast<android::status_t>(SetDisplayDppsAdROI(display_id, h_s, h_e, v_s,
                                                            v_e, f_in, f_out));
}

android::status_t HWCSession::SetFrameTriggerMode(const android::Parcel *input_parcel) {
  auto display_id = static_cast<int>(input_parcel->readInt32());
  auto mode = static_cast<uint32_t>(input_parcel->readInt32());

  int disp_idx = GetDisplayIndex(display_id);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", display_id);
    return -EINVAL;
  }

  auto err = CallDisplayFunction(static_cast<hwc2_display_t>(disp_idx),
                                 &HWCDisplay::SetFrameTriggerMode, mode);
  if (err != HWC2_ERROR_NONE)
    return -EINVAL;

  return 0;
}

android::status_t HWCSession::SetColorModeWithRenderIntentOverride(
    const android::Parcel *input_parcel) {
  auto display = static_cast<hwc2_display_t>(input_parcel->readInt32());
  auto mode = static_cast<ColorMode>(input_parcel->readInt32());
  auto int_intent = static_cast<int>(input_parcel->readInt32());

  if (mode < ColorMode::NATIVE || mode > ColorMode::DISPLAY_BT2020) {
    DLOGE("Invalid ColorMode: %d", mode);
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if ((int_intent < 0) || (int_intent > MAX_EXTENDED_RENDER_INTENT)) {
    DLOGE("Invalid RenderIntent: %d", int_intent);
    return HWC2_ERROR_BAD_PARAMETER;
  }

  auto intent = static_cast<RenderIntent>(int_intent);
  auto err =
      CallDisplayFunction(display, &HWCDisplay::SetColorModeWithRenderIntent, mode, intent);
  if (err != HWC2_ERROR_NONE)
    return -EINVAL;

  return 0;
}
android::status_t HWCSession::SetColorModeById(const android::Parcel *input_parcel) {
  int display = input_parcel->readInt32();
  auto mode = input_parcel->readInt32();

  int disp_idx = GetDisplayIndex(display);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", display);
    return -EINVAL;
  }

  auto err = CallDisplayFunction(static_cast<hwc2_display_t>(disp_idx),
                                 &HWCDisplay::SetColorModeById, mode);
  if (err != HWC2_ERROR_NONE)
    return -EINVAL;

  return 0;
}

android::status_t HWCSession::SetColorModeFromClient(const android::Parcel *input_parcel) {
  int display = input_parcel->readInt32();
  auto mode = input_parcel->readInt32();

  int disp_idx = GetDisplayIndex(display);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", display);
    return -EINVAL;
  }

  auto err = CallDisplayFunction(static_cast<hwc2_display_t>(disp_idx),
                                 &HWCDisplay::SetColorModeFromClientApi, mode);
  if (err != HWC2_ERROR_NONE)
    return -EINVAL;

  callbacks_.Refresh(static_cast<hwc2_display_t>(disp_idx));

  return 0;
}

android::status_t HWCSession::RefreshScreen(const android::Parcel *input_parcel) {
  int display = input_parcel->readInt32();

  int disp_idx = GetDisplayIndex(display);
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", display);
    return -EINVAL;
  }

  callbacks_.Refresh(static_cast<hwc2_display_t>(disp_idx));

  return 0;
}

void HWCSession::DynamicDebug(const android::Parcel *input_parcel) {
  int type = input_parcel->readInt32();
  bool enable = (input_parcel->readInt32() > 0);
  DLOGI("type = %d enable = %d", type, enable);
  int verbose_level = input_parcel->readInt32();

  switch (type) {
    case qService::IQService::DEBUG_ALL:
      HWCDebugHandler::DebugAll(enable, verbose_level);
      break;

    case qService::IQService::DEBUG_MDPCOMP:
      HWCDebugHandler::DebugStrategy(enable, verbose_level);
      HWCDebugHandler::DebugCompManager(enable, verbose_level);
      break;

    case qService::IQService::DEBUG_PIPE_LIFECYCLE:
      HWCDebugHandler::DebugResources(enable, verbose_level);
      HWCDebugHandler::DebugQos(enable, verbose_level);
      break;

    case qService::IQService::DEBUG_DRIVER_CONFIG:
      HWCDebugHandler::DebugDriverConfig(enable, verbose_level);
      break;

    case qService::IQService::DEBUG_ROTATOR:
      HWCDebugHandler::DebugResources(enable, verbose_level);
      HWCDebugHandler::DebugDriverConfig(enable, verbose_level);
      HWCDebugHandler::DebugRotator(enable, verbose_level);
      HWCDebugHandler::DebugQos(enable, verbose_level);
      break;

    case qService::IQService::DEBUG_QDCM:
      HWCDebugHandler::DebugQdcm(enable, verbose_level);
      break;

    case qService::IQService::DEBUG_SCALAR:
      HWCDebugHandler::DebugScalar(enable, verbose_level);
      break;

    case qService::IQService::DEBUG_CLIENT:
      HWCDebugHandler::DebugClient(enable, verbose_level);
      break;

    case qService::IQService::DEBUG_DISPLAY:
      HWCDebugHandler::DebugDisplay(enable, verbose_level);
      break;

    default:
      DLOGW("type = %d is not supported", type);
  }
}

android::status_t HWCSession::QdcmCMDDispatch(uint32_t display_id,
                                              const PPDisplayAPIPayload &req_payload,
                                              PPDisplayAPIPayload *resp_payload,
                                              PPPendingParams *pending_action) {
  int ret = 0;
  bool is_physical_display = false;

  if (display_id >= HWCCallbacks::kNumDisplays || !hwc_display_[display_id]) {
      DLOGW("Invalid display id or display = %d is not connected.", display_id);
      return -ENODEV;
  }

  if (display_id == map_info_primary_.client_id) {
    is_physical_display = true;
  } else {
    for (auto &map_info : map_info_builtin_) {
      if (map_info.client_id == display_id) {
        is_physical_display = true;
        break;
     }
    }
  }

  if (!is_physical_display) {
    DLOGW("Skipping QDCM command dispatch on display = %d", display_id);
    return ret;
  }

  ret = hwc_display_[display_id]->ColorSVCRequestRoute(req_payload, resp_payload, pending_action);

  return ret;
}

android::status_t HWCSession::QdcmCMDHandler(const android::Parcel *input_parcel,
                                             android::Parcel *output_parcel) {
  int ret = 0;
  float *brightness = NULL;
  uint32_t display_id(0);
  PPPendingParams pending_action;
  PPDisplayAPIPayload resp_payload, req_payload;
  uint8_t *disp_id = NULL;
  int32_t *mode_id = NULL;

  if (!color_mgr_) {
    DLOGW("color_mgr_ not initialized.");
    return -ENOENT;
  }

  pending_action.action = kNoAction;
  pending_action.params = NULL;

  // Read display_id, payload_size and payload from in_parcel.
  ret = HWCColorManager::CreatePayloadFromParcel(*input_parcel, &display_id, &req_payload);
  if (!ret) {
    ret = QdcmCMDDispatch(display_id, req_payload, &resp_payload, &pending_action);
  }

  if (ret) {
    output_parcel->writeInt32(ret);  // first field in out parcel indicates return code.
    req_payload.DestroyPayload();
    resp_payload.DestroyPayload();
    return ret;
  }

  if (kNoAction != pending_action.action) {
    int32_t action = pending_action.action;
    int count = -1;
    while (action > 0) {
      count++;
      int32_t bit = (action & 1);
      action = action >> 1;

      if (!bit)
        continue;

      DLOGV_IF(kTagQDCM, "pending action = %d, display_id = %d", BITMAP(count), display_id);
      switch (BITMAP(count)) {
        case kInvalidating:
          callbacks_.Refresh(display_id);
          break;
        case kEnterQDCMMode:
          ret = color_mgr_->EnableQDCMMode(true, hwc_display_[display_id]);
          break;
        case kExitQDCMMode:
          ret = color_mgr_->EnableQDCMMode(false, hwc_display_[display_id]);
          break;
        case kApplySolidFill:
          {
            SCOPE_LOCK(locker_[display_id]);
            ret = color_mgr_->SetSolidFill(pending_action.params,
                                           true, hwc_display_[display_id]);
          }
          callbacks_.Refresh(display_id);
          usleep(kSolidFillDelay);
          break;
        case kDisableSolidFill:
          {
            SCOPE_LOCK(locker_[display_id]);
            ret = color_mgr_->SetSolidFill(pending_action.params,
                                           false, hwc_display_[display_id]);
          }
          callbacks_.Refresh(display_id);
          usleep(kSolidFillDelay);
          break;
        case kSetPanelBrightness:
          ret = -EINVAL;
          brightness = reinterpret_cast<float *>(resp_payload.payload);
          if (brightness == NULL) {
            DLOGE("Brightness payload is Null");
          } else {
            ret = INT(SetDisplayBrightness(static_cast<hwc2_display_t>(display_id), *brightness));
          }
          break;
        case kEnableFrameCapture:
          ret = color_mgr_->SetFrameCapture(pending_action.params, true, hwc_display_[display_id]);
          callbacks_.Refresh(display_id);
          break;
        case kDisableFrameCapture:
          ret = color_mgr_->SetFrameCapture(pending_action.params, false,
                                            hwc_display_[display_id]);
          break;
        case kConfigureDetailedEnhancer:
          ret = color_mgr_->SetDetailedEnhancer(pending_action.params, hwc_display_[display_id]);
          callbacks_.Refresh(display_id);
          break;
        case kModeSet:
          ret = static_cast<int>
                  (hwc_display_[display_id]->RestoreColorTransform());
          callbacks_.Refresh(display_id);
          break;
        case kNoAction:
          break;
        case kMultiDispProc:
          for (auto &map_info : map_info_builtin_) {
            uint32_t id = UINT32(map_info.client_id);
            if (id < HWCCallbacks::kNumDisplays && hwc_display_[id]) {
              int result = 0;
              resp_payload.DestroyPayload();
              result = hwc_display_[id]->ColorSVCRequestRoute(req_payload, &resp_payload,
                                                              &pending_action);
              if (result) {
                DLOGW("Failed to dispatch action to disp %d ret %d", id, result);
                ret = result;
              }
            }
          }
          break;
        case kMultiDispGetId:
          ret = resp_payload.CreatePayloadBytes(HWCCallbacks::kNumDisplays, &disp_id);
          if (ret) {
            DLOGW("Unable to create response payload!");
          } else {
            for (int i = 0; i < HWCCallbacks::kNumDisplays; i++) {
              disp_id[i] = HWCCallbacks::kNumDisplays;
            }
            if (hwc_display_[HWC_DISPLAY_PRIMARY]) {
              disp_id[HWC_DISPLAY_PRIMARY] = HWC_DISPLAY_PRIMARY;
            }
            for (auto &map_info : map_info_builtin_) {
              uint64_t id = map_info.client_id;
              if (id < HWCCallbacks::kNumDisplays && hwc_display_[id]) {
                disp_id[id] = (uint8_t)id;
              }
            }
          }
          break;
        case kSetModeFromClient:
          {
            SCOPE_LOCK(locker_[display_id]);
            mode_id = reinterpret_cast<int32_t *>(resp_payload.payload);
            if (mode_id) {
              ret = static_cast<int>(hwc_display_[display_id]->SetColorModeFromClientApi(*mode_id));
            } else {
              DLOGE("mode_id is Null");
              ret = -EINVAL;
            }
          }
          if (!ret) {
            callbacks_.Refresh(display_id);
          }
          break;
        default:
          DLOGW("Invalid pending action = %d!", pending_action.action);
          break;
      }
    }
  }
  // for display API getter case, marshall returned params into out_parcel.
  output_parcel->writeInt32(ret);
  HWCColorManager::MarshallStructIntoParcel(resp_payload, output_parcel);
  req_payload.DestroyPayload();
  resp_payload.DestroyPayload();

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[display_id]);
  hwc_display_[display_id]->ResetValidation();

  return ret;
}

int GetEventValue(const char *uevent_data, int length, const char *event_info) {
  const char *iterator_str = uevent_data;
  while (((iterator_str - uevent_data) <= length) && (*iterator_str)) {
    const char *pstr = strstr(iterator_str, event_info);
    if (pstr != NULL) {
      return (atoi(iterator_str + strlen(event_info)));
    }
    iterator_str += strlen(iterator_str) + 1;
  }

  return -1;
}

const char *GetTokenValue(const char *uevent_data, int length, const char *token) {
  const char *iterator_str = uevent_data;
  const char *pstr = NULL;
  while (((iterator_str - uevent_data) <= length) && (*iterator_str)) {
    pstr = strstr(iterator_str, token);
    if (pstr) {
      break;
    }
    iterator_str += strlen(iterator_str) + 1;
  }

  if (pstr)
    pstr = pstr+strlen(token);

  return pstr;
}

android::status_t HWCSession::SetDsiClk(const android::Parcel *input_parcel) {
  int disp_id = input_parcel->readInt32();
  uint64_t clk = UINT64(input_parcel->readInt64());
  if (disp_id != HWC_DISPLAY_PRIMARY) {
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_id]);
  if (!hwc_display_[disp_id]) {
    return -EINVAL;
  }

  return hwc_display_[disp_id]->SetDynamicDSIClock(clk);
}

android::status_t HWCSession::GetDsiClk(const android::Parcel *input_parcel,
                                        android::Parcel *output_parcel) {
  int disp_id = input_parcel->readInt32();
  if (disp_id != HWC_DISPLAY_PRIMARY) {
    return -EINVAL;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_id]);
  if (!hwc_display_[disp_id]) {
    return -EINVAL;
  }

  uint64_t bitrate = 0;
  hwc_display_[disp_id]->GetDynamicDSIClock(&bitrate);
  output_parcel->writeUint64(bitrate);

  return 0;
}

android::status_t HWCSession::GetSupportedDsiClk(const android::Parcel *input_parcel,
                                                 android::Parcel *output_parcel) {
  int disp_id = input_parcel->readInt32();
  if (disp_id != HWC_DISPLAY_PRIMARY) {
    return -EINVAL;
  }

  SCOPE_LOCK(locker_[disp_id]);
  if (!hwc_display_[disp_id]) {
    return -EINVAL;
  }

  std::vector<uint64_t> bit_rates;
  hwc_display_[disp_id]->GetSupportedDSIClock(&bit_rates);
  output_parcel->writeInt32(INT32(bit_rates.size()));
  for (auto &bit_rate : bit_rates) {
    output_parcel->writeUint64(bit_rate);
  }

  return 0;
}

android::status_t HWCSession::SetPanelLuminanceAttributes(const android::Parcel *input_parcel) {
  int disp_id = input_parcel->readInt32();

  // currently doing only for virtual display
  if (disp_id != qdutils::DISPLAY_VIRTUAL) {
    return -EINVAL;
  }

  float min_lum = input_parcel->readFloat();
  float max_lum = input_parcel->readFloat();

  // check for out of range luminance values
  if (min_lum <= 0.0f || min_lum >= 1.0f || max_lum <= 100.0f || max_lum >= 1000.0f) {
    return -EINVAL;
  }

  std::lock_guard<std::mutex> obj(mutex_lum_);
  set_min_lum_ = min_lum;
  set_max_lum_ = max_lum;
  DLOGI("set max_lum %f, min_lum %f", set_max_lum_, set_min_lum_);

  return 0;
}

void HWCSession::UEventHandler(const char *uevent_data, int length) {
  // Drop hotplug uevents until SurfaceFlinger (the client) is connected. The equivalent of hotplug
  // uevent handling will be done once when SurfaceFlinger connects, at RegisterCallback(). Since
  // HandlePluggableDisplays() reads the latest connection states of all displays, no uevent is
  // lost.
  if (callbacks_.IsClientConnected() && strcasestr(uevent_data, HWC_UEVENT_DRM_EXT_HOTPLUG)) {
    // MST hotplug will not carry connection status/test pattern etc.
    // Pluggable display handler will check all connection status' and take action accordingly.
    const char *str_status = GetTokenValue(uevent_data, length, "status=");
    const char *str_mst = GetTokenValue(uevent_data, length, "MST_HOTPLUG=");
    if (!str_status && !str_mst) {
      return;
    }

    hpd_bpp_ = GetEventValue(uevent_data, length, "bpp=");
    hpd_pattern_ = GetEventValue(uevent_data, length, "pattern=");
    DLOGI("Uevent = %s, status = %s, MST_HOTPLUG = %s, bpp = %d, pattern = %d", uevent_data,
          str_status ? str_status : "NULL", str_mst ? str_mst : "NULL", hpd_bpp_, hpd_pattern_);

    hwc2_display_t virtual_display_index =
        (hwc2_display_t)GetDisplayIndex(qdutils::DISPLAY_VIRTUAL);

    std::bitset<kSecureMax> secure_sessions = 0;
    hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
    if (active_builtin_disp_id < HWCCallbacks::kNumDisplays) {
      Locker::ScopeLock lock_a(locker_[active_builtin_disp_id]);
      hwc_display_[active_builtin_disp_id]->GetActiveSecureSession(&secure_sessions);
    }
    if (secure_sessions[kSecureDisplay] || hwc_display_[virtual_display_index]) {
      // Defer hotplug handling.
      SCOPE_LOCK(pluggable_handler_lock_);
      DLOGI("Marking hotplug pending...");
      pending_hotplug_event_ = kHotPlugEvent;
    } else {
      // Handle hotplug.
      int32_t err = HandlePluggableDisplays(true);
      if (err) {
        DLOGW("Hotplug handling failed. Error %d '%s'. Hotplug handling %s.", err,
              strerror(abs(err)), (pending_hotplug_event_ == kHotPlugEvent) ?
              "deferred" : "dropped");
      }
    }

    if (str_status) {
      bool connected = (strncmp(str_status, "connected", strlen("connected")) == 0);
      DLOGI("Connected = %d", connected);
      // Pass on legacy HDMI hot-plug event.
      qservice_->onHdmiHotplug(INT(connected));
    }
  }
}

int32_t HWCSession::GetVsyncPeriod(hwc2_display_t disp, uint32_t *vsync_period) {
  if (disp >= HWCCallbacks::kNumDisplays) {
    DLOGW("Invalid Display : display = %" PRIu64, disp);
    return HWC2_ERROR_BAD_DISPLAY;
  }

  SCOPE_LOCK(locker_[(int)disp]);
  // default value
  *vsync_period = 1000000000ul / 60;

  if (hwc_display_[disp]) {
    hwc_display_[disp]->GetDisplayAttribute(0, HwcAttribute::VSYNC_PERIOD, (int32_t *)vsync_period);
  }

  return HWC2_ERROR_NONE;
}

void HWCSession::Refresh(hwc2_display_t display) {
  callbacks_.Refresh(display);
}

android::status_t HWCSession::GetPanelResolution(const android::Parcel *input_parcel,
                                                 android::Parcel *output_parcel) {
  SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  if (!hwc_display_[HWC_DISPLAY_PRIMARY]) {
    DLOGI("Primary display is not initialized");
    return -EINVAL;
  }
  auto panel_width = 0u;
  auto panel_height = 0u;

  hwc_display_[HWC_DISPLAY_PRIMARY]->GetPanelResolution(&panel_width, &panel_height);
  output_parcel->writeInt32(INT32(panel_width));
  output_parcel->writeInt32(INT32(panel_height));

  return android::NO_ERROR;
}

android::status_t HWCSession::GetVisibleDisplayRect(const android::Parcel *input_parcel,
                                                    android::Parcel *output_parcel) {
  int disp_idx = GetDisplayIndex(input_parcel->readInt32());
  if (disp_idx == -1) {
    DLOGE("Invalid display = %d", disp_idx);
    return android::BAD_VALUE;
  }

  SEQUENCE_WAIT_SCOPE_LOCK(locker_[disp_idx]);
  if (!hwc_display_[disp_idx]) {
    return android::NO_INIT;
  }

  hwc_rect_t visible_rect = {0, 0, 0, 0};
  int error = hwc_display_[disp_idx]->GetVisibleDisplayRect(&visible_rect);
  if (error < 0) {
    return error;
  }

  output_parcel->writeInt32(visible_rect.left);
  output_parcel->writeInt32(visible_rect.top);
  output_parcel->writeInt32(visible_rect.right);
  output_parcel->writeInt32(visible_rect.bottom);

  return android::NO_ERROR;
}

android::status_t HWCSession::SetStandByMode(const android::Parcel *input_parcel) {
  SCOPE_LOCK(locker_[HWC_DISPLAY_PRIMARY]);

  bool enable = (input_parcel->readInt32() > 0);
  bool is_twm = (input_parcel->readInt32() > 0);

  if (!hwc_display_[HWC_DISPLAY_PRIMARY]) {
    DLOGI("Primary display is not initialized");
    return -EINVAL;
  }

  DisplayError error = hwc_display_[HWC_DISPLAY_PRIMARY]->SetStandByMode(enable, is_twm);
  if (error != kErrorNone) {
    DLOGE("SetStandByMode failed. Error = %d", error);
    return -EINVAL;
  }

  return android::NO_ERROR;
}

android::status_t HWCSession::DelayFirstCommit() {
  if (!hwc_display_[HWC_DISPLAY_PRIMARY]) {
    DLOGI("Primary display is not initialized");
    return -EINVAL;
  }

  return hwc_display_[HWC_DISPLAY_PRIMARY]->DelayFirstCommit();
}

int HWCSession::CreatePrimaryDisplay() {
  int status = -EINVAL;
  HWDisplaysInfo hw_displays_info = {};

  if (null_display_mode_) {
    HWDisplayInfo hw_info = {};
    hw_info.display_type = kBuiltIn;
    hw_info.is_connected = 1;
    hw_info.is_primary = 1;
    hw_info.is_wb_ubwc_supported = 0;
    hw_info.display_id = 1;
    hw_displays_info[hw_info.display_id] = hw_info;
  } else {
    DisplayError error = core_intf_->GetDisplaysStatus(&hw_displays_info);
    if (error != kErrorNone) {
      DLOGE("Failed to get connected display list. Error = %d", error);
      return status;
    }
  }

  for (auto &iter : hw_displays_info) {
    auto &info = iter.second;
    if (!info.is_primary) {
      continue;
    }

    // todo (user): If primary display is not connected (e.g. hdmi as primary), a NULL display
    // need to be created. SF expects primary display hotplug during callback registration unlike
    // previous implementation where first hotplug could be notified anytime.
    if (!info.is_connected) {
      DLOGE("Primary display is not connected. Not supported at present.");
      break;
    }

    auto hwc_display = &hwc_display_[HWC_DISPLAY_PRIMARY];
    hwc2_display_t client_id = map_info_primary_.client_id;

    if (info.display_type == kBuiltIn) {
      status = HWCDisplayBuiltIn::Create(core_intf_, &buffer_allocator_, &callbacks_, this,
                                         qservice_, client_id, info.display_id, hwc_display);
    } else if (info.display_type == kPluggable) {
      status = HWCDisplayPluggable::Create(core_intf_, &buffer_allocator_, &callbacks_, this,
                                           qservice_, client_id, info.display_id, 0, 0, false,
                                           hwc_display);
    } else {
      DLOGE("Spurious primary display type = %d", info.display_type);
      break;
    }

    if (!status) {
      DLOGI("Created primary display type = %d, sdm id = %d, client id = %d", info.display_type,
             info.display_id, UINT32(client_id));
      {
         SCOPE_LOCK(hdr_locker_[client_id]);
         is_hdr_display_[UINT32(client_id)] = HasHDRSupport(*hwc_display);
      }

      map_info_primary_.disp_type = info.display_type;
      map_info_primary_.sdm_id = info.display_id;
      CreateDummyDisplay(HWC_DISPLAY_PRIMARY);
      color_mgr_ = HWCColorManager::CreateColorManager(&buffer_allocator_);
      if (!color_mgr_) {
        DLOGW("Failed to load HWCColorManager.");
      }
    } else {
      DLOGE("Primary display creation has failed! status = %d", status);
    }

    // Primary display is found, no need to parse more.
    break;
  }

  return status;
}

void HWCSession::CreateDummyDisplay(hwc2_display_t client_id) {
  if (!async_powermode_) {
    return;
  }

  hwc2_display_t dummy_disp_id = map_hwc_display_.find(client_id)->second;
  auto hwc_display_dummy = &hwc_display_[dummy_disp_id];
  HWCDisplayDummy::Create(core_intf_, &buffer_allocator_, &callbacks_, this, qservice_,
                    0, 0, hwc_display_dummy);
  if (!*hwc_display_dummy) {
    DLOGE("Dummy display creation failed for %d display\n", UINT32(client_id));
  }
}

int HWCSession::HandleBuiltInDisplays() {
  if (null_display_mode_) {
    DLOGW("Skipped BuiltIn display handling in null-display mode");
    return 0;
  }

  HWDisplaysInfo hw_displays_info = {};
  DisplayError error = core_intf_->GetDisplaysStatus(&hw_displays_info);
  if (error != kErrorNone) {
    DLOGE("Failed to get connected display list. Error = %d", error);
    return -EINVAL;
  }

  int status = 0;
  for (auto &iter : hw_displays_info) {
    auto &info = iter.second;

    // Do not recreate primary display.
    if (info.is_primary || info.display_type != kBuiltIn) {
      continue;
    }

    for (auto &map_info : map_info_builtin_) {
      hwc2_display_t client_id = map_info.client_id;

      {
        SCOPE_LOCK(locker_[client_id]);
        // Lock confined to this scope
        if (hwc_display_[client_id]) {
          continue;
        }

        DLOGI("Create builtin display, sdm id = %d, client id = %d", info.display_id,
              UINT32(client_id));
        status = HWCDisplayBuiltIn::Create(core_intf_, &buffer_allocator_, &callbacks_, this,
                                           qservice_, client_id, info.display_id,
                                           &hwc_display_[client_id]);
        if (status) {
          DLOGE("Builtin display creation failed.");
          break;
        }

        {
          SCOPE_LOCK(hdr_locker_[client_id]);
          is_hdr_display_[UINT32(client_id)] = HasHDRSupport(hwc_display_[client_id]);
        }

        DLOGI("Builtin display created: sdm id = %d, client id = %d", info.display_id,
              UINT32(client_id));
        map_info.disp_type = info.display_type;
        map_info.sdm_id = info.display_id;
        CreateDummyDisplay(client_id);
      }

      DLOGI("Hotplugging builtin display, sdm id = %d, client id = %d", info.display_id,
            UINT32(client_id));
      callbacks_.Hotplug(client_id, HWC2::Connection::Connected);
      break;
    }
  }

  return status;
}

int HWCSession::HandlePluggableDisplays(bool delay_hotplug) {
  SCOPE_LOCK(pluggable_handler_lock_);
  if (null_display_mode_) {
    DLOGW("Skipped pluggable display handling in null-display mode");
    return 0;
  }

  DLOGI("Handling hotplug...");
  HWDisplaysInfo hw_displays_info = {};
  DisplayError error = core_intf_->GetDisplaysStatus(&hw_displays_info);
  if (error != kErrorNone) {
    DLOGE("Failed to get connected display list. Error = %d", error);
    return -EINVAL;
  }

  int status = HandleDisconnectedDisplays(&hw_displays_info);
  if (status) {
    DLOGE("All displays could not be disconnected.");
    return status;
  }

  status = HandleConnectedDisplays(&hw_displays_info, delay_hotplug);
  if (status) {
    switch (status) {
      case -EAGAIN:
      case -ENODEV:
        // Errors like device removal or deferral for which we want to try another hotplug handling.
        pending_hotplug_event_ = kHotPlugEvent;
        status = 0;
        break;
      default:
        // Real errors we want to flag and stop hotplug handling.
        pending_hotplug_event_ = kHotPlugNone;
        DLOGE("All displays could not be connected. Error %d '%s'.", status, strerror(abs(status)));
    }
    DLOGI("Handling hotplug... %s", (kHotPlugNone == pending_hotplug_event_) ?
          "Stopped." : "Done. Hotplug events pending.");
    return status;
  }

  pending_hotplug_event_ = kHotPlugNone;

  DLOGI("Handling hotplug... Done.");
  return 0;
}

int HWCSession::HandleConnectedDisplays(HWDisplaysInfo *hw_displays_info, bool delay_hotplug) {
  int status = 0;
  std::vector<hwc2_display_t> pending_hotplugs = {};
  hwc2_display_t client_id = 0;

  for (auto &iter : *hw_displays_info) {
    auto &info = iter.second;

    // Do not recreate primary display or if display is not connected.
    if (info.is_primary || info.display_type != kPluggable || !info.is_connected) {
      continue;
    }

    // Check if we are already using the display.
    auto display_used = std::find_if(map_info_pluggable_.begin(), map_info_pluggable_.end(),
                                     [&](auto &p) {
                                       return (p.sdm_id == info.display_id);
                                     });
    if (display_used != map_info_pluggable_.end()) {
      // Display is already used in a slot.
      continue;
    }

    // Count active pluggable display slots and slots with no commits.
    bool first_commit_pending = false;
    std::for_each(map_info_pluggable_.begin(), map_info_pluggable_.end(),
                   [&](auto &p) {
                     SCOPE_LOCK(locker_[p.client_id]);
                     if (hwc_display_[p.client_id]) {
                       if (!hwc_display_[p.client_id]->IsFirstCommitDone()) {
                         DLOGI("Display commit pending on display %d-1", p.sdm_id);
                         first_commit_pending = true;
                       }
                     }
                   });

    if (!disable_hotplug_bwcheck_ && first_commit_pending) {
      // Hotplug bandwidth check is accomplished by creating and hotplugging a new display after
      // a display commit has happened on previous hotplugged displays. This allows the driver to
      // return updated modes for the new display based on available link bandwidth.
      DLOGI("Pending display commit on one of the displays. Deferring display creation.");
      status = -EAGAIN;
      if (callbacks_.IsClientConnected()) {
        // Trigger a display refresh since we depend on PresentDisplay() to handle pending hotplugs.
        hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
        if (active_builtin_disp_id >= HWCCallbacks::kNumDisplays) {
          active_builtin_disp_id = HWC_DISPLAY_PRIMARY;
        }
        callbacks_.Refresh(active_builtin_disp_id);
      }
      break;
    }

    // find an empty slot to create display.
    for (auto &map_info : map_info_pluggable_) {
      client_id = map_info.client_id;

      // Lock confined to this scope
      {
        SCOPE_LOCK(locker_[client_id]);
        auto &hwc_display = hwc_display_[client_id];
        if (hwc_display) {
          // Display slot is already used.
          continue;
        }

        DLOGI("Create pluggable display, sdm id = %d, client id = %d", info.display_id,
              UINT32(client_id));

        // Test pattern generation ?
        map_info.test_pattern = (hpd_bpp_ > 0) && (hpd_pattern_ > 0);
        int err = 0;
        if (!map_info.test_pattern) {
          err = HWCDisplayPluggable::Create(core_intf_, &buffer_allocator_,
                                            &callbacks_, this, qservice_, client_id,
                                            info.display_id, 0, 0, false, &hwc_display);
        } else {
          err = HWCDisplayPluggableTest::Create(core_intf_, &buffer_allocator_,
                                                &callbacks_, this, qservice_, client_id,
                                                info.display_id, UINT32(hpd_bpp_),
                                                UINT32(hpd_pattern_), &hwc_display);
        }

        if (err) {
          DLOGW("Pluggable display creation failed/aborted. Error %d '%s'.", err,
                strerror(abs(err)));
          status = err;
          // Attempt creating remaining pluggable displays.
          break;
        }

        {
          SCOPE_LOCK(hdr_locker_[client_id]);
          is_hdr_display_[UINT32(client_id)] = HasHDRSupport(hwc_display);
        }

        DLOGI("Created pluggable display successfully: sdm id = %d, client id = %d",
              info.display_id, UINT32(client_id));
        CreateDummyDisplay(client_id);
      }

      map_info.disp_type = info.display_type;
      map_info.sdm_id = info.display_id;

      pending_hotplugs.push_back((hwc2_display_t)client_id);

      // Display is created for this sdm id, move to next connected display.
      break;
    }
  }

  // No display was created.
  if (!pending_hotplugs.size()) {
    return status;
  }

  // Active builtin display needs revalidation
  hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
  if (active_builtin_disp_id < HWCCallbacks::kNumDisplays) {
    WaitForResources(delay_hotplug, active_builtin_disp_id, client_id);
  }

  for (auto client_id : pending_hotplugs) {
    DLOGI("Notify hotplug display connected: client id = %d", UINT32(client_id));
    callbacks_.Hotplug(client_id, HWC2::Connection::Connected);
  }

  return status;
}

bool HWCSession::HasHDRSupport(HWCDisplay *hwc_display) {
  // query number of hdr types
  uint32_t out_num_types = 0;
  float out_max_luminance = 0.0f;
  float out_max_average_luminance = 0.0f;
  float out_min_luminance = 0.0f;
  if (hwc_display->GetHdrCapabilities(&out_num_types, nullptr, &out_max_luminance,
                                      &out_max_average_luminance, &out_min_luminance)
                                      != HWC2::Error::None) {
    return false;
  }

  return (out_num_types > 0);
}

int HWCSession::HandleDisconnectedDisplays(HWDisplaysInfo *hw_displays_info) {
  // Destroy pluggable displays which were connected earlier but got disconnected now.
  for (auto &map_info : map_info_pluggable_) {
    bool disconnect = true;   // disconnect in case display id is not found in list.

    for (auto &iter : *hw_displays_info) {
      auto &info = iter.second;
      if (info.display_id != map_info.sdm_id) {
        continue;
      }
      if (info.is_connected) {
        disconnect = false;
      }
      break;
    }

    if (disconnect) {
      DestroyDisplay(&map_info);
    }
  }

  return 0;
}

void HWCSession::DestroyDisplay(DisplayMapInfo *map_info) {
  switch (map_info->disp_type) {
    case kPluggable:
      DestroyPluggableDisplay(map_info);
      break;
    default:
      DestroyNonPluggableDisplay(map_info);
      break;
    }
}

void HWCSession::DestroyPluggableDisplay(DisplayMapInfo *map_info) {
  hwc2_display_t client_id = map_info->client_id;

  DLOGI("Notify hotplug display disconnected: client id = %d", UINT32(client_id));
  callbacks_.Hotplug(client_id, HWC2::Connection::Disconnected);

  SCOPE_LOCK(system_locker_);
  {
    SEQUENCE_WAIT_SCOPE_LOCK(locker_[client_id]);
    auto &hwc_display = hwc_display_[client_id];
    if (!hwc_display) {
      return;
    }
    DLOGI("Destroy display %d-%d, client id = %d", map_info->sdm_id, map_info->disp_type,
         UINT32(client_id));
    {
      SCOPE_LOCK(hdr_locker_[client_id]);
      is_hdr_display_[UINT32(client_id)] = false;
    }

    if (!map_info->test_pattern) {
      HWCDisplayPluggable::Destroy(hwc_display);
    } else {
      HWCDisplayPluggableTest::Destroy(hwc_display);
    }

    if (async_powermode_) {
      hwc2_display_t dummy_disp_id = map_hwc_display_.find(client_id)->second;
      auto &hwc_display_dummy = hwc_display_[dummy_disp_id];
      display_ready_.reset(UINT32(dummy_disp_id));
      if (hwc_display_dummy) {
        HWCDisplayDummy::Destroy(hwc_display_dummy);
        hwc_display_dummy = nullptr;
      }
    }
    display_ready_.reset(UINT32(client_id));
    pending_power_mode_[client_id] = false;
    hwc_display = nullptr;
    map_info->Reset();
  }
}

void HWCSession::DestroyNonPluggableDisplay(DisplayMapInfo *map_info) {
  hwc2_display_t client_id = map_info->client_id;

  SCOPE_LOCK(locker_[client_id]);
  auto &hwc_display = hwc_display_[client_id];
  if (!hwc_display) {
    return;
  }
  DLOGI("Destroy display %d-%d, client id = %d", map_info->sdm_id, map_info->disp_type,
        UINT32(client_id));
  {
    SCOPE_LOCK(hdr_locker_[client_id]);
    is_hdr_display_[UINT32(client_id)] = false;
  }

  switch (map_info->disp_type) {
    case kBuiltIn:
      HWCDisplayBuiltIn::Destroy(hwc_display);
      break;
    default:
      virtual_display_factory_.Destroy(hwc_display);
      break;
    }

    if (async_powermode_ && map_info->disp_type == kBuiltIn) {
      hwc2_display_t dummy_disp_id = map_hwc_display_.find(client_id)->second;
      auto &hwc_display_dummy = hwc_display_[dummy_disp_id];
      display_ready_.reset(UINT32(dummy_disp_id));
      if (hwc_display_dummy) {
        HWCDisplayDummy::Destroy(hwc_display_dummy);
        hwc_display_dummy = nullptr;
      }
    }
    pending_power_mode_[client_id] = false;
    hwc_display = nullptr;
    display_ready_.reset(UINT32(client_id));
    map_info->Reset();
}

HWC2::Error HWCSession::ValidateDisplayInternal(hwc2_display_t display, uint32_t *out_num_types,
                                                uint32_t *out_num_requests) {
  HWCDisplay *hwc_display = hwc_display_[display];

  DTRACE_SCOPED();
  if (hwc_display->IsInternalValidateState()) {
    // Internal Validation has already been done on display, get the Output params.
    return hwc_display->GetValidateDisplayOutput(out_num_types, out_num_requests);
  }

  if (display == HWC_DISPLAY_PRIMARY) {
    // TODO(user): This can be moved to HWCDisplayPrimary
    if (need_invalidate_) {
      callbacks_.Refresh(display);
      need_invalidate_ = false;
    }
  }

  auto status = HWC2::Error::None;
  status = hwc_display->Validate(out_num_types, out_num_requests);
  SetCpuPerfHintLargeCompCycle();
  return status;
}

HWC2::Error HWCSession::PresentDisplayInternal(hwc2_display_t display) {
  HWCDisplay *hwc_display = hwc_display_[display];

  DTRACE_SCOPED();
  // If display is in Skip-Validate state and Validate cannot be skipped, do Internal
  // Validation to optimize for the frames which don't require the Client composition.
  if (hwc_display->IsSkipValidateState() && !hwc_display->CanSkipValidate()) {
    uint32_t out_num_types = 0, out_num_requests = 0;
    hwc_display->SetFastPathComposition(true);
    HWC2::Error error = ValidateDisplayInternal(display, &out_num_types, &out_num_requests);
    if ((error != HWC2::Error::None) || hwc_display->HWCClientNeedsValidate()) {
      hwc_display->SetValidationState(HWCDisplay::kInternalValidate);
      hwc_display->SetFastPathComposition(false);
      return HWC2::Error::NotValidated;
    }
  }
  return HWC2::Error::None;
}

void HWCSession::DisplayPowerReset() {
  // Acquire lock on all displays.
  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY;
    display < HWCCallbacks::kNumDisplays; display++) {
    locker_[display].Lock();
  }

  HWC2::Error status = HWC2::Error::None;
  HWC2::PowerMode last_power_mode[HWCCallbacks::kNumDisplays] = {};

  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY;
    display < HWCCallbacks::kNumDisplays; display++) {
    if (hwc_display_[display] != NULL) {
      last_power_mode[display] = hwc_display_[display]->GetCurrentPowerMode();
      DLOGI("Powering off display = %d", INT32(display));
      status = hwc_display_[display]->SetPowerMode(HWC2::PowerMode::Off,
                                                   true /* teardown */);
      if (status != HWC2::Error::None) {
        DLOGE("Power off for display = %d failed with error = %d", INT32(display), status);
      }
    }
  }
  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY;
    display < HWCCallbacks::kNumDisplays; display++) {
    if (hwc_display_[display] != NULL) {
      HWC2::PowerMode mode = last_power_mode[display];
      DLOGI("Setting display %d to mode = %d", INT32(display), mode);
      status = hwc_display_[display]->SetPowerMode(mode, false /* teardown */);
      if (status != HWC2::Error::None) {
        DLOGE("%d mode for display = %d failed with error = %d", mode, INT32(display), status);
      }
      ColorMode color_mode = hwc_display_[display]->GetCurrentColorMode();
      status = hwc_display_[display]->SetColorMode(color_mode);
      if (status != HWC2::Error::None) {
        DLOGE("SetColorMode failed for display = %d error = %d", INT32(display), status);
      }
    }
  }

  hwc2_display_t vsync_source = callbacks_.GetVsyncSource();
  status = hwc_display_[vsync_source]->SetVsyncEnabled(HWC2::Vsync::Enable);
  if (status != HWC2::Error::None) {
    DLOGE("Enabling vsync failed for disp: %" PRIu64 " with error = %d", vsync_source, status);
  }

  // Release lock on all displays.
  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY;
    display < HWCCallbacks::kNumDisplays; display++) {
    locker_[display].Unlock();
  }

  callbacks_.Refresh(vsync_source);
}

void HWCSession::HandleSecureSession() {
  std::bitset<kSecureMax> secure_sessions = 0;
  {
    // TODO(user): Revisit if supporting secure display on non-primary.
    hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
    if (active_builtin_disp_id >= HWCCallbacks::kNumDisplays) {
      return;
    }
    Locker::ScopeLock lock_pwr(power_state_[active_builtin_disp_id]);
    if (power_state_transition_[active_builtin_disp_id]) {
      // Route all interactions with client to dummy display.
      active_builtin_disp_id = map_hwc_display_.find(active_builtin_disp_id)->second;
    }
    Locker::ScopeLock lock_d(locker_[active_builtin_disp_id]);
    hwc_display_[active_builtin_disp_id]->GetActiveSecureSession(&secure_sessions);
  }

  if (secure_sessions.any()) {
    secure_session_active_ = true;
  } else if (!secure_session_active_) {
    // No secure session active. No secure session transition to handle. Skip remaining steps.
    return;
  }

  // If it is called during primary prepare/commit, we need to pause any ongoing commit on
  // external/virtual display.
  bool found_active_secure_display = false;
  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY;
       display < HWCCallbacks::kNumRealDisplays; display++) {
    Locker::ScopeLock lock_d(locker_[display]);
    HWCDisplay *hwc_display = hwc_display_[display];
    if (!hwc_display) {
      continue;
    }

    bool is_active_secure_display = false;
    // The first On/Doze/DozeSuspend built-in display is taken as the secure display.
    if (!found_active_secure_display &&
        hwc_display->GetDisplayClass() == DISPLAY_CLASS_BUILTIN &&
        hwc_display->GetCurrentPowerMode() != HWC2::PowerMode::Off) {
      is_active_secure_display = true;
      found_active_secure_display = true;
    }
    hwc_display->HandleSecureSession(secure_sessions, &pending_power_mode_[display],
                                     is_active_secure_display);
  }
}

void HWCSession::HandlePendingPowerMode(hwc2_display_t disp_id,
                                        const shared_ptr<Fence> &retire_fence) {
  if (!secure_session_active_) {
    // No secure session active. Skip remaining steps.
    return;
  }

  hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
  if (disp_id != active_builtin_disp_id) {
    return;
  }

  Locker::ScopeLock lock_d(locker_[active_builtin_disp_id]);
  bool pending_power_mode = false;
  std::bitset<kSecureMax> secure_sessions = 0;
  hwc_display_[active_builtin_disp_id]->GetActiveSecureSession(&secure_sessions);
  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY + 1;
    display < HWCCallbacks::kNumDisplays; display++) {
    if (display != active_builtin_disp_id) {
      Locker::ScopeLock lock_d(locker_[display]);
      if (pending_power_mode_[display]) {
        pending_power_mode = true;
        break;
      }
    }
  }

  if (!pending_power_mode) {
    if (!secure_sessions.any()) {
      secure_session_active_ = false;
    }
    return;
  }

  // retire fence is set only after successful primary commit, So check for retire fence to know
  // non secure commit went through to notify driver to change the CRTC mode to non secure.
  // Otherwise any commit to non-primary display would fail.
  if (retire_fence == nullptr) {
    return;
  }

  Fence::Wait(retire_fence);

  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY + 1;
    display < HWCCallbacks::kNumDisplays; display++) {
    if (display != active_builtin_disp_id) {
      Locker::ScopeLock lock_d(locker_[display]);
      if (pending_power_mode_[display] && hwc_display_[display]) {
        HWC2::Error error =
          hwc_display_[display]->SetPowerMode(hwc_display_[display]->GetPendingPowerMode(), false);
        if (HWC2::Error::None == error) {
          pending_power_mode_[display] = false;
          hwc_display_[display]->ClearPendingPowerMode();
          pending_refresh_.set(UINT32(HWC_DISPLAY_PRIMARY));
        } else {
          DLOGE("SetDisplayStatus error = %d (%s)", error, to_string(error).c_str());
        }
      }
    }
  }
  secure_session_active_ = false;
}

void HWCSession::HandlePendingHotplug(hwc2_display_t disp_id,
                                      const shared_ptr<Fence> &retire_fence) {
  hwc2_display_t active_builtin_disp_id = GetActiveBuiltinDisplay();
  if (disp_id != active_builtin_disp_id ||
      (kHotPlugNone == pending_hotplug_event_ && !destroy_virtual_disp_pending_)) {
    return;
  }

  std :: bitset < kSecureMax > secure_sessions = 0;
  if (active_builtin_disp_id < HWCCallbacks::kNumDisplays) {
    Locker::ScopeLock lock_d(locker_[active_builtin_disp_id]);
    hwc_display_[active_builtin_disp_id]->GetActiveSecureSession(&secure_sessions);
  }

  if (secure_sessions.any() || active_builtin_disp_id >= HWCCallbacks::kNumDisplays) {
    return;
  }

  if (destroy_virtual_disp_pending_ || kHotPlugEvent == pending_hotplug_event_) {
    Fence::Wait(retire_fence);

    // Destroy the pending virtual display if secure session not present.
    if (destroy_virtual_disp_pending_) {
      for (auto &map_info : map_info_virtual_) {
        DestroyDisplay(&map_info);
        destroy_virtual_disp_pending_ = false;
        virtual_id_ = HWCCallbacks::kNumDisplays;
      }
    }
    // Handle connect/disconnect hotplugs if secure session is not present.
    hwc2_display_t virtual_display_idx = (hwc2_display_t)GetDisplayIndex(qdutils::DISPLAY_VIRTUAL);
    if (!hwc_display_[virtual_display_idx] && kHotPlugEvent == pending_hotplug_event_) {
      // Handle deferred hotplug event.
      int32_t err = pluggable_handler_lock_.TryLock();
      if (!err) {
        // Do hotplug handling in a different thread to avoid blocking PresentDisplay.
        std::thread(&HWCSession::HandlePluggableDisplays, this, true).detach();
        pluggable_handler_lock_.Unlock();
      } else {
        // EBUSY means another thread is already handling hotplug. Skip deferred hotplug handling.
        if (EBUSY != err) {
          DLOGW("Failed to acquire pluggable display handler lock. Error %d '%s'.", err,
                strerror(abs(err)));
        }
      }
    }
  }
}

int32_t HWCSession::GetReadbackBufferAttributes(hwc2_display_t display, int32_t *format,
                                                int32_t *dataspace) {
  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (!format || !dataspace) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display != HWC_DISPLAY_PRIMARY) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  HWCDisplay *hwc_display = hwc_display_[display];
  if (hwc_display == nullptr) {
    return HWC2_ERROR_BAD_DISPLAY;
  } else if (!hwc_display->HasReadBackBufferSupport()) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  *format = HAL_PIXEL_FORMAT_RGB_888;
  *dataspace = GetDataspaceFromColorMode(hwc_display->GetCurrentColorMode());

  return HWC2_ERROR_NONE;
}

int32_t HWCSession::SetReadbackBuffer(hwc2_display_t display, const native_handle_t *buffer,
                                      const shared_ptr<Fence> &acquire_fence) {

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (!buffer) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display != HWC_DISPLAY_PRIMARY) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  int external_dpy_index = GetDisplayIndex(qdutils::DISPLAY_EXTERNAL);
  int virtual_dpy_index = GetDisplayIndex(qdutils::DISPLAY_VIRTUAL);
  if (((external_dpy_index != -1) && hwc_display_[external_dpy_index]) ||
      ((virtual_dpy_index != -1) && hwc_display_[virtual_dpy_index])) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  return CallDisplayFunction(display, &HWCDisplay::SetReadbackBuffer, buffer, acquire_fence,
                             false, kCWBClientComposer);
}

int32_t HWCSession::GetReadbackBufferFence(hwc2_display_t display,
                                           shared_ptr<Fence> *release_fence) {
  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (!release_fence) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display != HWC_DISPLAY_PRIMARY) {
    return HWC2_ERROR_UNSUPPORTED;
  }

  return CallDisplayFunction(display, &HWCDisplay::GetReadbackBufferFence, release_fence);
}

int32_t HWCSession::GetDisplayIdentificationData(hwc2_display_t display, uint8_t *outPort,
                                                 uint32_t *outDataSize, uint8_t *outData) {
  if (!outPort || !outDataSize) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  return CallDisplayFunction(display, &HWCDisplay::GetDisplayIdentificationData, outPort,
                             outDataSize, outData);
}

int32_t HWCSession::GetDisplayCapabilities(hwc2_display_t display,
                                           hidl_vec<HwcDisplayCapability> *capabilities) {
  if (!capabilities) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (!hwc_display_[display]) {
    DLOGE("Expected valid hwc_display");
    return HWC2_ERROR_BAD_PARAMETER;
  }

  bool isBuiltin = (hwc_display_[display]->GetDisplayClass() == DISPLAY_CLASS_BUILTIN);
  if (isBuiltin) {
    int32_t has_doze_support = 0;
    GetDozeSupport(display, &has_doze_support);

    // TODO(user): Handle SKIP_CLIENT_COLOR_TRANSFORM based on DSPP availability
    if (has_doze_support) {
      *capabilities = {HwcDisplayCapability::SKIP_CLIENT_COLOR_TRANSFORM,
                       HwcDisplayCapability::DOZE,
                       HwcDisplayCapability::BRIGHTNESS, HwcDisplayCapability::PROTECTED_CONTENTS};
    } else {
      *capabilities = {HwcDisplayCapability::SKIP_CLIENT_COLOR_TRANSFORM,
                       HwcDisplayCapability::BRIGHTNESS, HwcDisplayCapability::PROTECTED_CONTENTS};
    }
  }

  return HWC2_ERROR_NONE;
}

int32_t HWCSession::GetDisplayCapabilities2_3(hwc2_display_t display, uint32_t *outNumCapabilities,
                                              uint32_t *outCapabilities) {
  if (!outNumCapabilities) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (!hwc_display_[display]) {
    DLOGE("Expected valid hwc_display");
    return HWC2_ERROR_BAD_PARAMETER;
  }
  bool isBuiltin = (hwc_display_[display]->GetDisplayClass() == DISPLAY_CLASS_BUILTIN);
  if (!outCapabilities) {
    *outNumCapabilities = 0;
    if (isBuiltin) {
      *outNumCapabilities = 2;
    }
    return HWC2_ERROR_NONE;
  } else {
    if (isBuiltin) {
      uint32_t index = 0;
      int32_t has_doze_support = 0;
      GetDozeSupport(display, &has_doze_support);
      if (has_doze_support) {
        outCapabilities[index++] = HWC2_DISPLAY_CAPABILITY_DOZE;
      }
      outCapabilities[index++] = HWC2_DISPLAY_CAPABILITY_BRIGHTNESS;
      *outNumCapabilities = index;
    }
  }
  return HWC2_ERROR_NONE;
}
int32_t HWCSession::GetDisplayConnectionType(hwc2_display_t display,
                                             HwcDisplayConnectionType *type) {
  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (!type) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (!hwc_display_[display]) {
    DLOGE("Expected valid hwc_display");
    return HWC2_ERROR_BAD_DISPLAY;
  }
  *type = HwcDisplayConnectionType::EXTERNAL;
  if (hwc_display_[display]->GetDisplayClass() == DISPLAY_CLASS_BUILTIN) {
    *type = HwcDisplayConnectionType::INTERNAL;
  }

  return HWC2_ERROR_NONE;
}

int32_t HWCSession::GetClientTargetProperty(hwc2_display_t display,
                                            HwcClientTargetProperty *outClientTargetProperty) {
  if (!outClientTargetProperty) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  return CallDisplayFunction(display, &HWCDisplay::GetClientTargetProperty,
                             outClientTargetProperty);
}

int32_t HWCSession::GetDisplayBrightnessSupport(hwc2_display_t display, bool *outSupport) {
  if (!outSupport) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (!hwc_display_[display]) {
    DLOGE("Expected valid hwc_display");
    return HWC2_ERROR_BAD_PARAMETER;
  }
  *outSupport = (hwc_display_[display]->GetDisplayClass() == DISPLAY_CLASS_BUILTIN);
  return HWC2_ERROR_NONE;
}

int32_t HWCSession::SetDisplayBrightness(hwc2_display_t display, float brightness) {
  if (display >= HWCCallbacks::kNumDisplays) {
    return HWC2_ERROR_BAD_DISPLAY;
  }

  if (!hwc_display_[display]) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  return INT32(hwc_display_[display]->SetPanelBrightness(brightness));
}

android::status_t HWCSession::SetQSyncMode(const android::Parcel *input_parcel) {
  auto mode = input_parcel->readInt32();

  QSyncMode qsync_mode = kQSyncModeNone;
  switch (mode) {
    case qService::IQService::QSYNC_MODE_NONE:
      qsync_mode = kQSyncModeNone;
      break;
    case qService::IQService::QSYNC_MODE_CONTINUOUS:
      qsync_mode = kQSyncModeContinuous;
      break;
    case qService::IQService::QSYNC_MODE_ONESHOT:
      qsync_mode = kQsyncModeOneShot;
      break;
    default:
      DLOGE("Qsync mode not supported %d", mode);
      return -EINVAL;
  }
  return CallDisplayFunction(HWC_DISPLAY_PRIMARY, &HWCDisplay::SetQSyncMode, qsync_mode);
}

void HWCSession::UpdateThrottlingRate() {
  uint32_t new_min = 0;

  for (int i=0; i < HWCCallbacks::kNumDisplays; i++) {
    auto &display = hwc_display_[i];
    if (!display)
      continue;
    if (display->GetCurrentPowerMode() != HWC2::PowerMode::Off)
      new_min = (new_min == 0) ? display->GetMaxRefreshRate() :
        std::min(new_min, display->GetMaxRefreshRate());
  }

  SetNewThrottlingRate(new_min);
}

void HWCSession::SetNewThrottlingRate(const uint32_t new_rate) {
  if (new_rate !=0 && throttling_refresh_rate_ != new_rate) {
    HWCDisplay::SetThrottlingRefreshRate(new_rate);
    throttling_refresh_rate_ = new_rate;
  }
}

android::status_t HWCSession::SetIdlePC(const android::Parcel *input_parcel) {
  auto enable = input_parcel->readInt32();
  auto synchronous = input_parcel->readInt32();

  return static_cast<android::status_t>(ControlIdlePowerCollapse(enable, synchronous));
}

hwc2_display_t HWCSession::GetActiveBuiltinDisplay() {
  hwc2_display_t active_display = HWCCallbacks::kNumDisplays;
  // Get first active display among primary and built-in displays.
  std::vector<DisplayMapInfo> map_info = {map_info_primary_};
  std::copy(map_info_builtin_.begin(), map_info_builtin_.end(), std::back_inserter(map_info));

  for (auto &info : map_info) {
    hwc2_display_t target_display = info.client_id;
    SCOPE_LOCK(power_state_[target_display]);
    if (power_state_transition_[target_display]) {
      // Route all interactions with client to dummy display.
      target_display = map_hwc_display_.find(target_display)->second;
    }
    Locker::ScopeLock lock_d(locker_[target_display]);
    auto &hwc_display = hwc_display_[target_display];
    if (hwc_display && hwc_display->GetCurrentPowerMode() != HWC2::PowerMode::Off) {
      active_display = info.client_id;
      break;
    }
  }

  return active_display;
}

int32_t HWCSession::SetDisplayBrightnessScale(const android::Parcel *input_parcel) {
  auto display = input_parcel->readInt32();
  auto level = input_parcel->readInt32();
  if (level < 0 || level > kBrightnessScaleMax) {
    DLOGE("Invalid backlight scale level %d", level);
    return -EINVAL;
  }
  auto bl_scale = level * kSvBlScaleMax / kBrightnessScaleMax;
  auto error = CallDisplayFunction(display, &HWCDisplay::SetBLScale, (uint32_t)bl_scale);
  if (INT32(error) == HWC2_ERROR_NONE) {
    callbacks_.Refresh(display);
  }

  return INT32(error);
}

void HWCSession::NotifyClientStatus(bool connected) {
  for (uint32_t i = 0; i < HWCCallbacks::kNumDisplays; i++) {
    if (!hwc_display_[i]) {
      continue;
    }
    SCOPE_LOCK(locker_[i]);
    hwc_display_[i]->NotifyClientStatus(connected);
    hwc_display_[i]->SetVsyncEnabled(HWC2::Vsync::Disable);
  }
  callbacks_.UpdateVsyncSource(HWCCallbacks::kNumDisplays);
}

void HWCSession::WaitForResources(bool wait_for_resources, hwc2_display_t active_builtin_id,
                                  hwc2_display_t display_id) {
  std::vector<DisplayMapInfo> map_info = {map_info_primary_};
  std::copy(map_info_builtin_.begin(), map_info_builtin_.end(), std::back_inserter(map_info));

  for (auto &info : map_info) {
    hwc2_display_t target_display = info.client_id;
    {
      SCOPE_LOCK(power_state_[target_display]);
      if (power_state_transition_[target_display]) {
        // Route all interactions with client to dummy display.
        target_display = map_hwc_display_.find(target_display)->second;
      }
    }
    {
      SEQUENCE_WAIT_SCOPE_LOCK(locker_[target_display]);
      auto &hwc_display = hwc_display_[target_display];
      if (hwc_display && hwc_display->GetCurrentPowerMode() != HWC2::PowerMode::Off) {
        hwc_display->ResetValidation();
      }
    }
  }

  if (wait_for_resources) {
    bool res_wait = true;
    do {
      if (client_connected_) {
        Refresh(active_builtin_id);
      }
      {
        std::unique_lock<std::mutex> caller_lock(hotplug_mutex_);
        hotplug_cv_.wait(caller_lock);
      }
      res_wait = hwc_display_[display_id]->CheckResourceState();
    } while (res_wait);
  }
}

int32_t HWCSession::GetDisplayVsyncPeriod(hwc2_display_t disp, VsyncPeriodNanos *vsync_period) {
  if (vsync_period == nullptr) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  return CallDisplayFunction(disp, &HWCDisplay::GetDisplayVsyncPeriod, vsync_period);
}

int32_t HWCSession::SetActiveConfigWithConstraints(
    hwc2_display_t display, hwc2_config_t config,
    const VsyncPeriodChangeConstraints *vsync_period_change_constraints,
    VsyncPeriodChangeTimeline *out_timeline) {
  if ((vsync_period_change_constraints == nullptr) || (out_timeline == nullptr)) {
    return HWC2_ERROR_BAD_PARAMETER;
  }

  return CallDisplayFunction(display, &HWCDisplay::SetActiveConfigWithConstraints, config,
                             vsync_period_change_constraints, out_timeline);
}

void HWCSession::SetCpuPerfHintLargeCompCycle() {
  bool found_non_primary_active_display = false;

  // Check any non-primary display is active
  for (hwc2_display_t display = HWC_DISPLAY_PRIMARY + 1;
    display < HWCCallbacks::kNumDisplays; display++) {
    if (hwc_display_[display] == NULL) {
      continue;
    }
    if (hwc_display_[display]->GetCurrentPowerMode() != HWC2::PowerMode::Off) {
      found_non_primary_active_display = true;
      break;
    }
  }

  // send cpu hint for primary display
  if (!found_non_primary_active_display) {
    hwc_display_[HWC_DISPLAY_PRIMARY]->SetCpuPerfHintLargeCompCycle();
  }
}

}  // namespace sdm
