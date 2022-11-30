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

#ifndef __HWC_LAYERS_H__
#define __HWC_LAYERS_H__

/* This class translates HWC2 Layer functions to the SDM LayerStack
 */

#include <gralloc_priv.h>
#include <qdMetaData.h>
#include <core/layer_stack.h>
#include <core/layer_buffer.h>
#include <utils/utils.h>
#define HWC2_INCLUDE_STRINGIFICATION
#define HWC2_USE_CPP11
#include <hardware/hwcomposer2.h>
#undef HWC2_INCLUDE_STRINGIFICATION
#undef HWC2_USE_CPP11
#include <android/hardware/graphics/composer/2.3/IComposerClient.h>
#include <vendor/qti/hardware/display/composer/3.0/IQtiComposerClient.h>

#include <deque>
#include <map>
#include <set>

#include "core/buffer_allocator.h"
#include "hwc_buffer_allocator.h"

using PerFrameMetadataKey =
    android::hardware::graphics::composer::V2_3::IComposerClient::PerFrameMetadataKey;
using vendor::qti::hardware::display::composer::V3_0::IQtiComposerClient;
using android::hardware::graphics::common::V1_2::Dataspace;

namespace sdm {

DisplayError SetCSC(const private_handle_t *pvt_handle, ColorMetaData *color_metadata);
bool GetColorPrimary(const int32_t &dataspace, ColorPrimaries *color_primary);
bool GetTransfer(const int32_t &dataspace, GammaTransfer *gamma_transfer);
bool GetRange(const int32_t &dataspace, ColorRange *color_range);
bool GetSDMColorSpace(const int32_t &dataspace, ColorMetaData *color_metadata);
bool IsBT2020(const ColorPrimaries &color_primary);
int32_t TranslateFromLegacyDataspace(const int32_t &legacy_ds);
DisplayError ColorMetadataToDataspace(ColorMetaData color_metadata, Dataspace *dataspace);

enum GeometryChanges {
  kNone         = 0x000,
  kBlendMode    = 0x001,
  kDataspace    = 0x002,
  kDisplayFrame = 0x004,
  kPlaneAlpha   = 0x008,
  kSourceCrop   = 0x010,
  kTransform    = 0x020,
  kZOrder       = 0x040,
  kAdded        = 0x080,
  kRemoved      = 0x100,
  kBufferGeometry = 0x200,
};

enum LayerTypes {
  kLayerUnknown = 0,
  kLayerApp = 1,
  kLayerGame = 2,
  kLayerBrowser = 3,
};

class HWCLayer {
 public:
  explicit HWCLayer(hwc2_display_t display_id, HWCBufferAllocator *buf_allocator);
  ~HWCLayer();
  uint32_t GetZ() const { return z_; }
  hwc2_layer_t GetId() const { return id_; }
  LayerTypes GetType() const { return type_; }
  Layer *GetSDMLayer() { return layer_; }
  void ResetPerFrameData();

  HWC2::Error SetLayerBlendMode(HWC2::BlendMode mode);
  HWC2::Error SetLayerBuffer(buffer_handle_t buffer, shared_ptr<Fence> acquire_fence);
  HWC2::Error SetLayerColor(hwc_color_t color);
  HWC2::Error SetLayerCompositionType(HWC2::Composition type);
  HWC2::Error SetLayerDataspace(int32_t dataspace);
  HWC2::Error SetLayerDisplayFrame(hwc_rect_t frame);
  HWC2::Error SetCursorPosition(int32_t x, int32_t y);
  HWC2::Error SetLayerPlaneAlpha(float alpha);
  HWC2::Error SetLayerSourceCrop(hwc_frect_t crop);
  HWC2::Error SetLayerSurfaceDamage(hwc_region_t damage);
  HWC2::Error SetLayerTransform(HWC2::Transform transform);
  HWC2::Error SetLayerVisibleRegion(hwc_region_t visible);
  HWC2::Error SetLayerPerFrameMetadata(uint32_t num_elements, const PerFrameMetadataKey *keys,
                                       const float *metadata);
  HWC2::Error SetLayerPerFrameMetadataBlobs(uint32_t num_elements, const PerFrameMetadataKey *keys,
                                            const uint32_t *sizes, const uint8_t* metadata);
  HWC2::Error SetLayerZOrder(uint32_t z);
  HWC2::Error SetLayerType(IQtiComposerClient::LayerType type);
  HWC2::Error SetLayerColorTransform(const float *matrix);
  void SetComposition(const LayerComposition &sdm_composition);
  HWC2::Composition GetClientRequestedCompositionType() { return client_requested_; }
  void UpdateClientCompositionType(HWC2::Composition type) { client_requested_ = type; }
  HWC2::Composition GetDeviceSelectedCompositionType() { return device_selected_; }
  int32_t GetLayerDataspace() { return dataspace_; }
  uint32_t GetGeometryChanges() { return geometry_changes_; }
  void ResetGeometryChanges() { geometry_changes_ = GeometryChanges::kNone; }
  void PushBackReleaseFence(const shared_ptr<Fence> &fence);
  void PopBackReleaseFence(shared_ptr<Fence> *fence);
  void PopFrontReleaseFence(shared_ptr<Fence> *fence);
  void ResetValidation() { layer_->update_mask.reset(); }
  bool NeedsValidation() { return (geometry_changes_ || layer_->update_mask.any()); }
  bool IsSingleBuffered() { return single_buffer_; }
  bool IsScalingPresent();
  bool IsRotationPresent();
  bool IsDataSpaceSupported();
  bool IsProtected() { return secure_; }
  static LayerBufferFormat GetSDMFormat(const int32_t &source, const int flags);
  bool IsSurfaceUpdated() { return surface_updated_; }
  void SetPartialUpdate(bool enabled) { partial_update_enabled_ = enabled; }
  bool IsNonIntegralSourceCrop() { return non_integral_source_crop_; }
  bool HasMetaDataRefreshRate() { return has_metadata_refresh_rate_; }
  bool IsColorTransformSet() { return color_transform_matrix_set_; }
  void SetLayerAsMask();
  bool BufferLatched() { return buffer_flipped_; }
  void ResetBufferFlip() { buffer_flipped_ = false; }
#ifdef FOD_ZPOS
  bool IsFodPressed() { return fod_pressed_; }
#endif

 private:
  Layer *layer_ = nullptr;
  LayerTypes type_ = kLayerUnknown;
  uint32_t z_ = 0;
  const hwc2_layer_t id_;
  const hwc2_display_t display_id_;
  static std::atomic<hwc2_layer_t> next_id_;
  std::deque<shared_ptr<Fence>> release_fences_;
  HWCBufferAllocator *buffer_allocator_ = NULL;
  int32_t dataspace_ =  HAL_DATASPACE_UNKNOWN;
  LayerTransform layer_transform_ = {};
  LayerRect dst_rect_ = {};
  bool single_buffer_ = false;
  int buffer_fd_ = -1;
  bool dataspace_supported_ = false;
  bool partial_update_enabled_ = false;
  bool surface_updated_ = true;
  bool non_integral_source_crop_ = false;
  bool has_metadata_refresh_rate_ = false;
  bool color_transform_matrix_set_ = false;
  bool buffer_flipped_ = false;
  bool secure_ = false;
#ifdef FOD_ZPOS
  bool fod_pressed_ = false;
#endif

  // Composition requested by client(SF)
  HWC2::Composition client_requested_ = HWC2::Composition::Device;
  // Composition selected by SDM
  HWC2::Composition device_selected_ = HWC2::Composition::Device;
  uint32_t geometry_changes_ = GeometryChanges::kNone;

  void SetRect(const hwc_rect_t &source, LayerRect *target);
  void SetRect(const hwc_frect_t &source, LayerRect *target);
  uint32_t GetUint32Color(const hwc_color_t &source);
  void GetUBWCStatsFromMetaData(UBWCStats *cr_stats, UbwcCrStatsVector *cr_vec);
  DisplayError SetMetaData(const private_handle_t *pvt_handle, Layer *layer);
  uint32_t RoundToStandardFPS(float fps);
  void ValidateAndSetCSC(const private_handle_t *handle);
  void SetDirtyRegions(hwc_region_t surface_damage);
};

struct SortLayersByZ {
  bool operator()(const HWCLayer *lhs, const HWCLayer *rhs) const {
    return lhs->GetZ() < rhs->GetZ();
  }
};

}  // namespace sdm
#endif  // __HWC_LAYERS_H__
