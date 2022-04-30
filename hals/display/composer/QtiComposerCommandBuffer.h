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

#ifndef __QTICOMPOSERCOMMANDBUFFER_H__
#define __QTICOMPOSERCOMMANDBUFFER_H__

#include <log/log.h>
#include <sync/sync.h>
#include <fmq/MessageQueue.h>
#include <hidl/MQDescriptor.h>
#include <utils/fence.h>

#include <limits>
#include <algorithm>
#include <vector>
#include <string>

namespace vendor {
namespace qti {
namespace hardware {
namespace display {
namespace composer {
namespace V3_0 {

using ::android::hardware::graphics::common::V1_0::ColorTransform;
using ::android::hardware::graphics::common::V1_0::Dataspace;
using ::android::hardware::graphics::common::V1_0::Transform;
using ::android::hardware::graphics::composer::V2_1::Error;
using ::android::hardware::graphics::composer::V2_1::Display;
using ::android::hardware::graphics::composer::V2_1::Layer;
using ::android::hardware::MessageQueue;
using ::android::hardware::MQDescriptorSync;
using ::android::hardware::hidl_vec;
using ::android::hardware::hidl_handle;

using CommandQueueType = MessageQueue<uint32_t, ::android::hardware::kSynchronizedReadWrite>;

using std::shared_ptr;
using std::string;
using sdm::Fence;

// This class helps build a command queue.  Note that all sizes/lengths are in units of uint32_t's.
class CommandWriter {
 public:
  explicit CommandWriter(uint32_t initialMaxSize) : mDataMaxSize(initialMaxSize) {
    mData = std::make_unique<uint32_t[]>(mDataMaxSize);
    reset();
  }

  ~CommandWriter() { reset(); }

  void reset() {
    mDataWritten = 0;
    mCommandEnd = 0;

    // handles in mDataHandles are owned by the caller
    mDataHandles.clear();

    // handles in mTemporaryHandles are owned by the writer
    for (auto handle : mTemporaryHandles) {
      native_handle_close(handle);
      native_handle_delete(handle);
    }
    mTemporaryHandles.clear();
  }

  IQtiComposerClient::Command getCommand(uint32_t offset) {
    uint32_t val = (offset < mDataWritten) ? mData[offset] : 0;
    return static_cast<IQtiComposerClient::Command>(val &
      static_cast<uint32_t>(IQtiComposerClient::Command::OPCODE_MASK));
  }

  bool writeQueue(bool& queueChanged, uint32_t& commandLength,
                  hidl_vec<hidl_handle>& commandHandles) {
    if (mDataWritten == 0) {
      queueChanged = false;
      commandLength = 0;
      commandHandles.setToExternal(nullptr, 0);
      return true;
    }

    // After data are written to the queue, it may not be read by the
    // remote reader when
    //
    //  - the writer does not send them (because of other errors)
    //  - the hwbinder transaction fails
    //  - the reader does not read them (because of other errors)
    //
    // Discard the stale data here.
    size_t staleDataSize = mQueue ? mQueue->availableToRead() : 0;
    if (staleDataSize > 0) {
      ALOGW("discarding stale data from message queue");
      CommandQueueType::MemTransaction tx;
      if (mQueue->beginRead(staleDataSize, &tx)) {
        mQueue->commitRead(staleDataSize);
      }
    }
    // write data to queue, optionally resizing it
    if (mQueue && (mDataMaxSize <= mQueue->getQuantumCount())) {
      if (!mQueue->write(mData.get(), mDataWritten)) {
        ALOGE("failed to write commands to message queue");
        return false;
      }

      queueChanged = false;
    } else {
      auto newQueue = std::make_unique<CommandQueueType>(mDataMaxSize);
      if (!newQueue->isValid() || !newQueue->write(mData.get(), mDataWritten)) {
        ALOGE("failed to prepare a new message queue ");
        return false;
      }

      mQueue = std::move(newQueue);
      queueChanged = true;
    }

    commandLength = mDataWritten;
    commandHandles.setToExternal(const_cast<hidl_handle*>(mDataHandles.data()),
                                 mDataHandles.size());

    return true;
  }

  const MQDescriptorSync<uint32_t>* getMQDescriptor() const {
    return (mQueue) ? mQueue->getDesc() : nullptr;
  }

  // Commands from ::android::hardware::graphics::composer::V2_1::IComposerClient follow.
  static constexpr uint16_t kSelectDisplayLength = 2;
  void selectDisplay(Display display) {
    beginCommand(IQtiComposerClient::Command::SELECT_DISPLAY, kSelectDisplayLength);
    write64(display);
    endCommand();
  }

  static constexpr uint16_t kSelectLayerLength = 2;
  void selectLayer(Layer layer) {
    beginCommand(IQtiComposerClient::Command::SELECT_LAYER, kSelectLayerLength);
    write64(layer);
    endCommand();
  }

  static constexpr uint16_t kSetErrorLength = 2;
  void setError(uint32_t location, Error error) {
    beginCommand(IQtiComposerClient::Command::SET_ERROR, kSetErrorLength);
    write(location);
    writeSigned(static_cast<int32_t>(error));
    endCommand();
  }

  static constexpr uint32_t kPresentOrValidateDisplayResultLength = 1;
  void setPresentOrValidateResult(uint32_t state) {
    beginCommand(IQtiComposerClient::Command::SET_PRESENT_OR_VALIDATE_DISPLAY_RESULT,
                 kPresentOrValidateDisplayResultLength);
    write(state);
    endCommand();
  }

  void setChangedCompositionTypes(const std::vector<Layer>& layers,
                                  const std::vector<IQtiComposerClient::Composition>& types) {
    size_t totalLayers = std::min(layers.size(), types.size());
    size_t currentLayer = 0;

    while (currentLayer < totalLayers) {
      size_t count = std::min(totalLayers - currentLayer, static_cast<size_t>(kMaxLength) / 3);

      beginCommand(IQtiComposerClient::Command::SET_CHANGED_COMPOSITION_TYPES, count * 3);
      for (size_t i = 0; i < count; i++) {
        write64(layers[currentLayer + i]);
        writeSigned(static_cast<int32_t>(types[currentLayer + i]));
      }
      endCommand();

      currentLayer += count;
    }
  }

  void setDisplayRequests(uint32_t displayRequestMask, const std::vector<Layer>& layers,
                          const std::vector<uint32_t>& layerRequestMasks) {
    size_t totalLayers = std::min(layers.size(), layerRequestMasks.size());
    size_t currentLayer = 0;

    while (currentLayer < totalLayers) {
      size_t count = std::min(totalLayers - currentLayer, static_cast<size_t>(kMaxLength - 1) / 3);

      beginCommand(IQtiComposerClient::Command::SET_DISPLAY_REQUESTS, 1 + count * 3);
      write(displayRequestMask);
      for (size_t i = 0; i < count; i++) {
        write64(layers[currentLayer + i]);
        write(static_cast<int32_t>(layerRequestMasks[currentLayer + i]));
      }
      endCommand();

      currentLayer += count;
    }
  }

  static constexpr uint16_t kSetPresentFenceLength = 1;
  void setPresentFence(const shared_ptr<Fence> &presentFence) {
    beginCommand(IQtiComposerClient::Command::SET_PRESENT_FENCE, kSetPresentFenceLength);
    writeFence(presentFence);
    endCommand();
  }

  void setReleaseFences(const std::vector<Layer>& layers,
                        const std::vector<shared_ptr<Fence>>& releaseFences) {
    size_t totalLayers = std::min(layers.size(), releaseFences.size());
    size_t currentLayer = 0;

    while (currentLayer < totalLayers) {
      size_t count = std::min(totalLayers - currentLayer, static_cast<size_t>(kMaxLength) / 3);

      beginCommand(IQtiComposerClient::Command::SET_RELEASE_FENCES, count * 3);
      for (size_t i = 0; i < count; i++) {
        write64(layers[currentLayer + i]);
        writeFence(releaseFences[currentLayer + i]);
      }
      endCommand();

      currentLayer += count;
    }
  }

  static constexpr uint16_t kSetColorTransformLength = 17;
  void setColorTransform(const float* matrix, ColorTransform hint) {
    beginCommand(IQtiComposerClient::Command::SET_COLOR_TRANSFORM, kSetColorTransformLength);
    for (int i = 0; i < 16; i++) {
      writeFloat(matrix[i]);
    }
    writeSigned(static_cast<int32_t>(hint));
    endCommand();
  }

  void setClientTarget(uint32_t slot, const native_handle_t* target,
                       const shared_ptr<Fence>& acquireFence, Dataspace dataspace,
                       const std::vector<IQtiComposerClient::Rect>& damage) {
    bool doWrite = (damage.size() <= (kMaxLength - 4) / 4);
    size_t length = 4 + ((doWrite) ? damage.size() * 4 : 0);

    beginCommand(IQtiComposerClient::Command::SET_CLIENT_TARGET, length);
    write(slot);
    writeHandle(target, true);
    writeFence(acquireFence);
    writeSigned(static_cast<int32_t>(dataspace));
    // When there are too many rectangles in the damage region and doWrite
    // is false, we write no rectangle at all which means the entire
    // client target is damaged.
    if (doWrite) {
      writeRegion(damage);
    }
    endCommand();
  }

  static constexpr uint16_t kSetOutputBufferLength = 3;
  void setOutputBuffer(uint32_t slot, const native_handle_t* buffer,
                       const shared_ptr<Fence>& releaseFence) {
    beginCommand(IQtiComposerClient::Command::SET_OUTPUT_BUFFER, kSetOutputBufferLength);
    write(slot);
    writeHandle(buffer, true);
    writeFence(releaseFence);
    endCommand();
  }

  static constexpr uint16_t kValidateDisplayLength = 0;
  void validateDisplay() {
    beginCommand(IQtiComposerClient::Command::VALIDATE_DISPLAY, kValidateDisplayLength);
    endCommand();
  }

  static constexpr uint16_t kPresentOrValidateDisplayLength = 0;
  void presentOrvalidateDisplay() {
    beginCommand(IQtiComposerClient::Command::PRESENT_OR_VALIDATE_DISPLAY,
                 kPresentOrValidateDisplayLength);
    endCommand();
  }

  static constexpr uint16_t kAcceptDisplayChangesLength = 0;
  void acceptDisplayChanges() {
    beginCommand(IQtiComposerClient::Command::ACCEPT_DISPLAY_CHANGES, kAcceptDisplayChangesLength);
    endCommand();
  }

  static constexpr uint16_t kPresentDisplayLength = 0;
  void presentDisplay() {
    beginCommand(IQtiComposerClient::Command::PRESENT_DISPLAY, kPresentDisplayLength);
    endCommand();
  }

  static constexpr uint16_t kSetLayerCursorPositionLength = 2;
  void setLayerCursorPosition(int32_t x, int32_t y) {
    beginCommand(IQtiComposerClient::Command::SET_LAYER_CURSOR_POSITION,
                 kSetLayerCursorPositionLength);
    writeSigned(x);
    writeSigned(y);
    endCommand();
  }

  static constexpr uint16_t kSetLayerBufferLength = 3;
  void setLayerBuffer(uint32_t slot, const native_handle_t* buffer,
                      const shared_ptr<Fence>& acquireFence) {
    beginCommand(IQtiComposerClient::Command::SET_LAYER_BUFFER, kSetLayerBufferLength);
    write(slot);
    writeHandle(buffer, true);
    writeFence(acquireFence);
    endCommand();
  }

  void setLayerSurfaceDamage(const std::vector<IQtiComposerClient::Rect>& damage) {
    bool doWrite = (damage.size() <= kMaxLength / 4);
    size_t length = (doWrite) ? damage.size() * 4 : 0;

    beginCommand(IQtiComposerClient::Command::SET_LAYER_SURFACE_DAMAGE, length);
    // When there are too many rectangles in the damage region and doWrite
    // is false, we write no rectangle at all which means the entire
    // layer is damaged.
    if (doWrite) {
      writeRegion(damage);
    }
    endCommand();
  }

  static constexpr uint16_t kSetLayerBlendModeLength = 1;
  void setLayerBlendMode(IQtiComposerClient::BlendMode mode) {
    beginCommand(IQtiComposerClient::Command::SET_LAYER_BLEND_MODE, kSetLayerBlendModeLength);
    writeSigned(static_cast<int32_t>(mode));
    endCommand();
  }

  static constexpr uint16_t kSetLayerColorLength = 1;
  void setLayerColor(IQtiComposerClient::Color color) {
    beginCommand(IQtiComposerClient::Command::SET_LAYER_COLOR, kSetLayerColorLength);
    writeColor(color);
    endCommand();
  }

  static constexpr uint16_t kSetLayerCompositionTypeLength = 1;
  void setLayerCompositionType(IQtiComposerClient::Composition type) {
    beginCommand(IQtiComposerClient::Command::SET_LAYER_COMPOSITION_TYPE,
                 kSetLayerCompositionTypeLength);
    writeSigned(static_cast<int32_t>(type));
    endCommand();
  }

  static constexpr uint16_t kSetLayerDataspaceLength = 1;
  void setLayerDataspace(Dataspace dataspace) {
    beginCommand(IQtiComposerClient::Command::SET_LAYER_DATASPACE, kSetLayerDataspaceLength);
    writeSigned(static_cast<int32_t>(dataspace));
    endCommand();
  }

  static constexpr uint16_t kSetLayerDisplayFrameLength = 4;
  void setLayerDisplayFrame(const IQtiComposerClient::Rect& frame) {
    beginCommand(IQtiComposerClient::Command::SET_LAYER_DISPLAY_FRAME, kSetLayerDisplayFrameLength);
    writeRect(frame);
    endCommand();
  }

  static constexpr uint16_t kSetLayerPlaneAlphaLength = 1;
  void setLayerPlaneAlpha(float alpha) {
    beginCommand(IQtiComposerClient::Command::SET_LAYER_PLANE_ALPHA, kSetLayerPlaneAlphaLength);
    writeFloat(alpha);
    endCommand();
  }

  static constexpr uint16_t kSetLayerSidebandStreamLength = 1;
  void setLayerSidebandStream(const native_handle_t* stream) {
    beginCommand(IQtiComposerClient::Command::SET_LAYER_SIDEBAND_STREAM,
                 kSetLayerSidebandStreamLength);
    writeHandle(stream);
    endCommand();
  }

  static constexpr uint16_t kSetLayerSourceCropLength = 4;
  void setLayerSourceCrop(const IQtiComposerClient::FRect& crop) {
    beginCommand(IQtiComposerClient::Command::SET_LAYER_SOURCE_CROP, kSetLayerSourceCropLength);
    writeFRect(crop);
    endCommand();
  }

  static constexpr uint16_t kSetLayerTransformLength = 1;
  void setLayerTransform(Transform transform) {
    beginCommand(IQtiComposerClient::Command::SET_LAYER_TRANSFORM, kSetLayerTransformLength);
    writeSigned(static_cast<int32_t>(transform));
    endCommand();
  }

  void setLayerVisibleRegion(const std::vector<IQtiComposerClient::Rect>& visible) {
    bool doWrite = (visible.size() <= kMaxLength / 4);
    size_t length = (doWrite) ? visible.size() * 4 : 0;

    beginCommand(IQtiComposerClient::Command::SET_LAYER_VISIBLE_REGION, length);
    // When there are too many rectangles in the visible region and
    // doWrite is false, we write no rectangle at all which means the
    // entire layer is visible.
    if (doWrite) {
      writeRegion(visible);
    }
    endCommand();
  }

  static constexpr uint16_t kSetLayerZOrderLength = 1;
  void setLayerZOrder(uint32_t z) {
    beginCommand(IQtiComposerClient::Command::SET_LAYER_Z_ORDER, kSetLayerZOrderLength);
    write(z);
    endCommand();
  }

  static constexpr uint16_t kSetLayerTypeLength = 1;
  void setLayerType(uint32_t z) {
    beginCommand(IQtiComposerClient::Command::SET_LAYER_TYPE, kSetLayerTypeLength);
    write(z);
    endCommand();
  }

  // Commands from ::android::hardware::graphics::composer::V2_2::IComposerClient follow.
  static constexpr uint16_t kSetLayerFloatColorLength = 4;
  void setLayerFloatColor(IQtiComposerClient::FloatColor color) {
    beginCommand(IQtiComposerClient::Command::SET_LAYER_FLOAT_COLOR, kSetLayerFloatColorLength);
    writeFloatColor(color);
    endCommand();
  }

  void setLayerPerFrameMetadata(const hidl_vec<IQtiComposerClient::PerFrameMetadata>& metadataVec) {
    beginCommand(IQtiComposerClient::Command::SET_LAYER_PER_FRAME_METADATA,
                 metadataVec.size() * 2);
    for (const auto& metadata : metadataVec) {
      writeSigned(static_cast<int32_t>(metadata.key));
      writeFloat(metadata.value);
    }
    endCommand();
  }

  // Commands from ::android::hardware::graphics::composer::V2_3::IComposerClient follow.
  static constexpr uint16_t kSetLayerColorTransformLength = 16;
  void setLayerColorTransform(const float* matrix) {
    beginCommand(IQtiComposerClient::Command::SET_LAYER_COLOR_TRANSFORM,
                 kSetLayerColorTransformLength);
    for (int i = 0; i < 16; i++) {
      writeFloat(matrix[i]);
    }
    endCommand();
  }

  void setLayerPerFrameMetadataBlobs(
    const hidl_vec<IQtiComposerClient::PerFrameMetadataBlob>& metadata) {
    size_t commandLength = 0;

    if (metadata.size() > std::numeric_limits<uint32_t>::max()) {
      LOG_FATAL("too many metadata blobs - dynamic metadata size is too large");
      return;
    }

    // number of blobs
    commandLength += metadata.size();

    for (auto metadataBlob : metadata) {
      commandLength += sizeof(int32_t);  // key of metadata blob
      commandLength += 1;                // size information of metadata blob

      // metadata content size
      size_t metadataSize = metadataBlob.blob.size() / sizeof(uint32_t);
      commandLength += metadataSize;
      commandLength += (metadataBlob.blob.size() - (metadataSize * sizeof(uint32_t)) > 0) ? 1 : 0;
    }

    if (commandLength > std::numeric_limits<uint16_t>::max()) {
      LOG_FATAL("dynamic metadata size is too large");
      return;
    }

    // Blobs are written as:
    // {numElements, key1, size1, blob1, key2, size2, blob2, key3, size3...}
    uint16_t length = static_cast<uint16_t>(commandLength);
    beginCommand(IQtiComposerClient::Command::SET_LAYER_PER_FRAME_METADATA_BLOBS, length);
    write(static_cast<uint32_t>(metadata.size()));
    for (auto metadataBlob : metadata) {
      writeSigned(static_cast<int32_t>(metadataBlob.key));
      write(static_cast<uint32_t>(metadataBlob.blob.size()));
      writeBlob(static_cast<uint32_t>(metadataBlob.blob.size()), metadataBlob.blob.data());
    }
    endCommand();
  }

  static constexpr uint16_t kSetDisplayElapseTime = 2;
  void setDisplayElapseTime(uint64_t time) {
    beginCommand(IQtiComposerClient::Command::SET_DISPLAY_ELAPSE_TIME, kSetDisplayElapseTime);
    write64(time);
    endCommand();
  }

  // Commands from ::android::hardware::graphics::composer::V2_4::IComposerClient follow.
  static constexpr uint16_t kSetClientTargetPropertyLength = 2;
  void setClientTargetProperty(
        const IQtiComposerClient::ClientTargetProperty& clientTargetProperty) {
    beginCommand(IQtiComposerClient::Command::SET_CLIENT_TARGET_PROPERTY,
                 kSetClientTargetPropertyLength);
    writeSigned(static_cast<int32_t>(clientTargetProperty.pixelFormat));
    writeSigned(static_cast<int32_t>(clientTargetProperty.dataspace));
    endCommand();
  }

 protected:
  // Commands from ::android::hardware::graphics::composer::V2_1::IComposerClient follow.
  void beginCommand(IQtiComposerClient::Command command, uint16_t length) {
    if (mCommandEnd) {
      LOG_FATAL("endCommand was not called before command 0x%x", command);
    }

    growData(1 + length);
    write(static_cast<uint32_t>(command) | length);

    mCommandEnd = mDataWritten + length;
  }

  void endCommand() {
    if (!mCommandEnd) {
      LOG_FATAL("beginCommand was not called");
    } else if (mDataWritten > mCommandEnd) {
      LOG_FATAL("too much data written");
      mDataWritten = mCommandEnd;
    } else if (mDataWritten < mCommandEnd) {
      LOG_FATAL("too little data written");
      while (mDataWritten < mCommandEnd) {
        write(0);
      }
    }

    mCommandEnd = 0;
  }

  void write(uint32_t val) { mData[mDataWritten++] = val; }

  void writeSigned(int32_t val) { memcpy(&mData[mDataWritten++], &val, sizeof(val)); }

  void writeFloat(float val) { memcpy(&mData[mDataWritten++], &val, sizeof(val)); }

  void write64(uint64_t val) {
    uint32_t lo = static_cast<uint32_t>(val & 0xffffffff);
    uint32_t hi = static_cast<uint32_t>(val >> 32);
    write(lo);
    write(hi);
  }

  void writeRect(const IQtiComposerClient::Rect& rect) {
    writeSigned(rect.left);
    writeSigned(rect.top);
    writeSigned(rect.right);
    writeSigned(rect.bottom);
  }

  void writeRegion(const std::vector<IQtiComposerClient::Rect>& region) {
    for (const auto& rect : region) {
      writeRect(rect);
    }
  }

  void writeFRect(const IQtiComposerClient::FRect& rect) {
    writeFloat(rect.left);
    writeFloat(rect.top);
    writeFloat(rect.right);
    writeFloat(rect.bottom);
  }

  void writeColor(const IQtiComposerClient::Color& color) {
    write((color.r <<  0) | (color.g <<  8) | (color.b << 16) | (color.a << 24));
  }

  // ownership of handle is not transferred
  void writeHandle(const native_handle_t* handle, bool useCache) {
    if (!handle) {
      writeSigned(static_cast<int32_t>((useCache) ?
                IQtiComposerClient::HandleIndex::CACHED : IQtiComposerClient::HandleIndex::EMPTY));
      return;
    }

    mDataHandles.push_back(handle);
    writeSigned(mDataHandles.size() - 1);
  }

  void writeHandle(const native_handle_t* handle) {
    writeHandle(handle, false);
  }

  // Handle would own fence hereafter. Hence provide a dupped fd.
  void writeFence(const shared_ptr<Fence>& fence) {
    native_handle_t* handle = nullptr;
    if (fence) {
      handle = getTemporaryHandle(1, 0);
      if (handle) {
        handle->data[0] = Fence::Dup(fence);
      } else {
        ALOGW("failed to get temporary handle for fence %s", Fence::GetStr(fence).c_str());
        Fence::Wait(fence);
      }
    }

    writeHandle(handle);
  }

  native_handle_t* getTemporaryHandle(int numFds, int numInts) {
    native_handle_t* handle = native_handle_create(numFds, numInts);
    if (handle) {
      mTemporaryHandles.push_back(handle);
    }
    return handle;
  }

  static constexpr uint16_t kMaxLength = std::numeric_limits<uint16_t>::max();

  // Commands from ::android::hardware::graphics::composer::V2_2::IComposerClient follow.
  void writeFloatColor(const IQtiComposerClient::FloatColor& color) {
    writeFloat(color.r);
    writeFloat(color.g);
    writeFloat(color.b);
    writeFloat(color.a);
  }

  // Commands from ::android::hardware::graphics::composer::V2_3::IComposerClient follow.
  void writeBlob(uint32_t length, const unsigned char* blob) {
    memcpy(&mData[mDataWritten], blob, length);
    uint32_t numElements = length / 4;
    mDataWritten += numElements;
    mDataWritten += (length - (numElements * 4) > 0) ? 1 : 0;
  }

 private:
  void growData(uint32_t grow) {
    uint32_t newWritten = mDataWritten + grow;
    if (newWritten < mDataWritten) {
      LOG_ALWAYS_FATAL("buffer overflowed; data written %" PRIu32
                       ", growing by %" PRIu32, mDataWritten, grow);
    }

    if (newWritten <= mDataMaxSize) {
      return;
    }

    uint32_t newMaxSize = mDataMaxSize << 1;
    if (newMaxSize < newWritten) {
      newMaxSize = newWritten;
    }

    auto newData = std::make_unique<uint32_t[]>(newMaxSize);
    std::copy_n(mData.get(), mDataWritten, newData.get());
    mDataMaxSize = newMaxSize;
    mData = std::move(newData);
  }

  uint32_t mDataMaxSize;
  std::unique_ptr<uint32_t[]> mData;

  uint32_t mDataWritten;
  // end offset of the current command
  uint32_t mCommandEnd;

  std::vector<hidl_handle> mDataHandles;
  std::vector<native_handle_t *> mTemporaryHandles;

  std::unique_ptr<CommandQueueType> mQueue;
};

// This class helps parse a command queue.  Note that all sizes/lengths are in units of uint32_t's.
class CommandReaderBase {
 public:
  CommandReaderBase() : mDataMaxSize(0) { reset(); }

  bool setMQDescriptor(const MQDescriptorSync<uint32_t>& descriptor) {
    mQueue = std::make_unique<CommandQueueType>(descriptor, false);
    if (mQueue->isValid()) {
      return true;
    } else {
      mQueue = nullptr;
      return false;
    }
  }

  bool readQueue(uint32_t commandLength, const hidl_vec<hidl_handle>& commandHandles) {
    if (!mQueue) {
      return false;
    }

    auto quantumCount = mQueue->getQuantumCount();
    if (mDataMaxSize < quantumCount) {
      mDataMaxSize = quantumCount;
      mData = std::make_unique<uint32_t[]>(mDataMaxSize);
    }

    if (commandLength > mDataMaxSize || !mQueue->read(mData.get(), commandLength)) {
      ALOGE("failed to read commands from message queue");
      return false;
    }

    mDataSize = commandLength;
    mDataRead = 0;
    mCommandBegin = 0;
    mCommandEnd = 0;
    mDataHandles.setToExternal(const_cast<hidl_handle*>(commandHandles.data()),
                               commandHandles.size());

    return true;
  }

  void reset() {
    mDataSize = 0;
    mDataRead = 0;
    mCommandBegin = 0;
    mCommandEnd = 0;
    mDataHandles.setToExternal(nullptr, 0);
  }

 protected:
  bool isEmpty() const { return (mDataRead >= mDataSize); }

  bool beginCommand(IQtiComposerClient::Command& command, uint16_t& length) {
    if (mCommandEnd) {
      LOG_FATAL("endCommand was not called before command 0x%x", command);
    }

    constexpr uint32_t opcode_mask =
      static_cast<uint32_t>(IQtiComposerClient::Command::OPCODE_MASK);
    constexpr uint32_t length_mask =
      static_cast<uint32_t>(IQtiComposerClient::Command::LENGTH_MASK);

    uint32_t val = read();
    command = static_cast<IQtiComposerClient::Command>(val & opcode_mask);
    length = static_cast<uint16_t>(val & length_mask);

    if (mDataRead + length > mDataSize) {
      ALOGE("command 0x%x has invalid command length %" PRIu16, command, length);
      // undo the read() above
      mDataRead--;
      return false;
    }

    mCommandEnd = mDataRead + length;

    return true;
  }

  void endCommand() {
    if (!mCommandEnd) {
      LOG_FATAL("beginCommand was not called");
    } else if (mDataRead > mCommandEnd) {
      LOG_FATAL("too much data read");
      mDataRead = mCommandEnd;
    } else if (mDataRead < mCommandEnd) {
      LOG_FATAL("too little data read");
      mDataRead = mCommandEnd;
    }

    mCommandBegin = mCommandEnd;
    mCommandEnd = 0;
  }

  uint32_t getCommandLoc() const { return mCommandBegin; }

  uint32_t read() { return mData[mDataRead++]; }

  int32_t readSigned() {
    int32_t val;
    memcpy(&val, &mData[mDataRead++], sizeof(val));
    return val;
  }

  float readFloat() {
    float val;
    memcpy(&val, &mData[mDataRead++], sizeof(val));
    return val;
  }

  uint64_t read64() {
    uint32_t lo = read();
    uint32_t hi = read();
    return (static_cast<uint64_t>(hi) << 32) | lo;
  }

  void readBlob(uint32_t size, void* blob) {
    memcpy(blob, &mData[mDataRead], size);
    uint32_t numElements = size / sizeof(uint32_t);
    mDataRead += numElements;
    mDataRead += (size - numElements * sizeof(uint32_t) != 0) ? 1 : 0;
  }

  IQtiComposerClient::Color readColor() {
    uint32_t val = read();
    return IQtiComposerClient::Color{
      static_cast<uint8_t>((val >>  0) & 0xff),
      static_cast<uint8_t>((val >>  8) & 0xff),
      static_cast<uint8_t>((val >> 16) & 0xff),
      static_cast<uint8_t>((val >> 24) & 0xff),
    };
  }

  // ownership of handle is not transferred
  const native_handle_t* readHandle(bool& useCache) {
    const native_handle_t* handle = nullptr;

    int32_t index = readSigned();
    switch (index) {
    case static_cast<int32_t>(IQtiComposerClient::HandleIndex::EMPTY):
      useCache = false;
      break;
    case static_cast<int32_t>(IQtiComposerClient::HandleIndex::CACHED):
      useCache = true;
      break;
    default:
      if (static_cast<size_t>(index) < mDataHandles.size()) {
        handle = mDataHandles[index].getNativeHandle();
      } else {
        ALOGE("invalid handle index %zu", static_cast<size_t>(index));
      }
      useCache = false;
      break;
    }

    return handle;
  }

  const native_handle_t* readHandle() {
    bool useCache;
    return readHandle(useCache);
  }

  // Handle would still own original fence. Hence create a Fence object on duped fd.
  void readFence(shared_ptr<Fence>* fence, const string &name) {
    auto handle = readHandle();
    if (!handle || handle->numFds == 0) {
      return;
    }

    if (handle->numFds != 1) {
      ALOGE("invalid fence handle with %d fds", handle->numFds);
      return;
    }

    *fence = Fence::Create(dup(handle->data[0]), name);
    if (*fence == nullptr) {
      ALOGW("failed to dup fence %d", handle->data[0]);
      sync_wait(handle->data[0], -1);
    }
  }

 private:
  std::unique_ptr<CommandQueueType> mQueue;
  uint32_t mDataMaxSize;
  std::unique_ptr<uint32_t[]> mData;

  uint32_t mDataSize;
  uint32_t mDataRead;

  // begin/end offsets of the current command
  uint32_t mCommandBegin;
  uint32_t mCommandEnd;

  hidl_vec<hidl_handle> mDataHandles;
};

}  // namespace V3_0
}  // namespace composer
}  // namespace display
}  // namespace hardware
}  // namespace qti
}  // namespace vendor

#endif  // __QTICOMPOSERCOMMANDBUFFER_H__
