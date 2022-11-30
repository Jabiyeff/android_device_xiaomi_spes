/*
 * Copyright (c) 2016-2019, The Linux Foundation. All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of The Linux Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#define ATRACE_TAG (ATRACE_TAG_GRAPHICS | ATRACE_TAG_HAL)
#include <log/log.h>
#include <cutils/trace.h>
#include <sync/sync.h>
#include <utils/Trace.h>
#include <algorithm>
#include <sstream>
#include <string>

#include "gr_buf_descriptor.h"
#include "gr_device_impl.h"
#include "gr_utils.h"
#include "gralloc_priv.h"
#include "qdMetaData.h"
#include "qd_utils.h"

int gralloc_device_open(const struct hw_module_t *module, const char *name, hw_device_t **device);

int gralloc_device_close(struct hw_device_t *device);

static struct hw_module_methods_t gralloc_module_methods = {.open = gralloc_device_open};

struct gralloc_module_t HAL_MODULE_INFO_SYM = {
    // clang-format off
    .common = {
            .tag = HARDWARE_MODULE_TAG,
            .module_api_version = GRALLOC_MODULE_API_VERSION_1_0,
            .hal_api_version = HARDWARE_HAL_API_VERSION,
            .id = GRALLOC_HARDWARE_MODULE_ID,
            .name = "Graphics Memory Module",
            .author = "Code Aurora Forum",
            .methods = &gralloc_module_methods,
            .dso = 0,
            .reserved = {0},
        },
    // clang-format on
};

int gralloc_device_open(const struct hw_module_t *module, const char *name, hw_device_t **device) {
  int status = -EINVAL;
  if (module && device && !strcmp(name, GRALLOC_HARDWARE_MODULE_ID)) {
    gralloc::GrallocImpl * /*gralloc1_device_t*/ dev = gralloc::GrallocImpl::GetInstance(module);
    *device = reinterpret_cast<hw_device_t *>(dev);
    if (dev) {
      status = 0;
    } else {
      ALOGE("Fatal error opening gralloc1 device");
    }
  }
  return status;
}

namespace gralloc {

std::atomic<uint64_t> GrallocImpl::next_descriptor_id_(1);

GrallocImpl::GrallocImpl(const hw_module_t *module) {
  common.tag = HARDWARE_DEVICE_TAG;
  common.version = GRALLOC_MODULE_API_VERSION_1_0;
  common.module = const_cast<hw_module_t *>(module);
  common.close = CloseDevice;
  getFunction = GetFunction;
  getCapabilities = GetCapabilities;

  initialized_ = Init();
}

inline gralloc1_error_t ToError(Error error) {
  switch (error) {
    case Error::NONE:
      return GRALLOC1_ERROR_NONE;
    case Error::BAD_DESCRIPTOR:
      return GRALLOC1_ERROR_BAD_DESCRIPTOR;
    case Error::BAD_BUFFER:
      return GRALLOC1_ERROR_BAD_HANDLE;
    case Error::BAD_VALUE:
      return GRALLOC1_ERROR_BAD_VALUE;
    case Error::NO_RESOURCES:
      return GRALLOC1_ERROR_NO_RESOURCES;
    case Error::UNSUPPORTED:
    default:
      return GRALLOC1_ERROR_UNSUPPORTED;
  }
}

static uint64_t ProducerUsageToBufferUsage(uint64_t /*gralloc1_producer_usage_t*/ producer_usage) {
  uint64_t usage = producer_usage & ~(GRALLOC1_PRODUCER_USAGE_CPU_READ_OFTEN |
                                      GRALLOC1_PRODUCER_USAGE_CPU_WRITE_OFTEN);
  if ((producer_usage & GRALLOC1_PRODUCER_USAGE_CPU_READ_OFTEN) ==
      GRALLOC1_PRODUCER_USAGE_CPU_READ_OFTEN) {
    usage |= BufferUsage::CPU_READ_OFTEN;
  } else if ((producer_usage & GRALLOC1_PRODUCER_USAGE_CPU_READ) ==
             GRALLOC1_PRODUCER_USAGE_CPU_READ) {
    usage |= BufferUsage::CPU_READ_RARELY;
  }

  if ((producer_usage & GRALLOC1_PRODUCER_USAGE_CPU_WRITE_OFTEN) ==
      GRALLOC1_PRODUCER_USAGE_CPU_WRITE_OFTEN) {
    usage |= BufferUsage::CPU_WRITE_OFTEN;
  } else if ((producer_usage & GRALLOC1_PRODUCER_USAGE_CPU_WRITE) ==
             GRALLOC1_PRODUCER_USAGE_CPU_WRITE) {
    usage |= BufferUsage::CPU_WRITE_RARELY;
  }
  return usage;
}

static uint64_t ConsumerUsageToBufferUsage(uint64_t /*gralloc1_consumer_usage_t*/ consumer_usage) {
  uint64_t usage = consumer_usage & ~(GRALLOC1_CONSUMER_USAGE_CPU_READ_OFTEN);
  if ((consumer_usage & GRALLOC1_CONSUMER_USAGE_CPU_READ_OFTEN) ==
      GRALLOC1_CONSUMER_USAGE_CPU_READ_OFTEN) {
    usage |= BufferUsage::CPU_READ_OFTEN;
  } else if ((consumer_usage & GRALLOC1_CONSUMER_USAGE_CPU_READ) ==
             GRALLOC1_CONSUMER_USAGE_CPU_READ) {
    usage |= BufferUsage::CPU_READ_RARELY;
  }
  return usage;
}

bool GrallocImpl::Init() {
  buf_mgr_ = BufferManager::GetInstance();
  return buf_mgr_ != nullptr;
}

GrallocImpl::~GrallocImpl() {}

gralloc1_error_t GrallocImpl::CreateBufferDescriptorLocked(
    gralloc1_buffer_descriptor_t *descriptor_id) {
  std::lock_guard<std::mutex> lock(descriptor_lock_);
  auto descriptor = std::make_shared<BufferDescriptor>(next_descriptor_id_++);
  *descriptor_id = static_cast<gralloc1_buffer_descriptor_t>(descriptor->GetId());
  descriptors_map_.emplace(*descriptor_id, descriptor);
  return GRALLOC1_ERROR_NONE;
}

gralloc1_error_t GrallocImpl::DestroyBufferDescriptorLocked(
    gralloc1_buffer_descriptor_t descriptor_id) {
  std::lock_guard<std::mutex> lock(descriptor_lock_);
  const auto descriptor = descriptors_map_.find(descriptor_id);
  if (descriptor == descriptors_map_.end()) {
    return GRALLOC1_ERROR_BAD_DESCRIPTOR;
  }
  descriptors_map_.erase(descriptor);
  return GRALLOC1_ERROR_NONE;
}

int GrallocImpl::CloseDevice(hw_device_t *device __unused) {
  // No-op since the gralloc device is a singleton
  return 0;
}

void GrallocImpl::GetCapabilities(struct gralloc1_device *device, uint32_t *out_count,
                                  int32_t /*gralloc1_capability_t*/ *out_capabilities) {
  if (device != nullptr && out_count != nullptr) {
    if (out_capabilities != nullptr && *out_count >= 3) {
      out_capabilities[0] = GRALLOC1_CAPABILITY_TEST_ALLOCATE;
      out_capabilities[1] = GRALLOC1_CAPABILITY_LAYERED_BUFFERS;
      out_capabilities[2] = GRALLOC1_CAPABILITY_RELEASE_IMPLY_DELETE;
    }
    *out_count = 3;
  }
  return;
}

gralloc1_function_pointer_t GrallocImpl::GetFunction(gralloc1_device_t *device, int32_t function) {
  if (!device) {
    return NULL;
  }

  switch (function) {
    case GRALLOC1_FUNCTION_DUMP:
      return reinterpret_cast<gralloc1_function_pointer_t>(Dump);
    case GRALLOC1_FUNCTION_CREATE_DESCRIPTOR:
      return reinterpret_cast<gralloc1_function_pointer_t>(CreateBufferDescriptor);
    case GRALLOC1_FUNCTION_DESTROY_DESCRIPTOR:
      return reinterpret_cast<gralloc1_function_pointer_t>(DestroyBufferDescriptor);
    case GRALLOC1_FUNCTION_SET_CONSUMER_USAGE:
      return reinterpret_cast<gralloc1_function_pointer_t>(SetConsumerUsage);
    case GRALLOC1_FUNCTION_SET_DIMENSIONS:
      return reinterpret_cast<gralloc1_function_pointer_t>(SetBufferDimensions);
    case GRALLOC1_FUNCTION_SET_FORMAT:
      return reinterpret_cast<gralloc1_function_pointer_t>(SetColorFormat);
    case GRALLOC1_FUNCTION_SET_LAYER_COUNT:
      return reinterpret_cast<gralloc1_function_pointer_t>(SetLayerCount);
    case GRALLOC1_FUNCTION_SET_PRODUCER_USAGE:
      return reinterpret_cast<gralloc1_function_pointer_t>(SetProducerUsage);
    case GRALLOC1_FUNCTION_GET_BACKING_STORE:
      return reinterpret_cast<gralloc1_function_pointer_t>(GetBackingStore);
    case GRALLOC1_FUNCTION_GET_CONSUMER_USAGE:
      return reinterpret_cast<gralloc1_function_pointer_t>(GetConsumerUsage);
    case GRALLOC1_FUNCTION_GET_DIMENSIONS:
      return reinterpret_cast<gralloc1_function_pointer_t>(GetBufferDimensions);
    case GRALLOC1_FUNCTION_GET_FORMAT:
      return reinterpret_cast<gralloc1_function_pointer_t>(GetColorFormat);
    case GRALLOC1_FUNCTION_GET_LAYER_COUNT:
      return reinterpret_cast<gralloc1_function_pointer_t>(GetLayerCount);
    case GRALLOC1_FUNCTION_GET_PRODUCER_USAGE:
      return reinterpret_cast<gralloc1_function_pointer_t>(GetProducerUsage);
    case GRALLOC1_FUNCTION_GET_STRIDE:
      return reinterpret_cast<gralloc1_function_pointer_t>(GetBufferStride);
    case GRALLOC1_FUNCTION_ALLOCATE:
      return reinterpret_cast<gralloc1_function_pointer_t>(AllocateBuffers);
    case GRALLOC1_FUNCTION_RETAIN:
      return reinterpret_cast<gralloc1_function_pointer_t>(RetainBuffer);
    case GRALLOC1_FUNCTION_RELEASE:
      return reinterpret_cast<gralloc1_function_pointer_t>(ReleaseBuffer);
    case GRALLOC1_FUNCTION_GET_NUM_FLEX_PLANES:
      return reinterpret_cast<gralloc1_function_pointer_t>(GetNumFlexPlanes);
    case GRALLOC1_FUNCTION_LOCK:
      return reinterpret_cast<gralloc1_function_pointer_t>(LockBuffer);
    case GRALLOC1_FUNCTION_LOCK_FLEX:
      return reinterpret_cast<gralloc1_function_pointer_t>(LockFlex);
    case GRALLOC1_FUNCTION_UNLOCK:
      return reinterpret_cast<gralloc1_function_pointer_t>(UnlockBuffer);
    case GRALLOC1_FUNCTION_PERFORM:
      return reinterpret_cast<gralloc1_function_pointer_t>(Gralloc1Perform);
    default:
      ALOGE("%s:Gralloc Error. Client Requested for unsupported function", __FUNCTION__);
      return NULL;
  }

  return NULL;
}

void GrallocImpl::Dump(gralloc1_device_t *device, uint32_t *out_size,
                                   char *out_buffer) {
  if (!device || !out_size) {
    ALOGE("Gralloc Error : device=%p", (void *)device);
    return;
  }
  const size_t max_dump_size = 8192;
  if (out_buffer == nullptr) {
    *out_size = max_dump_size;
  } else {
    std::ostringstream os;
    os << "-------------------------------" << std::endl;
    os << "QTI gralloc dump:" << std::endl;
    os << "-------------------------------" << std::endl;
    GrallocImpl const *dev = GRALLOC_IMPL(device);
    dev->buf_mgr_->Dump(&os);
    os << "-------------------------------" << std::endl;
    auto copied = os.str().copy(out_buffer, std::min(os.str().size(), max_dump_size), 0);
    *out_size = UINT(copied);
  }

  return;
}

gralloc1_error_t GrallocImpl::CheckDeviceAndHandle(gralloc1_device_t *device,
                                                   buffer_handle_t buffer) {
  const private_handle_t *hnd = PRIV_HANDLE_CONST(buffer);
  if (!device || (private_handle_t::validate(hnd) != 0)) {
    ALOGE("Gralloc Error : device= %p, buffer-handle=%p", (void *)device, (void *)buffer);
    return GRALLOC1_ERROR_BAD_HANDLE;
  }

  return GRALLOC1_ERROR_NONE;
}

int32_t GrallocImpl::CreateBufferDescriptor(gralloc1_device_t *device,
                                                     gralloc1_buffer_descriptor_t *out_descriptor) {
  if (!device || !out_descriptor) {
    return GRALLOC1_ERROR_BAD_DESCRIPTOR;
  }
  auto *dev = reinterpret_cast<GrallocImpl *>(device);
  return static_cast<int32_t>(dev->CreateBufferDescriptorLocked(out_descriptor));
}

int32_t GrallocImpl::DestroyBufferDescriptor(gralloc1_device_t *device,
                                                      gralloc1_buffer_descriptor_t descriptor) {
  if (!device) {
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_DESCRIPTOR);
  }
  auto *dev = reinterpret_cast<GrallocImpl *>(device);
  return static_cast<int32_t>(dev->DestroyBufferDescriptorLocked(descriptor));
}

int32_t GrallocImpl::SetConsumerUsage(gralloc1_device_t *device,
                                               gralloc1_buffer_descriptor_t descriptor,
                                               uint64_t /*gralloc1_consumer_usage_t*/ usage) {
  if (!device) {
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_DESCRIPTOR);
  } else {
    auto *dev = reinterpret_cast<GrallocImpl *>(device);
    return static_cast<int32_t>(dev->CallBufferDescriptorFunction(descriptor,
                                &BufferDescriptor::SetUsage,
                                ConsumerUsageToBufferUsage(usage)));
  }
}

int32_t GrallocImpl::SetBufferDimensions(gralloc1_device_t *device,
                                                  gralloc1_buffer_descriptor_t descriptor,
                                                  uint32_t width, uint32_t height) {
  if (!device) {
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_DESCRIPTOR);
  } else {
    auto *dev = reinterpret_cast<GrallocImpl *>(device);
    return static_cast<int32_t>(dev->CallBufferDescriptorFunction(descriptor,
                                &BufferDescriptor::SetDimensions,
                                INT(width), INT(height)));
  }
}

int32_t GrallocImpl::SetColorFormat(gralloc1_device_t *device,
                                             gralloc1_buffer_descriptor_t descriptor,
                                             int32_t format) {
  if (!device) {
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_DESCRIPTOR);
  } else {
    auto *dev = reinterpret_cast<GrallocImpl *>(device);
    return static_cast<int32_t>(dev->CallBufferDescriptorFunction(descriptor,
                                &BufferDescriptor::SetColorFormat, format));
  }
}

int32_t GrallocImpl::SetLayerCount(gralloc1_device_t *device,
                                            gralloc1_buffer_descriptor_t descriptor,
                                            uint32_t layer_count) {
  if (!device) {
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_DESCRIPTOR);
  } else {
    auto *dev = reinterpret_cast<GrallocImpl *>(device);
    return static_cast<int32_t>(dev->CallBufferDescriptorFunction(descriptor,
                                &BufferDescriptor::SetLayerCount, layer_count));
  }
}

int32_t GrallocImpl::SetProducerUsage(gralloc1_device_t *device,
                                               gralloc1_buffer_descriptor_t descriptor,
                                               uint64_t /*gralloc1_producer_usage_t*/ usage) {
  if (!device) {
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_DESCRIPTOR);
  } else {
    auto *dev = reinterpret_cast<GrallocImpl *>(device);
    return static_cast<int32_t>(dev->CallBufferDescriptorFunction(descriptor,
                                &BufferDescriptor::SetUsage, ProducerUsageToBufferUsage(usage)));
  }
}

int32_t GrallocImpl::GetBackingStore(gralloc1_device_t *device, buffer_handle_t buffer,
                                              gralloc1_backing_store_t *out_backstore) {
  if (!out_backstore) {
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
  }

  gralloc1_error_t status = CheckDeviceAndHandle(device, buffer);
  if (status == GRALLOC1_ERROR_NONE) {
    *out_backstore =
        static_cast<gralloc1_backing_store_t>(PRIV_HANDLE_CONST(buffer)->GetBackingstore());
  }

  return status;
}

int32_t GrallocImpl::GetConsumerUsage(gralloc1_device_t *device, buffer_handle_t buffer,
                                               uint64_t /*gralloc1_consumer_usage_t*/ *outUsage) {
  if (!outUsage) {
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
  }

  gralloc1_error_t status = CheckDeviceAndHandle(device, buffer);
  if (status == GRALLOC1_ERROR_NONE) {
    *outUsage = static_cast<uint64_t>(PRIV_HANDLE_CONST(buffer)->GetUsage());
  }

  return static_cast<int32_t>(status);
}

int32_t GrallocImpl::GetBufferDimensions(gralloc1_device_t *device, buffer_handle_t buffer,
                                                  uint32_t *outWidth, uint32_t *outHeight) {
  if (!outWidth || !outHeight) {
    return GRALLOC1_ERROR_BAD_VALUE;
  }

  gralloc1_error_t status = CheckDeviceAndHandle(device, buffer);
  if (status == GRALLOC1_ERROR_NONE) {
    const private_handle_t *hnd = PRIV_HANDLE_CONST(buffer);
    *outWidth = UINT(hnd->GetUnalignedWidth());
    *outHeight = UINT(hnd->GetUnalignedHeight());
  }

  return static_cast<int32_t>(status);
}

int32_t GrallocImpl::GetColorFormat(gralloc1_device_t *device, buffer_handle_t buffer,
                                             int32_t *outFormat) {
  if (!outFormat) {
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
  }

  gralloc1_error_t status = CheckDeviceAndHandle(device, buffer);
  if (status == GRALLOC1_ERROR_NONE) {
    *outFormat = PRIV_HANDLE_CONST(buffer)->GetColorFormat();
  }

  return static_cast<int32_t>(status);
}

int32_t GrallocImpl::GetLayerCount(gralloc1_device_t *device, buffer_handle_t buffer,
                                            uint32_t *outLayerCount) {
  if (!outLayerCount) {
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
  }

  gralloc1_error_t status = CheckDeviceAndHandle(device, buffer);
  if (status == GRALLOC1_ERROR_NONE) {
    *outLayerCount = PRIV_HANDLE_CONST(buffer)->GetLayerCount();
  }

  return static_cast<int32_t>(status);
}

int32_t GrallocImpl::GetProducerUsage(gralloc1_device_t *device, buffer_handle_t buffer,
                                               uint64_t /*gralloc1_producer_usage_t*/ *outUsage) {
  if (!outUsage) {
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
  }

  gralloc1_error_t status = CheckDeviceAndHandle(device, buffer);
  if (status == GRALLOC1_ERROR_NONE) {
    *outUsage = static_cast<uint64_t>(PRIV_HANDLE_CONST(buffer)->GetUsage());
  }

  return static_cast<int32_t>(status);
}

int32_t GrallocImpl::GetBufferStride(gralloc1_device_t *device, buffer_handle_t buffer,
                                              uint32_t *outStride) {
  if (!outStride) {
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
  }

  gralloc1_error_t status = CheckDeviceAndHandle(device, buffer);
  if (status == GRALLOC1_ERROR_NONE) {
    *outStride = UINT(PRIV_HANDLE_CONST(buffer)->GetStride());
  }

  return static_cast<int32_t>(status);
}

gralloc1_error_t GrallocImpl::AllocateBuffer(const gralloc1_buffer_descriptor_t *descriptor_ids,
                                             buffer_handle_t *out_buffers) {
  gralloc1_error_t status = GRALLOC1_ERROR_NONE;

  // Validate descriptor
  std::lock_guard<std::mutex> descriptor_lock(descriptor_lock_);
  std::shared_ptr<gralloc::BufferDescriptor> descriptor;
  const auto map_descriptor = descriptors_map_.find(descriptor_ids[0]);
  if (map_descriptor == descriptors_map_.end()) {
    return GRALLOC1_ERROR_BAD_DESCRIPTOR;
  } else {
    descriptor = map_descriptor->second;
  }

  // Allocate separate buffer for each descriptor
  if (buf_mgr_->AllocateBuffer(*descriptor, &out_buffers[0]) != Error::NONE) {
    return GRALLOC1_ERROR_NO_RESOURCES;
  }

  return status;
}

int32_t GrallocImpl::AllocateBuffers(gralloc1_device_t *device, uint32_t num_descriptors,
                                              const gralloc1_buffer_descriptor_t *descriptors,
                                              buffer_handle_t *out_buffers) {
  if (!num_descriptors || !descriptors) {
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_DESCRIPTOR);
  }

  if (!device) {
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
  }

  if (num_descriptors != 1) {
    return static_cast<int32_t>(GRALLOC1_ERROR_UNSUPPORTED);
  }

  auto *dev = reinterpret_cast<GrallocImpl *>(device);
  gralloc1_error_t status = dev->AllocateBuffer(descriptors, out_buffers);

  return static_cast<int32_t>(status);
}

int32_t GrallocImpl::RetainBuffer(gralloc1_device_t *device, buffer_handle_t buffer) {
  gralloc1_error_t status = CheckDeviceAndHandle(device, buffer);
  if (status == GRALLOC1_ERROR_NONE) {
    const private_handle_t *hnd = PRIV_HANDLE_CONST(buffer);
    GrallocImpl const *dev = GRALLOC_IMPL(device);
    status = ToError(dev->buf_mgr_->RetainBuffer(hnd));
  }

  return static_cast<int32_t>(status);
}

int32_t GrallocImpl::ReleaseBuffer(gralloc1_device_t *device, buffer_handle_t buffer) {
  if (!device || !buffer) {
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_DESCRIPTOR);
  }

  const private_handle_t *hnd = PRIV_HANDLE_CONST(buffer);
  GrallocImpl const *dev = GRALLOC_IMPL(device);
  return static_cast<int32_t>((ToError(dev->buf_mgr_->ReleaseBuffer(hnd))));
}

int32_t GrallocImpl::GetFlexLayout(const private_handle_t *hnd,
                                            struct android_flex_layout *layout) {
  if (!IsYuvFormat(hnd->format)) {
    return static_cast<int32_t>(GRALLOC1_ERROR_UNSUPPORTED);
  }

  android_ycbcr yuvPlaneInfo[2];
  int err = GetYUVPlaneInfo(hnd, yuvPlaneInfo);

  if (err != 0) {
    return GRALLOC1_ERROR_BAD_HANDLE;
  }

  layout->format = FLEX_FORMAT_YCbCr;
  layout->num_planes = 3;

  for (uint32_t i = 0; i < layout->num_planes; i++) {
    layout->planes[i].bits_per_component = 8;
    layout->planes[i].bits_used = 8;
    layout->planes[i].h_increment = 1;
    layout->planes[i].v_increment = 1;
    layout->planes[i].h_subsampling = 2;
    layout->planes[i].v_subsampling = 2;
  }

  // We are only returning flex layout for progressive or single field formats.
  struct android_ycbcr ycbcr = yuvPlaneInfo[0];
  layout->planes[0].top_left = static_cast<uint8_t *>(ycbcr.y);
  layout->planes[0].component = FLEX_COMPONENT_Y;
  layout->planes[0].v_increment = static_cast<int32_t>(ycbcr.ystride);

  layout->planes[1].top_left = static_cast<uint8_t *>(ycbcr.cb);
  layout->planes[1].component = FLEX_COMPONENT_Cb;
  layout->planes[1].h_increment = static_cast<int32_t>(ycbcr.chroma_step);
  layout->planes[1].v_increment = static_cast<int32_t>(ycbcr.cstride);

  layout->planes[2].top_left = static_cast<uint8_t *>(ycbcr.cr);
  layout->planes[2].component = FLEX_COMPONENT_Cr;
  layout->planes[2].h_increment = static_cast<int32_t>(ycbcr.chroma_step);
  layout->planes[2].v_increment = static_cast<int32_t>(ycbcr.cstride);
  return static_cast<int32_t>(GRALLOC1_ERROR_NONE);
}

int32_t GrallocImpl::GetNumFlexPlanes(gralloc1_device_t *device, buffer_handle_t buffer,
                                               uint32_t *out_num_planes) {
  if (!out_num_planes) {
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
  }

  gralloc1_error_t status = CheckDeviceAndHandle(device, buffer);
  if (status == GRALLOC1_ERROR_NONE) {
    const private_handle_t *hnd = PRIV_HANDLE_CONST(buffer);
    if (!IsYuvFormat(hnd->format)) {
      status = GRALLOC1_ERROR_UNSUPPORTED;
    } else {
      *out_num_planes = 3;
    }
  }
  return static_cast<int32_t>(status);
}

static inline void CloseFdIfValid(int fd) {
  if (fd > 0) {
    close(fd);
  }
}

int32_t GrallocImpl::LockBuffer(gralloc1_device_t *device, buffer_handle_t buffer,
                                         uint64_t /*gralloc1_producer_usage_t*/ prod_usage,
                                         uint64_t /*gralloc1_consumer_usage_t*/ cons_usage,
                                         const gralloc1_rect_t *region, void **out_data,
                                         int32_t acquire_fence) {
  ATRACE_CALL();
  gralloc1_error_t status = CheckDeviceAndHandle(device, buffer);
  if (status != GRALLOC1_ERROR_NONE || !out_data ||
      !region) {  // currently we ignore the region/rect client wants to lock
    CloseFdIfValid(acquire_fence);
    return static_cast<int32_t>(status);
  }

  if (acquire_fence > 0) {
    ATRACE_BEGIN("fence wait");
    int error = sync_wait(acquire_fence, 1000);
    ATRACE_END();
    CloseFdIfValid(acquire_fence);
    if (error < 0) {
      ALOGE("%s: sync_wait timedout! error = %s", __FUNCTION__, strerror(errno));
      return static_cast<int32_t>(GRALLOC1_ERROR_UNDEFINED);
    }
  }

  const private_handle_t *hnd = PRIV_HANDLE_CONST(buffer);
  GrallocImpl const *dev = GRALLOC_IMPL(device);

  // Either producer usage or consumer usage must be *_USAGE_NONE
  if ((prod_usage != GRALLOC1_PRODUCER_USAGE_NONE) &&
      (cons_usage != GRALLOC1_CONSUMER_USAGE_NONE)) {
    // Current gralloc1 clients do not satisfy this restriction.
    // See b/33588773 for details
    // return GRALLOC1_ERROR_BAD_VALUE;
  }

  status = ToError(dev->buf_mgr_->LockBuffer(
      hnd, ProducerUsageToBufferUsage(prod_usage) | ConsumerUsageToBufferUsage(cons_usage)));
  *out_data = reinterpret_cast<void *>(hnd->base);

  return static_cast<int32_t>(status);
}

int32_t GrallocImpl::LockFlex(gralloc1_device_t *device, buffer_handle_t buffer,
                                       uint64_t /*gralloc1_producer_usage_t*/ prod_usage,
                                       uint64_t /*gralloc1_consumer_usage_t*/ cons_usage,
                                       const gralloc1_rect_t *region,
                                       struct android_flex_layout *out_flex_layout,
                                       int32_t acquire_fence) {
  if (!out_flex_layout) {
    CloseFdIfValid(acquire_fence);
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
  }

  void *out_data{};
  int32_t status = GrallocImpl::LockBuffer(device, buffer, prod_usage, cons_usage, region,
                                                    &out_data, acquire_fence);
  if (status != GRALLOC1_ERROR_NONE) {
    return static_cast<int32_t>(status);
  }

  auto *dev = reinterpret_cast<GrallocImpl *>(device);
  const private_handle_t *hnd = PRIV_HANDLE_CONST(buffer);
  dev->GetFlexLayout(hnd, out_flex_layout);
  return static_cast<int32_t>(status);
}

int32_t GrallocImpl::UnlockBuffer(gralloc1_device_t *device, buffer_handle_t buffer,
                                           int32_t *release_fence) {
  gralloc1_error_t status = CheckDeviceAndHandle(device, buffer);
  if (status != GRALLOC1_ERROR_NONE) {
    return static_cast<int32_t>(status);
  }

  if (!release_fence) {
    return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
  }

  const private_handle_t *hnd = PRIV_HANDLE_CONST(buffer);
  GrallocImpl const *dev = GRALLOC_IMPL(device);

  *release_fence = -1;

  return static_cast<int32_t>(ToError(dev->buf_mgr_->UnlockBuffer(hnd)));
}

static int32_t Perform(int operation, va_list args) {
  switch (operation) {
    case GRALLOC_MODULE_PERFORM_GET_STRIDE: {
      int width = va_arg(args, int);
      int format = va_arg(args, int);
      int *stride = va_arg(args, int *);
      unsigned int alignedw = 0, alignedh = 0;

      if (!stride) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
      }

      BufferInfo info(width, width, format);
      GetAlignedWidthAndHeight(info, &alignedw, &alignedh);
      *stride = INT(alignedw);
    } break;

    case GRALLOC_MODULE_PERFORM_GET_CUSTOM_STRIDE_FROM_HANDLE: {
      private_handle_t *hnd = va_arg(args, private_handle_t *);
      int *stride = va_arg(args, int *);
      if (private_handle_t::validate(hnd) != 0) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_HANDLE);
      }

      if (!stride) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
      }

      BufferDim_t buffer_dim;
      if (getMetaData(hnd, GET_BUFFER_GEOMETRY, &buffer_dim) == 0) {
        *stride = buffer_dim.sliceWidth;
      } else {
        *stride = hnd->width;
      }
    } break;

    case GRALLOC_MODULE_PERFORM_GET_CUSTOM_STRIDE_AND_HEIGHT_FROM_HANDLE: {
      private_handle_t *hnd = va_arg(args, private_handle_t *);
      int *stride = va_arg(args, int *);
      int *height = va_arg(args, int *);
      if (private_handle_t::validate(hnd) != 0) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_HANDLE);
      }

      if (!stride || !height) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
      }

      GetCustomDimensions(hnd, stride, height);
    } break;

    case GRALLOC_MODULE_PERFORM_GET_ATTRIBUTES: {
      int width = va_arg(args, int);
      int height = va_arg(args, int);
      int format = va_arg(args, int);
      uint64_t usage = va_arg(args, uint64_t);
      usage |= va_arg(args, uint64_t);

      int *aligned_width = va_arg(args, int *);
      int *aligned_height = va_arg(args, int *);
      int *tile_enabled = va_arg(args, int *);
      if (!aligned_width || !aligned_height || !tile_enabled) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
      }

      unsigned int alignedw, alignedh;
      BufferInfo info(width, height, format, usage);
      *tile_enabled = IsUBwcEnabled(format, usage);
      GetAlignedWidthAndHeight(info, &alignedw, &alignedh);
      *aligned_width = INT(alignedw);
      *aligned_height = INT(alignedh);
    } break;

    case GRALLOC_MODULE_PERFORM_GET_COLOR_SPACE_FROM_HANDLE: {
      private_handle_t *hnd = va_arg(args, private_handle_t *);
      int *color_space = va_arg(args, int *);

      if (private_handle_t::validate(hnd) != 0) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_HANDLE);
      }

      if (!color_space) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
      }

      *color_space = 0;
      GetColorSpaceFromMetadata(hnd, color_space);
    } break;
    case GRALLOC_MODULE_PERFORM_GET_YUV_PLANE_INFO: {
      private_handle_t *hnd = va_arg(args, private_handle_t *);
      android_ycbcr *ycbcr = va_arg(args, struct android_ycbcr *);
      if (private_handle_t::validate(hnd) != 0) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_HANDLE);
      }

      if (!ycbcr) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
      }

      if (GetYUVPlaneInfo(hnd, ycbcr)) {
        return static_cast<int32_t>(GRALLOC1_ERROR_UNDEFINED);
      }
    } break;

    case GRALLOC_MODULE_PERFORM_GET_MAP_SECURE_BUFFER_INFO: {
      private_handle_t *hnd = va_arg(args, private_handle_t *);
      int *map_secure_buffer = va_arg(args, int *);

      if (private_handle_t::validate(hnd) != 0) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_HANDLE);
      }

      if (!map_secure_buffer) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
      }

      if (getMetaData(hnd, GET_MAP_SECURE_BUFFER, map_secure_buffer) != 0) {
        *map_secure_buffer = 0;
      }
    } break;

    case GRALLOC_MODULE_PERFORM_GET_UBWC_FLAG: {
      private_handle_t *hnd = va_arg(args, private_handle_t *);
      int *flag = va_arg(args, int *);

      if (private_handle_t::validate(hnd) != 0) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_HANDLE);
      }

      if (!flag) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
      }

      *flag = hnd->flags & private_handle_t::PRIV_FLAGS_UBWC_ALIGNED;
      int linear_format = 0;
      if (getMetaData(hnd, GET_LINEAR_FORMAT, &linear_format) == 0) {
        if (linear_format) {
          *flag = 0;
        }
      }
    } break;

    case GRALLOC_MODULE_PERFORM_GET_RGB_DATA_ADDRESS: {
      private_handle_t *hnd = va_arg(args, private_handle_t *);
      void **rgb_data = va_arg(args, void **);

      if (private_handle_t::validate(hnd) != 0) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_HANDLE);
      }

      if (!rgb_data) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
      }

      if (GetRgbDataAddress(hnd, rgb_data)) {
        return static_cast<int32_t>(GRALLOC1_ERROR_UNDEFINED);
      }
    } break;

    case GRALLOC1_MODULE_PERFORM_GET_INTERLACE_FLAG: {
      private_handle_t *hnd = va_arg(args, private_handle_t *);
      int *flag = va_arg(args, int *);

      if (private_handle_t::validate(hnd) != 0) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_HANDLE);
      }

      if (!flag) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_VALUE);
      }

      if (getMetaData(hnd, GET_PP_PARAM_INTERLACED, flag) != 0) {
        *flag = 0;
      }
    } break;

    case GRALLOC_MODULE_PERFORM_SET_SINGLE_BUFFER_MODE: {
      private_handle_t *hnd = va_arg(args, private_handle_t *);
      uint32_t *enable = va_arg(args, uint32_t *);
      if (private_handle_t::validate(hnd) != 0) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_HANDLE);
      }
      if (setMetaData(hnd, SET_SINGLE_BUFFER_MODE, enable) != 0) {
        return static_cast<int32_t>(GRALLOC1_ERROR_UNSUPPORTED);
      }
    } break;

    case GRALLOC_MODULE_PERFORM_GET_GRAPHICS_METADATA: {
      private_handle_t* hnd = va_arg(args, private_handle_t *);

      if (private_handle_t::validate(hnd) != 0) {
        return static_cast<int32_t>(GRALLOC1_ERROR_BAD_HANDLE);
      }

      void* graphic_metadata = va_arg(args, void*);

      if (getMetaData(hnd, GET_GRAPHICS_METADATA, graphic_metadata) != 0) {
        graphic_metadata = NULL;
        return static_cast<int32_t>(GRALLOC1_ERROR_UNSUPPORTED);
      }
    } break;

    default:
      break;
  }
  return static_cast<int32_t>(GRALLOC1_ERROR_NONE);
}

int32_t GrallocImpl::Gralloc1Perform(gralloc1_device_t *device, int operation, ...) {
  if (!device) {
    return GRALLOC1_ERROR_BAD_VALUE;
  }

  va_list args;
  va_start(args, operation);
  int32_t err = Perform(operation, args);
  va_end(args);

  return err;
}

}  // namespace gralloc
