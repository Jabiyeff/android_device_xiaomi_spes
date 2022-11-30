/*
 * Copyright (c) 2011-2020, The Linux Foundation. All rights reserved.

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

#include <log/log.h>
#include <cutils/properties.h>
#include <algorithm>
#include <vector>

#ifndef QMAA
#include <linux/msm_ion.h>
#endif

#include "gr_allocator.h"
#include "gr_utils.h"
#include "gralloc_priv.h"

#include "qd_utils.h"

#ifndef ION_FLAG_CP_PIXEL
#define ION_FLAG_CP_PIXEL 0
#endif

#ifndef ION_FLAG_ALLOW_NON_CONTIG
#define ION_FLAG_ALLOW_NON_CONTIG 0
#endif

#ifndef ION_FLAG_CP_CAMERA_PREVIEW
#define ION_FLAG_CP_CAMERA_PREVIEW 0
#endif

#ifndef ION_SECURE
#define ION_SECURE ION_FLAG_SECURE
#endif

#ifndef ION_FLAG_CP_CDSP
#define ION_FLAG_CP_CDSP 0
#endif

#ifdef SLAVE_SIDE_CP
#define CP_HEAP_ID ION_CP_MM_HEAP_ID
#define SD_HEAP_ID CP_HEAP_ID
#define ION_CP_FLAGS (ION_SECURE | ION_FLAG_ALLOW_NON_CONTIG)
#define ION_SD_FLAGS ION_SECURE
#define ION_SC_FLAGS ION_SECURE
#define ION_SC_PREVIEW_FLAGS ION_SECURE
#else  // MASTER_SIDE_CP
#define CP_HEAP_ID ION_SECURE_HEAP_ID
#define SD_HEAP_ID ION_SECURE_DISPLAY_HEAP_ID
#define ION_CP_FLAGS (ION_SECURE | ION_FLAG_CP_PIXEL)
#define ION_SD_FLAGS (ION_SECURE | ION_FLAG_CP_SEC_DISPLAY)
#define ION_SC_FLAGS (ION_SECURE | ION_FLAG_CP_CAMERA)
#define ION_SC_PREVIEW_FLAGS (ION_SECURE | ION_FLAG_CP_CAMERA_PREVIEW)
#endif

using std::shared_ptr;
using std::vector;

namespace gralloc {

static BufferInfo GetBufferInfo(const BufferDescriptor &descriptor) {
  return BufferInfo(descriptor.GetWidth(), descriptor.GetHeight(), descriptor.GetFormat(),
                    descriptor.GetUsage());
}

Allocator::Allocator() : ion_allocator_(nullptr) {}

bool Allocator::Init() {
  ion_allocator_ = new IonAlloc();

  if (!ion_allocator_->Init()) {
    return false;
  }

  return true;
}

Allocator::~Allocator() {
  if (ion_allocator_) {
    delete ion_allocator_;
  }
}

void Allocator::SetProperties(gralloc::GrallocProperties props) {
  use_system_heap_for_sensors_ = props.use_system_heap_for_sensors;
}

int Allocator::AllocateMem(AllocData *alloc_data, uint64_t usage, int format) {
  int ret;
  alloc_data->uncached = UseUncached(format, usage);

  // After this point we should have the right heap set, there is no fallback
  GetIonHeapInfo(usage, &alloc_data->heap_id, &alloc_data->alloc_type, &alloc_data->flags);

  ret = ion_allocator_->AllocBuffer(alloc_data);
  if (ret >= 0) {
    alloc_data->alloc_type |= private_handle_t::PRIV_FLAGS_USES_ION;
  } else {
    ALOGE("%s: Failed to allocate buffer - heap: 0x%x flags: 0x%x", __FUNCTION__,
          alloc_data->heap_id, alloc_data->flags);
  }

  return ret;
}

int Allocator::MapBuffer(void **base, unsigned int size, unsigned int offset, int fd) {
  if (ion_allocator_) {
    return ion_allocator_->MapBuffer(base, size, offset, fd);
  }

  return -EINVAL;
}

int Allocator::ImportBuffer(int fd) {
  if (ion_allocator_) {
    return ion_allocator_->ImportBuffer(fd);
  }
  return -EINVAL;
}

int Allocator::FreeBuffer(void *base, unsigned int size, unsigned int offset, int fd, int handle) {
  if (ion_allocator_) {
    return ion_allocator_->FreeBuffer(base, size, offset, fd, handle);
  }

  return -EINVAL;
}

int Allocator::CleanBuffer(void *base, unsigned int size, unsigned int offset, int handle, int op,
                           int fd) {
  if (ion_allocator_) {
    return ion_allocator_->CleanBuffer(base, size, offset, handle, op, fd);
  }

  return -EINVAL;
}

bool Allocator::CheckForBufferSharing(uint32_t num_descriptors,
                                      const vector<shared_ptr<BufferDescriptor>> &descriptors,
                                      ssize_t *max_index) {
  unsigned int cur_heap_id = 0, prev_heap_id = 0;
  unsigned int cur_alloc_type = 0, prev_alloc_type = 0;
  unsigned int cur_ion_flags = 0, prev_ion_flags = 0;
  bool cur_uncached = false, prev_uncached = false;
  unsigned int alignedw, alignedh;
  unsigned int max_size = 0;

  *max_index = -1;
  for (uint32_t i = 0; i < num_descriptors; i++) {
    // Check Cached vs non-cached and all the ION flags
    cur_uncached = UseUncached(descriptors[i]->GetFormat(), descriptors[i]->GetUsage());
    GetIonHeapInfo(descriptors[i]->GetUsage(), &cur_heap_id, &cur_alloc_type, &cur_ion_flags);

    if (i > 0 && (cur_heap_id != prev_heap_id || cur_alloc_type != prev_alloc_type ||
                  cur_ion_flags != prev_ion_flags)) {
      return false;
    }

    // For same format type, find the descriptor with bigger size
    GetAlignedWidthAndHeight(GetBufferInfo(*descriptors[i]), &alignedw, &alignedh);
    unsigned int size = GetSize(GetBufferInfo(*descriptors[i]), alignedw, alignedh);
    if (max_size < size) {
      *max_index = INT(i);
      max_size = size;
    }

    prev_heap_id = cur_heap_id;
    prev_uncached = cur_uncached;
    prev_ion_flags = cur_ion_flags;
    prev_alloc_type = cur_alloc_type;
  }

  return true;
}

void Allocator::GetIonHeapInfo(uint64_t usage, unsigned int *ion_heap_id, unsigned int *alloc_type,
                               unsigned int *ion_flags) {
  unsigned int heap_id = 0;
  unsigned int type = 0;
  uint32_t flags = 0;
#ifndef QMAA
  if (usage & GRALLOC_USAGE_PROTECTED) {
    if (usage & GRALLOC_USAGE_PRIVATE_SECURE_DISPLAY) {
      heap_id = ION_HEAP(SD_HEAP_ID);
      /*
       * There is currently no flag in ION for Secure Display
       * VM. Please add it to the define once available.
       */
      flags |= UINT(ION_SD_FLAGS);
    } else if (usage & BufferUsage::CAMERA_OUTPUT) {
      heap_id = ION_HEAP(SD_HEAP_ID);
      if (usage & GRALLOC_USAGE_PRIVATE_CDSP) {
        flags |= UINT(ION_SECURE | ION_FLAG_CP_CDSP);
      }
      if (usage & BufferUsage::COMPOSER_OVERLAY) {
        flags |= UINT(ION_SC_PREVIEW_FLAGS);
      } else {
        flags |= UINT(ION_SC_FLAGS);
      }
    } else if (usage & GRALLOC_USAGE_PRIVATE_CDSP) {
      heap_id = ION_HEAP(ION_SECURE_CARVEOUT_HEAP_ID);
      flags |= UINT(ION_SECURE | ION_FLAG_CP_CDSP);
    } else {
      heap_id = ION_HEAP(CP_HEAP_ID);
      flags |= UINT(ION_CP_FLAGS);
    }
  }

  if (usage & BufferUsage::SENSOR_DIRECT_DATA) {
      if (use_system_heap_for_sensors_) {
        ALOGI("gralloc::sns_direct_data with system_heap");
        heap_id |= ION_HEAP(ION_SYSTEM_HEAP_ID);
      } else {
        ALOGI("gralloc::sns_direct_data with adsp_heap");
        heap_id |= ION_HEAP(ION_ADSP_HEAP_ID);
      }
  }

  if (flags & UINT(ION_SECURE)) {
    type |= private_handle_t::PRIV_FLAGS_SECURE_BUFFER;
  }

  // if no ion heap flags are set, default to system heap
  if (!heap_id) {
    heap_id = ION_HEAP(ION_SYSTEM_HEAP_ID);
  }
#endif

  *alloc_type = type;
  *ion_flags = flags;
  *ion_heap_id = heap_id;

  return;
}
}  // namespace gralloc
