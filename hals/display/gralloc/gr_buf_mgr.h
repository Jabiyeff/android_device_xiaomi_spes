/*
 * Copyright (c) 2011-2018, 2020 The Linux Foundation. All rights reserved.
 * Not a Contribution
 *
 * Copyright (C) 2008 The Android Open Source Project
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

#ifndef __GR_BUF_MGR_H__
#define __GR_BUF_MGR_H__

#include <pthread.h>

#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "gr_allocator.h"
#include "gr_buf_descriptor.h"
#include "gr_utils.h"
#include "gralloc_priv.h"

namespace gralloc {
using gralloc::Error;
class BufferManager {
 public:
  ~BufferManager();

  Error AllocateBuffer(const BufferDescriptor &descriptor, buffer_handle_t *handle,
                       unsigned int bufferSize = 0, bool testAlloc = false);
  Error RetainBuffer(private_handle_t const *hnd);
  Error ReleaseBuffer(private_handle_t const *hnd);
  Error LockBuffer(const private_handle_t *hnd, uint64_t usage);
  Error UnlockBuffer(const private_handle_t *hnd);
  Error Dump(std::ostringstream *os);
  void BuffersDump();
  Error ValidateBufferSize(private_handle_t const *hnd, BufferInfo info);
  Error IsBufferImported(const private_handle_t *hnd);
  static BufferManager *GetInstance();
  void SetGrallocDebugProperties(gralloc::GrallocProperties props);
  Error GetMetadata(private_handle_t *handle, int64_t metadatatype_value, hidl_vec<uint8_t> *out);
  Error SetMetadata(private_handle_t *handle, int64_t metadatatype_value, hidl_vec<uint8_t> in);
  Error GetReservedRegion(private_handle_t *handle, void **reserved_region,
                          uint64_t *reserved_region_size);
  Error FlushBuffer(const private_handle_t *handle);
  Error RereadBuffer(const private_handle_t *handle);
  Error GetAllHandles(std::vector<const private_handle_t *> *out_handle_list);

 private:
  BufferManager();
  Error MapBuffer(private_handle_t const *hnd);

  // Imports the ion fds into the current process. Returns an error for invalid handles
  Error ImportHandleLocked(private_handle_t *hnd);

  // Creates a Buffer from the valid private handle and adds it to the map
  void RegisterHandleLocked(const private_handle_t *hnd, int ion_handle, int ion_handle_meta);

  // Wrapper structure over private handle
  // Values associated with the private handle
  // that do not need to go over IPC can be placed here
  // This structure is also not expected to be ABI stable
  // unlike private_handle_t
  struct Buffer {
    const private_handle_t *handle = nullptr;
    int ref_count = 1;
    // Hold the main and metadata ion handles
    // Freed from the allocator process
    // and unused in the mapping process
    int ion_handle_main = -1;
    int ion_handle_meta = -1;

    Buffer() = delete;
    explicit Buffer(const private_handle_t *h, int ih_main = -1, int ih_meta = -1)
        : handle(h), ion_handle_main(ih_main), ion_handle_meta(ih_meta) {}
    void IncRef() { ++ref_count; }
    bool DecRef() { return --ref_count == 0; }
    uint64_t reserved_size = 0;
    void *reserved_region_ptr = nullptr;
  };

  Error FreeBuffer(std::shared_ptr<Buffer> buf);

  // Get the wrapper Buffer object from the handle, returns nullptr if handle is not found
  std::shared_ptr<Buffer> GetBufferFromHandleLocked(const private_handle_t *hnd);
  Allocator *allocator_ = NULL;
  std::mutex buffer_lock_;
  std::unordered_map<const private_handle_t *, std::shared_ptr<Buffer>> handles_map_ = {};
  std::atomic<uint64_t> next_id_;
  uint64_t allocated_ = 0;
  uint64_t kAllocThreshold = (uint64_t)2*1024*1024*1024;
  uint64_t kMemoryOffset = 50*1024*1024;
  struct {
    const char *kDumpFile = "/data/misc/wmtrace/bufferdump.txt";
    uint64_t position = 0;
  } file_dump_;
};

}  // namespace gralloc

#endif  // __GR_BUF_MGR_H__
