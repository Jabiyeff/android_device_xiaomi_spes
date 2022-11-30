/*
 * Copyright (c) 2011-2019, The Linux Foundation. All rights reserved.

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

#define DEBUG 0
#define ATRACE_TAG (ATRACE_TAG_GRAPHICS | ATRACE_TAG_HAL)
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <linux/dma-buf.h>
#include <ion/ion.h>
#include <stdlib.h>
#include <fcntl.h>
#include <log/log.h>
#include <cutils/trace.h>
#include <errno.h>
#include <utils/Trace.h>
#include <string>

#include "gr_utils.h"
#include "gralloc_priv.h"
#include "gr_ion_alloc.h"

namespace gralloc {

bool IonAlloc::Init() {
  if (ion_dev_fd_ == FD_INIT) {
    ion_dev_fd_ = OpenIonDevice();
  }

  if (ion_dev_fd_ < 0) {
    ALOGE("%s: Failed to open ion device - %s", __FUNCTION__, strerror(errno));
    ion_dev_fd_ = FD_INIT;
    return false;
  }

  return true;
}

int IonAlloc::OpenIonDevice() {
  return ion_open();
}

void IonAlloc::CloseIonDevice() {
  if (ion_dev_fd_ > FD_INIT) {
    ion_close(ion_dev_fd_);
  }

  ion_dev_fd_ = FD_INIT;
}

int IonAlloc::AllocBuffer(AllocData *data) {
  ATRACE_CALL();
  int err = 0;
  int fd = -1;
  unsigned int flags = data->flags;

  flags |= data->uncached ? 0 : ION_FLAG_CACHED;

  std::string tag_name{};
  if (ATRACE_ENABLED()) {
    tag_name = "libion alloc size: " + std::to_string(data->size);
  }

  ATRACE_BEGIN(tag_name.c_str());
  err = ion_alloc_fd(ion_dev_fd_, data->size, data->align, data->heap_id, flags, &fd);
  ATRACE_END();
  if (err) {
    ALOGE("libion alloc failed ion_fd %d size %d align %d heap_id %x flags %x",
          ion_dev_fd_, data->size, data->align, data->heap_id, flags);
    return err;
  }

  data->fd = fd;
  data->ion_handle = fd;  // For new ion api ion_handle does not exists so reusing fd for now
  ALOGD_IF(DEBUG, "libion: Allocated buffer size:%u fd:%d", data->size, data->fd);

  return 0;
}

int IonAlloc::FreeBuffer(void *base, unsigned int size, unsigned int offset, int fd,
                         int /*ion_handle*/) {
  ATRACE_CALL();
  int err = 0;
  ALOGD_IF(DEBUG, "libion: Freeing buffer base:%p size:%u fd:%d", base, size, fd);

  if (base) {
    err = UnmapBuffer(base, size, offset);
  }

  close(fd);
  return err;
}

int IonAlloc::ImportBuffer(int fd) {
  // For new ion api ion_handle does not exists so reusing fd for now
  return fd;
}

int IonAlloc::CleanBuffer(void */*base*/, unsigned int /*size*/, unsigned int /*offset*/,
                          int /*handle*/, int op, int dma_buf_fd) {
  ATRACE_CALL();
  ATRACE_INT("operation id", op);

  struct dma_buf_sync sync;
  int err = 0;

  switch (op) {
    case CACHE_CLEAN:
      sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;
      break;
    case CACHE_INVALIDATE:
      sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;
      break;
    case CACHE_READ_DONE:
      sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
      break;
    default:
      ALOGE("%s: Invalid operation %d", __FUNCTION__, op);
      return -1;
  }

  if (ioctl(dma_buf_fd, INT(DMA_BUF_IOCTL_SYNC), &sync)) {
    err = -errno;
    ALOGE("%s: DMA_BUF_IOCTL_SYNC failed with error - %s", __FUNCTION__, strerror(errno));
    return err;
  }

  return 0;
}

int IonAlloc::MapBuffer(void **base, unsigned int size, unsigned int offset, int fd) {
  ATRACE_CALL();
  int err = 0;
  void *addr = 0;

  addr = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  *base = addr;
  if (addr == MAP_FAILED) {
    err = -errno;
    ALOGE("ion: Failed to map memory in the client: %s", strerror(errno));
  } else {
    ALOGD_IF(DEBUG, "ion: Mapped buffer base:%p size:%u offset:%u fd:%d", addr, size, offset, fd);
  }

  return err;
}

int IonAlloc::UnmapBuffer(void *base, unsigned int size, unsigned int /*offset*/) {
  ATRACE_CALL();
  ALOGD_IF(DEBUG, "ion: Unmapping buffer  base:%p size:%u", base, size);

  int err = 0;
  if (munmap(base, size)) {
    err = -errno;
    ALOGE("ion: Failed to unmap memory at %p : %s", base, strerror(errno));
  }

  return err;
}

}  // namespace gralloc
