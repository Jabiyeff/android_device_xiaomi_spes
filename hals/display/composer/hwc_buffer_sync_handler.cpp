/*
* Copyright (c) 2015, 2019-2020, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above
*    copyright notice, this list of conditions and the following
*    disclaimer in the documentation and/or other materials provided
*    with the distribution.
*  * Neither the name of The Linux Foundation nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
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

#include <errno.h>
#include <sync/sync.h>
#include <utils/constants.h>
#include <utils/debug.h>
#include <utils/fence.h>

#include "hwc_debugger.h"
#include "hwc_buffer_sync_handler.h"

#define __CLASS__ "HWCBufferSyncHandler"

namespace sdm {

HWCBufferSyncHandler HWCBufferSyncHandler::g_hwc_buffer_sync_handler_;

HWCBufferSyncHandler::HWCBufferSyncHandler() {
  Fence::Set(this);
}

DisplayError HWCBufferSyncHandler::SyncWait(int fd, int timeout) {
  // Assume invalid fd as signaled.
  if (fd < 0) {
    return kErrorNone;
  }

  int error = sync_wait(fd, timeout);
  if (!error) {
    return kErrorNone;
  }

  // Fence is not signaled yet.
  if (errno == ETIME) {
    return kErrorTimeOut;
  }

  DLOGW("sync_wait fd = %d, timeout = %d ms, err = %d : %s", fd, timeout, errno, strerror(errno));

  return kErrorUndefined;
}

DisplayError HWCBufferSyncHandler::SyncMerge(int fd1, int fd2, int *merged_fd) {
  // Caller owns fds, hence, if
  //  one of the fence fd is invalid, create dup of valid fd and set to merged fd.
  //  both fence fds are same, create dup of one of the fd and set to merged fd.
  *merged_fd = -1;
  if (fd1 < 0) {
    *merged_fd = dup(fd2);
  } else if ((fd2 < 0) || (fd1 == fd2)) {
    *merged_fd = dup(fd1);
  } else {
    *merged_fd = sync_merge("SyncMerge", fd1, fd2);
  }

  return kErrorNone;
}

void HWCBufferSyncHandler::GetSyncInfo(int fd, std::ostringstream *os) {
  struct sync_file_info *file_info = sync_file_info(fd);
  if (!file_info) {
    return;
  }

  *os << "SYNC_File status:: " << file_info->status;
  *os << ", name: " << file_info->name;
  *os << ", num_fences: " << file_info->num_fences;

  struct sync_fence_info *fence_info = sync_get_fence_info(file_info);
  if (!fence_info) {
    return;
  }

  for (size_t i = 0; i < file_info->num_fences; i++) {
    *os << ", fence[" << i << "]:: ";
    *os << "status: " << fence_info[i].status;
    *os << ", drv_name: " << fence_info[i].driver_name;
    *os << ", obj_name: " << fence_info[i].obj_name;
    *os << ", ts: " << fence_info[i].timestamp_ns;
  }
}

DisplayError HWCBufferSyncHandler::SyncWait(int fd) {
  // Deprecated.
  assert(false);
  return kErrorUndefined;
}

bool HWCBufferSyncHandler::IsSyncSignaled(int fd) {
  // Deprecated.
  assert(false);
  return false;
}

}  // namespace sdm
