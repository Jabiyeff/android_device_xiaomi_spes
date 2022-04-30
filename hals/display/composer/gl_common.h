/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

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

#ifndef __GL_COMMON_H__
#define __GL_COMMON_H__

#include <sync/sync.h>
#include <utils/debug.h>
#include <vector>
#include <string>

#include "glengine.h"
#include "EGLImageWrapper.h"

namespace sdm {

struct GLRect {
  float left = 0.0f;
  float top = 0.0f;
  float right = 0.0f;
  float bottom = 0.0f;
};

struct GLContext {
  EGLDisplay egl_display = EGL_NO_DISPLAY;
  EGLContext egl_context = EGL_NO_CONTEXT;
  EGLSurface egl_surface = EGL_NO_SURFACE;
  uint32_t program_id = 0;
};

class GLCommon {
 public:
  virtual GLuint LoadProgram(int vertex_entries, const char **vertex, int fragment_entries,
                             const char **fragment);
  virtual void DumpShaderLog(int shader);
  virtual void MakeCurrent(const GLContext *ctx);
  virtual void SetProgram(uint32_t id);
  virtual void SetDestinationBuffer(const private_handle_t *dst_hnd);
  virtual void SetSourceBuffer(const private_handle_t *src_hnd);
  virtual void DestroyContext(GLContext *ctx);
  virtual void DeleteProgram(uint32_t id);
  virtual int WaitOnInputFence(const std::vector<shared_ptr<Fence>> &in_fences);
  virtual int CreateOutputFence(shared_ptr<Fence> *out_fence);
  virtual void ClearCache();
  virtual void SetRealTimePriority();
  virtual void SetViewport(const GLRect &dst_rect);

 protected:
  virtual ~GLCommon() { }

 private:
  EGLImageWrapper image_wrapper_;
};

}  // namespace sdm

#endif  // __GL_COMMON_H__
