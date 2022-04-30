/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
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

#include "gl_color_convert_impl.h"
#include "gl_color_convert.h"

#define __CLASS__ "GLColorConvert"

namespace sdm {

GLColorConvert* GLColorConvert::GetInstance(GLRenderTarget target, bool secure) {
  GLColorConvertImpl* color_convert = new GLColorConvertImpl(target, secure);
  if (color_convert == nullptr) {
    DLOGE("Failed to create color convert instance for %d target %d secure", target, secure);
    return nullptr;
  }

  int status = color_convert->Init();
  if (status != 0) {
    DLOGE("Failed to initialize GL Color convert instance %d", status);
    delete color_convert;
    return nullptr;
  }

  DLOGI("Created instance successfully");

  return color_convert;
}

void GLColorConvert::Destroy(GLColorConvert* intf) {
  GLColorConvertImpl* color_convert = static_cast<GLColorConvertImpl*>(intf);
  if (color_convert->Deinit() != 0) {
    DLOGE("De Init failed");
  }

  delete color_convert;
}

}  // namespace sdm
