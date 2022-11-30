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

#include "gl_layer_stitch_impl.h"
#include "gl_layer_stitch.h"

#define __CLASS__ "GLLayerStitch"

namespace sdm {

GLLayerStitch* GLLayerStitch::GetInstance(bool secure) {
  GLLayerStitchImpl *layer_stitch = new GLLayerStitchImpl(secure);
  if (layer_stitch == nullptr) {
    DLOGE("Failed to create layer stitch instance. secure: %d", secure);
    return nullptr;
  }

  int status = layer_stitch->Init();
  if (status != 0) {
    DLOGE("Failed to initialize GL layer stitch instance %d", status);
    delete layer_stitch;
    return nullptr;
  }

  DLOGI("Created instance successfully");

  return layer_stitch;
}

void GLLayerStitch::Destroy(GLLayerStitch *intf) {
  GLLayerStitchImpl *layer_stitch = static_cast<GLLayerStitchImpl *>(intf);
  if (layer_stitch->Deinit() != 0) {
    DLOGE("De Init failed");
  }

  delete layer_stitch;
}

}  // namespace sdm
