/*
* Copyright (c) 2014 - 2018, 2020 The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
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

#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <stdint.h>
#include <debug_handler.h>
#include <core/sdm_types.h>
#include <core/display_interface.h>
#include <display_properties.h>

namespace sdm {

using display::DebugHandler;

class Debug {
 public:
  static inline DebugHandler* Get() { return DebugHandler::Get(); }
  static int GetSimulationFlag();
  static bool GetExternalResolution(char *val);
  static void GetIdleTimeoutMs(uint32_t *active_ms, uint32_t *inactive_ms);
  static bool IsRotatorDownScaleDisabled();
  static bool IsDecimationDisabled();
  static int GetMaxPipesPerMixer(DisplayType display_type);
  static int GetMaxUpscale();
  static bool IsVideoModeEnabled();
  static bool IsRotatorUbwcDisabled();
  static bool IsRotatorSplitDisabled();
  static bool IsScalarDisabled();
  static bool IsUbwcTiledFrameBuffer();
  static bool IsAVRDisabled();
  static bool IsExtAnimDisabled();
  static bool IsPartialSplitDisabled();
  static bool IsSrcSplitPreferred();
  static DisplayError GetMixerResolution(uint32_t *width, uint32_t *height);
  static DisplayError GetWindowRect(float *left, float *top, float *right, float *bottom);
  static DisplayError GetReducedConfig(uint32_t *num_vig_pipes, uint32_t *num_dma_pipes);
  static int GetExtMaxlayers();
  static DisplayError GetProperty(const char *property_name, char *value);
  static DisplayError GetProperty(const char *property_name, int *value);
};

}  // namespace sdm

#endif  // __DEBUG_H__

