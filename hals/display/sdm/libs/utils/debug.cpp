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

#include <stdlib.h>
#include <utils/debug.h>
#include <utils/constants.h>
#include <string>
#include <algorithm>

namespace sdm {

int Debug::GetSimulationFlag() {
  int value = 0;
  DebugHandler::Get()->GetProperty(COMPOSITION_MASK_PROP, &value);

  return value;
}

bool Debug::GetExternalResolution(char *value) {
  uint32_t retval = 0;
  DebugHandler::Get()->GetProperty(HDMI_CONFIG_INDEX_PROP, value);
  if (value[0]) {
    retval = 1;
  }

  return retval;
}

void Debug::GetIdleTimeoutMs(uint32_t *active_ms, uint32_t *inactive_ms) {
  int active_val = IDLE_TIMEOUT_ACTIVE_MS;
  int inactive_val = IDLE_TIMEOUT_INACTIVE_MS;

  DebugHandler::Get()->GetProperty(IDLE_TIME_PROP, &active_val);
  DebugHandler::Get()->GetProperty(IDLE_TIME_INACTIVE_PROP, &inactive_val);

  *active_ms = UINT32(active_val);
  *inactive_ms = UINT32(inactive_val);
}

bool Debug::IsRotatorDownScaleDisabled() {
  int value = 0;
  DebugHandler::Get()->GetProperty(DISABLE_ROTATOR_DOWNSCALE_PROP, &value);

  return (value == 1);
}

bool Debug::IsDecimationDisabled() {
  int value = 0;
  DebugHandler::Get()->GetProperty(DISABLE_DECIMATION_PROP, &value);

  return (value == 1);
}

int Debug::GetMaxPipesPerMixer(DisplayType display_type) {
  int value = -1;
  switch (display_type) {
    case kBuiltIn:
      DebugHandler::Get()->GetProperty(PRIMARY_MIXER_STAGES_PROP, &value);
      break;
    case kPluggable:
      DebugHandler::Get()->GetProperty(EXTERNAL_MIXER_STAGES_PROP, &value);
      break;
    case kVirtual:
      DebugHandler::Get()->GetProperty(VIRTUAL_MIXER_STAGES_PROP, &value);
      break;
    default:
      break;
  }

  return value;
}

int Debug::GetMaxUpscale() {
  int value = 0;
  DebugHandler::Get()->GetProperty(MAX_UPSCALE_PROP, &value);

  return value;
}

bool Debug::IsVideoModeEnabled() {
  int value = 0;
  DebugHandler::Get()->GetProperty(VIDEO_MODE_PANEL_PROP, &value);

  return (value == 1);
}

bool Debug::IsRotatorUbwcDisabled() {
  int value = 0;
  DebugHandler::Get()->GetProperty(DISABLE_ROTATOR_UBWC_PROP, &value);

  return (value == 1);
}

bool Debug::IsRotatorSplitDisabled() {
  int value = 0;
  DebugHandler::Get()->GetProperty(DISABLE_ROTATOR_SPLIT_PROP, &value);

  return (value == 1);
}

bool Debug::IsScalarDisabled() {
  int value = 0;
  DebugHandler::Get()->GetProperty(DISABLE_SCALER_PROP, &value);

  return (value == 1);
}

bool Debug::IsUbwcTiledFrameBuffer() {
  int ubwc_disabled = 0;

  DebugHandler::Get()->GetProperty(DISABLE_UBWC_PROP, &ubwc_disabled);

  return (ubwc_disabled == 0);
}

bool Debug::IsAVRDisabled() {
  int value = 0;
  DebugHandler::Get()->GetProperty(DISABLE_AVR_PROP, &value);

  return (value == 1);
}

bool Debug::IsExtAnimDisabled() {
  int value = 0;
  DebugHandler::Get()->GetProperty(DISABLE_EXTERNAL_ANIMATION_PROP, &value);

  return (value == 1);
}

bool Debug::IsPartialSplitDisabled() {
  int value = 0;
  DebugHandler::Get()->GetProperty(DISABLE_PARTIAL_SPLIT_PROP, &value);

  return (value == 1);
}

bool Debug::IsSrcSplitPreferred() {
  int value = 0;
  DebugHandler::Get()->GetProperty(PREFER_SOURCE_SPLIT_PROP, &value);

  return (value == 1);
}

DisplayError Debug::GetMixerResolution(uint32_t *width, uint32_t *height) {
  char value[64] = {};

  int error = DebugHandler::Get()->GetProperty(MIXER_RESOLUTION_PROP, value);
  if (error != 0) {
    return kErrorUndefined;
  }

  std::string str(value);

  *width = UINT32(stoi(str));
  *height = UINT32(stoi(str.substr(str.find('x') + 1)));

  return kErrorNone;
}

DisplayError Debug::GetWindowRect(float *left, float *top, float *right, float *bottom) {
  char value[64] = {};

  int error = DebugHandler::Get()->GetProperty(WINDOW_RECT_PROP, value);
  if (error != 0) {
    return kErrorUndefined;
  }

  std::string str(value);
  *left = FLOAT(stof(str));
  str = (str.substr(str.find(',') + 1));
  *top = FLOAT(stof(str));
  str = (str.substr(str.find(',') + 1));
  *right = FLOAT(stof(str));
  str = (str.substr(str.find(',') + 1));
  *bottom = FLOAT(stof(str));

  if (*left < 0 || *top < 0 || *right < 0 || *bottom < 0) {
    *left = *top = *right = *bottom = 0;
  }

  return kErrorNone;
}

DisplayError Debug::GetReducedConfig(uint32_t *num_vig_pipes, uint32_t *num_dma_pipes) {
  char value[64] = {};

  int error = DebugHandler::Get()->GetProperty(SIMULATED_CONFIG_PROP, value);
  if (error != 0) {
    return kErrorUndefined;
  }

  std::string str(value);

  *num_vig_pipes = UINT32(stoi(str));
  *num_dma_pipes = UINT32(stoi(str.substr(str.find('x') + 1)));

  return kErrorNone;
}

int Debug::GetExtMaxlayers() {
  int max_external_layers = 0;
  DebugHandler::Get()->GetProperty(MAX_EXTERNAL_LAYERS_PROP, &max_external_layers);

  return std::max(max_external_layers, 2);
}

DisplayError Debug::GetProperty(const char *property_name, char *value) {
  if (DebugHandler::Get()->GetProperty(property_name, value)) {
    return kErrorUndefined;
  }

  return kErrorNone;
}

DisplayError Debug::GetProperty(const char *property_name, int *value) {
  if (DebugHandler::Get()->GetProperty(property_name, value)) {
    return kErrorUndefined;
  }

  return kErrorNone;
}

}  // namespace sdm

