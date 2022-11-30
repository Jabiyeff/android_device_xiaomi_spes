/*
* Copyright (c) 2019, The Linux Foundation. All rights reserved.
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

#ifndef __SNAPDRAGON_COLOR_INTF_H__
#define __SNAPDRAGON_COLOR_INTF_H__

#include <vector>
#include <array>
#include <tuple>
#include <functional>
#include <memory>
#include <string>
#include "display_color_processing.h"
#include "color_metadata.h"

namespace snapdragoncolor {

//<! Hardware assets strings
const std::string kPbGamut = "PostBlendGamut";
const std::string kPbIgc = "PostBlendIGC";
const std::string kPbGC = "PostBlendGC";

enum ScProperty {
  //<! Make value of zero as invalid.
  kInvalid = 0,
  //<! GetProperty - For client to Get list of supported ColorModes.
  //<! Payload -struct ColorModeList
  kModeList,
  //<! GetProperty - For client to check if Mode needs to be updated due to listener updates.
  //<! Payload - bool
  kNeedsUpdate,
  //<! ScOps - Prop for passing ModeRenderInputParams payload
  //<! Payload - ModeRenderInputParams
  kModeRenderInputParams,
  //<! ScOps - Prop for passing HwConfigOutputParams payload
  //<! Payload - HwConfigOutputParams
  kHwConfigPayloadParam,
  //<! SetProperty - Property to pass the ColorTransform matrix for display
  //<! Payload - struct ColorTransform
  kSetColorTransform,
  //<! SetProperty - Client will set the Gamut hw config data
  //<! Payload - PostBlendGamutHwConfig
  kPostBlendGamutHwConfig,
  //<! SetProperty - Client will set the Gamma hw config data
  //<! Payload - PostBlendGammaHwConfig
  kPostBlendGammaHwConfig,
  //<! SetProperty - Client will set the Inverse Gamma hw config data
  //<! Payload - PostBlendInverseGammaHwConfig
  kPostBlendInverseGammaHwConfig,
  //<! GetProperty - For client to check if library supports hdr tone-mapping.
  //<! Payload - bool
  kSupportToneMap,
  //<! Max value of public properties
  kPropertyMax = 511,
  //<! Custom Properties
  kCustomPropertyStart = 512,
  kCustomPropertyEnd = 1024,
};

enum ScOps {
  //<! Client will pass blend-space, render intent and color primaries along with hardware
  //<! assets that interface can use for generating the mode (ModeRenderInputParams).
  //<! Interface ProcessOps implementation will consume ModeRenderInputParams and generate
  //<! required mode and will update the OpsOutParams with HW asset configuration.
  kScModeRenderIntent,
  kScOpsMax,
};

static const uint32_t kMatrixSize = 4 * 4;
struct ColorTransform {
  const uint32_t version = sizeof(struct ColorTransform);
  std::array<float, kMatrixSize> coeff_array;
};

struct ScPayload {
  uint32_t version = sizeof(ScPayload);
  //<! len of the payload
  uint32_t len = 0;
  //<! ScProperty for the payload
  ScProperty prop = kInvalid;
  //<! payload pointer, has been made as uint64_t to support 32/64 bit platforms
  uint64_t payload = reinterpret_cast<uint64_t>(nullptr);
};


class ScPostBlendInterface {
 public:
  virtual ~ScPostBlendInterface() {}
  //<! Initialization related functions
  virtual int Init(const std::string &panel_name) = 0;
  virtual int DeInit() = 0;

  // Property functions
  virtual int SetProperty(const ScPayload &payload) = 0;
  virtual int GetProperty(ScPayload *payload) = 0;

  // ProcessOps functions
  virtual int ProcessOps(ScOps op, const ScPayload &input, ScPayload *output) = 0;
};

extern "C" ScPostBlendInterface* GetScPostBlendInterface(uint32_t major_version,
                                            uint32_t minor_version);

enum RenderIntent {
  //<! Colors with vendor defined gamut
  kNative,
  //<! Colors with in gamut are left untouched, out side the gamut are hard clipped
  kColorimetric,
  //<! Colors with in gamut are ehanced, out side the gamuat are hard clipped
  kEnhance,
  //<! Tone map hdr colors to display's dynamic range, mapping to display gamut is
  //<! defined in colormertic.
  kToneMapColorimetric,
  //<! Tone map hdr colors to display's dynamic range, mapping to display gamut is
  //<! defined in enhance.
  kToneMapEnhance,
  //<! Custom render intents range
  kOemCustomStart = 0x100,
  kOemCustomEnd = 0x1ff,
  kMaxRenderIntent = 0xffff
};

struct ColorMode {
  //<! Blend-Space gamut
  ColorPrimaries gamut;
  //<! Blend-space Gamma
  GammaTransfer gamma;
  //<! Intent of the mode
  RenderIntent intent;
  //<! Hardware assets needed for the mode
  std::vector<std::string> hw_assets;
};

struct ColorModeList {
  const uint32_t version = sizeof(struct ColorModeList) + sizeof(struct ColorMode);
  //<! List of the color modes
  std::vector<ColorMode> list;
};

struct GamutConfig {
  const uint32_t version = sizeof(struct GamutConfig);
  bool enabled = false;
  //<! Gamut table config
  lut3d_info gamut_info;
};

struct GammaPostBlendConfig {
  const uint32_t version = sizeof(struct GammaPostBlendConfig);
  bool enabled = false;
  //<! GC/IGC tables for each color component
  std::vector<uint32_t> r;
  std::vector<uint32_t> g;
  std::vector<uint32_t> b;
  explicit GammaPostBlendConfig(uint32_t len) {
    r.reserve(len);
    b.reserve(len);
    g.reserve(len);
  }
};

struct PostBlendGamutHwConfig {
  uint32_t gamut_version = sizeof(struct GamutConfig);
  uint32_t num_of_grid_entires = 17;
  uint32_t grid_entries_width = 10;
};

struct PostBlendGammaHwConfig {
  uint32_t gamma_version = sizeof(struct GammaPostBlendConfig);
  uint32_t num_of_entries = 1024;
  uint32_t entries_width = 10;
};

struct PostBlendInverseGammaHwConfig {
  uint32_t inverse_gamma_version = sizeof(struct GammaPostBlendConfig);
  uint32_t num_of_entries = 256;
  uint32_t entries_width = 12;
};

struct HwConfigPayload {
  const uint32_t version = sizeof(struct HwConfigPayload);
  //<! Memory for HW assets will be allocated by client
  //<! Hw asset
  std::string hw_asset;
  //<! payload for hw_asset
  std::shared_ptr<void> hw_payload;
  //<! payload len
  uint32_t hw_payload_len;
};

struct ModeRenderInputParams {
  const uint32_t version = sizeof(struct ModeRenderInputParams);
  //<! hw assets that shall be used for generating mode.
  //<! Optional parameter for now, added for future expansion.
  std::vector<std::string> hw_assets_for_mode;
  //<! Client will pass current blend space and render intent information.
  ColorMode color_mode;
  //<! flag will be true if we have valid color metadata(Ex: HDR10 playback)
  bool valid_meta_data;
  //<! if valid_meta_data is true, meta_data needs to be parsed.
  ColorMetaData meta_data;
  //<! QDCM color mode id.
  int32_t mode_id;
};

struct HwConfigOutputParams {
  const uint32_t version = sizeof(HwConfigOutputParams);
  //<! HW asset configurations that imp updates for kScModeRenderIntent ProcessOps call.
  std::vector<HwConfigPayload> payload;
};

}  // namespace snapdragoncolor
#endif  // __SNAPDRAGON_COLOR_INTF_H__
