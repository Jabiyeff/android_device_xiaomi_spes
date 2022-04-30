/*
* Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification, are permitted
* provided that the following conditions are met:
*    * Redistributions of source code must retain the above copyright notice, this list of
*      conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright notice, this list of
*      conditions and the following disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of The Linux Foundation nor the names of its contributors may be used to
*      endorse or promote products derived from this software without specific prior written
*      permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __HW_INFO_TYPES_H__
#define __HW_INFO_TYPES_H__

#include <stdint.h>
#include <core/display_interface.h>
#include <core/core_interface.h>
#include <utils/locker.h>
#include <utils/fence.h>
#include <utils/debug.h>
#include <bitset>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <utility>

namespace sdm {
using std::string;
using std::pair;
using std::vector;

const int kMaxSDELayers = 16;   // Maximum number of layers that can be handled by MDP5 hardware
                                // in a given layer stack.
#define MAX_PLANES 4
#define MAX_DETAIL_ENHANCE_CURVE 3
#define MAJOR 28
#define MINOR 16
#define SDEVERSION(major, minor, hw_rev) ((major) << MAJOR) | ((minor) << MINOR) | (hw_rev)

// CSC Max Size
#define MAX_CSC_MATRIX_COEFF_SIZE   9
#define MAX_CSC_CLAMP_SIZE          6
#define MAX_CSC_BIAS_SIZE           3

#define MAX_SPLIT_COUNT             2

enum HWDeviceType {
  kDeviceBuiltIn,
  kDevicePluggable,
  kDeviceVirtual,
  kDeviceRotator,
  kDeviceMax,
};

enum HWBlockType {
  kHWBuiltIn,
  kHWPluggable,
  kHWWriteback0,
  kHWWriteback1,
  kHWWriteback2,
  kHWBlockMax
};

enum HWDisplayMode {
  kModeDefault,
  kModeVideo,
  kModeCommand,
};

enum PipeType {
  kPipeTypeUnused,
  kPipeTypeVIG,
  kPipeTypeRGB,
  kPipeTypeDMA,
  kPipeTypeCursor,
};

enum HWSubBlockType {
  kHWVIGPipe,
  kHWRGBPipe,
  kHWDMAPipe,
  kHWCursorPipe,
  kHWRotatorInput,
  kHWRotatorOutput,
  kHWWBIntfOutput,
  kHWDestinationScalar,
  kHWSubBlockMax,
};

enum HWAlphaInterpolation {
  kInterpolationPixelRepeat,
  kInterpolationBilinear,
  kInterpolation2D,
  kInterpolationMax,
};

enum HWBlendingFilter {
  kBlendFilterCircular,
  kBlendFilterSeparable,
  kBlendFilterMax,
};

enum HWPipeFlags {
  kIGC = 1 << 0,
  kMultiRect = 1 << 1,
  kMultiRectParallelMode = 1 << 2,
  kFlipVertical = 1 << 5,
  kFlipHorizontal = 1 << 6,
  kSecure = 1 << 7,
  kDisplaySecure = 1 << 8,
  kCameraSecure = 1 << 9,
  kInterlaced = 1 << 10,
  kUpdating = 1 < 11,
  kSolidFill = 1 << 12,
  kTonemap1d = 1 << 13,
  kTonemap3d = 1 << 14,
};

enum HWAVRModes {
  kQsyncNone,       // Disables Qsync.
  kContinuousMode,  // Mode to enable AVR feature for every frame.
  kOneShotMode,     // Mode to enable AVR feature for particular frame.
};

enum HWTopology {
  kUnknown,
  kSingleLM,
  kSingleLMDSC,
  kDualLM,
  kDualLMDSC,
  kDualLMMerge,
  kDualLMMergeDSC,
  kDualLMDSCMerge,
  kQuadLMMerge,
  kQuadLMDSCMerge,
  kQuadLMMergeDSC,
  kPPSplit,
};

enum HWMixerSplit {
  kNoSplit,
  kDualSplit,
  kQuadSplit,
};

enum HwHdrEotf {
  kHdrEOTFInvalid = 0,
  kHdrEOTFSDR = 0x1,
  kHdrEOTFHdrLumRange = 0x2,
  kHdrEOTFHDR10 = 0x4,
  kHdrEOTFHLG = 0x8,
};

enum HwColorspace {
  kColorspaceXvycc601     = (1 << 0),
  kColorspaceXvycc709     = (1 << 1),
  kColorspaceSycc601      = (1 << 2),
  kColorspaceAdobeycc601  = (1 << 3),
  kColorspaceAdobergb     = (1 << 4),
  kColorspaceBt2020cycc   = (1 << 5),
  kColorspaceBt2020ycc    = (1 << 6),
  kColorspaceBt2020rgb    = (1 << 7),
  kColorspaceDcip3        = (1 << 15)
};

enum HWSrcTonemap {
  kSrcTonemapNone,
  kSrcTonemap1d,  // DMA
  kSrcTonemap3d,  // VIG
};

enum HWToneMapLut {
  kLutNone,     // No valid lut
  kDma1dIgc,    // DMA IGC Lut
  kDma1dGc,     // DMA GC Lut
  kVig1dIgc,    // VIG IGC Lut
  kVig3dGamut,  // 3D Gamut Lut
};

enum HWWriteOperation {
  kNoOp,   // No-op, previously set config holds good
  kSet,    // Sets the new config
  kReset,  // Resets/Clears the previously set config
};

enum class HWRecoveryEvent : uint32_t {
  kSuccess,            // driver succeeded recovery
  kCapture,            // driver PP_TIMEOUT, capture logs
  kDisplayPowerReset,  // driver requesting display power cycle
};

typedef std::map<HWSubBlockType, std::vector<LayerBufferFormat>> FormatsMap;
typedef std::map<LayerBufferFormat, float> CompRatioMap;

// Base Postprocessing features information.
class PPFeatureInfo {
 public:
  uint32_t enable_flags_ = 0;  // bitmap to indicate subset of parameters enabling or not.
  uint32_t feature_version_ = 0;
  uint32_t feature_id_ = 0;
  uint32_t disp_id_ = 0;
  uint32_t pipe_id_ = 0;

  virtual ~PPFeatureInfo() {}
  virtual void *GetConfigData(void) const = 0;
};

struct HWDynBwLimitInfo {
  uint32_t cur_mode = kBwVFEOn;
  uint64_t total_bw_limit[kBwModeMax] = { 0 };
  uint64_t pipe_bw_limit[kBwModeMax] = { 0 };
};

struct HWPipeCaps {
  PipeType type = kPipeTypeUnused;
  uint32_t id = 0;
  uint32_t master_pipe_id = 0;
  uint32_t max_rects = 1;
  bool inverse_pma = 0;
  uint32_t dgm_csc_version = 0;
  std::map<HWToneMapLut, uint32_t> tm_lut_version_map = {};
  bool block_sec_ui = false;
};

struct HWRotatorInfo {
  uint32_t num_rotator = 0;
  bool has_downscale = false;
  std::string device_path = "";
  float min_downscale = 2.0f;
  bool downscale_compression = false;
  uint32_t max_line_width = 0;
};

enum HWQseedStepVersion {
  kQseed3v2,
  kQseed3v3,
  kQseed3v4,
  kQseed3litev4,
  kQseed3litev5,
};

struct HWDestScalarInfo {
  uint32_t count = 0;
  uint32_t max_input_width = 0;
  uint32_t max_output_width = 0;
  uint32_t max_scale_up = 1;
  uint32_t prefill_lines = 4;
};

enum SmartDMARevision {
  V1,
  V2,
  V2p5
};

enum InlineRotationVersion {
  kInlineRotationNone,
  kInlineRotationV1,
  kInlineRotationV2,
};

struct InlineRotationInfo {
  InlineRotationVersion inrot_version = kInlineRotationNone;
  std::vector<LayerBufferFormat> inrot_fmts_supported;
  float max_downscale_rt = 2.2f;    // max downscale real time display
  float max_ds_without_pre_downscaler = 2.2f;
};


const int  kPipeVigLimit       = (1 << 0);
const int  kPipeDmaLimit       = (1 << 1);
const int  kPipeScalingLimit   = (1 << 2);
const int  kPipeRotationLimit  = (1 << 3);

struct HWResourceInfo {
  uint32_t hw_version = 0;
  uint32_t num_dma_pipe = 0;
  uint32_t num_vig_pipe = 0;
  uint32_t num_rgb_pipe = 0;
  uint32_t num_cursor_pipe = 0;
  uint32_t num_blending_stages = 0;
  uint32_t num_solidfill_stages = 0;
  uint32_t max_scale_up = 1;
  uint32_t max_scale_down = 1;
  uint64_t max_bandwidth_low = 0;
  uint64_t max_bandwidth_high = 0;
  uint32_t max_mixer_width = 2048;
  uint32_t max_pipe_width = 2048;
  uint32_t max_scaler_pipe_width = 2560;
  uint32_t max_rotation_pipe_width = 1088;
  uint32_t max_cursor_size = 0;
  uint64_t max_pipe_bw =  0;
  uint64_t max_pipe_bw_high = 0;
  uint32_t max_sde_clk = 0;
  float clk_fudge_factor = 1.0f;
  uint32_t macrotile_nv12_factor = 0;
  uint32_t macrotile_factor = 0;
  uint32_t linear_factor = 0;
  uint32_t scale_factor = 0;
  uint32_t extra_fudge_factor = 0;
  uint32_t amortizable_threshold = 0;
  uint32_t system_overhead_lines = 0;
  bool has_ubwc = false;
  bool has_decimation = false;
  bool has_non_scalar_rgb = false;
  bool is_src_split = false;
  bool separate_rotator = false;
  bool has_qseed3 = false;
  bool has_concurrent_writeback = false;
  bool has_ppp = false;
  bool has_excl_rect = false;
  uint32_t writeback_index = kHWBlockMax;
  HWDynBwLimitInfo dyn_bw_info;
  std::vector<HWPipeCaps> hw_pipes;
  FormatsMap supported_formats_map;
  HWRotatorInfo hw_rot_info;
  HWDestScalarInfo hw_dest_scalar_info;
  bool has_hdr = false;
  SmartDMARevision smart_dma_rev = SmartDMARevision::V1;
  float ib_fudge_factor = 1.0f;
  uint32_t undersized_prefill_lines = 0;
  CompRatioMap comp_ratio_rt_map;
  CompRatioMap comp_ratio_nrt_map;
  uint32_t cache_size = 0;  // cache size in bytes
  HWQseedStepVersion pipe_qseed3_version = kQseed3v2;  // only valid when has_qseed3=true
  uint32_t min_prefill_lines = 0;
  InlineRotationInfo inline_rot_info = {};
  std::bitset<32> src_tone_map = 0;  //!< Stores the bit mask of src tone map capability
  int secure_disp_blend_stage = -1;
  uint32_t line_width_constraints_count = 0;
  vector< pair <uint32_t, uint32_t> > line_width_limits;
  vector< pair <uint32_t, uint32_t> > line_width_constraints;
  uint32_t num_mnocports = 2;
  uint32_t mnoc_bus_width = 32;
  bool use_baselayer_for_stage = false;
  bool has_micro_idle = false;
  uint32_t ubwc_version = 1;
  uint32_t rc_total_mem_size = 0;
};

struct HWSplitInfo {
  uint32_t left_split = 0;
  uint32_t right_split = 0;

  bool operator !=(const HWSplitInfo &split_info) {
    return ((left_split != split_info.left_split) || (right_split != split_info.right_split));
  }

  bool operator ==(const HWSplitInfo &split_info) {
    return !(operator !=(split_info));
  }
};

struct HWColorPrimaries {
  uint32_t white_point[2] = {};       // White point
  uint32_t red[2] = {};               // Red color primary
  uint32_t green[2] = {};             // Green color primary
  uint32_t blue[2] = {};              // Blue color primary
};

struct HWPanelOrientation {
  bool rotation = false;
  bool flip_horizontal = false;
  bool flip_vertical = false;
};

struct HWPanelInfo {
  DisplayPort port = kPortDefault;    // Display port
  HWDisplayMode mode = kModeDefault;  // Display mode
  bool partial_update = false;        // Partial update feature
  int left_align = 1;                 // ROI left alignment restriction
  int width_align = 1;                // ROI width alignment restriction
  int top_align = 1;                  // ROI top alignment restriction
  int height_align = 1;               // ROI height alignment restriction
  int min_roi_width = 1;              // Min width needed for ROI
  int min_roi_height = 1;             // Min height needed for ROI
  bool needs_roi_merge = false;       // Merge ROI's of both the DSI's
  bool dynamic_fps = false;           // Panel Supports dynamic fps
  bool dfps_porch_mode = false;       // dynamic fps VFP or HFP mode
  bool ping_pong_split = false;       // Supports Ping pong split
  uint32_t min_fps = 0;               // Min fps supported by panel
  uint32_t max_fps = 0;               // Max fps supported by panel
  bool is_primary_panel = false;      // Panel is primary display
  bool is_pluggable = false;          // Panel is pluggable
  HWSplitInfo split_info;             // Panel split configuration
  char panel_name[256] = {0};         // Panel name
  float panel_max_brightness = 255.0f;  // Max panel brightness
  float panel_min_brightness = 1.0f;  // Min panel brightness
  uint32_t left_roi_count = 1;        // Number if ROI supported on left panel
  uint32_t right_roi_count = 1;       // Number if ROI supported on right panel
  bool hdr_enabled = false;           // HDR feature supported
  bool hdr_plus_enabled = false;      // HDR10+ feature supported
  bool hdr_metadata_type_one = false;     // Static HDR metadata type one
  uint32_t hdr_eotf = 0;              // Electro optical transfer function
  float peak_luminance = 0.0f;        // Panel's peak luminance level
  float average_luminance = 0.0f;     // Panel's average luminance level
  float blackness_level = 0.0f;       // Panel's blackness level
  HWColorPrimaries primaries = {};    // WRGB color primaries
  HWPanelOrientation panel_orientation = {};  // Panel Orientation
  uint32_t transfer_time_us = 0;       // transfer time in micro seconds to panel's active region
  bool qsync_support = false;          // Specifies panel supports qsync feature or not.
  bool dyn_bitclk_support = false;     // Bit clk can be updated to avoid RF interference.
  std::vector<uint64_t> bitclk_rates;  // Supported bit clk levels.
  uint32_t supported_colorspaces = 0;  // supported_colorspaces for DP displays.

  bool operator !=(const HWPanelInfo &panel_info) {
    return ((port != panel_info.port) || (mode != panel_info.mode) ||
            (partial_update != panel_info.partial_update) ||
            (left_align != panel_info.left_align) || (width_align != panel_info.width_align) ||
            (top_align != panel_info.top_align) || (height_align != panel_info.height_align) ||
            (min_roi_width != panel_info.min_roi_width) ||
            (min_roi_height != panel_info.min_roi_height) ||
            (needs_roi_merge != panel_info.needs_roi_merge) ||
            (dynamic_fps != panel_info.dynamic_fps) || (min_fps != panel_info.min_fps) ||
            (dfps_porch_mode != panel_info.dfps_porch_mode) ||
            (ping_pong_split != panel_info.ping_pong_split) ||
            (max_fps != panel_info.max_fps) || (is_primary_panel != panel_info.is_primary_panel) ||
            (split_info != panel_info.split_info) ||
            (left_roi_count != panel_info.left_roi_count) ||
            (right_roi_count != panel_info.right_roi_count) ||
            (transfer_time_us != panel_info.transfer_time_us) ||
            (qsync_support != panel_info.qsync_support) ||
            (dyn_bitclk_support != panel_info.dyn_bitclk_support) ||
            (bitclk_rates != panel_info.bitclk_rates));
  }

  bool operator ==(const HWPanelInfo &panel_info) {
    return !(operator !=(panel_info));
  }
};

struct HWSessionConfig {
  LayerRect src_rect {};
  LayerRect dst_rect {};
  uint32_t buffer_count = 0;
  bool secure = false;
  uint32_t frame_rate = 0;
  LayerTransform transform;
  bool secure_camera = false;

  bool operator==(const HWSessionConfig& config) const {
    return (src_rect == config.src_rect &&
            dst_rect == config.dst_rect &&
            buffer_count == config.buffer_count &&
            secure == config.secure &&
            frame_rate == config.frame_rate &&
            transform == config.transform &&
            secure_camera == config.secure_camera);
  }

  bool operator!=(const HWSessionConfig& config) const {
    return !operator==(config);
  }
};

enum HWRotatorMode {
  kRotatorNone,
  kRotatorOffline,
  kRotatorInline
};

struct HWRotateInfo {
  int pipe_id = -1;  // Not actual pipe id, but the relative DMA id
  int writeback_id = -1;  // Writeback block id, but this is the same as DMA id
  LayerRect src_roi {};  // Source crop of each split
  LayerRect dst_roi {};  // Destination crop of each split
  bool valid = false;
  int rotate_id = -1;  // Actual rotator session id with driver
};

struct HWRotatorSession {
  HWRotateInfo hw_rotate_info[kMaxRotatePerLayer] {};
  uint32_t hw_block_count = 0;  // number of rotator hw blocks used by rotator session
  int session_id = -1;  // A handle with Session Manager
  HWSessionConfig hw_session_config {};
  LayerBuffer input_buffer {};  // Input to rotator
  LayerBuffer output_buffer {};  // Output of rotator, crop width and stride are same
  float input_compression = 1.0f;
  float output_compression = 1.0f;
  bool is_buffer_cached = false;
  HWRotatorMode mode = kRotatorNone;
};

struct HWScaleLutInfo {
  uint32_t dir_lut_size = 0;
  uint32_t cir_lut_size = 0;
  uint32_t sep_lut_size = 0;
  uint64_t dir_lut = 0;
  uint64_t cir_lut = 0;
  uint64_t sep_lut = 0;
};

struct HWDetailEnhanceData : DisplayDetailEnhancerData {
  uint16_t prec_shift = 0;
  int16_t adjust_a[MAX_DETAIL_ENHANCE_CURVE] = {0};
  int16_t adjust_b[MAX_DETAIL_ENHANCE_CURVE] = {0};
  int16_t adjust_c[MAX_DETAIL_ENHANCE_CURVE] = {0};
};

struct HWPixelExtension {
  int32_t extension = 0;  // Number of pixels extension in left, right, top and bottom directions
                          // for all color components. This pixel value for each color component
                          // should be sum of fetch and repeat pixels.

  int32_t overfetch = 0;  // Number of pixels need to be overfetched in left, right, top and bottom
                          // directions from source image for scaling.

  int32_t repeat = 0;     // Number of pixels need to be repeated in left, right, top and bottom
                          // directions for scaling.
};

struct HWPlane {
  int32_t init_phase_x = 0;
  int32_t phase_step_x = 0;
  int32_t init_phase_y = 0;
  int32_t phase_step_y = 0;
  HWPixelExtension left {};
  HWPixelExtension top {};
  HWPixelExtension right {};
  HWPixelExtension bottom {};
  uint32_t roi_width = 0;
  int32_t preload_x = 0;
  int32_t preload_y = 0;
  uint32_t src_width = 0;
  uint32_t src_height = 0;
};

struct HWCsc {
  int64_t ctm_coeff[MAX_CSC_MATRIX_COEFF_SIZE] = {0};
  uint32_t pre_bias[MAX_CSC_BIAS_SIZE] = {0};
  uint32_t post_bias[MAX_CSC_BIAS_SIZE] = {0};
  uint32_t pre_clamp[MAX_CSC_CLAMP_SIZE] = {0};
  uint32_t post_clamp[MAX_CSC_CLAMP_SIZE] = {0};
};

struct HWScaleData {
  struct enable {
    uint8_t scale = 0;
    uint8_t direction_detection = 0;
    uint8_t detail_enhance = 0;
    uint8_t dyn_exp_disable = 0;     // RGB format dynamic pixel expansion disable.
  } enable;
  uint32_t dst_width = 0;
  uint32_t dst_height = 0;
  uint32_t dir_weight = 0;
  HWPlane plane[MAX_PLANES] {};
  // scale_v2_data fields
  ScalingFilterConfig y_rgb_filter_cfg = kFilterEdgeDirected;
  ScalingFilterConfig uv_filter_cfg = kFilterEdgeDirected;
  HWAlphaInterpolation alpha_filter_cfg = kInterpolationPixelRepeat;
  HWBlendingFilter blend_cfg = kBlendFilterCircular;

  struct lut_flags {
    uint8_t lut_swap = 0;
    uint8_t lut_dir_wr = 0;
    uint8_t lut_y_cir_wr = 0;
    uint8_t lut_uv_cir_wr = 0;
    uint8_t lut_y_sep_wr = 0;
    uint8_t lut_uv_sep_wr = 0;
  } lut_flag;

  uint32_t dir_lut_idx = 0;
  /* for Y(RGB) and UV planes*/
  uint32_t y_rgb_cir_lut_idx = 0;
  uint32_t uv_cir_lut_idx = 0;
  uint32_t y_rgb_sep_lut_idx = 0;
  uint32_t uv_sep_lut_idx = 0;
  HWDetailEnhanceData detail_enhance {};

  uint32_t src_x_pre_down_scale_0 = 0;
  uint32_t src_x_pre_down_scale_1 = 0;
  uint32_t src_y_pre_down_scale_0 = 0;
  uint32_t src_y_pre_down_scale_1 = 0;
};

struct HWDestScaleInfo {
  uint32_t mixer_width = 0;
  uint32_t mixer_height = 0;
  bool scale_update = false;
  HWScaleData scale_data = {};
  LayerRect panel_roi = {};
};

typedef std::map<uint32_t, HWDestScaleInfo *> DestScaleInfoMap;

struct HWAVRInfo {
  bool update = false;                // Update avr setting.
  HWAVRModes mode = kContinuousMode;  // Specifies the AVR mode
};

struct HWPipeCscInfo {
  HWWriteOperation op = kNoOp;
  HWCsc csc = {};
};

struct HWPipeTonemapLutInfo {
  HWWriteOperation op = kNoOp;
  HWToneMapLut type = kLutNone;
  std::shared_ptr<PPFeatureInfo> pay_load = nullptr;
};

struct HWPipeTonemapInversePma {
  HWWriteOperation op = kNoOp;
  bool inverse_pma = false;
};

struct HWPipeInfo {
  HWPipeInfo *pair = NULL;
  uint8_t rect = 255;
  uint32_t pipe_id = 0;
  HWSubBlockType sub_block_type = kHWSubBlockMax;
  LayerRect src_roi {};
  LayerRect dst_roi {};
  LayerRect excl_rect {};  // exclusion rectangle per pipe rectangle
  uint8_t horizontal_decimation = 0;
  uint8_t vertical_decimation = 0;
  HWScaleData scale_data {};
  uint32_t z_order = 0;
  uint32_t flags = 0;
  bool valid = false;
  bool is_virtual = 0;
  HWPipeTonemapInversePma inverse_pma_info = {};
  HWPipeCscInfo dgm_csc_info = {};
  std::vector<HWPipeTonemapLutInfo> lut_info = {};
  LayerTransform transform;
  HWSrcTonemap tonemap = kSrcTonemapNone;
  LayerBufferFormat format = kFormatARGB8888;  // src format of the buffer
  bool is_solid_fill = false;
};

struct HWSolidfillStage {
  uint32_t z_order = kMaxSDELayers;
  uint32_t color = 0;
  LayerRect roi = {};
  bool is_exclusion_rect = false;
  LayerSolidFill solid_fill_info = {};
};

struct HWLayerConfig {
  HWPipeInfo left_pipe {};           // pipe for left side of output
  HWPipeInfo right_pipe {};          // pipe for right side of output
  HWRotatorSession hw_rotator_session {};
  bool use_inline_rot = false;             // keep track of which layers inline rotation
  HWSolidfillStage hw_solidfill_stage {};
  float compression = 1.0f;
  bool use_solidfill_stage = false;
};

struct HWHDRLayerInfo {
  enum HDROperation {
    kNoOp,   // No-op.
    kSet,    // Sets the HDR MetaData - Start of HDR
    kReset,  // resets the previously set HDR Metadata, End of HDR
  };

  int32_t layer_index = -1;
  HDROperation operation = kNoOp;
  bool in_hdr_mode = false;  // True if already in HDR mode with static metadata.
  bool blend_space_layer_changed = false;  // True if HDR layer's index changed.
  std::set<uint32_t> hdr_layers;  // Non-tonemapped HDR layer indices.
  std::vector<uint8_t> dyn_hdr_vsif_payload;  // Dynamic HDR VSIF data.
};

struct RCLayersInfo {
  int top_width = 0;
  int top_height = 0;
  int bottom_width = 0;
  int bottom_height = 0;
};

struct LayerExt {
  std::vector<LayerRect> excl_rects = {};  // list of exclusion rects
};

struct HWLayersInfo {
  LayerStack *stack = NULL;          // Input layer stack. Set by the caller.
  uint32_t app_layer_count = 0;      // Total number of app layers. Must not be 0.
  uint32_t gpu_target_index = 0;     // GPU target layer index. 0 if not present.
  uint32_t stitch_target_index = 0;  // Blit target layer index. 0 if not present.
  std::vector<ColorPrimaries> wide_color_primaries = {};  // list of wide color primaries

  std::vector<Layer> hw_layers = {};  // Layers which need to be programmed on the HW
  std::vector<LayerExt> layer_exts = {};  // Extention layer having list of
                                          // exclusion rectangles for each layer
  std::vector<uint32_t> index {};   // Indexes of the layers from the layer stack which need to
                                 // be programmed on hardware.
  std::vector<uint32_t> roi_index {};  // Stores the ROI index where the layers are visible.
  shared_ptr<Fence> sync_handle = nullptr;  // Release fence id for current draw cycle.
  int set_idle_time_ms = -1;    // Set idle time to the new specified value.
                                //    -1 indicates no change in idle time since last set value.
  std::vector<LayerRect> left_frame_roi = {};   // Left ROI.
  std::vector<LayerRect> right_frame_roi = {};  // Right ROI.
  LayerRect partial_fb_roi = {};   // Damaged area in framebuffer.
  bool roi_split = false;          // Indicates separated left and right ROI
  bool async_cursor_updates = false;  // Cursor layer allowed to have async updates
  bool fast_path_composition = false;  // Indicates frame has fast path composition
  DestScaleInfoMap dest_scale_info_map = {};
  HWHDRLayerInfo hdr_layer_info = {};
  Handle pvt_data = NULL;   // Private data used by sdm extension only.
  bool game_present = false;  // Indicates there is game layer or not
  bool rc_config = false;
  RCLayersInfo rc_layers_info = {};
};

struct HWQosData {
  uint64_t core_ab_bps = 0;
  uint64_t core_ib_bps = 0;
  uint64_t llcc_ab_bps = 0;
  uint64_t llcc_ib_bps = 0;
  uint64_t dram_ab_bps = 0;
  uint64_t dram_ib_bps = 0;
  uint64_t rot_prefill_bw_bps = 0;
  uint32_t clock_hz = 0;
  uint32_t rot_clock_hz = 0;
};

enum UpdateType {
  kUpdateResources,  // Indicates Strategy & RM execution, which can update resources.
  kSwapBuffers,      // Indicates Strategy & RM execution, which can update buffer handler and crop.
  kUpdateMax,
};

struct HWLayers {
  HWLayersInfo info {};
  HWLayerConfig config[kMaxSDELayers] {};
  float output_compression = 1.0f;
  HWQosData qos_data = {};
  HWAVRInfo hw_avr_info = {};
  std::bitset<kUpdateMax> updates_mask = 0;
  uint64_t elapse_timestamp = 0;
};

struct HWDisplayAttributes : DisplayConfigVariableInfo {
  bool is_device_split = false;
  uint32_t v_front_porch = 0;  //!< Vertical front porch of panel
  uint32_t v_back_porch = 0;   //!< Vertical back porch of panel
  uint32_t v_pulse_width = 0;  //!< Vertical pulse width of panel
  uint32_t h_total = 0;        //!< Total width of panel (hActive + hFP + hBP + hPulseWidth)
  uint32_t v_total = 0;        //!< Total height of panel (vActive + vFP + vBP + vPulseWidth)
  uint32_t clock_khz = 0;      //!< Stores the pixel clock of panel in khz
  HWTopology topology = kUnknown;  //!< Stores the topology information.

  bool operator !=(const HWDisplayAttributes &display_attributes) {
    return ((is_device_split != display_attributes.is_device_split) ||
            (x_pixels != display_attributes.x_pixels) ||
            (y_pixels != display_attributes.y_pixels) ||
            (x_dpi != display_attributes.x_dpi) ||
            (y_dpi != display_attributes.y_dpi) ||
            (fps != display_attributes.fps) ||
            (vsync_period_ns != display_attributes.vsync_period_ns) ||
            (v_front_porch != display_attributes.v_front_porch) ||
            (v_back_porch != display_attributes.v_back_porch) ||
            (v_pulse_width != display_attributes.v_pulse_width) ||
            (h_total != display_attributes.h_total) ||
            (is_yuv != display_attributes.is_yuv) ||
            (clock_khz != display_attributes.clock_khz) ||
            (topology != display_attributes.topology));
  }

  bool operator ==(const HWDisplayAttributes &display_attributes) {
    return !(operator !=(display_attributes));
  }

  bool OnlyFpsChanged(const HWDisplayAttributes &display_attributes) {
    return ((fps != display_attributes.fps) &&
            (vsync_period_ns != display_attributes.vsync_period_ns) &&
            (x_pixels == display_attributes.x_pixels) &&
            (y_pixels == display_attributes.y_pixels) &&
            (x_dpi == display_attributes.x_dpi) &&
            (y_dpi == display_attributes.y_dpi) &&
            (topology == display_attributes.topology) &&
            (is_device_split == display_attributes.is_device_split) &&
            (v_front_porch == display_attributes.v_front_porch) &&
            (v_back_porch == display_attributes.v_back_porch) &&
            (v_pulse_width == display_attributes.v_pulse_width) &&
            (h_total == display_attributes.h_total) &&
            (is_yuv == display_attributes.is_yuv) &&
            (clock_khz == display_attributes.clock_khz));
  }
};

struct HWMixerAttributes {
  uint32_t width = 0;                                  // Layer mixer width
  uint32_t height = 0;                                 // Layer mixer height
  uint32_t split_left = 0;                             // Left portion of layer mixer
  HWMixerSplit split_type = kNoSplit;                  // Mixer topology
  LayerBufferFormat output_format = kFormatRGB101010;  // Layer mixer output format
  uint32_t dest_scaler_blocks_used = 0;                // Count of dest scaler blocks used

  bool operator !=(const HWMixerAttributes &mixer_attributes) {
    return ((width != mixer_attributes.width) ||
            (height != mixer_attributes.height) ||
            (split_type != mixer_attributes.split_type) ||
            (output_format != mixer_attributes.output_format) ||
            (split_left != mixer_attributes.split_left) ||
            (dest_scaler_blocks_used != mixer_attributes.dest_scaler_blocks_used));
  }

  bool operator ==(const HWMixerAttributes &mixer_attributes) {
    return !(operator !=(mixer_attributes));
  }

  bool IsValid() {
    return (width > 0 && height > 0);
  }
};

}  // namespace sdm

#endif  // __HW_INFO_TYPES_H__

