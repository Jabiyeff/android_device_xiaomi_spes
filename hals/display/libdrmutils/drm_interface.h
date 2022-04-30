/*
* Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
*
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

#ifndef __DRM_INTERFACE_H__
#define __DRM_INTERFACE_H__

#include <map>
#include <string>
#include <utility>
#include <vector>
#include <array>

#include "xf86drm.h"
#include "xf86drmMode.h"
#include <drm/msm_drm.h>
#include <drm/msm_drm_pp.h>

#ifdef KERNEL_5_4
#include <drm/sde_drm.h>
#endif

namespace sde_drm {

typedef std::map<std::pair<uint32_t, uint64_t>, float> CompRatioMap;

/*
 * Drm Atomic Operation Codes
 */
enum struct DRMOps {
  /*
   * Op: Sets plane source crop
   * Arg: uint32_t - Plane ID
   *      DRMRect  - Source Rectangle
   */
  PLANE_SET_SRC_RECT,
  /*
   * Op: Sets plane destination rect
   * Arg: uint32_t - Plane ID
   *      DRMRect - Dst Rectangle
   */
  PLANE_SET_DST_RECT,
  /*
   * Op: Sets plane exclusion rect
   * Arg: uint32_t - Plane ID
   *      drm_clip_rect - Exclusion Rectangle
   */
  PLANE_SET_EXCL_RECT,
  /*
   * Op: Sets plane zorder
   * Arg: uint32_t - Plane ID
   *      uint32_t - zorder
   */
  PLANE_SET_ZORDER,
  /*
   * Op: Sets plane rotation flags
   * Arg: uint32_t - Plane ID
   *      uint32_t - bit mask of rotation flags (See drm_mode.h for enums)
   */
  PLANE_SET_ROTATION,
  /*
   * Op: Sets plane alpha
   * Arg: uint32_t - Plane ID
   *      uint32_t - alpha value
   */
  PLANE_SET_ALPHA,
  /*
   * Op: Sets the blend type
   * Arg: uint32_t - Plane ID
   *      uint32_t - blend type (see DRMBlendType)
   */
  PLANE_SET_BLEND_TYPE,
  /*
   * Op: Sets horizontal decimation
   * Arg: uint32_t - Plane ID
   *      uint32_t - decimation factor
   */
  PLANE_SET_H_DECIMATION,
  /*
   * Op: Sets vertical decimation
   * Arg: uint32_t - Plane ID
   *      uint32_t - decimation factor
   */
  PLANE_SET_V_DECIMATION,
  /*
   * Op: Sets source config flags
   * Arg: uint32_t - Plane ID
   *      uint32_t - flags to enable or disable a specific op. E.g. deinterlacing
   */
  PLANE_SET_SRC_CONFIG,
  /*
   * Op: Sets frame buffer ID for plane. Set together with CRTC.
   * Arg: uint32_t - Plane ID
   *      uint32_t - Framebuffer ID
   */
  PLANE_SET_FB_ID,
  /*
   * Op: Sets the crtc for this plane. Set together with FB_ID.
   * Arg: uint32_t - Plane ID
   *      uint32_t - CRTC ID
   */
  PLANE_SET_CRTC,
  /*
   * Op: Sets acquire fence for this plane's buffer. Set together with FB_ID, CRTC.
   * Arg: uint32_t - Plane ID
   *      uint32_t - Input fence
   */
  PLANE_SET_INPUT_FENCE,
  /*
   * Op: Sets scaler config on this plane.
   * Arg: uint32_t - Plane ID
   *      uint64_t - Address of the scaler config object (version based)
   */
  PLANE_SET_SCALER_CONFIG,
  /*
   * Op: Sets FB Secure mode for this plane.
   * Arg: uint32_t - Plane ID
   *      uint32_t - Value of the FB Secure mode.
   */
  PLANE_SET_FB_SECURE_MODE,
  /*
   * Op: Sets csc config on this plane.
   * Arg: uint32_t - Plane ID
   *      uint32_t* - pointer to csc type
   */
  PLANE_SET_CSC_CONFIG,
  /*
   * Op: Sets multirect mode on this plane.
   * Arg: uint32_t - Plane ID
   *      uint32_t - multirect mode
   */
  PLANE_SET_MULTIRECT_MODE,
  /*
   * Op: Sets sspp layout on this plane.
   * Arg: uint32_t - Plane ID
   *      uint32_t - SSPP Layout Index
   */
  PLANE_SET_SSPP_LAYOUT,
  /*
   * Op: Sets rotator output frame buffer ID for plane.
   * Arg: uint32_t - Plane ID
   *      uint32_t - Framebuffer ID
   */
  PLANE_SET_ROT_FB_ID,
  /*
   * Op: Sets inverse pma mode on this plane.
   * Arg: uint32_t - Plane ID
   *      uint32_t - enable/disable inverse pma.
   */
  PLANE_SET_INVERSE_PMA,
  /*
   * Op: Sets csc config on this plane.
   * Arg: uint32_t - Plane ID
   *      uint64_t - Address of the csc config object(version based)
   */
  PLANE_SET_DGM_CSC_CONFIG,
  /*
   * Op: Sets SSPP Feature
   * Arg: uint32_t - Plane ID
   *      DRMPPFeatureInfo * - PP feature data pointer
   */
  PLANE_SET_POST_PROC,
  /*
   * Op: Activate or deactivate a CRTC
   * Arg: uint32_t - CRTC ID
   *      uint32_t - 1 to enable, 0 to disable
   */
  CRTC_SET_ACTIVE,
  /*
   * Op: Sets display mode
   * Arg: uint32_t - CRTC ID
   *      drmModeModeInfo* - Pointer to display mode
   */
  CRTC_SET_MODE,
  /*
   * Op: Sets an offset indicating when a release fence should be signalled.
   * Arg: uint32_t - offset
   *      0: non-speculative, default
   *      1: speculative
   */
  CRTC_SET_OUTPUT_FENCE_OFFSET,
  /*
   * Op: Sets overall SDE core clock
   * Arg: uint32_t - CRTC ID
   *      uint32_t - core_clk
   */
  CRTC_SET_CORE_CLK,
  /*
   * Op: Sets MNOC bus average bandwidth
   * Arg: uint32_t - CRTC ID
   *      uint32_t - core_ab
   */
  CRTC_SET_CORE_AB,
   /*
   * Op: Sets MNOC bus instantaneous bandwidth
   * Arg: uint32_t - CRTC ID
   *      uint32_t - core_ib
   */
  CRTC_SET_CORE_IB,
  /*
   * Op: Sets LLCC Bus average bandwidth
   * Arg: uint32_t - CRTC ID
   *      uint32_t - llcc_ab
   */
  CRTC_SET_LLCC_AB,
  /*
   * Op: Sets LLCC Bus instantaneous bandwidth
   * Arg: uint32_t - CRTC ID
   *      uint32_t - llcc_ib
   */
  CRTC_SET_LLCC_IB,
  /*
   * Op: Sets DRAM bus average bandwidth
   * Arg: uint32_t - CRTC ID
   *      uint32_t - dram_ab
   */
  CRTC_SET_DRAM_AB,
  /*
   * Op: Sets DRAM bus instantaneous bandwidth
   * Arg: uint32_t - CRTC ID
   *      uint32_t - dram_ib
   */
  CRTC_SET_DRAM_IB,
  /*
   * Op: Sets Rotator BW for inline rotation
   * Arg: uint32_t - CRTC ID
   *      uint32_t - rot_bw
   */
  CRTC_SET_ROT_PREFILL_BW,
  /*
   * Op: Sets rotator clock for inline rotation
   * Arg: uint32_t - CRTC ID
   *      uint32_t - rot_clk
   */
  CRTC_SET_ROT_CLK,
  /*
   * Op: Sets destination scalar data
   * Arg: uint32_t - CRTC ID
   *      uint64_t - Pointer to destination scalar data
   */
  CRTC_SET_DEST_SCALER_CONFIG,
  /*
   * Op: Returns release fence for this frame. Should be called after Commit() on
   * DRMAtomicReqInterface.
   * Arg: uint32_t - CRTC ID
   *      int * - Pointer to an integer that will hold the returned fence
   */
  CRTC_GET_RELEASE_FENCE,
  /*
   * Op: Sets PP feature
   * Arg: uint32_t - CRTC ID
   *      DRMPPFeatureInfo * - PP feature data pointer
   */
  CRTC_SET_POST_PROC,
  /*
   * Op: Sets CRTC ROIs.
   * Arg: uint32_t - CRTC ID
   *      uint32_t - number of ROIs
   *      DRMRect * - Array of CRTC ROIs
   */
  CRTC_SET_ROI,
  /*
   * Op: Sets Security level for CRTC.
   * Arg: uint32_t - CRTC ID
   *      uint32_t - Security level
   */
  CRTC_SET_SECURITY_LEVEL,
  /*
   * Op: sets solid fill stages
   * Arg: uint32_t - CRTC ID
   *      Vector of DRMSolidfillStage
   */
  CRTC_SET_SOLIDFILL_STAGES,
  /*
   * Op: Sets idle timeout.
   * Arg: uint32_t - CRTC ID
   *      uint32_t - idle timeout in ms
   */
  CRTC_SET_IDLE_TIMEOUT,
  /*
   * Op: Sets Capture mode for Concurrent Writeback feature.
   * Arg: uint32_t - CRTC ID
   *      uint32_t - Capture mode
   */
  CRTC_SET_CAPTURE_MODE,
  /*
   * Op: Sets Idle PC state for CRTC.
   * Arg: uint32_t - CRTC ID
   *      uint32_t - idle pc state
   */
  CRTC_SET_IDLE_PC_STATE,
  /*
   * Op: Returns retire fence for this commit. Should be called after Commit() on
   * DRMAtomicReqInterface.
   * Arg: uint32_t - Connector ID
   *      int * - Pointer to an integer that will hold the returned fence
   */
  CONNECTOR_GET_RETIRE_FENCE,
  /*
   * Op: Sets writeback connector destination rect
   * Arg: uint32_t - Connector ID
   *      DRMRect - Dst Rectangle
   */
  CONNECTOR_SET_OUTPUT_RECT,
  /*
   * Op: Sets frame buffer ID for writeback connector.
   * Arg: uint32_t - Connector ID
   *      uint32_t - Framebuffer ID
   */
  CONNECTOR_SET_OUTPUT_FB_ID,
  /*
   * Op: Sets power mode for connector.
   * Arg: uint32_t - Connector ID
   *      uint32_t - Power Mode
   */
  CONNECTOR_SET_POWER_MODE,
  /*
   * Op: Sets panel ROIs.
   * Arg: uint32_t - Connector ID
   *      uint32_t - number of ROIs
   *      DRMRect * - Array of Connector ROIs
   */
  CONNECTOR_SET_ROI,
  /*
   * Op: Sets the connector to autorefresh mode.
   * Arg: uint32_t - Connector ID
   *      uint32_t - Enable-1, Disable-0
   */
  CONNECTOR_SET_AUTOREFRESH,
  /*
   * Op: Set FB secure mode for Writeback connector.
   * Arg: uint32_t - Connector ID
   *      uint32_t - FB Secure mode
   */
  CONNECTOR_SET_FB_SECURE_MODE,
  /*
   * Op: Sets a crtc id to this connector
   * Arg: uint32_t - Connector ID
   *      uint32_t - CRTC ID
   */
  CONNECTOR_SET_CRTC,
  /*
   * Op: Sets PP feature
   * Arg: uint32_t - Connector ID
   * DRMPPFeatureInfo * - PP feature data pointer
   */
  CONNECTOR_SET_POST_PROC,
  /*
   * Op: Sets connector hdr metadata
   * Arg: uint32_t - Connector ID
   *      drm_msm_ext_hdr_metadata - hdr_metadata
   */
  CONNECTOR_SET_HDR_METADATA,
  /*
   * Op: Cache Dpps features.
   * Arg: uint32_t - Object ID
          uint32_t - Feature ID
   *      uint64_t - Pointer to feature config data
   */
  DPPS_CACHE_FEATURE,
  /*
   * Op: Commit Dpps features.
   * Arg: drmModeAtomicReq - Atomic request
   */
  DPPS_COMMIT_FEATURE,
  /*
   * Op: Commit panel features.
   * Arg: drmModeAtomicReq - Atomic request
   */
  COMMIT_PANEL_FEATURES,
  /*
   * Op: Sets qsync mode on connector
   * Arg: uint32_t - Connector ID
   *     uint32_t - qsync mode
   */
  CONNECTOR_SET_QSYNC_MODE,
  /*
   * Op: Sets topology control on this connector
   * Arg: uint32_t - Connector ID
   *      uint32_t - Topology control bit-mask
   */
  CONNECTOR_SET_TOPOLOGY_CONTROL,
  /*
   * Op: Sets frame trigger mode on this connector
   * Arg: uint32_t - Connector ID
   *      uint32_t - Frame trigger mode
   */
  CONNECTOR_SET_FRAME_TRIGGER,
  /*
   * Op: Sets colorspace on DP connector
   * Arg: uint32_t - Connector ID
   *      uint32_t - colorspace value bit-mask
   */
  CONNECTOR_SET_COLORSPACE,
};

enum struct DRMRotation {
  FLIP_H = 0x1,
  FLIP_V = 0x2,
  ROT_180 = FLIP_H | FLIP_V,
  ROT_90 = 0x4,
};

enum struct DRMPowerMode {
  ON,
  DOZE,
  DOZE_SUSPEND,
  OFF,
};

enum struct DRMBlendType {
  UNDEFINED = 0,
  OPAQUE = 1,
  PREMULTIPLIED = 2,
  COVERAGE = 3,
};

enum struct DRMSrcConfig {
  DEINTERLACE = 0,
};

enum struct DRMIdlePCState {
  NONE,
  ENABLE,
  DISABLE,
};


/* Display type to identify a suitable connector */
enum struct DRMDisplayType {
  PERIPHERAL,
  TV,
  VIRTUAL,
};

struct DRMRect {
  uint32_t left;    // Left-most pixel coordinate.
  uint32_t top;     // Top-most pixel coordinate.
  uint32_t right;   // Right-most pixel coordinate.
  uint32_t bottom;  // Bottom-most pixel coordinate.
};

//------------------------------------------------------------------------
// DRM Info Query Types
//------------------------------------------------------------------------

enum struct QSEEDVersion {
  V1,
  V2,
  V3,
  V3LITE,
};

/* QSEED3 Step version */
enum struct QSEEDStepVersion {
  V2,
  V3,
  V4,
  V3LITE_V4,
  V3LITE_V5,
};

enum struct SmartDMARevision {
  V1,
  V2,
  V2p5
};

/* Inline Rotation version */
enum struct InlineRotationVersion {
  kInlineRotationNone,
  kInlineRotationV1,
  kInlineRotationV2,
};

/* Per CRTC Resource Info*/
struct DRMCrtcInfo {
  bool has_src_split;
  bool has_hdr;
  uint32_t max_blend_stages;
  uint32_t max_solidfill_stages;
  QSEEDVersion qseed_version;
  SmartDMARevision smart_dma_rev;
  float ib_fudge_factor;
  float clk_fudge_factor;
  uint32_t dest_scale_prefill_lines;
  uint32_t undersized_prefill_lines;
  uint32_t macrotile_prefill_lines;
  uint32_t nv12_prefill_lines;
  uint32_t linear_prefill_lines;
  uint32_t downscale_prefill_lines;
  uint32_t extra_prefill_lines;
  uint32_t amortized_threshold;
  uint64_t max_bandwidth_low;
  uint64_t max_bandwidth_high;
  uint32_t max_sde_clk;
  CompRatioMap comp_ratio_rt_map;
  CompRatioMap comp_ratio_nrt_map;
  uint32_t hw_version;
  uint32_t dest_scaler_count = 0;
  uint32_t max_dest_scaler_input_width = 0;
  uint32_t max_dest_scaler_output_width = 0;
  uint32_t max_dest_scale_up = 1;
  uint32_t min_prefill_lines = 0;
  int secure_disp_blend_stage = -1;
  bool concurrent_writeback = false;
  uint32_t vig_limit_index = 0;
  uint32_t dma_limit_index = 0;
  uint32_t scaling_limit_index = 0;
  uint32_t rotation_limit_index = 0;
  uint32_t line_width_constraints_count = 0;
  std::vector< std::pair <uint32_t, uint32_t> > line_width_limits;
  uint32_t num_mnocports;
  uint32_t mnoc_bus_width;
  bool use_baselayer_for_stage = false;
  bool has_micro_idle = false;
  uint32_t ubwc_version = 1;
  uint64_t rc_total_mem_size = 0;
};

enum struct DRMPlaneType {
  // Has CSC and scaling capability
  VIG = 0,
  // Has scaling capability but no CSC
  RGB,
  // No scaling support
  DMA,
  // Supports a small dimension and doesn't use a CRTC stage
  CURSOR,
  MAX,
};

enum struct DRMTonemapLutType {
  DMA_1D_GC,
  DMA_1D_IGC,
  VIG_1D_IGC,
  VIG_3D_GAMUT,
};

struct DRMPlaneTypeInfo {
  DRMPlaneType type;
  uint32_t master_plane_id;
  // FourCC format enum and modifier
  std::vector<std::pair<uint32_t, uint64_t>> formats_supported;
  uint32_t max_linewidth;
  uint32_t max_scaler_linewidth;
  uint32_t max_rotation_linewidth; // inline rotation limitation
  uint32_t max_upscale;
  uint32_t max_downscale;
  uint32_t max_horizontal_deci;
  uint32_t max_vertical_deci;
  uint64_t max_pipe_bandwidth;
  uint64_t max_pipe_bandwidth_high;
  uint32_t cache_size;  // cache size in bytes for inline rotation support.
  bool has_excl_rect = false;
  QSEEDStepVersion qseed3_version;
  bool multirect_prop_present = false;
  InlineRotationVersion inrot_version;  // inline rotation version
  std::vector<std::pair<uint32_t, uint64_t>> inrot_fmts_supported;
  float true_inline_dwnscale_rt_num = 11.0;
  float true_inline_dwnscale_rt_denom = 5.0;
  bool inverse_pma = false;
  uint32_t dgm_csc_version = 0;  // csc used with DMA
  std::map<DRMTonemapLutType, uint32_t> tonemap_lut_version_map = {};
  bool block_sec_ui = false;
};

// All DRM Planes as map<Plane_id , plane_type_info> listed from highest to lowest priority
typedef std::vector<std::pair<uint32_t, DRMPlaneTypeInfo>>  DRMPlanesInfo;

enum struct DRMTopology {
  UNKNOWN,  // To be compat with driver defs in sde_rm.h
  SINGLE_LM,
  SINGLE_LM_DSC,
  DUAL_LM,
  DUAL_LM_DSC,
  DUAL_LM_MERGE,
  DUAL_LM_MERGE_DSC,
  DUAL_LM_DSCMERGE,
  QUAD_LM_MERGE,
  QUAD_LM_DSCMERGE,
  QUAD_LM_MERGE_DSC,
  PPSPLIT,
};

enum struct DRMPanelMode {
  VIDEO,
  COMMAND,
};

/* Per mode info */
struct DRMModeInfo {
  drmModeModeInfo mode;
  DRMTopology topology;
  // Valid only if mode is command
  int num_roi;
  int xstart;
  int ystart;
  int walign;
  int halign;
  int wmin;
  int hmin;
  bool roi_merge;
  uint64_t bit_clk_rate;
  uint32_t transfer_time_us;
};

/* Per Connector Info*/
struct DRMConnectorInfo {
  uint32_t mmWidth;
  uint32_t mmHeight;
  uint32_t type;
  uint32_t type_id;
  std::vector<DRMModeInfo> modes;
  std::string panel_name;
  DRMPanelMode panel_mode;
  bool is_primary;
  // Valid only if DRMPanelMode is VIDEO
  bool dynamic_fps;
  // FourCC format enum and modifier
  std::vector<std::pair<uint32_t, uint64_t>> formats_supported;
  // Valid only if type is DRM_MODE_CONNECTOR_VIRTUAL
  uint32_t max_linewidth;
  DRMRotation panel_orientation;
  drm_panel_hdr_properties panel_hdr_prop;
  drm_msm_ext_hdr_properties ext_hdr_prop;
  bool qsync_support;
  // Connection status of this connector
  bool is_connected;
  bool is_wb_ubwc_supported;
  uint32_t topology_control;
  bool dyn_bitclk_support;
  std::vector<uint8_t> edid;
  uint32_t supported_colorspaces;
};

// All DRM Connectors as map<Connector_id , connector_info>
typedef std::map<uint32_t, DRMConnectorInfo> DRMConnectorsInfo;

/* Per Encoder Info */
struct DRMEncoderInfo {
  uint32_t type;
};

// All DRM Encoders as map<Encoder_id , encoder_info>
typedef std::map<uint32_t, DRMEncoderInfo> DRMEncodersInfo;

/* Identifier token for a display */
struct DRMDisplayToken {
  uint32_t conn_id;
  uint32_t crtc_id;
  uint32_t crtc_index;
  uint32_t encoder_id;
  uint8_t hw_port;
};

enum DRMPPFeatureID {
  kFeaturePcc,
  kFeatureIgc,
  kFeaturePgc,
  kFeatureMixerGc,
  kFeaturePaV2,
  kFeatureDither,
  kFeatureGamut,
  kFeaturePADither,
  kFeaturePAHsic,
  kFeaturePASixZone,
  kFeaturePAMemColSkin,
  kFeaturePAMemColSky,
  kFeaturePAMemColFoliage,
  kFeaturePAMemColProt,
  kFeatureDgmIgc,
  kFeatureDgmGc,
  kFeatureVigIgc,
  kFeatureVigGamut,
  kPPFeaturesMax,
};

enum DRMPropType {
  kPropEnum,
  kPropRange,
  kPropBlob,
  kPropTypeMax,
};

struct DRMPPFeatureInfo {
  DRMPPFeatureID id;
  DRMPropType type;
  uint32_t version;
  uint32_t payload_size;
  void *payload;
  uint32_t object_type;
};

enum DRMDPPSFeatureID {
  // Ad4 properties
  kFeatureAd4Mode,
  kFeatureAd4Init,
  kFeatureAd4Cfg,
  kFeatureAd4Input,
  kFeatureAd4Roi,
  kFeatureAd4Backlight,
  kFeatureAd4Assertiveness,
  kFeatureAd4ManualStrength,
  // ABA properties
  kFeatureAbaHistCtrl,
  kFeatureAbaHistIRQ,
  kFeatureAbaLut,
  // BL scale properties
  kFeatureSvBlScale,
  kFeatureBacklightScale,
  // Events
  kFeaturePowerEvent,
  kFeatureAbaHistEvent,
  kFeatureBackLightEvent,
  kFeatureAdAttBlEvent,
  kFeatureLtmHistEvent,
  kFeatureLtmWbPbEvent,
  kFeatureLtmOffEvent,
  // LTM properties
  kFeatureLtm,
  kFeatureLtmInit,
  kFeatureLtmCfg,
  kFeatureLtmNoiseThresh,
  kFeatureLtmBufferCtrl,
  kFeatureLtmQueueBuffer,
  kFeatureLtmQueueBuffer2,
  kFeatureLtmQueueBuffer3,
  kFeatureLtmHistCtrl,
  kFeatureLtmVlut,
  // Insert features above
  kDppsFeaturesMax,
};

struct DppsFeaturePayload {
  uint32_t object_type;
  uint32_t feature_id;
  uint64_t value;
};

struct DRMDppsLtmBuffers {
  uint32_t num_of_buffers;
  uint32_t buffer_size;
  std::array<int, LTM_BUFFER_SIZE> ion_buffer_fd;
  std::array<int, LTM_BUFFER_SIZE> drm_fb_id;
  std::array<void*, LTM_BUFFER_SIZE> uva;
  int status;
};

struct DRMDppsFeatureInfo {
  DRMDPPSFeatureID id;
  uint32_t obj_id;
  uint32_t version;
  uint32_t payload_size;
  void *payload;
};

enum DRMPanelFeatureID {
  kDRMPanelFeatureRCInit,
  kDRMPanelFeatureDsppRCInfo,
  kDRMPanelFeatureMax,
};

struct DRMPanelFeatureInfo  {
  DRMPanelFeatureID prop_id;
  uint32_t obj_type;
  uint32_t obj_id;
  uint32_t version;
  uint32_t prop_size;
  uint64_t prop_ptr;
};

enum AD4Modes {
  kAd4Off,
  kAd4AutoStrength,
  kAd4Calibration,
  kAd4Manual,
  kAd4ModeMax,
};

enum HistModes {
  kHistDisabled,
  kHistEnabled,
};

struct DRMDppsEventInfo {
  uint32_t object_type;
  uint32_t event_type;
  int drm_fd;
  bool enable;
};

enum DRMCscType {
  kCscYuv2Rgb601L,
  kCscYuv2Rgb601FR,
  kCscYuv2Rgb709L,
  kCscYuv2Rgb2020L,
  kCscYuv2Rgb2020FR,
  kCscTypeMax,
};

struct DRMScalerLUTInfo {
  uint32_t dir_lut_size = 0;
  uint32_t cir_lut_size = 0;
  uint32_t sep_lut_size = 0;
  uint64_t dir_lut = 0;
  uint64_t cir_lut = 0;
  uint64_t sep_lut = 0;
};

enum struct DRMSecureMode {
  NON_SECURE,
  SECURE,
  NON_SECURE_DIR_TRANSLATION,
  SECURE_DIR_TRANSLATION,
};

enum struct DRMSecurityLevel {
  SECURE_NON_SECURE,
  SECURE_ONLY,
};

enum struct DRMMultiRectMode {
  NONE = 0,
  PARALLEL = 1,
  SERIAL = 2,
};

enum struct DRMSSPPLayoutIndex {
  NONE = 0,
  LEFT = 1,
  RIGHT = 2,
};

enum struct DRMCWbCaptureMode {
  MIXER_OUT = 0,
  DSPP_OUT = 1,
};

enum struct DRMQsyncMode {
  NONE = 0,
  CONTINUOUS,
  ONESHOT,
};

enum struct DRMTopologyControl {
  NONE          = 0,
  RESERVE_LOCK  = 1 << 0,
  RESERVE_CLEAR = 1 << 1,
  DSPP          = 1 << 2,
  DEST_SCALER   = 1 << 3,
};

struct DRMSolidfillStage {
  DRMRect bounding_rect {};
  bool is_exclusion_rect = false;
  uint32_t color = 0xff000000;  // in 8bit argb
  uint32_t red = 0;
  uint32_t blue = 0;
  uint32_t green = 0;
  uint32_t alpha = 0xff;
  uint32_t color_bit_depth = 0;
  uint32_t z_order = 0;
  uint32_t plane_alpha = 0xff;
};

enum struct DRMFrameTriggerMode {
  FRAME_DONE_WAIT_DEFAULT = 0,
  FRAME_DONE_WAIT_SERIALIZE,
  FRAME_DONE_WAIT_POSTED_START,
};

/* DRM Color spaces exposed by the DP connector */
enum struct DRMColorspace {
  DEFAULT = 0,
  SMPTE_170M_YCC,
  BT709_YCC,
  XVYCC_601,
  XVYCC_709,
  SYCC_601,
  OPYCC_601,
  OPRGB,
  BT2020_CYCC,
  BT2020_RGB,
  BT2020_YCC,
  DCI_P3_RGB_D65,
  DCI_P3_RGB_THEATER,
};

/* DRM Atomic Request Property Set.
 *
 * Helper class to create and populate atomic properties of DRM components
 * when rendered in DRM atomic mode */
class DRMAtomicReqInterface {
 public:
  virtual ~DRMAtomicReqInterface() {}
  /* Perform request operation.
   *
   * [input]: opcode: operation code from DRMOps list.
   *          obj_id: Relevant crtc, connector, plane id
   *          var_arg: arguments for DRMOps's can differ in number and
   *          data type. Refer above DRMOps to details.
   * [return]: Error code if the API fails, 0 on success.
   */
  virtual int Perform(DRMOps opcode, uint32_t obj_id, ...) = 0;

  /*
   * Commit the params set via Perform(). Also resets the properties after commit. Needs to be
   * called every frame.
   * [input]: synchronous: Determines if the call should block until a h/w flip
   * [input]: retain_planes: Retains already staged planes. Useful when not explicitly programming
   *          planes but still need the previously staged ones to not be unstaged
   * [return]: Error code if the API fails, 0 on success.
   */
  virtual int Commit(bool synchronous, bool retain_planes) = 0;

  /*
   * Validate the params set via Perform().
   * [return]: Error code if the API fails, 0 on success.
   */
  virtual int Validate() = 0;
};

class DRMManagerInterface;

/* Populates a singleton instance of DRMManager */
typedef int (*GetDRMManager)(int fd, DRMManagerInterface **intf);

/* Destroy DRMManager instance */
typedef int (*DestroyDRMManager)();

/*
 * DRM Manager Interface - Any class which plans to implement helper function for vendor
 * specific DRM driver implementation must implement the below interface routines to work
 * with SDM.
 */

class DRMManagerInterface {
 public:
  virtual ~DRMManagerInterface() {}

  /*
   * Since SDM completely manages the planes. GetPlanesInfo will provide all
   * the plane information.
   * [output]: DRMPlanesInfo: Resource Info for planes.
   */
  virtual void GetPlanesInfo(DRMPlanesInfo *info) = 0;

  /*
   * Will provide all the information of a selected crtc.
   * [input]: Use crtc id 0 to obtain system wide info
   * [output]: DRMCrtcInfo: Resource Info for the given CRTC id.
   * [return]: 0 on success, a negative error value otherwise.
   */
  virtual int GetCrtcInfo(uint32_t crtc_id, DRMCrtcInfo *info) = 0;

  /*
   * Will provide all the information of a selected connector.
   * [output]: DRMConnectorInfo: Resource Info for the given connector id
   * [return]: 0 on success, a negative error value otherwise.
   */
  virtual int GetConnectorInfo(uint32_t conn_id, DRMConnectorInfo *info) = 0;

  /*
   * Provides information on all connectors.
   * [output]: DRMConnectorsInfo: Resource info for connectors.
   * [return]: 0 on success, a negative error value otherwise.
   */
  virtual int GetConnectorsInfo(DRMConnectorsInfo *info) = 0;

  /*
   * Provides information on a selected encoder.
   * [output]: DRMEncoderInfo: Resource info for the given encoder id.
   * [return]: 0 on success, a negative error value otherwise.
   */
  virtual int GetEncoderInfo(uint32_t encoder_id, DRMEncoderInfo *info) = 0;

  /*
   * Provides information on all encoders.
   * [output]: DRMEncodersInfo: Resource info for encoders.
   * [return]: 0 on success, a negative error value otherwise.
   */
  virtual int GetEncodersInfo(DRMEncodersInfo *info) = 0;

  /*
   * Will query post propcessing feature info of a CRTC.
   * [output]: DRMPPFeatureInfo: CRTC post processing feature info
   */
  virtual void GetCrtcPPInfo(uint32_t crtc_id, DRMPPFeatureInfo *info) = 0;

  /*
   * Register a logical display to receive a token.
   * Each display pipeline in DRM is identified by its CRTC and Connector(s). On display connect
   * (bootup or hotplug), clients should invoke this interface to establish the pipeline for the
   * display and should get a DisplayToken populated with crtc, encoder and connnector(s) id's. Here
   * onwards, Client should use this token to represent the display for any Perform operations if
   * needed.
   *
   * [input]: disp_type - Peripheral / TV / Virtual
   * [output]: DRMDisplayToken - CRTC and Connector IDs for the display.
   * [return]: 0 on success, a negative error value otherwise.
   */
  virtual int RegisterDisplay(DRMDisplayType disp_type, DRMDisplayToken *tok) = 0;

  /*
   * Register a logical display to receive a token.
   * Each display pipeline in DRM is identified by its CRTC and Connector(s). On display connect
   * (bootup or hotplug), clients should invoke this interface to establish the pipeline for the
   * display and should get a DisplayToken populated with crtc, encoder and connnector(s) id's. Here
   * onwards, Client should use this token to represent the display for any Perform operations if
   * needed.
   *
   * [input]: display_id - Connector ID
   * [output]: DRMDisplayToken - CRTC and Connector id's for the display.
   * [return]: 0 on success, a negative error value otherwise.
   */
  virtual int RegisterDisplay(int32_t display_id, DRMDisplayToken *token) = 0;

  /* Client should invoke this interface on display disconnect.
   * [input]: DRMDisplayToken - identifier for the display.
   */
  virtual void UnregisterDisplay(DRMDisplayToken *token) = 0;

  /*
   * Creates and returns an instance of DRMAtomicReqInterface corresponding to a display token
   * returned as part of RegisterDisplay API. Needs to be called per display.
   * [input]: DRMDisplayToken that identifies a display pipeline
   * [output]: Pointer to an instance of DRMAtomicReqInterface.
   * [return]: Error code if the API fails, 0 on success.
   */
  virtual int CreateAtomicReq(const DRMDisplayToken &token, DRMAtomicReqInterface **intf) = 0;

  /*
   * Destroys the instance of DRMAtomicReqInterface
   * [input]: Pointer to a DRMAtomicReqInterface
   * [return]: Error code if the API fails, 0 on success.
   */
  virtual int DestroyAtomicReq(DRMAtomicReqInterface *intf) = 0;

  /*
   * Sets the global scaler LUT
   * [input]: LUT Info
   * [return]: Error code if the API fails, 0 on success.
   */
  virtual int SetScalerLUT(const DRMScalerLUTInfo &lut_info) = 0;

  /*
   * Unsets the global scaler LUT
   * [input]: None
   * [return]: Error code if the API fails, 0 on success.
   */
  virtual int UnsetScalerLUT() = 0;

  /*
   * Get the DPPS feature info
   * [input]: Dpps feature id, info->id
   * [output]: Dpps feature version, info->version
   */
  virtual void GetDppsFeatureInfo(DRMDppsFeatureInfo *info) = 0;

  /*
   * Get the Panel feature info
   * [output]: panel feature info data
   */
  virtual void GetPanelFeature(DRMPanelFeatureInfo *info) = 0;

  /*
   * Set the Panel feature
   * [input]: panel feature info data
   */
  virtual void SetPanelFeature(const DRMPanelFeatureInfo &info) = 0;
};

}  // namespace sde_drm
#endif  // __DRM_INTERFACE_H__
