/*
* Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
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

#ifndef __DISPLAY_PROPERTIES_H__
#define __DISPLAY_PROPERTIES_H__

#define DISP_PROP_PREFIX                     "vendor.display."
#define GRALLOC_PROP_PREFIX                  "vendor.gralloc."
#define PERSIST_DISP_PROP_PREFIX             "persist.vendor.display."

#define DISPLAY_PROP(prop_name)              DISP_PROP_PREFIX prop_name
#define GRALLOC_PROP(prop_name)              GRALLOC_PROP_PREFIX prop_name
#define PERSIST_DISPLAY_PROP(prop_name)      PERSIST_DISP_PROP_PREFIX prop_name

// Start of property
#define COMPOSITION_MASK_PROP                DISPLAY_PROP("comp_mask")
#define HDMI_CONFIG_INDEX_PROP               DISPLAY_PROP("hdmi_cfg_idx")
#define IDLE_TIME_PROP                       DISPLAY_PROP("idle_time")
#define IDLE_TIME_INACTIVE_PROP              DISPLAY_PROP("idle_time_inactive")
#define DISABLE_ROTATOR_DOWNSCALE_PROP       DISPLAY_PROP("disable_rotator_downscale")
// Enables rotator for UI
#define ENABLE_ROTATOR_UI_PROP               DISPLAY_PROP("enable_rotator_ui")
#define DISABLE_DECIMATION_PROP              DISPLAY_PROP("disable_decimation")
#define PRIMARY_MIXER_STAGES_PROP            DISPLAY_PROP("primary_mixer_stages")
#define EXTERNAL_MIXER_STAGES_PROP           DISPLAY_PROP("external_mixer_stages")
#define VIRTUAL_MIXER_STAGES_PROP            DISPLAY_PROP("virtual_mixer_stages")
#define MAX_UPSCALE_PROP                     DISPLAY_PROP("max_upscale")
#define VIDEO_MODE_PANEL_PROP                DISPLAY_PROP("video_mode_panel")
#define DISABLE_ROTATOR_UBWC_PROP            DISPLAY_PROP("disable_rotator_ubwc")
#define DISABLE_ROTATOR_SPLIT_PROP           DISPLAY_PROP("disable_rotator_split")
#define DISABLE_SCALER_PROP                  DISPLAY_PROP("disable_scaler")
#define DISABLE_AVR_PROP                     DISPLAY_PROP("disable_avr")
#define DISABLE_EXTERNAL_ANIMATION_PROP      DISPLAY_PROP("disable_ext_anim")
#define DISABLE_PARTIAL_SPLIT_PROP           DISPLAY_PROP("disable_partial_split")
#define PREFER_SOURCE_SPLIT_PROP             DISPLAY_PROP("prefer_source_split")
#define MIXER_RESOLUTION_PROP                DISPLAY_PROP("mixer_resolution")
#define SIMULATED_CONFIG_PROP                DISPLAY_PROP("simulated_config")
#define MAX_EXTERNAL_LAYERS_PROP             DISPLAY_PROP("max_external_layers")
#define PERF_HINT_WINDOW_PROP                DISPLAY_PROP("perf_hint_window")
#define ENABLE_EXTERNAL_DOWNSCALE_PROP       DISPLAY_PROP("enable_external_downscale")
#define EXTERNAL_ACTION_SAFE_WIDTH_PROP      DISPLAY_PROP("external_action_safe_width")
#define EXTERNAL_ACTION_SAFE_HEIGHT_PROP     DISPLAY_PROP("external_action_safe_height")
#define FB_WIDTH_PROP                        DISPLAY_PROP("fb_width")
#define FB_HEIGHT_PROP                       DISPLAY_PROP("fb_height")
#define DISABLE_METADATA_DYNAMIC_FPS_PROP    DISPLAY_PROP("disable_metadata_dynamic_fps")
#define DISABLE_SKIP_VALIDATE_PROP           DISPLAY_PROP("disable_skip_validate")
#define DISABLE_DESTINATION_SCALER_PROP      DISPLAY_PROP("disable_dest_scaler")
#define DISABLE_SECURE_INLINE_ROTATOR_PROP   DISPLAY_PROP("disable_secure_inline_rotator")
#define DISABLE_MULTIRECT_PROP               DISPLAY_PROP("disable_multirect")
#define DISABLE_UBWC_FF_VOTING_PROP          DISPLAY_PROP("disable_ubwc_ff_voting")
#define DISABLE_INLINE_ROTATOR_PROP          DISPLAY_PROP("disable_inline_rotator")
#define DISABLE_OFFLINE_ROTATOR_PROP         DISPLAY_PROP("disable_offline_rotator")
#define DISABLE_FB_CROPPING_PROP             DISPLAY_PROP("disable_fb_cropping")
#define PRIORITIZE_CACHE_COMPOSITION_PROP    DISPLAY_PROP("prioritize_cache_comp")
#define DISABLE_HW_RECOVERY_PROP             DISPLAY_PROP("disable_hw_recovery")
#define DISABLE_HW_RECOVERY_DUMP_PROP        DISPLAY_PROP("disable_hw_recovery_dump")
#define DISABLE_SRC_TONEMAP_PROP             DISPLAY_PROP("disable_src_tonemap")
#define ENABLE_NULL_DISPLAY_PROP             DISPLAY_PROP("enable_null_display")
#define DISABLE_EXCL_RECT_PROP               DISPLAY_PROP("disable_excl_rect")
#define ENABLE_PIPE_PRIORITY_PROP            DISPLAY_PROP("enable_pipe_priority")
#define DISABLE_EXCl_RECT_PARTIAL_FB         DISPLAY_PROP("disable_excl_rect_partial_fb")
#define DISABLE_FBID_CACHE                   DISPLAY_PROP("disable_fbid_cache")
#define DISABLE_HOTPLUG_BWCHECK              DISPLAY_PROP("disable_hotplug_bwcheck")
#define DISABLE_MASK_LAYER_HINT              DISPLAY_PROP("disable_mask_layer_hint")
#define DISABLE_HDR_LUT_GEN                  DISPLAY_PROP("disable_hdr_lut_gen")
#define DISABLE_HDR                          DISPLAY_PROP("hwc_disable_hdr")
#define QDCM_PCC_TRANS_PROP                  DISPLAY_PROP("qdcm.pcc_for_trans")
#define QDCM_DIAGONAL_MATRIXMODE_PROP        DISPLAY_PROP("qdcm.diagonal_matrix_mode")
#define QDCM_MODE_COMBINE_PROP               DISPLAY_PROP("qdcm.mode_combine")
#define PREFER_MULTIRECT_PROP                DISPLAY_PROP("prefer_multirect")
#define DROP_SKEWED_VSYNC                    DISPLAY_PROP("drop_skewed_vsync")
#define DISABLE_FAST_PATH                    DISPLAY_PROP("disable_fast_path")
#define DISABLE_SYSTEM_LOAD_CHECK            DISPLAY_PROP("disable_system_load_check")
#define DISABLE_IDLE_SCALING_LAYERS          DISPLAY_PROP("disable_idle_scaling_layers")
// Disable microidle condition
#define DISABLE_SINGLE_LM_SPLIT_PROP         DISPLAY_PROP("disable_single_lm_split")
// Enable posted start dynamic
#define ENABLE_POSTED_START_DYN_PROP         DISPLAY_PROP("enable_posted_start_dyn")
#define ENABLE_OPTIMIZE_REFRESH              DISPLAY_PROP("enable_optimize_refresh")
#define DISABLE_PARALLEL_CACHE               DISPLAY_PROP("disable_parallel_cache")
#define DISABLE_LAYER_STITCH                 DISPLAY_PROP("disable_layer_stitch")
// Disable 3d tonemap support for UI layers
#define DISABLE_UI_3D_TONEMAP                DISPLAY_PROP("disable_ui_3d_tonemap")
#define QDCM_DISABLE_FACTORY_MODE_PROP       DISPLAY_PROP("qdcm.disable_factory_mode")
#define ENABLE_ASYNC_POWERMODE               DISPLAY_PROP("enable_async_powermode")
#define ENABLE_GPU_TONEMAPPER_PROP           DISPLAY_PROP("enable_gpu_tonemapper")
#define ENABLE_FORCE_SPLIT                   DISPLAY_PROP("enable_force_split")
#define DISABLE_GPU_COLOR_CONVERT            DISPLAY_PROP("disable_gpu_color_convert")
#define ENABLE_ASYNC_VDS_CREATION            DISPLAY_PROP("enable_async_vds_creation")
// MMNOC efficiency factor for Camera and Non-Camera cases
#define NORMAL_NOC_EFFICIENCY_FACTOR         DISPLAY_PROP("normal_noc_efficiency_factor")
#define CAMERA_NOC_EFFICIENCY_FACTOR         DISPLAY_PROP("camera_noc_efficiency_factor")
#define ENABLE_HISTOGRAM_INTR                DISPLAY_PROP("enable_hist_intr")
#define DEFER_FPS_FRAME_COUNT                DISPLAY_PROP("defer_fps_frame_count")
#define ENABLE_BW_LIMITS                     DISPLAY_PROP("enable_bw_limits")
#define DISABLE_ROTATOR_PRE_DOWNSCALER_PROP  DISPLAY_PROP("disable_pre_downscaler")
#define DISABLE_INLINE_ROTATOR_UI_PROP       DISPLAY_PROP("disable_inline_rotator_ui")
#define ENABLE_POMS_DURING_DOZE              DISPLAY_PROP("enable_poms_during_doze")
// RC
#define ENABLE_ROUNDED_CORNER                DISPLAY_PROP("enable_rounded_corner")
#define DISABLE_ROUNDED_CORNER_THREAD        DISPLAY_PROP("disable_rounded_corner_thread")
#define DISABLE_DYNAMIC_FPS                  DISPLAY_PROP("disable_dynamic_fps")
#define ENHANCE_IDLE_TIME                    DISPLAY_PROP("enhance_idle_time")
#define ENABLE_QSYNC_IDLE                    DISPLAY_PROP("enable_qsync_idle")

// PERF hint properties
#define ENABLE_PERF_HINT_LARGE_COMP_CYCLE    DISPLAY_PROP("enable_perf_hint_large_comp_cycle")
#define ENABLE_HDR10_GPU_TARGET              DISPLAY_PROP("enable_hdr10_gpu_target")

// Add all vendor.display properties above

#define DISABLE_UBWC_PROP                    GRALLOC_PROP("disable_ubwc")

// Add all vendor.gralloc.properties above

#define QDCM_DISABLE_TIMEOUT_PROP            PERSIST_DISPLAY_PROP("qdcm.disable_timeout")

// Add all persist.vendor.display.properties above

#define ZERO_SWAP_INTERVAL                   "vendor.debug.egl.swapinterval"
#define WINDOW_RECT_PROP                     DISPLAY_PROP("window_rect")
#define DISABLE_IDLE_TIME_HDR                DISPLAY_PROP("disable_idle_time_hdr")
#define DISABLE_IDLE_TIME_VIDEO              DISPLAY_PROP("disable_idle_time_video")
// Add all other.properties above
// End of property
#endif  // __DISPLAY_PROPERTIES_H__
