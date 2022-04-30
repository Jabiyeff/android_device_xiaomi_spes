/*
 * Copyright (c) 2014-2021, The Linux Foundation. All rights reserved.
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

#ifndef __HWC_DISPLAY_BUILTIN_H__
#define __HWC_DISPLAY_BUILTIN_H__

#include <thermal_client.h>
#include <mutex>
#include <string>
#include <vector>

#include "utils/sync_task.h"
#include "utils/constants.h"
#include "cpuhint.h"
#include "hwc_display.h"
#include "hwc_layers.h"
#include "display_null.h"

#include "gl_layer_stitch.h"

namespace sdm {

enum class LayerStitchTaskCode : int32_t {
  kCodeGetInstance,
  kCodeStitch,
  kCodeDestroyInstance,
};

struct LayerStitchGetInstanceContext : public SyncTask<LayerStitchTaskCode>::TaskContext {
  LayerBuffer *output_buffer = NULL;
};

struct LayerStitchContext : public SyncTask<LayerStitchTaskCode>::TaskContext {
  vector<StitchParams> stitch_params;
  shared_ptr<Fence> src_acquire_fence = nullptr;
  shared_ptr<Fence> dst_acquire_fence = nullptr;
  shared_ptr<Fence> release_fence = nullptr;
};

class HWCDisplayBuiltIn : public HWCDisplay, public SyncTask<LayerStitchTaskCode>::TaskHandler {
 public:
  enum {
    SET_METADATA_DYN_REFRESH_RATE,
    SET_BINDER_DYN_REFRESH_RATE,
    SET_DISPLAY_MODE,
    SET_QDCM_SOLID_FILL_INFO,
    UNSET_QDCM_SOLID_FILL_INFO,
    SET_QDCM_SOLID_FILL_RECT,
  };

  static int Create(CoreInterface *core_intf, BufferAllocator *buffer_allocator,
                    HWCCallbacks *callbacks,  HWCDisplayEventHandler *event_handler,
                    qService::QService *qservice, hwc2_display_t id, int32_t sdm_id,
                    HWCDisplay **hwc_display);
  static void Destroy(HWCDisplay *hwc_display);
  virtual int Init();
  virtual HWC2::Error Validate(uint32_t *out_num_types, uint32_t *out_num_requests);
  virtual HWC2::Error Present(shared_ptr<Fence> *out_retire_fence);
  virtual HWC2::Error CommitLayerStack();
  virtual HWC2::Error GetColorModes(uint32_t *out_num_modes, ColorMode *out_modes);
  virtual HWC2::Error SetColorMode(ColorMode mode);
  virtual HWC2::Error GetRenderIntents(ColorMode mode, uint32_t *out_num_intents,
                                       RenderIntent *out_intents);
  virtual HWC2::Error SetColorModeWithRenderIntent(ColorMode mode, RenderIntent intent);
  virtual HWC2::Error SetColorModeById(int32_t color_mode_id);
  virtual HWC2::Error SetColorModeFromClientApi(int32_t color_mode_id);
  virtual HWC2::Error SetColorTransform(const float *matrix, android_color_transform_t hint);
  virtual HWC2::Error RestoreColorTransform();
  virtual int Perform(uint32_t operation, ...);
  virtual int HandleSecureSession(const std::bitset<kSecureMax> &secure_session,
                                  bool *power_on_pending, bool is_active_secure_display);
  virtual void SetIdleTimeoutMs(uint32_t timeout_ms, uint32_t inactive_ms);
  virtual HWC2::Error SetFrameDumpConfig(uint32_t count, uint32_t bit_mask_layer_type,
                                         int32_t format, bool post_processed);
  virtual int FrameCaptureAsync(const BufferInfo &output_buffer_info, bool post_processed);
  virtual int GetFrameCaptureStatus() { return frame_capture_status_; }
  virtual DisplayError SetDetailEnhancerConfig(const DisplayDetailEnhancerData &de_data);
  virtual DisplayError SetHWDetailedEnhancerConfig(void *params);
  virtual DisplayError ControlPartialUpdate(bool enable, uint32_t *pending);
  virtual HWC2::Error SetReadbackBuffer(const native_handle_t *buffer,
                                        shared_ptr<Fence> acquire_fence,
                                        bool post_processed_output, CWBClient client);
  virtual HWC2::Error GetReadbackBufferFence(shared_ptr<Fence> *release_fence);
  virtual HWC2::Error SetQSyncMode(QSyncMode qsync_mode);
  virtual DisplayError ControlIdlePowerCollapse(bool enable, bool synchronous);
  virtual HWC2::Error SetDisplayDppsAdROI(uint32_t h_start, uint32_t h_end, uint32_t v_start,
                                          uint32_t v_end, uint32_t factor_in, uint32_t factor_out);
  virtual DisplayError SetDynamicDSIClock(uint64_t bitclk);
  virtual DisplayError GetDynamicDSIClock(uint64_t *bitclk);
  virtual DisplayError GetSupportedDSIClock(std::vector<uint64_t> *bitclk_rates);
  virtual DisplayError SetStandByMode(bool enable);
  virtual HWC2::Error UpdateDisplayId(hwc2_display_t id);
  virtual HWC2::Error SetPendingRefresh();
  virtual HWC2::Error SetPanelBrightness(float brightness);
  virtual HWC2::Error GetPanelBrightness(float *brightness);
  virtual HWC2::Error GetPanelMaxBrightness(uint32_t *max_brightness_level);
  virtual DisplayError TeardownConcurrentWriteback(void);
  virtual void SetFastPathComposition(bool enable) {
    fast_path_composition_ = enable && !readback_buffer_queued_;
  }
  virtual HWC2::Error SetFrameTriggerMode(uint32_t mode);
  virtual HWC2::Error SetBLScale(uint32_t level);
  virtual HWC2::Error UpdatePowerMode(HWC2::PowerMode mode);
  virtual HWC2::Error SetClientTarget(buffer_handle_t target, shared_ptr<Fence> acquire_fence,
                                      int32_t dataspace, hwc_region_t damage);
  virtual bool IsSmartPanelConfig(uint32_t config_id);
  virtual bool HasSmartPanelConfig(void);
  virtual int Deinit();
  virtual bool IsQsyncCallbackNeeded(bool *qsync_enabled, int32_t *refresh_rate,
                                     int32_t *qsync_refresh_rate);
  virtual int PostInit();

  virtual HWC2::Error SetDisplayedContentSamplingEnabledVndService(bool enabled);
  virtual HWC2::Error SetDisplayedContentSamplingEnabled(int32_t enabled, uint8_t component_mask,
                                                         uint64_t max_frames);
  virtual HWC2::Error GetDisplayedContentSamplingAttributes(int32_t *format, int32_t *dataspace,
                                                            uint8_t *supported_components);
  virtual HWC2::Error GetDisplayedContentSample(
      uint64_t max_frames, uint64_t timestamp, uint64_t *numFrames,
      int32_t samples_size[NUM_HISTOGRAM_COLOR_COMPONENTS],
      uint64_t *samples[NUM_HISTOGRAM_COLOR_COMPONENTS]);
  void Dump(std::ostringstream *os) override;
  virtual HWC2::Error SetPowerMode(HWC2::PowerMode mode, bool teardown);
  virtual bool HasReadBackBufferSupport();
  virtual bool IsDisplayIdle();

 private:
  HWCDisplayBuiltIn(CoreInterface *core_intf, BufferAllocator *buffer_allocator,
                    HWCCallbacks *callbacks, HWCDisplayEventHandler *event_handler,
                    qService::QService *qservice, hwc2_display_t id, int32_t sdm_id);
  void SetMetaDataRefreshRateFlag(bool enable);
  virtual DisplayError SetDisplayMode(uint32_t mode);
  virtual DisplayError DisablePartialUpdateOneFrame();
  void ProcessBootAnimCompleted(void);
  void SetQDCMSolidFillInfo(bool enable, const LayerSolidFill &color);
  void ToggleCPUHint(bool set);
  void ForceRefreshRate(uint32_t refresh_rate);
  uint32_t GetOptimalRefreshRate(bool one_updating_layer);
  void HandleFrameOutput();
  void HandleFrameDump();
  void HandleFrameCapture();
  bool CanSkipCommit();
  DisplayError SetMixerResolution(uint32_t width, uint32_t height);
  DisplayError GetMixerResolution(uint32_t *width, uint32_t *height);
  HWC2::Error CommitStitchLayers();
  void AppendStitchLayer();
  bool InitLayerStitch();
  void InitStitchTarget();
  bool AllocateStitchBuffer();
  void CacheAvrStatus();
  void PostCommitStitchLayers();
  void SetCpuPerfHintLargeCompCycle();
  int GetBwCode(const DisplayConfigVariableInfo &attr);
  void SetBwLimitHint(bool enable);
  void SetPartialUpdate(DisplayConfigFixedInfo fixed_info);
  uint32_t GetUpdatingAppLayersCount();
  void ValidateUiScaling();

  // SyncTask methods.
  void OnTask(const LayerStitchTaskCode &task_code,
              SyncTask<LayerStitchTaskCode>::TaskContext *task_context);

  constexpr static int kBwLow = 2;
  constexpr static int kBwMedium = 3;
  constexpr static int kBwHigh = 4;
  const int kPerfHintLargeCompCycle = 0x00001097;
  BufferAllocator *buffer_allocator_ = nullptr;
  CPUHint *cpu_hint_ = nullptr;
  CWBClient cwb_client_ = kCWBClientNone;

  // Builtin readback buffer configuration
  LayerBuffer output_buffer_ = {};
  bool post_processed_output_ = false;
  bool readback_buffer_queued_ = false;
  bool readback_configured_ = false;

  // Members for N frame output dump to file
  bool dump_output_to_file_ = false;
  BufferInfo output_buffer_info_ = {};
  void *output_buffer_base_ = nullptr;
  bool pending_refresh_ = true;
  bool enable_optimize_refresh_ = false;
  bool enable_poms_during_doze_ = false;

  // Members for 1 frame capture in a client provided buffer
  bool frame_capture_buffer_queued_ = false;
  int frame_capture_status_ = -EAGAIN;
  bool is_primary_ = false;
  bool disable_layer_stitch_ = true;
  HWCLayer* stitch_target_ = nullptr;
  SyncTask<LayerStitchTaskCode> layer_stitch_task_;
  GLLayerStitch* gl_layer_stitch_ = nullptr;
  BufferInfo buffer_info_ = {};
  DisplayConfigVariableInfo fb_config_ = {};

  bool qsync_enabled_ = false;
  bool qsync_reconfigured_ = false;
  // Members for Color sampling feature
  DisplayError HistogramEvent(int fd, uint32_t blob_id) override;
  histogram::HistogramCollector histogram;
  std::mutex sampling_mutex;
  bool api_sampling_vote = false;
  bool vndservice_sampling_vote = false;
  int perf_hint_window_ = 0;
  int perf_hint_large_comp_cycle_ = 0;
  int curr_refresh_rate_ = 0;
  bool is_smart_panel_ = false;
  const char *kDisplayBwName = "display_bw";
  bool enable_bw_limits_ = false;
  bool disable_dyn_fps_ = false;
  bool enhance_idle_time_ = false;
  bool force_reset_validate_ = false;

  // NULL display
  DisplayNull display_null_;
  DisplayInterface *stored_display_intf_ = NULL;
};

}  // namespace sdm

#endif  // __HWC_DISPLAY_BUILTIN_H__
