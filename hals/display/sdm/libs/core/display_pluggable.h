/*
* Copyright (c) 2014-2020, The Linux Foundation. All rights reserved.
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

#ifndef __DISPLAY_PLUGGABLE_H__
#define __DISPLAY_PLUGGABLE_H__

#include <map>
#include <string>
#include <vector>

#include "display_base.h"
#include "hw_events_interface.h"

namespace sdm {

class DisplayPluggable : public DisplayBase, HWEventHandler {
 public:
  DisplayPluggable(DisplayEventHandler *event_handler, HWInfoInterface *hw_info_intf,
                   BufferAllocator *buffer_allocator, CompManager *comp_manager);
  DisplayPluggable(int32_t display_id, DisplayEventHandler *event_handler,
                   HWInfoInterface *hw_info_intf, BufferAllocator *buffer_allocator,
                   CompManager *comp_manager);
  virtual DisplayError Init();
  virtual DisplayError Prepare(LayerStack *layer_stack);
  virtual DisplayError GetRefreshRateRange(uint32_t *min_refresh_rate, uint32_t *max_refresh_rate);
  virtual DisplayError SetRefreshRate(uint32_t refresh_rate, bool final_rate, bool idle_screen);
  virtual bool IsUnderscanSupported();
  virtual DisplayError InitializeColorModes();
  virtual DisplayError SetColorMode(const std::string &color_mode);
  virtual DisplayError GetColorModeCount(uint32_t *mode_count);
  virtual DisplayError GetColorModes(uint32_t *mode_count, std::vector<std::string> *color_modes);
  virtual DisplayError GetColorModeAttr(const std::string &color_mode, AttrVal *attr);
  virtual DisplayError SetColorTransform(const uint32_t length, const double *color_transform) {
    return kErrorNone;
  }
  virtual DisplayError TeardownConcurrentWriteback(void) { return kErrorNotSupported; }
  virtual DisplayError colorSamplingOn();
  virtual DisplayError colorSamplingOff();

  // Implement the HWEventHandlers
  virtual DisplayError VSync(int64_t timestamp);
  virtual DisplayError Blank(bool blank) { return kErrorNone; }
  virtual void IdleTimeout() {}
  virtual void ThermalEvent(int64_t thermal_level) {}
  virtual void CECMessage(char *message);
  virtual void IdlePowerCollapse() {}
  virtual void PingPongTimeout() {}
  virtual void PanelDead() {}
  virtual void HwRecovery(const HWRecoveryEvent sdm_event_code);
  void Histogram(int histogram_fd, uint32_t blob_id) override;

  void UpdateColorModes();
  void InitializeColorModesFromColorspace();

 private:
  DisplayError GetOverrideConfig(uint32_t *mode_index);
  void GetScanSupport();

  static const int kPropertyMax = 256;

  bool underscan_supported_ = false;
  HWScanSupport scan_support_;
  std::vector<HWEvent> event_list_ = {HWEvent::VSYNC, HWEvent::IDLE_NOTIFY, HWEvent::EXIT,
                                      HWEvent::CEC_READ_MESSAGE, HWEvent::HW_RECOVERY};
  uint32_t current_refresh_rate_ = 0;
};

}  // namespace sdm

#endif  // __DISPLAY_PLUGGABLE_H__
