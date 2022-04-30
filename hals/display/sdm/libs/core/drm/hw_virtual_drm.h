/*
Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __HW_VIRTUAL_DRM_H__
#define __HW_VIRTUAL_DRM_H__

#include "hw_device_drm.h"
#include <drm/msm_drm.h>
#include <drm/sde_drm.h>
#include <vector>

namespace sdm {

class HWVirtualDRM : public HWDeviceDRM {
 public:
  HWVirtualDRM(int32_t display_id, BufferAllocator *buffer_allocator,
               HWInfoInterface *hw_info_intf);
  virtual ~HWVirtualDRM() {}
  virtual DisplayError SetVSyncState(bool enable) { return kErrorNotSupported; }
  virtual DisplayError SetMixerAttributes(const HWMixerAttributes &mixer_attributes) {
    return kErrorNotSupported;
  }
  virtual DisplayError SetDisplayAttributes(const HWDisplayAttributes &display_attributes);

 protected:
  virtual DisplayError Validate(HWLayers *hw_layers);
  virtual DisplayError Commit(HWLayers *hw_layers);
  virtual DisplayError Flush(HWLayers *hw_layers);
  virtual DisplayError GetPPFeaturesVersion(PPFeatureVersion *vers);
  virtual DisplayError PowerOn(const HWQosData &qos_data, shared_ptr<Fence> *release_fence);
  virtual DisplayError SetScaleLutConfig(HWScaleLutInfo *lut_info) {
    return kErrorNotSupported;
  }
  virtual DisplayError UnsetScaleLutConfig() {
    return kErrorNotSupported;
  }
  virtual DisplayError GetDisplayIdentificationData(uint8_t *out_port, uint32_t *out_data_size,
                                                    uint8_t *out_data);

 private:
  void ConfigureWbConnectorFbId(uint32_t fb_id);
  void ConfigureWbConnectorDestRect();
  void ConfigureWbConnectorSecureMode(bool secure);
  void InitializeConfigs();
  DisplayError SetWbConfigs(const HWDisplayAttributes &display_attributes);
  void GetModeIndex(const HWDisplayAttributes &display_attributes, int *mode_index);
};

}  // namespace sdm

#endif  // __HW_VIRTUAL_DRM_H__

