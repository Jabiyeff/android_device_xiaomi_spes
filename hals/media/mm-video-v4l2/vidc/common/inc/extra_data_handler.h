/*--------------------------------------------------------------------------
Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.

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
--------------------------------------------------------------------------*/

#ifndef __EXTRA_DATA_HANDLER_H__
#define __EXTRA_DATA_HANDLER_H__
#include <media/msm_vidc_utils.h>

class client_extradata_info {
    private:
        OMX_U32 size; // size of extradata of each frame
        OMX_U32 buffer_count;
        OMX_BOOL enable;

    public:
        client_extradata_info() {
            size = VENUS_EXTRADATA_SIZE(4096, 2160);
            buffer_count = 0;
            enable = OMX_FALSE;
        }

        ~client_extradata_info() {
        }

        bool set_extradata_info(OMX_U32 size, OMX_U32 buffer_count) {
            this->size = size;
            this->buffer_count = buffer_count;
            return true;
        }
        void enable_client_extradata(OMX_BOOL enable) {
            this->enable = enable;
        }
        bool is_client_extradata_enabled() {
            return enable;
        }
        OMX_U32 getSize() const {
            return size;
        }
        OMX_U32 getBufferCount() const {
            return buffer_count;
        }
};

#endif
