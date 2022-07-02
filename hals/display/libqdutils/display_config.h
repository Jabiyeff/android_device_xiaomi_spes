/*
 * Copyright (c) 2013 - 2016, 2018 - 2021, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *    * Neither the name of The Linux Foundation. nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
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

/*
* Changes from Qualcomm Innovation Center are provided under the following license:
*
* Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the
* disclaimer below) provided that the following conditions are met:
*
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*
*    * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
* GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
* HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
* IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _DISPLAY_CONFIG_H
#define _DISPLAY_CONFIG_H

#include <vector>

#include <gralloc_priv.h>
#include <qdMetaData.h>
#include <hardware/hwcomposer.h>

// This header is for clients to use to set/get global display configuration.
// Only primary and external displays are supported here.

namespace qdutils {


/* TODO: Have all the common enums that need be exposed to clients and which
 * are also needed in hwc defined here. Remove such definitions we have in
 * hwc_utils.h
 */

// Use this enum to specify the dpy parameters where needed
enum {
    // DO NOT CHANGE THE LEGACY DEFINES
    DISPLAY_PRIMARY = 0,    // = HWC_DISPLAY_PRIMARY
    DISPLAY_EXTERNAL = 1,   // = HWC_DISPLAY_EXTERNAL
    DISPLAY_VIRTUAL = 2,    // = HWC_DISPLAY_VIRTUAL

    // Additional displays only for vendor client (e.g. pp) reference
    DISPLAY_BUILTIN_2 = 3,
    DISPLAY_EXTERNAL_2 = 4,
};

// External Display states - used in setSecondaryDisplayStatus()
// To be consistent with the same defined in hwc_utils.h
enum {
    EXTERNAL_OFFLINE = 0,
    EXTERNAL_ONLINE,
    EXTERNAL_PAUSE,
    EXTERNAL_RESUME,
};

enum {
    DISABLE_METADATA_DYN_REFRESH_RATE = 0,
    ENABLE_METADATA_DYN_REFRESH_RATE,
    SET_BINDER_DYN_REFRESH_RATE,
};

enum {
    DEFAULT_MODE = 0,
    VIDEO_MODE,
    COMMAND_MODE,
};

enum {
    DISPLAY_PORT_DEFAULT = 0,
    DISPLAY_PORT_DSI,
    DISPLAY_PORT_DTV,
    DISPLAY_PORT_WRITEBACK,
    DISPLAY_PORT_LVDS,
    DISPLAY_PORT_EDP,
    DISPLAY_PORT_DP,
};

// Display Attributes that are available to clients of this library
// Not to be confused with a similar struct in hwc_utils (in the hwc namespace)
typedef struct DisplayAttributes {
    uint32_t vsync_period = 0; //nanoseconds
    uint32_t xres = 0;
    uint32_t yres = 0;
    float xdpi = 0.0f;
    float ydpi = 0.0f;
    int panel_type = DISPLAY_PORT_DEFAULT;
    bool is_yuv = false;
} DisplayAttributes_t;

//=============================================================================
// The functions below run in the client process and wherever necessary
// do a binder call to HWC to get/set data.

// Check if external display is connected. Useful to check before making
// calls for external displays
// Returns 1 if connected, 0 if disconnected, negative values on errors
int isExternalConnected(void);

// Get display vsync period which is in nanoseconds
// i.e vsync_period = 1000000000l / fps
// Returns 0 on success, negative values on errors
int getDisplayAttributes(int dpy, DisplayAttributes_t& dpyattr);

// get the active visible region for the display
// Returns 0 on success, negative values on errors
int getDisplayVisibleRegion(int dpy, hwc_rect_t &rect);

// set the view frame information in hwc context from surfaceflinger
int setViewFrame(int dpy, int l, int t, int r, int b);

// Set the secondary display status(pause/resume/offline etc.,)
int setSecondaryDisplayStatus(int dpy, uint32_t status);

// Enable/Disable/Set refresh rate dynamically
int configureDynRefreshRate(uint32_t op, uint32_t refreshRate);

// Returns the number of configs supported for the display on success.
// Returns -1 on error.
// Only primary display supported for now, value of dpy ignored.
int getConfigCount(int dpy);

// Returns the index of config that is current set for the display on success.
// Returns -1 on error.
// Only primary display supported for now, value of dpy ignored.
int getActiveConfig(int dpy);

// Sets the config for the display on success and returns 0.
// Returns -1 on error.
// Only primary display supported for now, value of dpy ignored
int setActiveConfig(int configIndex, int dpy);

// Returns the attributes for the specified config for the display on success.
// Returns xres and yres as 0 on error.
// Only primary display supported for now, value of dpy ignored
DisplayAttributes getDisplayAttributes(int configIndex, int dpy);

// Set the primary display mode to command or video mode
int setDisplayMode(int mode);

// Sets the panel brightness of the primary display
int setPanelBrightness(int level);

// Retrieves the current panel brightness value
int getPanelBrightness();

// Sets the specified bit clk value.
int setDsiClk(int dpy, uint64_t bitClk);

// Retrieves the current bit clk value.
uint64_t getDsiClk(int dpy);

// Get supported bit clk values.
int getSupportedBitClk(int dpy, std::vector<uint64_t>& bit_rates);

// Sets the specified min and max luminance values.
int setPanelLuminanceAttributes(int dpy, float min_lum, float max_lum);

// Informs HWC about TWM entry/exit based on which NULL Display is connected
// or disconnected.
// mode   -> 0 for exit sequence, 1 for entry sequence.
// is_twm ->
//    0 for regular ambient mode
//    1 for TWM with framework shutdown mode (If display is in ON state, then
//      this would put display in DOZE state).
//      Using 0 for mode value and 1 for is_twm is not valid, it may lead to
//      state inconsistency between Android Framework and HAL.
int setStandByMode(int mode, int is_twm);

// Get Panel Resolution
extern "C" int getPanelResolution(int *width, int *height);

extern "C" int delayFirstCommit(void);

}; //namespace


extern "C" int waitForComposerInit();
#endif
