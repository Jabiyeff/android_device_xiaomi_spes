/*
 * Copyright (c) 2017 - 2018, The Linux Foundation. All rights reserved.
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

#ifndef __CONFIG_PARSER_H__
#define __CONFIG_PARSER_H__

//////////////////////////////////////////////////////////////////////////////
//                             Include Files
//////////////////////////////////////////////////////////////////////////////
#include <string>
#include <expat.h>
#include <utils/Log.h>
#include <sys/types.h>
#include <errno.h>
#include <map>

#define BUF_SIZE                    1024

#ifdef __cplusplus
    extern "C" {
#endif

#define VIDC_PLAT_LOGH(fmt, args...) ({ \
          ALOGD(fmt,##args); \
      })

namespace Platform {

class ConfigParser {
    private:
        static void processProperty(const XML_Char **attr, ConfigMap &confMap);
        static void endTag(void *userdata __unused, const XML_Char *tagName __unused);
        static void startTag(void *userdata, const XML_Char *tagName,
            const XML_Char **attr);
        ConfigParser() { }
    public:
        static std::string propFile;
        static int initAndParse(std::string propFile, ConfigMap &confMap);
};

}
#ifdef __cplusplus
}
#endif
#endif // __CONFIG_PARSER_H__
