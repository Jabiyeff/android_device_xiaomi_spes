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

#define LOG_TAG "ConfigParser"

#include <string.h>
#include <errno.h>
#include <utils/Log.h>
#include <sys/mman.h>
#include "PlatformConfig.h"
#include "ConfigParser.h"

namespace Platform {

#define BUF_SIZE                    1024

void ConfigParser::processProperty(const XML_Char **attr, ConfigMap &configMap) {
    if (strcmp(attr[0], "name") != 0) {
        VIDC_PLAT_LOGH("%s: Element 'name' not found!", __func__);
        return;
    }

    std::string propName(attr[1]);

    if (strcmp(attr[2], "value") != 0) {
        VIDC_PLAT_LOGH("%s: Element 'value' not found for %s!", __func__, propName.c_str());
        return;
    }

    std::string propValue(attr[3]);

    configMap[propName] = propValue;

    return;
}

void ConfigParser::startTag(void *userdata, const XML_Char *tagName,
        const XML_Char **attr) {
    if (strcmp(tagName, "property") == 0) {
        processProperty(attr, *static_cast<ConfigMap *>(userdata));
    }
    return;
}

void ConfigParser::endTag(void *userdata __unused, const XML_Char *tagName __unused) {
    return;
}

int ConfigParser::initAndParse(std::string configFile, ConfigMap &configMap) {
    int err = 1;
    XML_Parser parser;
    FILE *file;
    file = fopen(configFile.c_str(), "r");
    if (!file) {
        VIDC_PLAT_LOGH("%s: Error: %d (%s). Using defaults!",
            __func__, errno, strerror(errno));
        return err;
    }

    // Create Parser
    parser = XML_ParserCreate(NULL);
    if (!parser) {
        VIDC_PLAT_LOGH("%s: Failed to create XML parser!", __func__);
        err = -ENODEV;
        goto fileError;
    }

    // Set XML element handlers
    XML_SetUserData(parser, &configMap);
    XML_SetElementHandler(parser, startTag, endTag);
    void *buf;
    int bytesRead;

    // Parse
    while (1) {
        buf = XML_GetBuffer(parser, BUF_SIZE);
        if (buf == NULL) {
            VIDC_PLAT_LOGH("%s: XML_GetBuffer failed", __func__);
            err = -ENOMEM;
            goto parserError;
        }

        bytesRead = fread(buf, 1, BUF_SIZE, file);
        if (bytesRead < 0) {
            VIDC_PLAT_LOGH("%s: fread failed, bytes read = %d", __func__, bytesRead);
            err = bytesRead;
            goto parserError;
        }

        if (XML_ParseBuffer(parser, bytesRead,
                bytesRead == 0) == XML_STATUS_ERROR) {
            VIDC_PLAT_LOGH("%s: XML_ParseBuffer failed, for %s",
                    __func__, configFile.c_str());
            err = -EINVAL;
            goto parserError;
        }
        if (bytesRead == 0)
            break;
    }

    // Free parser and close file in error/ at exit
    parserError:
        XML_ParserFree(parser);
    fileError:
        fclose(file);
    return err;
}

}
