#
# Copyright (C) 2020-2021 The LineageOS Project
#
# SPDX-License-Identifier: Apache-2.0
#

import common
import re

def FullOTA_InstallEnd(info):
    OTA_InstallEnd(info)
    return

def IncrementalOTA_InstallEnd(info):
    OTA_InstallEnd(info)
    return

def AddImage(info, basename, dest):
    name = basename
    data = info.input_zip.read("IMAGES/" + basename)
    common.ZipWriteStr(info.output_zip, name, data)
    info.script.AppendExtra('package_extract_file("%s", "%s");' % (name, dest))

def OTA_InstallEnd(info):
    info.script.Print("Patching firmware images...")
    AddImage(info, "dtbo.img", "/dev/block/bootdevice/by-name/dtbo")
    AddImage(info, "vbmeta.img", "/dev/block/bootdevice/by-name/vbmeta")
    AddImage(info, "vbmeta_system.img", "/dev/block/bootdevice/by-name/vbmeta_system")
    return
