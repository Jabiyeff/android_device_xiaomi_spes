/*
 * Copyright (C) 2021 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <libinit_dalvik_heap.h>
#include <libinit_variant.h>

#include "vendor_init.h"

static const variant_info_t spes_info = {
    .hwc_value = "Global",
    .sku_value = "",

    .brand = "Redmi",
    .device = "spes",
    .marketname = "Redmi Note 11",
    .model = "2201117TG",

    .nfc = false,
};

static const variant_info_t spes_in_info = {
    .hwc_value = "India",
    .sku_value = "",

    .brand = "Redmi",
    .device = "spes",
    .marketname = "Redmi Note 11",
    .model = "2201117TI",

    .nfc = false,
};

static const variant_info_t spesn_info = {
    .hwc_value = "",
    .sku_value = "k7tn",

    .brand = "Redmi",
    .device = "spesn",
    .marketname = "Redmi Note 11",
    .model = "2201117TY",

    .nfc = true,
};

static const std::vector<variant_info_t> variants = {
    spes_info,
    spes_in_info,
    spesn_info,
};

void vendor_load_properties() {
    search_variant(variants);
    set_dalvik_heap();
}
