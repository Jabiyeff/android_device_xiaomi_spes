/*
 * Copyright (C) 2018 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fcntl.h>
#include <log/log.h>
#include <pthread.h>
#include <sys/epoll.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <drm/msm_drm.h>
#include <drm/msm_drm_pp.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include "histogram_collector.h"
#include "ringbuffer.h"

constexpr static auto implementation_defined_max_frame_ringbuffer = 300;

histogram::HistogramCollector::HistogramCollector()
    : histogram(histogram::Ringbuffer::create(implementation_defined_max_frame_ringbuffer,
                                              std::make_unique<histogram::DefaultTimeKeeper>())) {}

histogram::HistogramCollector::~HistogramCollector() {
  stop();
}

namespace {
static constexpr size_t numBuckets = 8;
static_assert((HIST_V_SIZE % numBuckets) == 0,
              "histogram cannot be rebucketed to smaller number of buckets");
static constexpr int bucket_compression = HIST_V_SIZE / numBuckets;

std::array<uint64_t, numBuckets> rebucketTo8Buckets(
    std::array<uint64_t, HIST_V_SIZE> const &frame) {
  std::array<uint64_t, numBuckets> bins;
  bins.fill(0);
  for (auto i = 0u; i < HIST_V_SIZE; i++)
    bins[i / bucket_compression] += frame[i];
  return bins;
}
}  // namespace

std::string histogram::HistogramCollector::Dump() const {
  uint64_t num_frames = 0;
  std::array<uint64_t, HIST_V_SIZE> all_sample_buckets;
  std::tie(num_frames, all_sample_buckets) = histogram->collect_cumulative();
  std::array<uint64_t, numBuckets> samples = rebucketTo8Buckets(all_sample_buckets);

  std::stringstream ss;
  ss << "Color Sampling, dark (0.0) to light (1.0): sampled frames: " << num_frames << '\n';
  if (num_frames == 0) {
    ss << "\tno color statistics collected\n";
    return ss.str();
  }

  ss << std::fixed << std::setprecision(3);
  ss << "\tbucket\t\t: # of displayed pixels at bucket value\n";
  for (auto i = 0u; i < samples.size(); i++) {
    ss << "\t" << i / static_cast<float>(samples.size()) << " to "
       << (i + 1) / static_cast<float>(samples.size()) << "\t: " << samples[i] << '\n';
  }

  return ss.str();
}

HWC2::Error histogram::HistogramCollector::collect(
    uint64_t max_frames, uint64_t timestamp,
    int32_t out_samples_size[NUM_HISTOGRAM_COLOR_COMPONENTS],
    uint64_t *out_samples[NUM_HISTOGRAM_COLOR_COMPONENTS], uint64_t *out_num_frames) const {
  if (!out_samples_size || !out_num_frames)
    return HWC2::Error::BadParameter;

  out_samples_size[0] = 0;
  out_samples_size[1] = 0;
  out_samples_size[2] = numBuckets;
  out_samples_size[3] = 0;

  uint64_t num_frames = 0;
  std::array<uint64_t, HIST_V_SIZE> samples;

  if (max_frames == 0 && timestamp == 0) {
    std::tie(num_frames, samples) = histogram->collect_cumulative();
  } else if (max_frames == 0) {
    std::tie(num_frames, samples) = histogram->collect_after(timestamp);
  } else if (timestamp == 0) {
    std::tie(num_frames, samples) = histogram->collect_max(max_frames);
  } else {
    std::tie(num_frames, samples) = histogram->collect_max_after(timestamp, max_frames);
  }

  auto samples_rebucketed = rebucketTo8Buckets(samples);
  *out_num_frames = num_frames;
  if (out_samples && out_samples[2])
    memcpy(out_samples[2], samples_rebucketed.data(), sizeof(uint64_t) * samples_rebucketed.size());

  return HWC2::Error::None;
}

HWC2::Error histogram::HistogramCollector::getAttributes(int32_t *format, int32_t *dataspace,
                                                         uint8_t *supported_components) const {
  if (!format || !dataspace || !supported_components)
    return HWC2::Error::BadParameter;

  *format = HAL_PIXEL_FORMAT_HSV_888;
  *dataspace = HAL_DATASPACE_UNKNOWN;
  *supported_components = HWC2_FORMAT_COMPONENT_2;
  return HWC2::Error::None;
}

void histogram::HistogramCollector::start() {
  start(implementation_defined_max_frame_ringbuffer);
}

void histogram::HistogramCollector::start(uint64_t max_frames) {
  std::unique_lock<decltype(mutex)> lk(mutex);
  if (started) {
    return;
  }

  started = true;
  histogram =
      histogram::Ringbuffer::create(max_frames, std::make_unique<histogram::DefaultTimeKeeper>());
  monitoring_thread = std::thread(&HistogramCollector::blob_processing_thread, this);
}

void histogram::HistogramCollector::stop() {
  std::unique_lock<decltype(mutex)> lk(mutex);
  if (!started) {
    return;
  }

  started = false;
  cv.notify_all();
  lk.unlock();

  if (monitoring_thread.joinable())
    monitoring_thread.join();
}

void histogram::HistogramCollector::notify_histogram_event(int blob_source_fd, BlobId id) {
  std::unique_lock<decltype(mutex)> lk(mutex);
  if (!started) {
    ALOGW("Discarding event blob-id: %X", id);
    return;
  }
  if (work_available) {
    ALOGI("notified of histogram event before consuming last one. prior event discarded");
  }

  work_available = true;
  blobwork = HistogramCollector::BlobWork{blob_source_fd, id};
  cv.notify_all();
}

void histogram::HistogramCollector::blob_processing_thread() {
  pthread_setname_np(pthread_self(), "histogram_blob");

  std::unique_lock<decltype(mutex)> lk(mutex);

  while (true) {
    cv.wait(lk, [this] { return !started || work_available; });
    if (!started) {
      return;
    }

    auto work = blobwork;
    work_available = false;
    lk.unlock();

    drmModePropertyBlobPtr blob = drmModeGetPropertyBlob(work.fd, work.id);
    if (!blob || !blob->data) {
      lk.lock();
      continue;
    }
    histogram->insert(*static_cast<struct drm_msm_hist *>(blob->data));
    drmModeFreePropertyBlob(blob);

    lk.lock();
  }
}
