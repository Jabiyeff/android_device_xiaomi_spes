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

#ifndef HISTOGRAM_HISTOGRAM_COLLECTOR_H_
#define HISTOGRAM_HISTOGRAM_COLLECTOR_H_
#include <android-base/thread_annotations.h>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#define HWC2_INCLUDE_STRINGIFICATION
#define HWC2_USE_CPP11
#include <hardware/hwcomposer2.h>
#undef HWC2_INCLUDE_STRINGIFICATION
#undef HWC2_USE_CPP11

// number of enums in hwc2_format_color_component_t;
#define NUM_HISTOGRAM_COLOR_COMPONENTS 4

namespace histogram {
typedef uint32_t BlobId;

class Ringbuffer;
class HistogramCollector {
 public:
  HistogramCollector();
  ~HistogramCollector();

  void start();
  void start(uint64_t max_frames);
  void stop();

  void notify_histogram_event(int blob_source_fd, BlobId id);

  std::string Dump() const;

  HWC2::Error collect(uint64_t max_frames, uint64_t timestamp,
                      int32_t samples_size[NUM_HISTOGRAM_COLOR_COMPONENTS],
                      uint64_t *samples[NUM_HISTOGRAM_COLOR_COMPONENTS], uint64_t *numFrames) const;
  HWC2::Error getAttributes(int32_t *format, int32_t *dataspace,
                            uint8_t *supported_components) const;

 private:
  HistogramCollector(HistogramCollector const &) = delete;
  HistogramCollector &operator=(HistogramCollector const &) = delete;
  void blob_processing_thread();

  std::condition_variable cv;
  std::mutex mutable mutex;
  bool started /* GUARDED_BY(mutex) */ = false;

  struct BlobWork {
    int fd; /* non-owning! */
    BlobId id;
  } blobwork /* GUARDED_BY(mutex) */;
  // no optional in c++14.
  bool work_available = false; /* GUARDED_BY(mutex) */
  ;

  std::thread monitoring_thread;

  std::unique_ptr<histogram::Ringbuffer> histogram;
};

}  // namespace histogram

#endif  // HISTOGRAM_HISTOGRAM_COLLECTOR_H_
