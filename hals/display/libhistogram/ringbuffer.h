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

#pragma once
#include <drm/msm_drm.h>
#include <drm/msm_drm_pp.h>
#include <sys/types.h>
#include <unistd.h>
#include <utils/Timers.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <array>
#include <deque>
#include <memory>
#include <mutex>
#include <tuple>

namespace histogram {

struct TimeKeeper {
  virtual nsecs_t current_time() const = 0;
  virtual ~TimeKeeper() = default;

 protected:
  TimeKeeper() = default;
  TimeKeeper &operator=(TimeKeeper const &) = delete;
  TimeKeeper(TimeKeeper const &) = delete;
};

struct DefaultTimeKeeper final : TimeKeeper {
  nsecs_t current_time() const final;
};

class Ringbuffer {
 public:
  static std::unique_ptr<Ringbuffer> create(size_t ringbuffer_size, std::unique_ptr<TimeKeeper> tk);
  void insert(drm_msm_hist const &frame);
  bool resize(size_t ringbuffer_size);

  using Sample = std::tuple<uint64_t /* numFrames */, std::array<uint64_t, HIST_V_SIZE> /* bins */>;
  Sample collect_cumulative() const;
  Sample collect_ringbuffer_all() const;
  Sample collect_after(nsecs_t timestamp) const;
  Sample collect_max(uint32_t max_frames) const;
  Sample collect_max_after(nsecs_t timestamp, uint32_t max_frames) const;
  ~Ringbuffer() = default;

 private:
  Ringbuffer(size_t ringbuffer_size, std::unique_ptr<TimeKeeper> tk);
  Ringbuffer(Ringbuffer const &) = delete;
  Ringbuffer &operator=(Ringbuffer const &) = delete;

  Sample collect_max(uint32_t max_frames, std::unique_lock<std::mutex> const &) const;
  Sample collect_max_after(nsecs_t timestamp, uint32_t max_frames,
                           std::unique_lock<std::mutex> const &) const;
  void update_cumulative(nsecs_t now, uint64_t &count,
                         std::array<uint64_t, HIST_V_SIZE> &bins) const;

  std::mutex mutable mutex;
  struct HistogramEntry {
    drm_msm_hist histogram;
    nsecs_t start_timestamp;
    nsecs_t end_timestamp;
  };
  std::deque<HistogramEntry> ringbuffer;
  size_t rb_max_size;
  std::unique_ptr<TimeKeeper> const timekeeper;

  uint64_t cumulative_frame_count;
  std::array<uint64_t, HIST_V_SIZE> cumulative_bins;
};

}  // namespace histogram
