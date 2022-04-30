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
#include <cutils/compiler.h>
#include <log/log.h>
#include <unistd.h>
#include <algorithm>
#include <iostream>

#include "ringbuffer.h"

nsecs_t histogram::DefaultTimeKeeper::current_time() const {
  return systemTime(SYSTEM_TIME_MONOTONIC);
}

histogram::Ringbuffer::Ringbuffer(size_t ringbuffer_size, std::unique_ptr<histogram::TimeKeeper> tk)
    : rb_max_size(ringbuffer_size), timekeeper(std::move(tk)), cumulative_frame_count(0) {
  cumulative_bins.fill(0);
}

std::unique_ptr<histogram::Ringbuffer> histogram::Ringbuffer::create(
    size_t ringbuffer_size, std::unique_ptr<histogram::TimeKeeper> tk) {
  if ((ringbuffer_size == 0) || !tk)
    return nullptr;
  return std::unique_ptr<histogram::Ringbuffer>(
      new histogram::Ringbuffer(ringbuffer_size, std::move(tk)));
}

void histogram::Ringbuffer::update_cumulative(nsecs_t now, uint64_t &count,
                                              std::array<uint64_t, HIST_V_SIZE> &bins) const {
  if (ringbuffer.empty())
    return;

  count++;
  ALOGI("count : %llu", static_cast<unsigned long long>(count));

  const auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::nanoseconds(now - ringbuffer.front().start_timestamp));

  for (auto i = 0u; i < bins.size(); i++) {
    ALOGI("histogram.data[%d]: %u\n", i, ringbuffer.front().histogram.data[i]);
    auto const increment = ringbuffer.front().histogram.data[i] * delta.count();
    if (CC_UNLIKELY((bins[i] + increment < bins[i]) ||
                    (increment < ringbuffer.front().histogram.data[i]))) {
      bins[i] = std::numeric_limits<uint64_t>::max();
    } else {
      bins[i] += increment;
    }
    ALOGI("bins[%d]: %llu\n", i, static_cast<unsigned long long>(bins[i]));
  }
}

void histogram::Ringbuffer::insert(drm_msm_hist const &frame) {
  std::cout << "Enter insert ringbuffer" << std::endl;

  std::unique_lock<decltype(mutex)> lk(mutex);
  auto now = timekeeper->current_time();

  std::cout << "update cumulative count" << std::endl;
  update_cumulative(now, cumulative_frame_count, cumulative_bins);

  if (ringbuffer.size() == rb_max_size)
    ringbuffer.pop_back();
  if (!ringbuffer.empty())
    ringbuffer.front().end_timestamp = now;

  std::cout << "Exit insert ringbuffer" << std::endl;
  ringbuffer.push_front({frame, now, 0});
}

bool histogram::Ringbuffer::resize(size_t ringbuffer_size) {
  std::unique_lock<decltype(mutex)> lk(mutex);
  if (ringbuffer_size == 0)
    return false;
  rb_max_size = ringbuffer_size;
  if (ringbuffer.size() > rb_max_size)
    ringbuffer.resize(rb_max_size);
  return true;
}

histogram::Ringbuffer::Sample histogram::Ringbuffer::collect_cumulative() const {
  std::unique_lock<decltype(mutex)> lk(mutex);
  histogram::Ringbuffer::Sample sample{cumulative_frame_count, cumulative_bins};
  update_cumulative(timekeeper->current_time(), std::get<0>(sample), std::get<1>(sample));
  return sample;
}

histogram::Ringbuffer::Sample histogram::Ringbuffer::collect_ringbuffer_all() const {
  std::unique_lock<decltype(mutex)> lk(mutex);
  return collect_max(ringbuffer.size(), lk);
}

histogram::Ringbuffer::Sample histogram::Ringbuffer::collect_after(nsecs_t timestamp) const {
  std::unique_lock<decltype(mutex)> lk(mutex);
  return collect_max_after(timestamp, ringbuffer.size(), lk);
}

histogram::Ringbuffer::Sample histogram::Ringbuffer::collect_max(uint32_t max_frames) const {
  std::unique_lock<decltype(mutex)> lk(mutex);
  return collect_max(max_frames, lk);
}

histogram::Ringbuffer::Sample histogram::Ringbuffer::collect_max_after(nsecs_t timestamp,
                                                                       uint32_t max_frames) const {
  std::unique_lock<decltype(mutex)> lk(mutex);
  return collect_max_after(timestamp, max_frames, lk);
}

histogram::Ringbuffer::Sample histogram::Ringbuffer::collect_max(
    uint32_t max_frames, std::unique_lock<std::mutex> const &) const {
  auto collect_first = std::min(static_cast<size_t>(max_frames), ringbuffer.size());
  if (collect_first == 0)
    return {0, {}};
  std::array<uint64_t, HIST_V_SIZE> bins;
  bins.fill(0);
  for (auto it = ringbuffer.begin(); it != ringbuffer.end() &&
    it != ringbuffer.begin() + collect_first; it++) {
    nsecs_t end_timestamp = it->end_timestamp;
    if (it == ringbuffer.begin()) {
      end_timestamp = timekeeper->current_time();
    }
    const auto time_displayed = std::chrono::nanoseconds(end_timestamp - it->start_timestamp);
    const auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(time_displayed);
    for (auto i = 0u; i < HIST_V_SIZE; i++) {
      bins[i] += it->histogram.data[i] * delta.count();
    }
  }
  return {collect_first, bins};
}

histogram::Ringbuffer::Sample histogram::Ringbuffer::collect_max_after(
    nsecs_t timestamp, uint32_t max_frames, std::unique_lock<std::mutex> const &lk) const {
  auto ts_filter_begin = std::lower_bound(
      ringbuffer.begin(), ringbuffer.end(), HistogramEntry{{}, timestamp, 0},
      [](auto const &a, auto const &b) { return a.start_timestamp >= b.start_timestamp; });

  auto collect_last = std::min(std::distance(ringbuffer.begin(), ts_filter_begin),
                               static_cast<std::ptrdiff_t>(max_frames));
  return collect_max(collect_last, lk);
}
