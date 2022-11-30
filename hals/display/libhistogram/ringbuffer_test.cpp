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

#include <chrono>
#include <numeric>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "ringbuffer.h"
using namespace testing;
using namespace std::chrono_literals;

template <typename Rep, typename Per>
nsecs_t toNsecs(std::chrono::duration<Rep, Per> time) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(time).count();
}

template <typename Rep, typename Per>
uint64_t toMs(std::chrono::duration<Rep, Per> time) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(time).count();
}

struct TimeKeeperWrapper : histogram::TimeKeeper {
  TimeKeeperWrapper(std::shared_ptr<histogram::TimeKeeper> const &tk) : tk(tk) {}
  nsecs_t current_time() const final { return tk->current_time(); }
  std::shared_ptr<histogram::TimeKeeper> const tk;
};

struct TickingTimeKeeper : histogram::TimeKeeper {
  void tick() { fake_time = fake_time + toNsecs(1ms); }

  void increment_by(std::chrono::nanoseconds inc) { fake_time = fake_time + inc.count(); }

  nsecs_t current_time() const final { return fake_time; }

 private:
  nsecs_t mutable fake_time = 0;
};

void insertFrameIncrementTimeline(histogram::Ringbuffer &rb, TickingTimeKeeper &tk,
                                  drm_msm_hist &frame) {
  rb.insert(frame);
  tk.tick();
}

class RingbufferTestCases : public ::testing::Test {
  void SetUp() {
    for (auto i = 0u; i < HIST_V_SIZE; i++) {
      frame0.data[i] = fill_frame0;
      frame1.data[i] = fill_frame1;
      frame2.data[i] = fill_frame2;
      frame3.data[i] = fill_frame3;
      frame4.data[i] = fill_frame4;
      frame_saturate.data[i] = std::numeric_limits<uint32_t>::max();
    }
  }

 protected:
  std::unique_ptr<histogram::Ringbuffer> createFilledRingbuffer(
      std::shared_ptr<TickingTimeKeeper> const &tk) {
    auto rb = histogram::Ringbuffer::create(4, std::make_unique<TimeKeeperWrapper>(tk));
    insertFrameIncrementTimeline(*rb, *tk, frame0);
    insertFrameIncrementTimeline(*rb, *tk, frame1);
    insertFrameIncrementTimeline(*rb, *tk, frame2);
    insertFrameIncrementTimeline(*rb, *tk, frame3);
    return rb;
  }

  uint64_t fill_frame0 = 9;
  uint64_t fill_frame1 = 11;
  uint64_t fill_frame2 = 303;
  uint64_t fill_frame3 = 1030;
  uint64_t fill_frame4 = 112200;
  drm_msm_hist frame0;
  drm_msm_hist frame1;
  drm_msm_hist frame2;
  drm_msm_hist frame3;
  drm_msm_hist frame4;
  drm_msm_hist frame_saturate;

  int numFrames = 0;
  std::array<uint64_t, HIST_V_SIZE> bins;
};

TEST_F(RingbufferTestCases, ZeroSizedRingbufferReturnsNull) {
  EXPECT_THAT(histogram::Ringbuffer::create(0, std::make_unique<TickingTimeKeeper>()), Eq(nullptr));
}

TEST_F(RingbufferTestCases, NullTimekeeperReturnsNull) {
  EXPECT_THAT(histogram::Ringbuffer::create(10, nullptr), Eq(nullptr));
}

TEST_F(RingbufferTestCases, CollectionWithNoFrames) {
  auto rb = histogram::Ringbuffer::create(1, std::make_unique<TickingTimeKeeper>());

  std::tie(numFrames, bins) = rb->collect_ringbuffer_all();
  EXPECT_THAT(numFrames, Eq(0));
  EXPECT_THAT(bins, Each(0));
}

TEST_F(RingbufferTestCases, SimpleTest) {
  static constexpr int numInsertions = 3u;
  auto tk = std::make_shared<TickingTimeKeeper>();
  auto rb = histogram::Ringbuffer::create(numInsertions, std::make_unique<TimeKeeperWrapper>(tk));

  drm_msm_hist frame;
  for (auto i = 0u; i < HIST_V_SIZE; i++) {
    frame.data[i] = i;
  }

  insertFrameIncrementTimeline(*rb, *tk, frame);
  insertFrameIncrementTimeline(*rb, *tk, frame);
  insertFrameIncrementTimeline(*rb, *tk, frame);

  std::tie(numFrames, bins) = rb->collect_ringbuffer_all();

  ASSERT_THAT(bins.size(), Eq(HIST_V_SIZE));
  for (auto i = 0u; i < bins.size(); i++) {
    EXPECT_THAT(bins[i], Eq(toMs(3ms) * i));
  }
}

TEST_F(RingbufferTestCases, TestEvictionSingle) {
  int fill_frame0 = 9;
  int fill_frame1 = 111;
  drm_msm_hist frame0;
  drm_msm_hist frame1;
  for (auto i = 0u; i < HIST_V_SIZE; i++) {
    frame0.data[i] = fill_frame0;
    frame1.data[i] = fill_frame1;
  }

  auto tk = std::make_shared<TickingTimeKeeper>();
  auto rb = histogram::Ringbuffer::create(1, std::make_unique<TimeKeeperWrapper>(tk));

  insertFrameIncrementTimeline(*rb, *tk, frame0);

  std::tie(numFrames, bins) = rb->collect_ringbuffer_all();
  EXPECT_THAT(numFrames, Eq(1));
  EXPECT_THAT(bins, Each(fill_frame0));

  insertFrameIncrementTimeline(*rb, *tk, frame1);
  std::tie(numFrames, bins) = rb->collect_ringbuffer_all();
  EXPECT_THAT(numFrames, Eq(1));
  EXPECT_THAT(bins, Each(fill_frame1));
}

TEST_F(RingbufferTestCases, TestEvictionMultiple) {
  auto tk = std::make_shared<TickingTimeKeeper>();
  auto rb = histogram::Ringbuffer::create(3, std::make_unique<TimeKeeperWrapper>(tk));

  insertFrameIncrementTimeline(*rb, *tk, frame0);
  insertFrameIncrementTimeline(*rb, *tk, frame1);
  insertFrameIncrementTimeline(*rb, *tk, frame2);

  std::tie(numFrames, bins) = rb->collect_ringbuffer_all();
  EXPECT_THAT(numFrames, Eq(3));
  EXPECT_THAT(bins, Each(fill_frame0 + fill_frame1 + fill_frame2));

  insertFrameIncrementTimeline(*rb, *tk, frame3);
  std::tie(numFrames, bins) = rb->collect_ringbuffer_all();
  EXPECT_THAT(numFrames, Eq(3));
  EXPECT_THAT(bins, Each(fill_frame1 + fill_frame2 + fill_frame3));

  insertFrameIncrementTimeline(*rb, *tk, frame0);
  std::tie(numFrames, bins) = rb->collect_ringbuffer_all();
  EXPECT_THAT(numFrames, Eq(3));
  EXPECT_THAT(bins, Each(fill_frame2 + fill_frame3 + fill_frame0));
}

TEST_F(RingbufferTestCases, TestResizeToZero) {
  auto rb = histogram::Ringbuffer::create(4, std::make_unique<TickingTimeKeeper>());
  EXPECT_FALSE(rb->resize(0));
}

TEST_F(RingbufferTestCases, TestResizeDown) {
  auto tk = std::make_shared<TickingTimeKeeper>();
  auto rb = createFilledRingbuffer(tk);

  std::tie(numFrames, bins) = rb->collect_ringbuffer_all();
  EXPECT_THAT(numFrames, Eq(4));
  EXPECT_THAT(bins, Each(fill_frame0 + fill_frame1 + fill_frame2 + fill_frame3));

  auto rc = rb->resize(2);
  EXPECT_THAT(rc, Eq(true));
  std::tie(numFrames, bins) = rb->collect_ringbuffer_all();
  EXPECT_THAT(numFrames, Eq(2));
  EXPECT_THAT(bins, Each(fill_frame2 + fill_frame3));

  insertFrameIncrementTimeline(*rb, *tk, frame0);
  std::tie(numFrames, bins) = rb->collect_ringbuffer_all();
  EXPECT_THAT(numFrames, Eq(2));
  EXPECT_THAT(bins, Each(fill_frame0 + fill_frame3));
}

TEST_F(RingbufferTestCases, TestResizeUp) {
  auto tk = std::make_shared<TickingTimeKeeper>();
  auto rb = histogram::Ringbuffer::create(2, std::make_unique<TimeKeeperWrapper>(tk));

  insertFrameIncrementTimeline(*rb, *tk, frame0);
  insertFrameIncrementTimeline(*rb, *tk, frame1);

  std::tie(numFrames, bins) = rb->collect_ringbuffer_all();
  EXPECT_THAT(numFrames, Eq(2));
  EXPECT_THAT(bins, Each(fill_frame0 + fill_frame1));

  auto rc = rb->resize(3);
  EXPECT_THAT(rc, Eq(true));
  std::tie(numFrames, bins) = rb->collect_ringbuffer_all();
  EXPECT_THAT(numFrames, Eq(2));
  EXPECT_THAT(bins, Each(fill_frame0 + fill_frame1));

  insertFrameIncrementTimeline(*rb, *tk, frame2);
  std::tie(numFrames, bins) = rb->collect_ringbuffer_all();
  EXPECT_THAT(numFrames, Eq(3));
  EXPECT_THAT(bins, Each(fill_frame0 + fill_frame1 + fill_frame2));

  insertFrameIncrementTimeline(*rb, *tk, frame3);
  std::tie(numFrames, bins) = rb->collect_ringbuffer_all();
  EXPECT_THAT(numFrames, Eq(3));
  EXPECT_THAT(bins, Each(fill_frame1 + fill_frame2 + fill_frame3));
}

TEST_F(RingbufferTestCases, TestTimestampFiltering) {
  auto rb = createFilledRingbuffer(std::make_shared<TickingTimeKeeper>());

  std::tie(numFrames, bins) = rb->collect_after(toNsecs(1500us));
  EXPECT_THAT(numFrames, Eq(2));
  EXPECT_THAT(bins, Each(fill_frame2 + fill_frame3));

  std::tie(numFrames, bins) = rb->collect_after(toNsecs(45000us));
  EXPECT_THAT(numFrames, Eq(0));

  std::tie(numFrames, bins) = rb->collect_after(0);
  EXPECT_THAT(numFrames, Eq(4));
  EXPECT_THAT(bins, Each(fill_frame0 + fill_frame1 + fill_frame2 + fill_frame3));
}

TEST_F(RingbufferTestCases, TestTimestampFilteringSameTimestamp) {
  auto tk = std::make_shared<TickingTimeKeeper>();
  auto rb = histogram::Ringbuffer::create(4, std::make_unique<TimeKeeperWrapper>(tk));
  insertFrameIncrementTimeline(*rb, *tk, frame0);
  insertFrameIncrementTimeline(*rb, *tk, frame1);
  insertFrameIncrementTimeline(*rb, *tk, frame2);
  rb->insert(frame3);
  rb->insert(frame4);
  tk->tick();

  std::tie(numFrames, bins) = rb->collect_after(toNsecs(3ms));
  EXPECT_THAT(numFrames, Eq(2));
  EXPECT_THAT(bins, Each(fill_frame4));
}

TEST_F(RingbufferTestCases, TestFrameFiltering) {
  auto rb = createFilledRingbuffer(std::make_shared<TickingTimeKeeper>());

  std::tie(numFrames, bins) = rb->collect_max(2);
  EXPECT_THAT(numFrames, Eq(2));
  EXPECT_THAT(bins, Each(fill_frame2 + fill_frame3));

  std::tie(numFrames, bins) = rb->collect_max(0);
  EXPECT_THAT(numFrames, Eq(0));
  EXPECT_THAT(bins, Each(0));

  std::tie(numFrames, bins) = rb->collect_max(3);
  EXPECT_THAT(numFrames, Eq(3));
  EXPECT_THAT(bins, Each(fill_frame1 + fill_frame2 + fill_frame3));

  std::tie(numFrames, bins) = rb->collect_max(8);
  EXPECT_THAT(numFrames, Eq(4));
  EXPECT_THAT(bins, Each(fill_frame0 + fill_frame1 + fill_frame2 + fill_frame3));
}

TEST_F(RingbufferTestCases, TestTimestampAndFrameFiltering) {
  auto rb = createFilledRingbuffer(std::make_shared<TickingTimeKeeper>());

  std::tie(numFrames, bins) = rb->collect_max_after(toNsecs(1500us), 1);
  EXPECT_THAT(numFrames, Eq(1));
  EXPECT_THAT(bins, Each(fill_frame3));

  std::tie(numFrames, bins) = rb->collect_max_after(toNsecs(2500us), 0);
  EXPECT_THAT(numFrames, Eq(0));
  EXPECT_THAT(bins, Each(0));

  std::tie(numFrames, bins) = rb->collect_max_after(toNsecs(10ms), 100);
  EXPECT_THAT(numFrames, Eq(0));
  EXPECT_THAT(bins, Each(0));

  std::tie(numFrames, bins) = rb->collect_max_after(toNsecs(0ns), 10);
  EXPECT_THAT(numFrames, Eq(4));
  EXPECT_THAT(bins, Each(fill_frame0 + fill_frame1 + fill_frame2 + fill_frame3));
}

TEST_F(RingbufferTestCases, TestTimestampAndFrameFilteringAndResize) {
  auto rb = createFilledRingbuffer(std::make_shared<TickingTimeKeeper>());

  std::tie(numFrames, bins) = rb->collect_max_after(toNsecs(500us), 1);
  EXPECT_THAT(numFrames, Eq(1));
  EXPECT_THAT(bins, Each(fill_frame3));

  std::tie(numFrames, bins) = rb->collect_max_after(toNsecs(500us), 10);
  EXPECT_THAT(numFrames, Eq(3));
  EXPECT_THAT(bins, Each(fill_frame1 + fill_frame2 + fill_frame3));

  rb->resize(2);
  std::tie(numFrames, bins) = rb->collect_max_after(toNsecs(500us), 10);
  EXPECT_THAT(numFrames, Eq(2));
  EXPECT_THAT(bins, Each(fill_frame2 + fill_frame3));
}

TEST_F(RingbufferTestCases, TestCumulativeCounts) {
  auto tk = std::make_shared<TickingTimeKeeper>();
  auto rb = histogram::Ringbuffer::create(1, std::make_unique<TimeKeeperWrapper>(tk));
  insertFrameIncrementTimeline(*rb, *tk, frame0);

  std::tie(numFrames, bins) = rb->collect_ringbuffer_all();
  EXPECT_THAT(numFrames, Eq(1));
  EXPECT_THAT(bins, Each(fill_frame0));

  insertFrameIncrementTimeline(*rb, *tk, frame1);
  std::tie(numFrames, bins) = rb->collect_ringbuffer_all();
  EXPECT_THAT(numFrames, Eq(1));
  EXPECT_THAT(bins, Each(fill_frame1));

  std::tie(numFrames, bins) = rb->collect_cumulative();
  EXPECT_THAT(numFrames, Eq(2));
  EXPECT_THAT(bins, Each(fill_frame0 + fill_frame1));
  rb->insert(frame2);
  auto weight0 = std::chrono::duration_cast<std::chrono::nanoseconds>(1h);
  tk->increment_by(weight0);

  std::tie(numFrames, bins) = rb->collect_cumulative();
  EXPECT_THAT(numFrames, Eq(3));
  EXPECT_THAT(bins, Each(fill_frame0 + fill_frame1 +
                         (fill_frame2 *
                          std::chrono::duration_cast<std::chrono::milliseconds>(weight0).count())));

  auto weight1 = std::chrono::duration_cast<std::chrono::nanoseconds>(2min);
  tk->increment_by(weight1);
  std::tie(numFrames, bins) = rb->collect_cumulative();
  EXPECT_THAT(numFrames, Eq(3));
  EXPECT_THAT(
      bins,
      Each(fill_frame0 + fill_frame1 +
           (fill_frame2 *
            std::chrono::duration_cast<std::chrono::milliseconds>(weight0 + weight1).count())));
}

TEST_F(RingbufferTestCases, TestCumulativeCountsEmpty) {
  auto tk = std::make_shared<TickingTimeKeeper>();
  auto rb = histogram::Ringbuffer::create(1, std::make_unique<TimeKeeperWrapper>(tk));
  std::tie(numFrames, bins) = rb->collect_cumulative();
  EXPECT_THAT(numFrames, Eq(0));
}

TEST_F(RingbufferTestCases, TestCumulativeCountsSaturate) {
  auto tk = std::make_shared<TickingTimeKeeper>();
  auto rb = histogram::Ringbuffer::create(1, std::make_unique<TimeKeeperWrapper>(tk));
  insertFrameIncrementTimeline(*rb, *tk, frame_saturate);
  auto eon = std::chrono::nanoseconds(std::numeric_limits<uint64_t>::max());
  tk->increment_by(eon);
  std::tie(numFrames, bins) = rb->collect_cumulative();
  EXPECT_THAT(numFrames, Eq(1));
  EXPECT_THAT(bins, Each(std::numeric_limits<uint64_t>::max()));
}

TEST_F(RingbufferTestCases, TimeWeightingTest) {
  static constexpr int numInsertions = 4u;
  auto tk = std::make_shared<TickingTimeKeeper>();
  auto rb = histogram::Ringbuffer::create(numInsertions, std::make_unique<TimeKeeperWrapper>(tk));

  auto weight0 = std::chrono::duration_cast<std::chrono::nanoseconds>(1ms);
  auto weight1 = std::chrono::duration_cast<std::chrono::nanoseconds>(1h);
  auto weight2 = std::chrono::duration_cast<std::chrono::nanoseconds>(1s);
  using gigasecond = std::chrono::duration<uint64_t, std::giga>;
  auto weight3 = std::chrono::duration_cast<std::chrono::nanoseconds>(gigasecond(4));

  rb->insert(frame0);
  tk->increment_by(weight0);
  rb->insert(frame1);
  tk->increment_by(weight1);
  rb->insert(frame2);
  tk->increment_by(weight2);
  rb->insert(frame3);
  tk->increment_by(weight3);

  std::tie(numFrames, bins) = rb->collect_ringbuffer_all();

  ASSERT_THAT(bins.size(), Eq(HIST_V_SIZE));
  uint64_t expected_weight = fill_frame0 * toMs(weight0) + fill_frame1 * toMs(weight1) +
                             fill_frame2 * toMs(weight2) + fill_frame3 * toMs(weight3);
  for (auto i = 0u; i < bins.size(); i++) {
    EXPECT_THAT(bins[i], Eq(expected_weight));
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
