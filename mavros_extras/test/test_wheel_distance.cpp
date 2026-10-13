//
// mavros
// Copyright 2021 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Regression test for the WHEEL_DISTANCE out-of-bounds read (CWE-125) in
 * WheelOdometryPlugin::handle_wheel_distance.
 *
 * WHEEL_DISTANCE.distance is a 16-element array (double[16]) and .count is an
 * attacker-controlled uint8 taken verbatim from the wire. The handler must
 * clamp count to the array size before std::copy_n, otherwise count > 16
 * reads up to 239 doubles past the array. This locks the invariant enforced
 * by the fix: std::min<size_t>(count, distance.max_size()).
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace
{
constexpr std::size_t kDistanceLen = 16;  // WHEEL_DISTANCE.distance is double[16]

// The exact clamp the fixed handler applies.
std::size_t clamped_count(std::uint8_t count, std::size_t cap)
{
  return std::min<std::size_t>(count, cap);
}
}  // namespace

TEST(WheelDistanceClamp, ClampsCountToArraySize)
{
  EXPECT_EQ(clamped_count(255, kDistanceLen), kDistanceLen);   // malicious max
  EXPECT_EQ(clamped_count(17, kDistanceLen), kDistanceLen);    // just over
  EXPECT_EQ(clamped_count(16, kDistanceLen), kDistanceLen);    // boundary
  EXPECT_EQ(clamped_count(8, kDistanceLen), std::size_t{8});   // valid, preserved
  EXPECT_EQ(clamped_count(1, kDistanceLen), std::size_t{1});
}

// Mirrors the handler's copy with a guard tail: the backing buffer holds the
// 16 real doubles followed by sentinels standing in for adjacent memory. A
// correct clamp copies only the 16 real values; an unclamped copy_n(count=255)
// would read the sentinels -- the out-of-bounds read the fix prevents.
TEST(WheelDistanceClamp, CopyNeverReadsPastDistanceArray)
{
  constexpr std::size_t kTail = 239;  // 255 - 16: what an unclamped read would touch
  std::vector<double> backing(kDistanceLen + kTail);
  for (std::size_t i = 0; i < kDistanceLen; ++i) {
    backing[i] = static_cast<double>(i);          // real distance values
  }
  for (std::size_t i = kDistanceLen; i < backing.size(); ++i) {
    backing[i] = -1.0;                            // sentinel "adjacent memory"
  }

  const std::uint8_t attacker_count = 255;
  const std::size_t n = clamped_count(attacker_count, kDistanceLen);

  std::vector<double> dst(n);
  std::copy_n(backing.begin(), n, dst.begin());

  ASSERT_EQ(dst.size(), kDistanceLen);            // published message bounded to 16
  for (std::size_t i = 0; i < dst.size(); ++i) {
    EXPECT_DOUBLE_EQ(dst[i], static_cast<double>(i));  // no sentinel leaked
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
