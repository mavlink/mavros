//
// mavros
// Copyright 2026 Vladimir Ermakov, All rights reserved.
//
// This file is part of the mavros package and subject to the license terms
// in the top-level LICENSE file of the mavros repository.
// https://github.com/mavlink/mavros/tree/master/LICENSE.md
//

/**
 * Tests for sys_status BatteryStatusDiag move operations.
 */

#include <gtest/gtest.h>

#include <string>
#include <utility>

#include "../src/plugins/sys_status.cpp"  // NOLINT(build/include)

using mavros::std_plugins::BatteryStatusDiag;

static std::string run_value(BatteryStatusDiag & d, const std::string & key)
{
  diagnostic_updater::DiagnosticStatusWrapper stat;
  d.run(stat);
  for (const auto & kv : stat.values) {
    if (kv.key == key) {
      return kv.value;
    }
  }
  return {};
}

TEST(BatteryStatusDiag, move_construct_keeps_name_and_state)
{
  BatteryStatusDiag a("Battery 2");
  a.set(12.5f, 1.5f, 0.5f);

  BatteryStatusDiag b(std::move(a));
  EXPECT_EQ(b.getName(), "Battery 2");
  EXPECT_EQ(run_value(b, "Voltage"), "12.50");
}

TEST(BatteryStatusDiag, move_assign_copies_state)
{
  BatteryStatusDiag a("Battery");
  BatteryStatusDiag b("Battery 1");
  a.set(11.1f, 2.0f, 0.25f);
  a.setcell_v({3.7f, 3.7f, 3.7f});

  b = std::move(a);
  EXPECT_EQ(b.getName(), "Battery 1");
  EXPECT_EQ(run_value(b, "Voltage"), "11.10");
  EXPECT_EQ(run_value(b, "Cell 3"), "3.70");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
