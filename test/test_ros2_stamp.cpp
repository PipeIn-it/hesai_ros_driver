// Copyright 2026 PipeIn Development
// SPDX-License-Identifier: MIT
//
// ToRos2Stamp (9B.1): seconds as a double to builtin_interfaces/Time, rounded to the nearest
// nanosecond, nanoseconds always below one second (H4, fixed at ab2af59).
#include <gtest/gtest.h>

#include <cmath>

#include "source_driver_ros2.hpp"

TEST(ToRos2Stamp, SplitsSecondsAndNanoseconds) {
  const auto t = ToRos2Stamp(1.5);
  EXPECT_EQ(t.sec, 1);
  EXPECT_EQ(t.nanosec, 500000000u);
}

TEST(ToRos2Stamp, NanosecondsStayBelowOneSecond) {
  const auto t = ToRos2Stamp(std::nextafter(2.0, 0.0));
  EXPECT_LT(t.nanosec, 1000000000u);
  EXPECT_EQ(t.sec, 2);
  EXPECT_EQ(t.nanosec, 0u);
}

TEST(ToRos2Stamp, ATodaysStampKeepsItsMicroseconds) {
  const auto t = ToRos2Stamp(1790000000.123456);
  EXPECT_EQ(t.sec, 1790000000);
  EXPECT_NEAR(static_cast<double>(t.nanosec), 123456000.0, 300.0);  // double spacing ~240 ns
}
