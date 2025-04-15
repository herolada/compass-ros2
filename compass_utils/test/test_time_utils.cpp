/**
 * \file
 * \brief Unit test for time_utils.hpp.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <limits>
#include <memory>
#include <thread>

#include <compass_utils/time_utils.hpp>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

using namespace compass_utils;
using namespace rclcpp;

TEST(TimeUtils, GetYear)  // NOLINT
{
  EXPECT_EQ(1970, compass_utils::getYear(rclcpp::Time()));
  std::chrono::system_clock::time_point wall_tp = std::chrono::system_clock::time_point{};
  EXPECT_EQ(1970, compass_utils::getYear(wall_tp));
  std::chrono::steady_clock::time_point steady_tp = std::chrono::steady_clock::time_point{};
  EXPECT_EQ(1970, compass_utils::getYear(steady_tp));

  EXPECT_EQ(2024, compass_utils::getYear(rclcpp::Time(1731505444, 0)));
  wall_tp = std::chrono::system_clock::time_point{} + std::chrono::seconds(1731505444);
  EXPECT_EQ(2024, compass_utils::getYear(wall_tp));
  steady_tp = std::chrono::steady_clock::time_point{} + std::chrono::seconds(1731505444);
  EXPECT_EQ(2024, compass_utils::getYear(steady_tp));
  EXPECT_EQ(2024, compass_utils::getYear(rclcpp::Time(1731505444, 500000000)));
}

TEST(TimeUtils, FromStructTm)  // NOLINT
{
  tm t{};

  EXPECT_FALSE(compass_utils::fromStructTm(t).has_value());

  t.tm_year = 1970 - 1900; t.tm_mon = 1 - 1; t.tm_mday = 1; t.tm_hour = 0; t.tm_min = 0; t.tm_sec = 0;
  ASSERT_TRUE(compass_utils::fromStructTm(t).has_value());
  EXPECT_EQ(rclcpp::Time(), compass_utils::fromStructTm(t));

  t.tm_year = 2024 - 1900; t.tm_mon = 11 - 1; t.tm_mday = 13; t.tm_hour = 13; t.tm_min = 44; t.tm_sec = 4;
  ASSERT_TRUE(compass_utils::fromStructTm(t).has_value());
  EXPECT_EQ(rclcpp::Time(1731505444, 0), compass_utils::fromStructTm(t));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
