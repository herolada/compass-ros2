// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for string_utils.hpp
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>

#include <compass_utils/string_utils.hpp>

using namespace compass_utils;

TEST(StringUtils, ToStringRos)  // NOLINT
{
  EXPECT_EQ("1970-01-01T00:00:01.500000Z", to_pretty_string(rclcpp::Time(1, 500000000)));
  std::chrono::system_clock::time_point wall_tp = std::chrono::system_clock::time_point{} + std::chrono::seconds{1} + std::chrono::nanoseconds{500000000};
  EXPECT_EQ("1970-01-01T00:00:01.500000Z", to_pretty_string(wall_tp));
  // std::chrono::steady_clock::time_point steady_tp = std::chrono::steady_clock::time_point{} + std::chrono::seconds{1} + std::chrono::nanoseconds{500000000};
  // EXPECT_EQ("1970-01-01T00:00:01.500000Z", to_pretty_string(steady_tp));

  EXPECT_EQ("2024-11-13T13:44:04Z", to_pretty_string(rclcpp::Time(1731505444, 0)));
  wall_tp = std::chrono::system_clock::time_point{} + std::chrono::seconds{1731505444};
  EXPECT_EQ("2024-11-13T13:44:04Z", to_pretty_string(wall_tp));
  // steady_tp = std::chrono::steady_clock::time_point{} + std::chrono::seconds{1731505444};
  // EXPECT_EQ("2024-11-13T13:44:04Z", to_pretty_string(steady_tp));
}

TEST(StringUtils, ToLower)  // NOLINT
{
  EXPECT_EQ("test", toLower("test"));
  EXPECT_EQ("test", toLower("Test"));
  EXPECT_EQ("test", toLower("TEST"));
  EXPECT_EQ("test", toLower("TeST"));
//  EXPECT_EQ("ěščřžýáíéďťňúů", toLower("ĚŠČŘŽÝÁÍÉĎŤŇÚŮ"));  // not yet working
}

TEST(StringUtils, AppendIfNonempty)  // NOLINT
{
  EXPECT_EQ("test/", appendIfNonEmpty("test", "/"));
  EXPECT_EQ("", appendIfNonEmpty("", "/"));
  EXPECT_EQ("//", appendIfNonEmpty("/", "/"));
  EXPECT_EQ("/test", appendIfNonEmpty("/", "test"));
  EXPECT_EQ("testpostfix", appendIfNonEmpty("test", "postfix"));
}

TEST(StringUtils, Split)  // NOLINT
{
  using v = std::vector<std::string>;

  EXPECT_EQ(v({""}), split("", "t"));
  EXPECT_EQ(v({"test"}), split("test", "a"));
  EXPECT_EQ(v({"", "es", ""}), split("test", "t"));
  EXPECT_EQ(v({"a", "es", "a"}), split("atesta", "t"));
  EXPECT_EQ(v({"a", "es", "a"}), split("atesta", "t", -1));
  EXPECT_EQ(v({"a", "es", "a"}), split("atesta", "t", 2));
  EXPECT_EQ(v({"a", "es", "a"}), split("atesta", "t", 100));
  EXPECT_EQ(v({"a", "esta"}), split("atesta", "t", 1));
  EXPECT_EQ(v({"atesta"}), split("atesta", "t", 0));

  EXPECT_EQ(v({""}), split("", "te"));
  EXPECT_EQ(v({"test"}), split("test", "ae"));
  EXPECT_EQ(v({"", "st"}), split("test", "te"));
  EXPECT_EQ(v({"a", "sta"}), split("atesta", "te"));
  EXPECT_EQ(v({"a", "sta"}), split("atesta", "te", -1));
  EXPECT_EQ(v({"a", "sta"}), split("atesta", "te", 2));
  EXPECT_EQ(v({"a", "sta"}), split("atesta", "te", 100));
  EXPECT_EQ(v({"a", "sta"}), split("atesta", "te", 1));
  EXPECT_EQ(v({"atesta"}), split("atesta", "te", 0));

  EXPECT_EQ(v({"", "home", "cras", "file", "path"}), split("/home/cras/file/path", "/"));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
