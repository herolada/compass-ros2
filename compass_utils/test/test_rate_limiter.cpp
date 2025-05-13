/**
 * \file
 * \brief Unit test for rate_limiter.h .
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/duration.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include "rclcpp/rclcpp.hpp"

#include <compass_utils/rate_limiter.h>


using namespace compass_utils;

std::vector<rclcpp::Time> createRegularSequence(const rclcpp::Time& start, const rclcpp::Duration& period, const size_t numTimes)
{
  const rclcpp::Logger log = rclcpp::get_logger("test_logger");

  std::vector<rclcpp::Time> result;
  result.resize(numTimes);
  for (size_t i = 0; i < numTimes; ++i)
  {
    result[i] = start + period * i;
  }
  return result;
}

TEST(ThrottleLimiter, BadConstruct)  // NOLINT
{
  const auto clk = std::make_shared<rclcpp::Clock>();
  EXPECT_THROW(compass_utils::ThrottleLimiter(clk, rclcpp::Rate(-1)), std::invalid_argument);
  EXPECT_THROW(compass_utils::ThrottleLimiter(clk, rclcpp::Duration(-1,0)), std::invalid_argument);
}

TEST(ThrottleLimiter, RegularSequence)  // NOLINT
{
  const auto clk = std::make_shared<rclcpp::Clock>();
  compass_utils::ThrottleLimiter limiter(clk, rclcpp::Rate(7));

  const auto times = createRegularSequence({1, 0}, rclcpp::Duration(0,0.1*1e9), 10);
  const std::vector<bool> results = {true, false, true, false, true, false, true, false, true, false};

  for (size_t i = 0; i < times.size(); ++i)
  {
    SCOPED_TRACE("Iteration " + std::to_string(i));
    SCOPED_TRACE("Times s " + std::to_string(times[i].seconds()));
    SCOPED_TRACE("Times ns " + std::to_string(times[i].nanoseconds()));
    EXPECT_EQ(results[i], limiter.shouldPublish(times[i]));
  }
}

TEST(ThrottleLimiter, RegularSequenceRatio)  // NOLINT
{
  const auto clk = std::make_shared<rclcpp::Clock>();
  compass_utils::ThrottleLimiter limiter(clk, rclcpp::Rate(7));

  const auto times = createRegularSequence({1, 0}, rclcpp::Duration(0,0.1*1e9), 1000);
  size_t numPublished {0};

  for (const auto& time : times)
    numPublished += limiter.shouldPublish(time);

  EXPECT_EQ(500, numPublished);  // Ideally 700. But throttle is not very good at achieving the 70% throughput rate.
}

TEST(ThrottleLimiter, IrregularSequence)  // NOLINT
{
  const auto clk = std::make_shared<rclcpp::Clock>();
  compass_utils::ThrottleLimiter limiter(clk, rclcpp::Rate(7));

  const std::vector<rclcpp::Time> times = {
    rclcpp::Time(1e9*1.0), rclcpp::Time(1e9*1.01), rclcpp::Time(1e9*1.02), rclcpp::Time(1e9*1.03),
    rclcpp::Time(1e9*1.15), rclcpp::Time(1e9*1.16), rclcpp::Time(1e9*1.17), rclcpp::Time(1e9*1.27),
    rclcpp::Time(1e9*1.30), rclcpp::Time(1e9*1.31), rclcpp::Time(1e9*1.32), rclcpp::Time(1e9*1.33),
  };
  // the period is slightly less than 0.15 seconds
  const std::vector<bool> results = {
    true, false, false, false,
    true, false, false, false,
    true, false, false, false
  };

  for (size_t i = 0; i < times.size(); ++i)
  {
    SCOPED_TRACE("Iteration " + std::to_string(i));
    EXPECT_EQ(results[i], limiter.shouldPublish(times[i]));
  }
}

TEST(ThrottleLimiter, Reset)  // NOLINT
{
  const auto clk = std::make_shared<rclcpp::Clock>();
  compass_utils::ThrottleLimiter limiter(clk, rclcpp::Rate(1));

  EXPECT_TRUE(limiter.shouldPublish(rclcpp::Time(1e9*1)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*1.1)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*1.2)));

  limiter.reset();
  EXPECT_TRUE(limiter.shouldPublish(rclcpp::Time(1e9*1.3)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*1.4)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*1.5)));
}

TEST(ThrottleLimiter, JumpBack)  // NOLINT
{
  const auto clk = std::make_shared<rclcpp::Clock>();
  compass_utils::ThrottleLimiter limiter(clk, rclcpp::Rate(1));

  EXPECT_TRUE(limiter.shouldPublish(rclcpp::Time(1e9*10)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*10.1)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*9.9)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*8.9)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*7.9)));

  // Jump back more than 3 seconds
  EXPECT_TRUE(limiter.shouldPublish(rclcpp::Time(1e9*1.3)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*1.4)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*1.5)));

  EXPECT_THROW(limiter.setJumpBackTolerance(rclcpp::Duration(-1,0)), std::invalid_argument);

  // Set jump tolerance to 5
  EXPECT_TRUE(limiter.shouldPublish(rclcpp::Time(1e9*10)));
  limiter.setJumpBackTolerance(rclcpp::Duration(5,0));
  // And jump by 4 seconds, should not trigger reset
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*6)));
  // Now jump by more than 5 seconds, should trigger reset
  EXPECT_TRUE(limiter.shouldPublish(rclcpp::Time(1)));
}

TEST(TokenBucketLimiter, BadConstruct)  // NOLINT
{
  const auto clk = std::make_shared<rclcpp::Clock>();
  EXPECT_THROW(compass_utils::TokenBucketLimiter(clk, rclcpp::Rate(-1)), std::invalid_argument);
  EXPECT_THROW(compass_utils::TokenBucketLimiter(clk, rclcpp::Duration(-1,0)), std::invalid_argument);
}

TEST(TokenBucketLimiter, RegularSequence)  // NOLINT
{
  const auto clk = std::make_shared<rclcpp::Clock>();
  std::vector<compass_utils::TokenBucketLimiter*> limiters;
  std::map<compass_utils::TokenBucketLimiter*, std::string> names;

  compass_utils::TokenBucketLimiter limiter03(clk, rclcpp::Duration(3, 0), 2, 1); limiters.push_back(&limiter03); names[&limiter03] = "03";  // NOLINT
  compass_utils::TokenBucketLimiter limiter05(clk, rclcpp::Rate(0.5), 2, 1); limiters.push_back(&limiter05); names[&limiter05] = "05";
  compass_utils::TokenBucketLimiter limiter1(clk, rclcpp::Rate(1), 2, 1); limiters.push_back(&limiter1); names[&limiter1] = "1";
  compass_utils::TokenBucketLimiter limiter2(clk, rclcpp::Rate(2), 2, 1); limiters.push_back(&limiter2); names[&limiter2] = "2";
  compass_utils::TokenBucketLimiter limiter4(clk, rclcpp::Rate(4), 2, 1); limiters.push_back(&limiter4); names[&limiter4] = "4";
  compass_utils::TokenBucketLimiter limiter5(clk, rclcpp::Rate(5), 2, 1); limiters.push_back(&limiter5); names[&limiter5] = "5";
  compass_utils::TokenBucketLimiter limiter7(clk, rclcpp::Rate(7), 2, 1); limiters.push_back(&limiter7); names[&limiter7] = "7";
  compass_utils::TokenBucketLimiter limiter10(clk, rclcpp::Rate(10), 2, 1); limiters.push_back(&limiter10); names[&limiter10] = "10";
  compass_utils::TokenBucketLimiter limiter20(clk, rclcpp::Rate(20), 2, 1); limiters.push_back(&limiter20); names[&limiter20] = "20";

  const auto times = createRegularSequence({1, 0}, rclcpp::Duration(0,1e9*0.1), 32);

  std::map<compass_utils::TokenBucketLimiter*, std::vector<bool>> expected = {
    //             1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32
    {&limiter03, { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}},  // NOLINT
    {&limiter05, { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}},  // NOLINT
    {&limiter1,  { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0}},  // NOLINT
    {&limiter2,  { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0}},  // NOLINT
    {&limiter4,  { 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0}},  // NOLINT
    {&limiter5,  { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0}},  // NOLINT
    {&limiter7,  { 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0}},  // NOLINT
    {&limiter10, { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}},  // NOLINT
    {&limiter20, { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}},  // NOLINT
  };

  std::map<compass_utils::TokenBucketLimiter*, std::vector<bool>> results;

  for (const auto& time : times)
  {
    for (const auto& limiter : limiters)
      results[limiter].push_back(limiter->shouldPublish(time));
  }

  for (const auto& limiter : limiters)
  {
    SCOPED_TRACE("limiter" + names[limiter]);
    EXPECT_EQ(expected[limiter], results[limiter]);
  }
}

TEST(TokenBucketLimiter, RegularSequenceRatio)  // NOLINT
{
  std::vector<double> rates = {0.3, 0.5, 1, 2, 4, 5, 7, 10, 20};

  std::map<compass_utils::TokenBucketLimiter*, std::string> names;
  std::vector<std::unique_ptr<compass_utils::TokenBucketLimiter>> limiters;
  std::map<compass_utils::TokenBucketLimiter*, size_t> numPublished;
  std::map<compass_utils::TokenBucketLimiter*, size_t> expectedPublished;

  const auto clk = std::make_shared<rclcpp::Clock>();

  for (const auto& rate : rates)
  {
    auto limiter = std::make_unique<compass_utils::TokenBucketLimiter>(clk, rclcpp::Rate(rate));
    names[limiter.get()] = "limiter " + std::to_string(rate);
    numPublished[limiter.get()] = 0;
    const auto expectedRate = (std::min)(10.0, rate);
    expectedPublished[limiter.get()] = 100 * expectedRate;
    limiters.push_back(std::move(limiter));
  }

  const auto times = createRegularSequence({1, 0}, rclcpp::Duration(0,1e9*0.1), 1000);
  for (const auto& time : times)
  {
    for (const auto& limiter : limiters)
      numPublished[limiter.get()] += limiter->shouldPublish(time);
  }

  for (const auto& limiter : limiters)
  {
    SCOPED_TRACE(names[limiter.get()]);
    EXPECT_NEAR(expectedPublished[limiter.get()], numPublished[limiter.get()], 1);
  }
}

TEST(TokenBucketLimiter, IrregularSequence)  // NOLINT
{
  const auto clk = std::make_shared<rclcpp::Clock>();
  compass_utils::TokenBucketLimiter limiter(clk, rclcpp::Rate(7), 2, 1);

  const std::vector<rclcpp::Time> times = {
    rclcpp::Time(1e9*1.0), rclcpp::Time(1e9*1.01), rclcpp::Time(1e9*1.02), rclcpp::Time(1e9*1.03),
    rclcpp::Time(1e9*1.15), rclcpp::Time(1e9*1.16), rclcpp::Time(1e9*1.17), rclcpp::Time(1e9*1.27),
    rclcpp::Time(1e9*1.30), rclcpp::Time(1e9*1.31), rclcpp::Time(1e9*1.32), rclcpp::Time(1e9*1.33),
  };
  // the period is slightly less than 0.15 seconds
  const std::vector<bool> results = {
    true, false, false, false,
    true, false, false, false,
    true, false, false, false
  };

  for (size_t i = 0; i < times.size(); ++i)
  {
    SCOPED_TRACE("Iteration " + std::to_string(i));
    EXPECT_EQ(results[i], limiter.shouldPublish(times[i]));
  }
}

TEST(TokenBucketLimiter, Reset)  // NOLINT
{
  const auto clk = std::make_shared<rclcpp::Clock>();
  compass_utils::TokenBucketLimiter limiter(clk, rclcpp::Rate(1), 2, 1);

  EXPECT_TRUE(limiter.shouldPublish(rclcpp::Time(1e9*1)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*1.1)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*1.2)));

  limiter.reset();
  EXPECT_TRUE(limiter.shouldPublish(rclcpp::Time(1e9*1.3)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*1.4)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*1.5)));
}

TEST(TokenBucketLimiter, JumpBack)  // NOLINT
{
  const auto clk = std::make_shared<rclcpp::Clock>();
  compass_utils::TokenBucketLimiter limiter(clk, rclcpp::Rate(1), 2, 1);

  EXPECT_TRUE(limiter.shouldPublish(rclcpp::Time(1e9*10)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*10.1)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*9.9)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*8.9)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*7.9)));

  // Jump back more than 3 seconds
  EXPECT_TRUE(limiter.shouldPublish(rclcpp::Time(1e9*1.3)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*1.4)));
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*1.5)));

  EXPECT_THROW(limiter.setJumpBackTolerance(rclcpp::Duration(-1,0)), std::invalid_argument);

  // Set jump tolerance to 5
  EXPECT_TRUE(limiter.shouldPublish(rclcpp::Time(1e9*10)));
  limiter.setJumpBackTolerance(rclcpp::Duration(5,0));
  // And jump by 4 seconds, should not trigger reset
  EXPECT_FALSE(limiter.shouldPublish(rclcpp::Time(1e9*6)));
  // Now jump by more than 5 seconds, should trigger reset
  EXPECT_TRUE(limiter.shouldPublish(rclcpp::Time( 1)));
}

TEST(TokenBucketLimiter, Params)  // NOLINT
{
  const auto clk = std::make_shared<rclcpp::Clock>();
  compass_utils::TokenBucketLimiter emptyStart(clk, rclcpp::Rate(1), 2, 0);
  compass_utils::TokenBucketLimiter fullStart(clk, rclcpp::Rate(1), 2, 2);

  EXPECT_FALSE(emptyStart.shouldPublish(rclcpp::Time(1e9*10)));
  EXPECT_TRUE(fullStart.shouldPublish(rclcpp::Time(1e9*10)));

  EXPECT_FALSE(emptyStart.shouldPublish(rclcpp::Time(1e9*10.1)));
  EXPECT_TRUE(fullStart.shouldPublish(rclcpp::Time(1e9*10.1)));

  EXPECT_FALSE(emptyStart.shouldPublish(rclcpp::Time(1e9*10.2)));
  EXPECT_FALSE(fullStart.shouldPublish(rclcpp::Time(1e9*10.2)));

  EXPECT_FALSE(emptyStart.shouldPublish(rclcpp::Time(1e9*10.9)));
  EXPECT_FALSE(fullStart.shouldPublish(rclcpp::Time(1e9*10.9)));

  EXPECT_TRUE(emptyStart.shouldPublish(rclcpp::Time(1e9*11.1)));
  EXPECT_TRUE(fullStart.shouldPublish(rclcpp::Time(1e9*11.1)));

  EXPECT_FALSE(emptyStart.shouldPublish(rclcpp::Time(1e9*11.2)));
  EXPECT_FALSE(fullStart.shouldPublish(rclcpp::Time(1e9*11.2)));

  EXPECT_FALSE(emptyStart.shouldPublish(rclcpp::Time(1e9*11.9)));
  EXPECT_FALSE(fullStart.shouldPublish(rclcpp::Time(1e9*11.9)));

  EXPECT_TRUE(emptyStart.shouldPublish(rclcpp::Time(1e9*12.1)));
  EXPECT_TRUE(fullStart.shouldPublish(rclcpp::Time(1e9*12.1)));

  EXPECT_FALSE(emptyStart.shouldPublish(rclcpp::Time(1e9*12.2)));
  EXPECT_FALSE(fullStart.shouldPublish(rclcpp::Time(1e9*12.2)));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  // rclcpp::Time::init();
  return RUN_ALL_TESTS();
}
