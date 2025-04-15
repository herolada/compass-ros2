/**
 * \file
 * \brief Various implementations of rate-limiting algorithms.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <algorithm>
#include <stdexcept>

#include <rclcpp/duration.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/time.hpp>

#include <compass_utils/rate_limiter.h>

namespace compass_utils
{

RateLimiter::RateLimiter(const rclcpp::Clock::SharedPtr clock, const ::rclcpp::Rate& rate) : clock(clock), rate(rate.period(), clock), period(rate.period())
{
  if (this->period < rclcpp::Duration::from_seconds(0.))
    throw std::invalid_argument("Negative rate is not supported.");
}

RateLimiter::RateLimiter(const rclcpp::Clock::SharedPtr clock, const rclcpp::Duration& period) : rate(period, clock), period(period)
{
  if (this->period < rclcpp::Duration::from_seconds(0.))
    throw std::invalid_argument("Negative rate is not supported.");
}

void RateLimiter::setJumpBackTolerance(const rclcpp::Duration& tolerance)
{
  if (tolerance < rclcpp::Duration::from_seconds(0.))
    throw std::invalid_argument("Jump back tolerance cannot be negative");
  this->jumpBackTolerance = tolerance;
}

bool RateLimiter::jumpedBack(const ::rclcpp::Time& stamp, const ::rclcpp::Time& previousStamp) const
{
  // It is important to not subtract the tolerance on the right - we could get into negative time, which is not allowed
  return stamp + this->jumpBackTolerance < previousStamp;
}

ThrottleLimiter::ThrottleLimiter(const rclcpp::Clock::SharedPtr clock, const rclcpp::Rate& rate) : clock(clock), RateLimiter(clock, rate)
{
}

ThrottleLimiter::ThrottleLimiter(const rclcpp::Clock::SharedPtr clock, const rclcpp::Duration& period) : clock(clock), RateLimiter(clock, period)
{
}

bool ThrottleLimiter::shouldPublish(const rclcpp::Time& stamp)
{
  // If time jumped back, always allow
  if (this->jumpedBack(stamp, this->lastPublishTime))
  {
    this->lastPublishTime = stamp;
    return true;
  }

  bool result {false};
  if (stamp >= (this->lastPublishTime + this->period))
  {
    result = true;
    this->lastPublishTime = stamp;
  }

  return result;
}

void ThrottleLimiter::reset()
{
  this->lastPublishTime = {0, 0};
}

TokenBucketLimiter::TokenBucketLimiter(const rclcpp::Clock::SharedPtr clock, const rclcpp::Rate& rate, const size_t bucketCapacity,
  const double initialTokensAvailable) : clock(clock), RateLimiter(clock, rate), tokensAvailable(rclcpp::Duration::from_seconds(initialTokensAvailable))
{
  this->bucketCapacity = bucketCapacity;
  this->initialTokensAvailable = (std::min)(initialTokensAvailable, static_cast<double>(bucketCapacity));
  // this->tokensAvailable = rclcpp::Duration::from_seconds(this->initialTokensAvailable);
}

TokenBucketLimiter::TokenBucketLimiter(const rclcpp::Clock::SharedPtr clock, const rclcpp::Duration& period, const size_t bucketCapacity,
  const double initialTokensAvailable) : clock(clock), RateLimiter(clock, period), tokensAvailable(rclcpp::Duration::from_seconds(initialTokensAvailable))
{
  this->bucketCapacity = bucketCapacity;
  this->initialTokensAvailable = (std::min)(initialTokensAvailable, static_cast<double>(bucketCapacity));
  // this->tokensAvailable = rclcpp::Duration::from_seconds(this->initialTokensAvailable);
}

bool TokenBucketLimiter::shouldPublish(const rclcpp::Time& stamp)
{
  // If time jumped back by a lot, reset
  if (this->jumpedBack(stamp, this->lastCheckTime))
    this->reset();

  // If we're processing the first message, record its stamp and say that dt == 0, so nothing will be added to bucket
  if (this->lastCheckTime == rclcpp::Time(0))
    this->lastCheckTime = stamp;

  // Do not allow if time jumped back just a bit (large jumps are solved above)
  if (stamp < this->lastCheckTime)
  {
    this->lastCheckTime = stamp;
    return false;
  }

  bool result {false};

  const auto dt = stamp - this->lastCheckTime;
  this->lastCheckTime = stamp;

  // Refill rate is 1 token per every period
  this->tokensAvailable += rclcpp::Duration::from_nanoseconds((dt.nanoseconds() / (this->period).nanoseconds()));

  // Limit by bucket capacity
  this->tokensAvailable = (std::min)(this->tokensAvailable, rclcpp::Duration::from_seconds(this->bucketCapacity));

  // If there is at least one whole token in the bucket, allow publishing
  if (this->tokensAvailable >= rclcpp::Duration::from_seconds(1.))
  {
    result = true;
    this->tokensAvailable -= rclcpp::Duration::from_seconds(1.);
  }

  return result;
}

void TokenBucketLimiter::reset()
{
  this->lastCheckTime = {0, 0};
  this->tokensAvailable = rclcpp::Duration::from_seconds(this->initialTokensAvailable);
}

}
