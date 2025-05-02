#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Various implementations of rate-limiting algorithms for azimuth visualization node. Adapted from cras_cpp_common.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <rclcpp/duration.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/time.hpp>

namespace compass_utils
{

/**
 * \brief Generic rate-limiter interface.
 */
class RateLimiter
{
public:
  /**
   * \brief Create limiter with the given rate.
   * \param[in] rate The desired rate of messages.
   */
  explicit RateLimiter(const rclcpp::Clock::SharedPtr clock, const ::rclcpp::Rate& rate);

  /**
   * \brief Create limiter with rate corresponding to the given period.
   * \param[in] period Average delay between two desired output messages.
   */
  explicit RateLimiter(const rclcpp::Clock::SharedPtr clock, const ::rclcpp::Duration& period);

  /**
   * \brief Call this function whenever a message is received. It tells whether the message has passed the rate-limiting
   *        and should be published, or whether it should be skipped.
   * \param[in] stamp Time when the message should be sent (usually not header.stamp!).
   * \return Whether to continue publishing the message.
   */
  virtual bool shouldPublish(const ::rclcpp::Time& stamp) = 0;

  /**
   * \brief Reset the rate-limiter as if it were newly created with the same parameters.
   */
  virtual void reset() = 0;

  /**
   * \brief Set the limit for telling between small and large backwards time jumps. Small jumps result in ignoring the
   *        messages, while a large jump results in a reset of the rate-limter.
   * \param[in] tolerance If a new message comes that is `tolerance` seconds older than the last seen message, it should
   *                      be treated as a large jump.
   */
  void setJumpBackTolerance(const ::rclcpp::Duration& tolerance);

protected:
  /**
   * \brief Decide whether the newly coming message should be treated as a backwards jump in time.
   * \param[in] stamp Reception time of the new message.
   * \param[in] previousStamp Reception time of the previous message.
   * \return True if the message should be treated as a large time jump.
   */
  bool jumpedBack(const ::rclcpp::Time& stamp, const ::rclcpp::Time& previousStamp) const;

  //! \brief The desired rate (1/period).
  ::rclcpp::Rate rate;

  //! \brief The desired period between message (1/rate).
  ::rclcpp::Duration period;

  //! \brief Threshold for jump back detection.
  ::rclcpp::Duration jumpBackTolerance {3, 0};

  const rclcpp::Clock::SharedPtr clock;

};

/**
 * \brief The (not so clever) algorithm used by topic_tools/throttle node.
 * \note It is not very good at achieving the requested if it isn't orders of magnitude smaller than the incoming rate.
 */
class ThrottleLimiter : public ::compass_utils::RateLimiter
{
public:
  explicit ThrottleLimiter(const rclcpp::Clock::SharedPtr clock, const ::rclcpp::Rate& rate);
  explicit ThrottleLimiter(const rclcpp::Clock::SharedPtr clock, const ::rclcpp::Duration& period);

  bool shouldPublish(const ::rclcpp::Time& stamp) override;
  void reset() override;

protected:
  //! \brief Stamp of the last message for which `shouldPublish()` returned trued.
  ::rclcpp::Time lastPublishTime {0, 0};
  const rclcpp::Clock::SharedPtr clock;
};

/**
 * \brief Token bucket rate limiting algorithm.
 * \note Generally, it should be quite good at achieving the desired rate.
 * \note The limiter has a bucket of a given capacity. It is refilled over time by a constant number of tokens per
 *       second. To publish a message, there has to be at least one token in the bucket. If more tokens should be put
 *       in the bucket than is its capacity, they are ignored.
 * \note The bucket capacity basically specifies the size of the burst that can happen after some period of inactivity
 *       when tokens are just collected and not consumed.
 */
class TokenBucketLimiter : public ::compass_utils::RateLimiter
{
public:
  /**
   * \brief Create the rate-limiter limiting to the desired rate.
   * \param[in] rate Desired rate.
   * \param[in] bucketCapacity Capacity of the bucket (in tokens).
   * \param[in] initialTokensAvailable Number of tokens available in the bucket at the beginning. Set to 1 to always
   *                                   let the first packet through. This number should not be higher than
   *                                   `bucketCapacity`.
   */
  explicit TokenBucketLimiter(const rclcpp::Clock::SharedPtr clock, const ::rclcpp::Rate& rate, size_t bucketCapacity = 2, double initialTokensAvailable = 1.0);

  /**
   * \brief Create rate-limiter with rate corresponding to the given period.
   * \param[in] period Average delay between two desired output messages.
   * \param[in] bucketCapacity Capacity of the bucket (in tokens).
   * \param[in] initialTokensAvailable Number of tokens available in the bucket at the beginning. Set to 1 to always
   *                                   let the first packet through. This number should not be higher than
   *                                   `bucketCapacity`.
   */
  explicit TokenBucketLimiter(const rclcpp::Clock::SharedPtr clock, const ::rclcpp::Duration& period, size_t bucketCapacity = 2,
    double initialTokensAvailable = 1.0);

  bool shouldPublish(const ::rclcpp::Time& stamp) override;
  void reset() override;

protected:
  //! \brief Stamp of the last incoming message. Zero at the beginning.
  ::rclcpp::Time lastCheckTime {0, 0};

  //! \brief Number of tokens that can fit into the bucket. This influences the maximum burst size.
  size_t bucketCapacity;

  //! \brief The number of currently available tokens. This units of this number are actually not seconds, but Duration
  //!        is used here to achieve higher decimal point accuracy.
  rclcpp::Duration tokensAvailable;

  //! \brief The number of tokens that are initially in the buffer (and after reset).
  double initialTokensAvailable;

  const rclcpp::Clock::SharedPtr clock;
};

}
