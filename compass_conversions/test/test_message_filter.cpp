// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for compass message filter.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <memory>
#include <string>

#include <boost/functional.hpp>

#include <angles/angles.h>
#include <compass_conversions/message_filter.h>
#include <compass_interfaces/msg/azimuth.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/memory.h>
#include <cras_cpp_common/string_utils/ros.hpp>
#include <message_filters/simple_filter.h>
#include <message_filters/message_event.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <message_filters/message_event.h>

using Az = compass_interfaces::msg::Azimuth;

template<class T>
class TestInput : public message_filters::SimpleFilter<T>
{
public:
  void add(const typename T::ConstPtr& msg)
  {
    // Pass a complete MessageEvent to avoid calling this->clock.now() to determine the missing timestamp
    this->signalMessage(message_filters::MessageEvent<T const>(msg, msg->header.stamp));
  }
};

TEST(MessageFilter, NoNavSatNeeded)  // NOLINT
{
  rclcpp::Logger log = rclcpp::get_logger("test_logger");
  TestInput<Az> azimuthInput;
  compass_conversions::CompassFilter filter(
    log, nullptr, azimuthInput, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);

  Az::ConstPtr outMessage;
  const auto cb = [&outMessage](const message_filters::MessageEvent<Az const>& filteredMessage)
  {
    outMessage = filteredMessage.getConstMessage();
  };
  filter.registerCallback(boost::function<void(const message_filters::MessageEvent<Az const>&)>(cb));

  Az::Ptr inMessage(new Az);
  inMessage->header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  inMessage->unit = Az::UNIT_DEG;
  inMessage->orientation = Az::ORIENTATION_NED;
  inMessage->reference = Az::REFERENCE_GEOGRAPHIC;
  inMessage->azimuth = 90;

  outMessage.reset();
  azimuthInput.add(inMessage);

  ASSERT_NE(nullptr, outMessage);
  EXPECT_NEAR(0, outMessage->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, outMessage->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, outMessage->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, outMessage->reference);
}

TEST(MessageFilter, NavSatNeededButNotGiven)  // NOLINT
{
  rclcpp::Logger log = rclcpp::get_logger("test_logger");
  TestInput<Az> azimuthInput;
  TestInput<sensor_msgs::msg::NavSatFix> fixInput;
  compass_conversions::CompassFilter filter(
    log, nullptr, azimuthInput, fixInput, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);

  Az::ConstPtr outMessage;
  const auto cb = [&outMessage](const message_filters::MessageEvent<Az const>& filteredMessage)
  {
    outMessage = filteredMessage.getConstMessage();
  };
  filter.registerCallback(boost::function<void(const message_filters::MessageEvent<Az const>&)>(cb));

  Az::Ptr inMessage(new Az);
  inMessage->header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  inMessage->unit = Az::UNIT_DEG;
  inMessage->orientation = Az::ORIENTATION_NED;
  inMessage->reference = Az::REFERENCE_MAGNETIC;
  inMessage->azimuth = 90;

  outMessage.reset();
  azimuthInput.add(inMessage);

  ASSERT_EQ(nullptr, outMessage);
}

TEST(MessageFilter, NavSatNeeded)  // NOLINT
{
  rclcpp::Logger log = rclcpp::get_logger("test_logger");
  TestInput<Az> azimuthInput;
  TestInput<sensor_msgs::msg::NavSatFix> fixInput;
  compass_conversions::CompassFilter filter(
    log, nullptr, azimuthInput, fixInput, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);

  Az::ConstPtr outMessage;
  const auto cb = [&outMessage](const message_filters::MessageEvent<Az const>& filteredMessage)
  {
    outMessage = filteredMessage.getConstMessage();
  };
  filter.registerCallback(boost::function<void(const message_filters::MessageEvent<Az const>&)>(cb));

  Az::Ptr inMessage(new Az);
  inMessage->header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  inMessage->unit = Az::UNIT_DEG;
  inMessage->orientation = Az::ORIENTATION_NED;
  inMessage->reference = Az::REFERENCE_MAGNETIC;
  inMessage->azimuth = 90;

  outMessage.reset();
  azimuthInput.add(inMessage);

  ASSERT_EQ(nullptr, outMessage);

  sensor_msgs::msg::NavSatFix::Ptr fixMessage(new sensor_msgs::msg::NavSatFix);
  fixMessage->header.stamp = inMessage->header.stamp;
  fixMessage->latitude = 51;
  fixMessage->longitude = 10;
  fixMessage->altitude = 200;
  fixInput.add(fixMessage);

  ASSERT_EQ(nullptr, outMessage);

  azimuthInput.add(inMessage);

  ASSERT_NE(nullptr, outMessage);
  EXPECT_NEAR(-angles::from_degrees(4.04) + 2 * M_PI, outMessage->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, outMessage->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, outMessage->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, outMessage->reference);

  azimuthInput.add(inMessage);

  ASSERT_NE(nullptr, outMessage);
  EXPECT_NEAR(-angles::from_degrees(4.04) + 2 * M_PI, outMessage->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, outMessage->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, outMessage->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, outMessage->reference);
}

TEST(MessageFilter, NavSatNeededAndGivenAsInitValue)  // NOLINT
{
  rclcpp::Logger log = rclcpp::get_logger("test_logger");
  TestInput<Az> azimuthInput;
  auto converter = std::make_shared<compass_conversions::CompassConverter>(log, true);
  compass_conversions::CompassFilter filter(
    log, converter, azimuthInput, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);

  Az::ConstPtr outMessage;
  const auto cb = [&outMessage](const message_filters::MessageEvent<Az const>& filteredMessage)
  {
    outMessage = filteredMessage.getConstMessage();
  };
  filter.registerCallback(boost::function<void(const message_filters::MessageEvent<Az const>&)>(cb));

  Az::Ptr inMessage(new Az);
  inMessage->header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  inMessage->unit = Az::UNIT_DEG;
  inMessage->orientation = Az::ORIENTATION_NED;
  inMessage->reference = Az::REFERENCE_MAGNETIC;
  inMessage->azimuth = 90;

  outMessage.reset();
  azimuthInput.add(inMessage);

  ASSERT_EQ(nullptr, outMessage);

  sensor_msgs::msg::NavSatFix::Ptr fixMessage(new sensor_msgs::msg::NavSatFix);
  fixMessage->header.stamp = inMessage->header.stamp;
  fixMessage->latitude = 51;
  fixMessage->longitude = 10;
  fixMessage->altitude = 200;
  converter->setNavSatPos(*fixMessage);

  ASSERT_EQ(nullptr, outMessage);

  azimuthInput.add(inMessage);

  ASSERT_NE(nullptr, outMessage);
  EXPECT_NEAR(-angles::from_degrees(4.04) + 2 * M_PI, outMessage->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, outMessage->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, outMessage->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, outMessage->reference);

  azimuthInput.add(inMessage);

  ASSERT_NE(nullptr, outMessage);
  EXPECT_NEAR(-angles::from_degrees(4.04) + 2 * M_PI, outMessage->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, outMessage->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, outMessage->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, outMessage->reference);
}

TEST(MessageFilter, ForcedDeclination)  // NOLINT
{
  rclcpp::Logger log = rclcpp::get_logger("test_logger");
  TestInput<Az> azimuthInput;
  auto converter = std::make_shared<compass_conversions::CompassConverter>(log, true);
  compass_conversions::CompassFilter filter(
    log, converter, azimuthInput, Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_GEOGRAPHIC);

  Az::ConstPtr outMessage;
  const auto cb = [&outMessage](const message_filters::MessageEvent<Az const>& filteredMessage)
  {
    outMessage = filteredMessage.getConstMessage();
  };
  filter.registerCallback(boost::function<void(const message_filters::MessageEvent<Az const>&)>(cb));

  Az::Ptr inMessage(new Az);
  inMessage->header.stamp = cras::parseTime("2024-11-18T13:00:00.000Z");
  inMessage->unit = Az::UNIT_DEG;
  inMessage->orientation = Az::ORIENTATION_NED;
  inMessage->reference = Az::REFERENCE_MAGNETIC;
  inMessage->azimuth = 90;

  outMessage.reset();
  azimuthInput.add(inMessage);

  ASSERT_EQ(nullptr, outMessage);

  converter->forceMagneticDeclination(angles::from_degrees(4.04));

  ASSERT_EQ(nullptr, outMessage);

  azimuthInput.add(inMessage);

  ASSERT_NE(nullptr, outMessage);
  EXPECT_NEAR(-angles::from_degrees(4.04) + 2 * M_PI, outMessage->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, outMessage->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, outMessage->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, outMessage->reference);

  azimuthInput.add(inMessage);

  ASSERT_NE(nullptr, outMessage);
  EXPECT_NEAR(-angles::from_degrees(4.04) + 2 * M_PI, outMessage->azimuth, 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, outMessage->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, outMessage->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, outMessage->reference);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
