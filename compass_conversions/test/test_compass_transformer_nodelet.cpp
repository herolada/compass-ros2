// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for transformations of compass_interfaces.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <cmath>
#include <memory>
#include <string>

#include <angles/angles.h>
#include <compass_interfaces/msg/azimuth.hpp>
#include <compass_conversions/compass_transformer.h>
#include <compass_utils/time_utils.hpp>
#include <compass_utils/tf2_utils.hpp>
// #include <cras_cpp_common/log_utils/memory.h>
// #include <cras_cpp_common/log_utils/node.h>
// #include <cras_cpp_common/nodelet_utils.hpp>
// #include <cras_cpp_common/tf2_utils.hpp>
// #include <cras_cpp_common/param_utils/param_helper.hpp>
// #include <cras_cpp_common/string_utils/ros.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
// #include <nodelet/nodelet.h>
// #include <ros/callback_queue.h>
// #include <ros/names.h>
// #include <ros/ros.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <rclcpp/utilities.hpp>

namespace ros
{
namespace names
{
extern void init(const std::map< std::string, std::string >& remappings);
}
}

using Az = compass_interfaces::msg::Azimuth;

std::vector< std::string > my_argv;

// template<typename NodeletType = compass_utils::Nodelet>
// std::unique_ptr<NodeletType> createNodelet(const rclcpp::Logger& log,
//   const rclcpp::std::map< std::string, std::string >& remaps = {},
//   const std::shared_ptr<tf2_ros::Buffer>& tf = nullptr)
// {
//   // Declaration order of these variables is important to make sure they can be properly stopped and destroyed.
//   auto nodelet = class_loader::impl::createInstance<nodelet::Nodelet>(
//     "compass_conversions::CompassTransformerNodelet", nullptr);
//   if (nodelet == nullptr)
//     return nullptr;

//   {
//     const auto paramHelper = dynamic_cast<compass_utils::ParamHelper*>(nodelet);
//     if (paramHelper != nullptr)
//       paramHelper->setLogger(log);
//   }

//   const auto targetNodelet = dynamic_cast<NodeletType*>(nodelet);
//   if (targetNodelet == nullptr)
//   {
//     delete nodelet;
//     return nullptr;
//   }

//   if (tf != nullptr)
//     targetNodelet->setBuffer(tf);

//   nodelet->init(rclcpp::this_node::getName(), remaps, my_argv, nullptr, nullptr);

//   return std::unique_ptr<NodeletType>(targetNodelet);
// }
std::shared_ptr<compass_conversions::CompassTransformerNodelet> createNodelet()
{
  // Declaration order of these variables is important to make sure they can be properly stopped and destroyed.
  // auto nodelet = class_loader::impl::createInstance<rclcpp::Node>(
  //   "compass_conversions::CompassTransformerNodelet", nullptr);
  // if (nodelet == nullptr)
  //   return nullptr;

  auto nodelet = std::make_shared<compass_conversions::CompassTransformerNodelet>();

  return nodelet;
}

TEST(CompassTransformerNodelet, BasicConversion)  // NOLINT
{
  auto node = createNodelet();
  auto nodelet = compass_conversions::CompassTransformerNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  node->declare_parameter("target_unit", "rad");
  rclcpp::Parameter parameter1("target_unit", "rad");
  node->set_parameter(parameter1);
  node->declare_parameter("target_orientation", "enu");
  rclcpp::Parameter parameter2("target_orientation", "enu");
  node->set_parameter(parameter2);
  node->declare_parameter("target_reference", "magnetic");
  rclcpp::Parameter parameter3("target_reference", "magnetic");
  node->set_parameter(parameter3);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);

  // const auto log = std::make_shared<rclcpp::Logger>();
  

  // auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, TfConversion)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  
  node->declare_parameter("target_unit", "deg");
  node->declare_parameter("target_orientation", "ned");
  node->declare_parameter("target_reference", "magnetic");
  node->declare_parameter("target_frame", "test2");

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);

  // const auto log = std::make_shared<rclcpp::Logger>();
  

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = node->now();
  tf.header.frame_id = "test";
  tf.child_frame_id = "test2";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, tf.transform.rotation);

  auto tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tfBuffer->setTransform(tf, "test", true);

  node->setBuffer(tfBuffer);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0;
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ("test2", lastAz->header.frame_id);
  EXPECT_NEAR(180.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_DEG, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, TfConversionFail)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  node->declare_parameter("target_unit", "deg");
  rclcpp::Parameter parameter1("target_unit", "deg");
  node->set_parameter(parameter1);
  node->declare_parameter("target_orientation", "ned");
  rclcpp::Parameter parameter2("target_orientation", "ned");
  node->set_parameter(parameter2);
  node->declare_parameter("target_reference", "magnetic");
  rclcpp::Parameter parameter3("target_reference", "magnetic");
  node->set_parameter(parameter3);
  node->declare_parameter("target_frame", "test_nonexistent");
  rclcpp::Parameter parameter4("target_frame", "test_nonexistent");
  node->set_parameter(parameter4);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);

  // const auto log = std::make_shared<rclcpp::Logger>();
  

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = node->now();
  tf.header.frame_id = "test";
  tf.child_frame_id = "test2";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, tf.transform.rotation);

  auto tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tfBuffer->setTransform(tf, "test", true);

  node->setBuffer(tfBuffer);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0;
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_FALSE(lastAz.has_value());
}

TEST(CompassTransformerNodelet, FixMissing)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  
  node->declare_parameter("target_reference", "utm");
  rclcpp::Parameter parameter1("target_reference", "utm");
  node->set_parameter(parameter1);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);

  // const auto log = std::make_shared<rclcpp::Logger>();
  

  // auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0;
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_FALSE(lastAz.has_value());
}

TEST(CompassTransformerNodelet, FixFromParams)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  node->declare_parameter("target_reference", "geographic");
  rclcpp::Parameter parameter1("target_reference", "geographic");
  node->set_parameter(parameter1);
  node->declare_parameter("initial_lat", 51.0);
  rclcpp::Parameter parameter2("initial_lat", 51.0);
  node->set_parameter(parameter2);
  node->declare_parameter("initial_lon", 15.0);
  rclcpp::Parameter parameter3("initial_lon", 15.0);
  node->set_parameter(parameter3);
  node->declare_parameter("initial_alt", 200.0);
  rclcpp::Parameter parameter4("initial_alt", 200.0);
  node->set_parameter(parameter4);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);

  // const auto log = std::make_shared<rclcpp::Logger>();
  

  // auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = compass_utils::parseTime("2024-11-18T13:00:00Z");
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(-5.333 + 360, angles::to_degrees(lastAz->azimuth), 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, FixFromMsg)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  
  node->declare_parameter("target_reference", "geographic");
  rclcpp::Parameter parameter1("target_reference", "geographic");
  node->set_parameter(parameter1);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto fixPub = node->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);

  // const auto log = std::make_shared<rclcpp::Logger>();
  

  // auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 &&
    (azimuthPub->get_subscription_count() == 0 || fixPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for fix and azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(fixPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  const auto time = compass_utils::parseTime("2024-11-18T13:00:00Z");

  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = time;
  fix.header.frame_id = "test";
  fix.latitude = 51.0;
  fix.longitude = 15.0;
  fix.altitude = 200.0;
  fixPub->publish(fix);

  // Wait until the fix message is received
  for (size_t i = 0; i < 10; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
  }

  Az in;
  in.header.stamp = time;
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(-5.333 + 360, angles::to_degrees(lastAz->azimuth), 1e-3);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, SubImuNameDetect)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  
  node->declare_parameter("target_unit", "rad");
  rclcpp::Parameter parameter1("target_unit", "rad");
  node->set_parameter(parameter1);
  
  node->declare_parameter("target_orientation", "enu");
  rclcpp::Parameter parameter2("target_orientation", "enu");
  node->set_parameter(parameter2);
  
  node->declare_parameter("target_reference", "magnetic");
  rclcpp::Parameter parameter3("target_reference", "magnetic");
  node->set_parameter(parameter3);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  // TODO SOLVE REMAPPING IN ROS2 NODES !!
  auto azimuthPub = node->create_publisher<sensor_msgs::msg::Imu>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);  
  // auto azimuthPub = node->create_publisher<sensor_msgs::msg::Imu>("imu/data/mag/ned/imu", 1);
  // auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);  

  // const std::map< std::string, std::string > remaps = {
  //   {pnh.resolveName("azimuth_in"), nh.resolveName("imu/data/mag/ned/imu")},
  // };

  // auto nodelet = createNodelet(log, remaps);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  sensor_msgs::msg::Imu in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.orientation);
  in.orientation_covariance[2 * 3 + 2] = 4.0;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, SubImuNoDetect)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  node->declare_parameter("target_unit", "rad");
  rclcpp::Parameter target_unit_param("target_unit", "rad");
  node->set_parameter(target_unit_param);

  node->declare_parameter("target_orientation", "enu"); 
  rclcpp::Parameter target_orient_param("target_orientation", "enu");
  node->set_parameter(target_orient_param);

  node->declare_parameter("target_reference", "magnetic");
  rclcpp::Parameter target_ref_param("target_reference", "magnetic");
  node->set_parameter(target_ref_param);

  node->declare_parameter("input_orientation", "ned");
  rclcpp::Parameter input_orient_param("input_orientation", "ned");
  node->set_parameter(input_orient_param);

  node->declare_parameter("input_reference", "magnetic");
  rclcpp::Parameter input_ref_param("input_reference", "magnetic");
  node->set_parameter(input_ref_param);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<sensor_msgs::msg::Imu>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);

  // const auto log = std::make_shared<rclcpp::Logger>();
  

  // auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  sensor_msgs::msg::Imu in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.orientation);
  in.orientation_covariance[2 * 3 + 2] = 4.0;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, SubPoseNameDetect)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  node->declare_parameter("target_unit", "rad");
  rclcpp::Parameter parameter1("target_unit", "rad");
  node->set_parameter(parameter1);

  node->declare_parameter("target_orientation", "enu");
  rclcpp::Parameter parameter2("target_orientation", "enu");
  node->set_parameter(parameter2);

  node->declare_parameter("target_reference", "magnetic");
  rclcpp::Parameter parameter3("target_reference", "magnetic");
  node->set_parameter(parameter3);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  // TODO SOLVE REMAPPING IN ROS2 NODES !!
  auto azimuthPub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);  
  // auto azimuthPub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose/mag/ned/pose", 1);
  // auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);  

  // const rclcpp::std::map< std::string, std::string > remaps = {
  //   {pnh.resolveName("azimuth_in"), nh.resolveName("pose/mag/ned/pose")},
  // };

  // auto nodelet = createNodelet(log, remaps);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  geometry_msgs::msg::PoseWithCovarianceStamped in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.pose.pose.orientation);
  in.pose.covariance[5 * 6 + 5] = 4;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, SubPoseNoDetect)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  
  node->declare_parameter("target_unit", "rad");
  rclcpp::Parameter parameter1("target_unit", "rad");
  node->set_parameter(parameter1);

  node->declare_parameter("target_orientation", "enu");
  rclcpp::Parameter parameter2("target_orientation", "enu");
  node->set_parameter(parameter2);

  node->declare_parameter("target_reference", "magnetic");
  rclcpp::Parameter parameter3("target_reference", "magnetic");
  node->set_parameter(parameter3);

  node->declare_parameter("input_orientation", "ned");
  rclcpp::Parameter parameter4("input_orientation", "ned");
  node->set_parameter(parameter4);
  
  node->declare_parameter("input_reference", "magnetic");
  rclcpp::Parameter parameter5("input_reference", "magnetic");
  node->set_parameter(parameter5);
  
  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);

  // const auto log = std::make_shared<rclcpp::Logger>();
  

  // auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  geometry_msgs::msg::PoseWithCovarianceStamped in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.pose.pose.orientation);
  in.pose.covariance[5 * 6 + 5] = 4.0;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, SubQuatNameDetect)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  
  node->declare_parameter("target_unit", "rad");
  rclcpp::Parameter parameter1("target_unit", "rad");
  node->set_parameter(parameter1);

  node->declare_parameter("target_orientation", "enu");
  rclcpp::Parameter parameter2("target_orientation", "enu");
  node->set_parameter(parameter2);

  node->declare_parameter("target_reference", "magnetic");
  rclcpp::Parameter parameter3("target_reference", "magnetic");
  node->set_parameter(parameter3);

  node->declare_parameter("input_variance", 4.0);
  rclcpp::Parameter parameter4("input_variance", 4.0);
  node->set_parameter(parameter4);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };


  auto azimuthPub = node->create_publisher<geometry_msgs::msg::QuaternionStamped>("quat/mag/ned/quat", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);  
  // auto azimuthPub = node->create_publisher<geometry_msgs::msg::QuaternionStamped>("quat/mag/ned/quat", 1);
  // auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);  

  // const rclcpp::std::map< std::string, std::string > remaps = {
  //   {pnh.resolveName("azimuth_in"), nh.resolveName("quat/mag/ned/quat")},
  // };

  // auto nodelet = createNodelet(log, remaps);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  geometry_msgs::msg::QuaternionStamped in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.quaternion);

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, SubQuatNoDetect)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  
  node->declare_parameter("target_unit", "rad");
  rclcpp::Parameter parameter1("target_unit", "rad");
  node->set_parameter(parameter1);

  node->declare_parameter("target_orientation", "enu");
  rclcpp::Parameter parameter2("target_orientation", "enu");
  node->set_parameter(parameter2);

  node->declare_parameter("target_reference", "magnetic");
  rclcpp::Parameter parameter3("target_reference", "magnetic");
  node->set_parameter(parameter3);

  node->declare_parameter("input_orientation", "ned");
  rclcpp::Parameter parameter4("input_orientation", "ned");
  node->set_parameter(parameter4);

  node->declare_parameter("input_reference", "magnetic");
  rclcpp::Parameter parameter5("input_reference", "magnetic");
  node->set_parameter(parameter5);

  node->declare_parameter("input_variance", 4.0);
  rclcpp::Parameter parameter6("input_variance", 4.0);
  node->set_parameter(parameter6);
  std::optional<Az> lastAz;

  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<geometry_msgs::msg::QuaternionStamped>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);

  // const auto log = std::make_shared<rclcpp::Logger>();
  

  // auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  geometry_msgs::msg::QuaternionStamped in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.quaternion);

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, PubImu)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  
  node->declare_parameter("target_unit", "rad");
  rclcpp::Parameter parameter1("target_unit", "rad");
  node->set_parameter(parameter1);

  node->declare_parameter("target_orientation", "enu");
  rclcpp::Parameter parameter2("target_orientation", "enu");
  node->set_parameter(parameter2);

  node->declare_parameter("target_reference", "magnetic");
  rclcpp::Parameter parameter3("target_reference", "magnetic");
  node->set_parameter(parameter3);
  
  node->declare_parameter("target_type", "imu");
  rclcpp::Parameter parameter4("target_type", "imu");
  node->set_parameter(parameter4);

  std::optional<sensor_msgs::msg::Imu> lastAz;
  auto cb = [&lastAz](const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<sensor_msgs::msg::Imu>("azimuth_out", 1, cb);

  // const auto log = std::make_shared<rclcpp::Logger>();
  

  // auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, compass_utils::getYaw(lastAz->orientation), 1e-6);
  EXPECT_EQ(4.0, lastAz->orientation_covariance[2 * 3 + 2]);
}

TEST(CompassTransformerNodelet, PubImuSuffix)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  
  node->declare_parameter("target_unit", "rad");
  rclcpp::Parameter parameter1("target_unit", "rad");
  node->set_parameter(parameter1);

  node->declare_parameter("target_orientation", "enu");
  rclcpp::Parameter parameter2("target_orientation", "enu");
  node->set_parameter(parameter2);

  node->declare_parameter("target_reference", "magnetic");
  rclcpp::Parameter parameter3("target_reference", "magnetic");
  node->set_parameter(parameter3);

  node->declare_parameter("target_type", "imu");
  rclcpp::Parameter parameter4("target_type", "imu");
  node->set_parameter(parameter4);

  node->declare_parameter("target_append_suffix", true);
  rclcpp::Parameter parameter5("target_append_suffix", true);
  node->set_parameter(parameter5);


  std::optional<sensor_msgs::msg::Imu> lastAz;
  auto cb = [&lastAz](const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<sensor_msgs::msg::Imu>("azimuth_out/mag/enu/imu", 1, cb);

  
  // const auto log = std::make_shared<rclcpp::Logger>();

  // auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, compass_utils::getYaw(lastAz->orientation), 1e-6);
  EXPECT_EQ(4.0, lastAz->orientation_covariance[2 * 3 + 2]);
}

TEST(CompassTransformerNodelet, PubPose)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  
  node->declare_parameter("target_unit", "rad");
  rclcpp::Parameter parameter1("target_unit", "rad");
  node->set_parameter(parameter1);

  node->declare_parameter("target_orientation", "enu");
  rclcpp::Parameter parameter2("target_orientation", "enu");
  node->set_parameter(parameter2);

  node->declare_parameter("target_reference", "magnetic");
  rclcpp::Parameter parameter3("target_reference", "magnetic");
  node->set_parameter(parameter3);

  node->declare_parameter("target_type", "pose");
  rclcpp::Parameter parameter4("target_type", "pose");
  node->set_parameter(parameter4);
  
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("azimuth_out", 1, cb);

  // const auto log = std::make_shared<rclcpp::Logger>();
  

  // auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, compass_utils::getYaw(lastAz->pose.pose.orientation), 1e-6);
  EXPECT_EQ(4.0, lastAz->pose.covariance[5 * 6 + 5]);
}

TEST(CompassTransformerNodelet, PubPoseSuffix)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  
  node->declare_parameter("target_unit", "rad");
  rclcpp::Parameter parameter1("target_unit", "rad");
  node->set_parameter(parameter1);

  node->declare_parameter("target_orientation", "enu");
  rclcpp::Parameter parameter2("target_orientation", "enu");
  node->set_parameter(parameter2);

  node->declare_parameter("target_reference", "magnetic");
  rclcpp::Parameter parameter3("target_reference", "magnetic");
  node->set_parameter(parameter3);
  
  node->declare_parameter("target_type", "pose");
  rclcpp::Parameter parameter4("target_type", "pose");
  node->set_parameter(parameter4);

  node->declare_parameter("target_append_suffix", true);
  rclcpp::Parameter parameter5("target_append_suffix", true);
  node->set_parameter(parameter5);

  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("azimuth_out/mag/enu/pose", 1, cb);

  
  // const auto log = std::make_shared<rclcpp::Logger>();

  // auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, compass_utils::getYaw(lastAz->pose.pose.orientation), 1e-6);
  EXPECT_EQ(4.0, lastAz->pose.covariance[5 * 6 + 5]);
}

TEST(CompassTransformerNodelet, PubQuat)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  
  node->declare_parameter("target_unit", "rad");
  rclcpp::Parameter parameter1("target_unit", "rad");
  node->set_parameter(parameter1);

  node->declare_parameter("target_orientation", "enu");
  rclcpp::Parameter parameter2("target_orientation", "enu");
  node->set_parameter(parameter2);

  node->declare_parameter("target_reference", "magnetic");
  rclcpp::Parameter parameter3("target_reference", "magnetic");
  node->set_parameter(parameter3);

  node->declare_parameter("target_type", "quaternion");
  rclcpp::Parameter parameter4("target_type", "quaternion");
  node->set_parameter(parameter4);

  std::optional<geometry_msgs::msg::QuaternionStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<geometry_msgs::msg::QuaternionStamped>("azimuth_out", 1, cb);

  // const auto log = std::make_shared<rclcpp::Logger>();
  

  // auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, compass_utils::getYaw(lastAz->quaternion), 1e-6);
}

TEST(CompassTransformerNodelet, PubQuatSuffix)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  
  node->declare_parameter("target_unit", "rad");
  rclcpp::Parameter parameter1("target_unit", "rad");
  node->set_parameter(parameter1);

  node->declare_parameter("target_orientation", "enu");
  rclcpp::Parameter parameter2("target_orientation", "enu");
  node->set_parameter(parameter2);

  node->declare_parameter("target_reference", "magnetic");
  rclcpp::Parameter parameter3("target_reference", "magnetic");
  node->set_parameter(parameter3);

  node->declare_parameter("target_type", "quaternion");
  rclcpp::Parameter parameter4("target_type", "quaternion");
  node->set_parameter(parameter4);
  
  node->declare_parameter("target_append_suffix", true);
  rclcpp::Parameter parameter5("target_append_suffix", true);
  node->set_parameter(parameter5);

  std::optional<geometry_msgs::msg::QuaternionStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<geometry_msgs::msg::QuaternionStamped>("azimuth_out/mag/enu/quat", 1, cb);

  
  // const auto log = std::make_shared<rclcpp::Logger>();

  // auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, compass_utils::getYaw(lastAz->quaternion), 1e-6);
}

TEST(CompassTransformerNodelet, CrossType)  // NOLINT
{
  auto node = createNodelet();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  
  node->declare_parameter("target_unit", "rad");
  rclcpp::Parameter parameter1("target_unit", "rad");
  node->set_parameter(parameter1);

  node->declare_parameter("target_orientation", "enu");
  rclcpp::Parameter parameter2("target_orientation", "enu");
  node->set_parameter(parameter2);

  node->declare_parameter("target_reference", "magnetic");
  rclcpp::Parameter parameter3("target_reference", "magnetic");
  node->set_parameter(parameter3);
  
  node->declare_parameter("target_type", "quaternion");
  rclcpp::Parameter parameter4("target_type", "quaternion");
  node->set_parameter(parameter4);

  node->declare_parameter("input_orientation", "ned");
  rclcpp::Parameter parameter5("input_orientation", "ned");
  node->set_parameter(parameter5);
  
  node->declare_parameter("input_reference", "magnetic");
  rclcpp::Parameter parameter6("input_reference", "magnetic");
  node->set_parameter(parameter6);

  std::optional<geometry_msgs::msg::QuaternionStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<sensor_msgs::msg::Imu>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<geometry_msgs::msg::QuaternionStamped>("azimuth_out", 1, cb);

  // const auto log = std::make_shared<rclcpp::Logger>();
  

  // auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  sensor_msgs::msg::Imu in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.orientation);
  in.orientation_covariance[2 * 3 + 2] = 4.0;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 10 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, compass_utils::getYaw(lastAz->quaternion), 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // Remove the program name from argv because the nodelet handling code does not expect it
  argc -= 1;
  argv += 1;
  auto my_argv = rclcpp::remove_ros_arguments(argc, argv);
  // uint32_t initOptions = rclcpp::init_options::AnonymousName;
  rclcpp::init(argc, argv);//, initOptions);
  // Anonymous nodes have a problem that topic remappings of type ~a:=b are resolved against the node name without the
  // anonymous part. Fix that by running names::init() again after rclcpp::init() finishes and the full node name is known.
  // This was reported and a fix provided in https://github.com/ros/ros_comm/issues/2324, but the fix never landed.
  // rclcpp::names::init(rclcpp::names::getUnresolvedRemappings());

  rclcpp::Node node("prevent_uninitialized");  // Just prevent ROS being uninited when the test-private nodehandles go out of scope

  return RUN_ALL_TESTS();
}
