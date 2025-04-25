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
#include <chrono>
#include <builtin_interfaces/msg/time.hpp>

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
#include <tf2_ros/create_timer_ros.h>
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

/* builtin_interfaces::msg::Time get_current_ros_time()
{
    using namespace std::chrono;

    // Get current time since epoch in nanoseconds
    auto now = system_clock::now();
    auto duration = now.time_since_epoch();
    auto sec = duration_cast<seconds>(duration);
    auto nanosec = duration_cast<nanoseconds>(duration - sec);

    // Fill builtin_interfaces::msg::Time
    builtin_interfaces::msg::Time ros_time;
    ros_time.sec = static_cast<int32_t>(sec.count());
    ros_time.nanosec = static_cast<uint32_t>(nanosec.count());

    return ros_time;
} */


std::shared_ptr<compass_conversions::CompassTransformerNodelet> createNodelet(rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
{
  auto nodelet = std::make_shared<compass_conversions::CompassTransformerNodelet>(node_options);
  return nodelet;
}

/* TEST(CompassTransformerNodelet, JustFix)  // NOLINT
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node >("test_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  auto fixPub = node->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 1);

  const auto time = compass_utils::parseTime("2024-11-18T13:00:00Z");

  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = time;
  fix.header.frame_id = "test";
  fix.latitude = 51.0;
  fix.longitude = 15.0;
  fix.altitude = 200.0;
  fixPub->publish(fix);

  for (size_t i = 0; i < 100; ++i)
  {
    printf("BB\n");
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
  }
} */

TEST(CompassTransformerNodelet, BasicConversion)  // NOLINT
{
  // Apparently it works just as well if you have just the one node and create any additional
  // pubs and subs out of it...
  // rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_node");

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  auto node = createNodelet(node_options);
  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // node->declare_parameter("target_unit", "rad");
  // rclcpp::Parameter parameter1("target_unit", "rad");
  // node->set_parameter(parameter1);
  // node->declare_parameter("target_orientation", "enu");
  // rclcpp::Parameter parameter2("target_orientation", "enu");
  // node->set_parameter(parameter2);
  // node->declare_parameter("target_reference", "magnetic");
  // rclcpp::Parameter parameter3("target_reference", "magnetic");
  // node->set_parameter(parameter3);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);
  
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
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

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
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

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_unit", "deg");
  node_options.append_parameter_override("target_orientation", "ned");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_frame", "test2");

  auto node = createNodelet(node_options);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = node->now();
  tf.header.frame_id = "test";
  tf.child_frame_id = "test2";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, tf.transform.rotation);

  auto tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());
  tfBuffer->setCreateTimerInterface(timer_interface);
  
  tfBuffer->setTransform(tf, "test", true);

  node->setBuffer(tfBuffer);
  node->init();

  ASSERT_NE(nullptr, node);  

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);  

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
    printf("D");

  }
  printf("E");

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
  printf("F");

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
    printf("G");

  }
  ASSERT_TRUE(lastAz.has_value());
  printf("H");

  // EXPECT_EQ(in.header.stamp, lastAz->header.stamp); //TODO buffer->transform changes timestamp...
  EXPECT_EQ("test2", lastAz->header.frame_id);
  EXPECT_NEAR(180.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_DEG, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_NED, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
}

TEST(CompassTransformerNodelet, TfConversionFail)  // NOLINT
{

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_unit", "deg");
  node_options.append_parameter_override("target_orientation", "ned");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_frame", "test_nonexistent");

  auto node = createNodelet(node_options);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = node->now();
  tf.header.frame_id = "test";
  tf.child_frame_id = "test2";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, tf.transform.rotation);

  // printf("node clk sec %u", node->get_clock()->now().seconds());
  // printf("node clk nanosec %u", node->get_clock()->now().nanoseconds());

  auto tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());
  tfBuffer->setCreateTimerInterface(timer_interface);
  
  tfBuffer->setTransform(tf, "test", true);

  node->setBuffer(tfBuffer);
  node->init();

  ASSERT_NE(nullptr, node);  

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  Az in;
  in.header.stamp = node->now();
  // printf("node now sec %u", in.header.stamp.sec);
  // printf("node now nanosec %u", in.header.stamp.nanosec);
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0;
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  azimuthPub->publish(in);

  for (size_t i = 0; i < 500 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once(std::chrono::nanoseconds(1'000'000));
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
  }
  ASSERT_FALSE(lastAz.has_value());
}

TEST(CompassTransformerNodelet, FixMissing)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_reference", "utm");

  auto node = createNodelet(node_options);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);

  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
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

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once(std::chrono::nanoseconds(10'000'000));
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
  }
  ASSERT_FALSE(lastAz.has_value());
}

TEST(CompassTransformerNodelet, FixFromParams)  // NOLINT
{

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_reference", "geographic");
  node_options.append_parameter_override("initial_lat", 51.0);
  node_options.append_parameter_override("initial_lon", 15.0);
  node_options.append_parameter_override("initial_alt", 200.0);

  auto node = createNodelet(node_options);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);

  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
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

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
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

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_reference", "geographic");

  auto node = createNodelet(node_options);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto fixPub = node->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);


  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 &&
    (azimuthPub->get_subscription_count() == 0 || fixPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for fix and azimuth input and output topics.");
  }

  printf("A");

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
  printf("B");

  // Wait until the fix message is received
  for (size_t i = 0; i < 10; ++i)
  {
    executor.spin_once(std::chrono::nanoseconds(10'000'000));
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
  }

  Az in;
  in.header.stamp = time;
  in.header.frame_id = "test";
  in.azimuth = 90.0;
  in.variance = 4.0 * std::pow(180.0 / M_PI, 2.0);
  in.unit = Az::UNIT_DEG;
  in.orientation = Az::ORIENTATION_NED;
  in.reference = Az::REFERENCE_MAGNETIC;

  printf("BBB\n");

  azimuthPub->publish(in);
  printf("C");

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
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

/* TEST(CompassTransformerNodelet, SubImuNameDetect)  // NOLINT
{

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");

  auto node = createNodelet(node_options);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  // TODO SOLVE REMAPPING IN ROS2 NODES !!
  auto azimuthPub = node->create_publisher<sensor_msgs::msg::Imu>("azimuth_in/imu", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);
  // auto azimuthPub = node->create_publisher<sensor_msgs::msg::Imu>("imu/data/mag/ned/imu", 1);
  // auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);  

  // const std::map< std::string, std::string > remaps = {
  //   {pnh.resolveName("azimuth_in"), nh.resolveName("imu/data/mag/ned/imu")},
  // };

  // auto nodelet = createNodelet(log, remaps);
  ASSERT_NE(nullptr, node);
  printf("A\n");

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
    executor.spin_once();
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for azimuth input and output topics.");
  }
  printf("B\n");

  ASSERT_GT(azimuthPub->get_subscription_count(), 0);
  ASSERT_GT(azimuthSub->get_publisher_count(), 0);

  sensor_msgs::msg::Imu in;
  in.header.stamp = node->now();
  in.header.frame_id = "test";
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI_2);
  tf2::convert(q, in.orientation);
  in.orientation_covariance[2 * 3 + 2] = 4.0;
  printf("C\n");

  azimuthPub->publish(in);

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    printf("D\n");
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
} */

TEST(CompassTransformerNodelet, SubImuNoDetect)  // NOLINT
{

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("input_orientation", "ned");
  node_options.append_parameter_override("input_reference", "magnetic");

  auto node = createNodelet(node_options);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<sensor_msgs::msg::Imu>("azimuth_in/imu", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);

  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
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

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
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

/* TEST(CompassTransformerNodelet, SubPoseNameDetect)  // NOLINT
{

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");

  auto node = createNodelet(node_options);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

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
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
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

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
} */

TEST(CompassTransformerNodelet, SubPoseNoDetect)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("input_orientation", "ned");
  node_options.append_parameter_override("input_reference", "magnetic");

  auto node = createNodelet(node_options);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("azimuth_in/pose", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);

  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
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

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
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

/* TEST(CompassTransformerNodelet, SubQuatNameDetect)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("input_variance", 4.0);

  auto node = createNodelet(node_options);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;
  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<geometry_msgs::msg::QuaternionStamped>("quat/mag/ned/quat", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);  
  //TODO remaps
  // auto azimuthPub = node->create_publisher<geometry_msgs::msg::QuaternionStamped>("quat/mag/ned/quat", 1);
  // auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);  

  // const rclcpp::std::map< std::string, std::string > remaps = {
  //   {pnh.resolveName("azimuth_in"), nh.resolveName("quat/mag/ned/quat")},
  // };

  // auto nodelet = createNodelet(log, remaps);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
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

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, lastAz->azimuth, 1e-6);
  EXPECT_EQ(Az::UNIT_RAD, lastAz->unit);
  EXPECT_EQ(Az::ORIENTATION_ENU, lastAz->orientation);
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, lastAz->reference);
  EXPECT_EQ(4.0, lastAz->variance);
} */

TEST(CompassTransformerNodelet, SubQuatNoDetect)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("input_orientation", "ned");
  node_options.append_parameter_override("input_reference", "magnetic");
  node_options.append_parameter_override("input_variance", 4.0);

  auto node = createNodelet(node_options);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Az> lastAz;

  auto cb = [&lastAz](const Az::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<geometry_msgs::msg::QuaternionStamped>("azimuth_in/quat", 1);
  auto azimuthSub = node->create_subscription<Az>("azimuth_out", 1, cb);

  // const auto log = std::make_shared<rclcpp::Logger>();
  

  // auto nodelet = createNodelet(log);
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
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

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
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
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_type", "imu");

  auto node = createNodelet(node_options);

  node->init();
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

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
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
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

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, compass_utils::getYaw(lastAz->orientation), 1e-6);
  EXPECT_EQ(4.0, lastAz->orientation_covariance[2 * 3 + 2]);
}

TEST(CompassTransformerNodelet, PubImuSuffix)  // NOLINT
{

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_type", "imu");
  node_options.append_parameter_override("target_append_suffix", true);

  auto node = createNodelet(node_options);

  node->init();
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<sensor_msgs::msg::Imu> lastAz;
  auto cb = [&lastAz](const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<sensor_msgs::msg::Imu>("azimuth_out/mag/enu/imu", 1, cb);

  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
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

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, compass_utils::getYaw(lastAz->orientation), 1e-6);
  EXPECT_EQ(4.0, lastAz->orientation_covariance[2 * 3 + 2]);
}

TEST(CompassTransformerNodelet, PubPose)  // NOLINT
{

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_type", "pose");

  auto node = createNodelet(node_options);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
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
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
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

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, compass_utils::getYaw(lastAz->pose.pose.orientation), 1e-6);
  EXPECT_EQ(4.0, lastAz->pose.covariance[5 * 6 + 5]);
}

TEST(CompassTransformerNodelet, PubPoseSuffix)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_type", "pose");
  node_options.append_parameter_override("target_append_suffix", true);

  auto node = createNodelet(node_options);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("azimuth_out/mag/enu/pose", 1, cb);

  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
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

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, compass_utils::getYaw(lastAz->pose.pose.orientation), 1e-6);
  EXPECT_EQ(4.0, lastAz->pose.covariance[5 * 6 + 5]);
}

TEST(CompassTransformerNodelet, PubQuat)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_type", "quaternion");

  auto node = createNodelet(node_options);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<geometry_msgs::msg::QuaternionStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<geometry_msgs::msg::QuaternionStamped>("azimuth_out", 1, cb);

  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
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

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, compass_utils::getYaw(lastAz->quaternion), 1e-6);
}

TEST(CompassTransformerNodelet, PubQuatSuffix)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_type", "quaternion");
  node_options.append_parameter_override("target_append_suffix", true);

  auto node = createNodelet(node_options);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<geometry_msgs::msg::QuaternionStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<Az>("azimuth_in", 1);
  auto azimuthSub = node->create_subscription<geometry_msgs::msg::QuaternionStamped>("azimuth_out/mag/enu/quat", 1, cb);

  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
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

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, compass_utils::getYaw(lastAz->quaternion), 1e-6);
}

TEST(CompassTransformerNodelet, CrossType)  // NOLINT
{
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("target_unit", "rad");
  node_options.append_parameter_override("target_orientation", "enu");
  node_options.append_parameter_override("target_reference", "magnetic");
  node_options.append_parameter_override("target_type", "quaternion");
  node_options.append_parameter_override("input_orientation", "ned");
  node_options.append_parameter_override("input_reference", "magnetic");

  auto node = createNodelet(node_options);

  node->init();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<geometry_msgs::msg::QuaternionStamped> lastAz;
  auto cb = [&lastAz](const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& msg)
  {
    lastAz = *msg;
  };

  auto azimuthPub = node->create_publisher<sensor_msgs::msg::Imu>("azimuth_in/imu", 1);
  auto azimuthSub = node->create_subscription<geometry_msgs::msg::QuaternionStamped>("azimuth_out", 1, cb);
;
  ASSERT_NE(nullptr, node);

  for (size_t i = 0; i < 1000 && (azimuthPub->get_subscription_count() == 0 || azimuthSub->get_publisher_count() == 0); ++i)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
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

  for (size_t i = 0; i < 50 && !lastAz.has_value() && rclcpp::ok() ; ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
  }
  ASSERT_TRUE(lastAz.has_value());

  EXPECT_EQ(in.header.stamp, lastAz->header.stamp);
  EXPECT_EQ(in.header.frame_id, lastAz->header.frame_id);
  EXPECT_NEAR(0.0, compass_utils::getYaw(lastAz->quaternion), 1e-6);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  rclcpp::shutdown();
}
