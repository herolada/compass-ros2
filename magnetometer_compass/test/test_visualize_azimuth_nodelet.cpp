// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for azimuth visualization.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <memory>
#include <string>
#include <optional>

#include <compass_interfaces/msg/azimuth.hpp>
#include <compass_utils/time_utils.hpp>
//#include <cras_cpp_common/log_utils/memory.h>
//#include <cras_cpp_common/log_utils/node.h>
//#include <cras_cpp_common/param_utils/param_helper.hpp>
//#include <cras_cpp_common/string_utils/ros.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
//#include <ros/ros.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/utilities.hpp>

using Az = compass_interfaces::msg::Azimuth;
using Quat = geometry_msgs::msg::QuaternionStamped;
using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
using Fix = sensor_msgs::msg::NavSatFix;

TEST(VisualizeAzimuthNodelet, Basic)  // NOLINT
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("test_node");

  std::optional<Pose> lastPose;
  auto poseCb = [&lastPose](const Pose::ConstSharedPtr& msg)
  {
    lastPose = *msg;
  };

  std::list<rclcpp::PublisherBase::SharedPtr> pubs;
  auto azPub = node->create_publisher<Az>("visualize_azimuth/azimuth", 1); pubs.push_back(azPub);
  auto fixPub = node->create_publisher<Fix>("gps/fix", rclcpp::QoS(1).transient_local()); pubs.push_back(fixPub);

  std::list<rclcpp::SubscriptionBase::SharedPtr> subs;
  auto visSub = node->create_subscription<Pose>("visualize_azimuth/azimuth_vis", 1, poseCb); subs.push_back(visSub);

  const auto pubTest = [](const rclcpp::PublisherBase::SharedPtr p) {return p->get_subscription_count() == 0;};

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  for (size_t i = 0; i < 1000 && std::any_of(pubs.begin(), pubs.end(), pubTest); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 0.2, "Waiting for publisher connections.");
  }

  const auto subTest = [](const rclcpp::SubscriptionBase::SharedPtr p) {return p->get_publisher_count() == 0;};
  for (size_t i = 0; i < 1000 && std::any_of(subs.begin(), subs.end(), subTest); ++i)
  {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.01*1e09)));
    executor.spin_once();
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 0.2, "Waiting for subscriber connections.");
  }

  ASSERT_FALSE(std::any_of(pubs.begin(), pubs.end(), pubTest));
  ASSERT_FALSE(std::any_of(subs.begin(), subs.end(), subTest));

  rclcpp::Time time = compass_utils::parseTime("2024-11-18T13:00:00Z");

  Az azimuth;
  azimuth.header.frame_id = "base_link";
  azimuth.header.stamp = time;
  azimuth.azimuth = 0;
  azimuth.variance = 0;
  azimuth.unit = Az::UNIT_RAD;
  azimuth.orientation = Az::ORIENTATION_ENU;
  azimuth.reference = Az::REFERENCE_UTM;
  azPub->publish(azimuth);

  for (size_t i = 0; i < 10 && !lastPose.has_value() && rclcpp::ok(); ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastPose.has_value());
  EXPECT_EQ(time, lastPose->header.stamp);
  EXPECT_EQ("base_link", lastPose->header.frame_id);
  EXPECT_EQ(0, lastPose->pose.covariance[0]);
  EXPECT_EQ(0, lastPose->pose.covariance[35]);
  EXPECT_EQ(0, lastPose->pose.pose.position.x);
  EXPECT_EQ(0, lastPose->pose.pose.position.y);
  EXPECT_EQ(0, lastPose->pose.pose.position.z);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.x);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.y);
  EXPECT_NEAR(M_SQRT1_2, lastPose->pose.pose.orientation.z, 1e-6);
  EXPECT_NEAR(M_SQRT1_2, lastPose->pose.pose.orientation.w, 1e-6);

  lastPose.reset();
  azimuth.azimuth = M_PI_2;
  azPub->publish(azimuth);

  for (size_t i = 0; i < 10 && !lastPose.has_value() && rclcpp::ok(); ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastPose.has_value());
  EXPECT_EQ(time, lastPose->header.stamp);
  EXPECT_EQ("base_link", lastPose->header.frame_id);
  EXPECT_EQ(0, lastPose->pose.covariance[0]);
  EXPECT_EQ(0, lastPose->pose.covariance[35]);
  EXPECT_EQ(0, lastPose->pose.pose.position.x);
  EXPECT_EQ(0, lastPose->pose.pose.position.y);
  EXPECT_EQ(0, lastPose->pose.pose.position.z);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.x);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.y);
  EXPECT_NEAR(0, lastPose->pose.pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, lastPose->pose.pose.orientation.w, 1e-6);

  lastPose.reset();
  azimuth.azimuth = M_PI;
  azPub->publish(azimuth);

  for (size_t i = 0; i < 10 && !lastPose.has_value() && rclcpp::ok(); ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastPose.has_value());
  EXPECT_EQ(time, lastPose->header.stamp);
  EXPECT_EQ("base_link", lastPose->header.frame_id);
  EXPECT_EQ(0, lastPose->pose.covariance[0]);
  EXPECT_EQ(0, lastPose->pose.covariance[35]);
  EXPECT_EQ(0, lastPose->pose.pose.position.x);
  EXPECT_EQ(0, lastPose->pose.pose.position.y);
  EXPECT_EQ(0, lastPose->pose.pose.position.z);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.x);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.y);
  EXPECT_NEAR(-M_SQRT1_2, lastPose->pose.pose.orientation.z, 1e-6);
  EXPECT_NEAR(M_SQRT1_2, lastPose->pose.pose.orientation.w, 1e-6);

  // We haven't yet supplied a fix, so change of reference is not possible

  lastPose.reset();
  azimuth.azimuth = M_PI;
  azimuth.reference = Az::REFERENCE_MAGNETIC;
  azPub->publish(azimuth);

  for (size_t i = 0; i < 5 && !lastPose.has_value() && rclcpp::ok(); ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_FALSE(lastPose.has_value());

  Fix fix;
  fix.header.stamp = time;
  fix.header.frame_id = "gps";
  fix.latitude = 50.090806436;
  fix.longitude = 14.133202857;
  fix.altitude = 445.6146;
  fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  fixPub->publish(fix);

  // Wait until the fix arrives
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.2*1e09)));

  lastPose.reset();
  azimuth.azimuth = M_PI;
  azPub->publish(azimuth);

  for (size_t i = 0; i < 10 && !lastPose.has_value() && rclcpp::ok(); ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1*1e09)));
  }
  ASSERT_TRUE(lastPose.has_value());
  EXPECT_EQ(time, lastPose->header.stamp);
  EXPECT_EQ("base_link", lastPose->header.frame_id);
  EXPECT_EQ(0, lastPose->pose.covariance[0]);
  EXPECT_EQ(0, lastPose->pose.covariance[35]);
  EXPECT_EQ(0, lastPose->pose.pose.position.x);
  EXPECT_EQ(0, lastPose->pose.pose.position.y);
  EXPECT_EQ(0, lastPose->pose.pose.position.z);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.x);
  EXPECT_EQ(0, lastPose->pose.pose.orientation.y);
  EXPECT_NEAR(-0.671109, lastPose->pose.pose.orientation.z, 1e-6);
  EXPECT_NEAR(0.741358, lastPose->pose.pose.orientation.w, 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);
  rclcpp::Node node("prevent_uninitalized");  // Just prevent ROS being uninited when the test-private nodehandles go out of scope

  return RUN_ALL_TESTS();
}
