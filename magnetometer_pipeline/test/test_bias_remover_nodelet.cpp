// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for magnetometer_bias_remover.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <cmath>
#include <gtest/gtest.h>
#include <magnetometer_pipeline/magnetometer_bias_remover_nodelet.hpp>
#include <map>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <string>
#include <utility>

using Field = sensor_msgs::msg::MagneticField;
using namespace std::chrono_literals;

std::shared_ptr<magnetometer_pipeline::MagnetometerBiasRemoverNodelet> createNodelet(rclcpp::NodeOptions node_options = rclcpp::NodeOptions())
{
  auto nodelet = std::make_shared<magnetometer_pipeline::MagnetometerBiasRemoverNodelet>(node_options);
  return nodelet;
}

TEST(MagnetometerBiasRemoverNodelet, Basic)  // NOLINT
{
  // The values in this test are extracted from a real-world bag file recording.

  auto node = createNodelet();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::optional<Field> lastField;
  auto magCb = [&lastField](const Field::ConstSharedPtr& msg)
  {
    lastField = *msg;
  };

  std::list<rclcpp::PublisherBase::SharedPtr> pubs;  
  auto magPub = node->create_publisher<Field>("imu/mag", 1); pubs.push_back(magPub);
  auto magBiasPub = node->create_publisher<Field>("imu/mag_bias", rclcpp::QoS(1).transient_local()); pubs.push_back(magBiasPub);

  std::list<rclcpp::SubscriptionBase::SharedPtr> subs;
  auto magUnbiasedSub = node->create_subscription<Field>("imu/mag_unbiased", 1, magCb); subs.push_back(magUnbiasedSub);

  const auto pubTest = [](const rclcpp::PublisherBase::SharedPtr p) {return p->get_subscription_count() == 0;};

  for (size_t i = 0; i < 1000 && std::any_of(pubs.begin(), pubs.end(), pubTest); ++i)
  {
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
    executor.spin_once();
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for publisher connections.");
  }

  RCLCPP_INFO(node->get_logger(), "A");

  const auto subTest = [](const rclcpp::SubscriptionBase::SharedPtr p) {return p->get_publisher_count() == 0;};
    
  RCLCPP_INFO(node->get_logger(), "B");

  for (size_t i = 0; i < 1000 && std::any_of(subs.begin(), subs.end(), subTest); ++i)
  {
    RCLCPP_INFO(node->get_logger(), "C");
    rclcpp::sleep_for(std::chrono::nanoseconds(10'000'000));
    executor.spin_once();
    RCLCPP_WARN_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 200., "Waiting for subscriber connections.");
  }

  RCLCPP_INFO(node->get_logger(), "D");

  ASSERT_FALSE(std::any_of(pubs.begin(), pubs.end(), pubTest));
  ASSERT_FALSE(std::any_of(subs.begin(), subs.end(), subTest));

    builtin_interfaces::msg::Time time;
  time.sec = 1664286802;
  time.nanosec = 187375068;

  Field mag;
  mag.header.stamp = time;
  mag.header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.263093;
  mag.magnetic_field.y = -0.538677;
  mag.magnetic_field.z = 0.157033;
  magPub->publish(mag);
  RCLCPP_INFO(node->get_logger(), "E");

  for (size_t i = 0; i < 5 && !lastField.has_value() && rclcpp::ok(); ++i)
  {
    RCLCPP_INFO(node->get_logger(), "F");
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
  }
  // Missing bias, nothing published
  RCLCPP_INFO(node->get_logger(), "G");

  ASSERT_FALSE(lastField.has_value());
  RCLCPP_INFO(node->get_logger(), "H");

  // Publish bias. Now it should have everything.

  Field bias;
  bias.header.stamp = time;
  bias.header.frame_id = "imu";
  bias.magnetic_field.x = -0.097227663;
  bias.magnetic_field.y = -0.692264333;
  bias.magnetic_field.z = 0;
  magBiasPub->publish(bias);

  executor.spin_once();

  // Wait until the latched messages are received

  rclcpp::sleep_for(std::chrono::nanoseconds(200'000'000));
  executor.spin_once();


  magPub->publish(mag);

  for (size_t i = 0; i < 10 && !lastField.has_value() && rclcpp::ok(); ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
  }
  ASSERT_TRUE(lastField.has_value());

  EXPECT_EQ(time, lastField->header.stamp);
  EXPECT_EQ("imu", lastField->header.frame_id);
  EXPECT_NEAR(0.360320, lastField->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.153587, lastField->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.157033, lastField->magnetic_field.z, 1e-6);

  // New data
  lastField.reset();
  time.sec = 1664286802;
  time.nanosec = 197458028;

  mag.header.stamp = time;
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag.magnetic_field.x = 0.264200;
  mag.magnetic_field.y = -0.533960;
  mag.magnetic_field.z = 0.149800;
  magPub->publish(mag);

  for (size_t i = 0; i < 10 && !lastField.has_value() && rclcpp::ok(); ++i)
  {
    executor.spin_once();
    rclcpp::sleep_for(std::chrono::nanoseconds(100'000'000));
  }
  ASSERT_TRUE(lastField.has_value());

  EXPECT_EQ(time, lastField->header.stamp);
  EXPECT_EQ("imu", lastField->header.frame_id);
  EXPECT_NEAR(0.361427, lastField->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.158304, lastField->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.149800, lastField->magnetic_field.z, 1e-6);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  rclcpp::shutdown();
}
