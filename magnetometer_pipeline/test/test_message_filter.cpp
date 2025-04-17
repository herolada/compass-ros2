// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for magnetometer bias removing message filter.
 * \author Martin Pecka
 */

#include "gtest/gtest.h"

#include <memory>
#include <string>

// #include <boost/functional.hpp>

// #include <cras_cpp_common/log_utils.h>
// #include <cras_cpp_common/log_utils/memory.h>
// #include <cras_cpp_common/log_utils/node.h>
// #include <cras_cpp_common/param_utils/get_param_adapters/xmlrpc_value.hpp>
#include <magnetometer_pipeline/message_filter.h>
#include <message_filters/simple_filter.h>
#include <message_filters/message_event.h>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <rclcpp/logger.hpp>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/time.hpp>

using Field = sensor_msgs::msg::MagneticField;

template<class T>
class TestInput : public message_filters::SimpleFilter<T>
{
public:
  void add(const typename T::ConstSharedPtr& msg)
  {
    // Pass a complete MessageEvent to avoid calling node->now() to determine the missing timestamp
    this->signalMessage(message_filters::MessageEvent<T const>(msg, msg->header.stamp));
  }

  void subscribe()
  {
  }
};

TEST(MessageFilter, Basic)  // NOLINT
{
  
  
  
  const auto log = rclcpp::get_logger("test");
  const auto clk = rclcpp::Clock();

  TestInput<Field> magInput;
  TestInput<Field> magBiasInput;
  magnetometer_pipeline::BiasRemoverFilter filter(log, clk, magInput, magBiasInput);

  Field::ConstSharedPtr outMessage;
  const auto cb = [&outMessage](const message_filters::MessageEvent<Field const>& filteredMessage)
  {
    outMessage = filteredMessage.getConstMessage();
  };
  filter.registerCallback(std::function<void(const message_filters::MessageEvent<Field const>&)>(cb));

  rclcpp::Time time(1664286802, 187375068);

  Field::SharedPtr mag(new Field);
  mag->header.stamp = time;
  mag->header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag->magnetic_field.x = 0.263093;
  mag->magnetic_field.y = -0.538677;
  mag->magnetic_field.z = 0.157033;

  Field::SharedPtr bias(new Field);
  bias->header.stamp = time;
  bias->header.frame_id = "imu";
  bias->magnetic_field.x = -0.097227663;
  bias->magnetic_field.y = -0.692264333;
  bias->magnetic_field.z = 0;

  outMessage.reset();
  magInput.add(mag);

  EXPECT_EQ(nullptr, outMessage);

  outMessage.reset();
  magBiasInput.add(bias);
  EXPECT_EQ(nullptr, outMessage);

  outMessage.reset();
  magInput.add(mag);
  ASSERT_NE(nullptr, outMessage);
  EXPECT_EQ(time, outMessage->header.stamp);
  EXPECT_EQ("imu", outMessage->header.frame_id);
  EXPECT_NEAR(0.360320, outMessage->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.153587, outMessage->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.157033, outMessage->magnetic_field.z, 1e-6);

  // New data

  time = {1664286802, 197458028};

  mag->header.stamp = time;
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag->magnetic_field.x = 0.264200;
  mag->magnetic_field.y = -0.533960;
  mag->magnetic_field.z = 0.149800;

  outMessage.reset();
  magInput.add(mag);
  ASSERT_NE(nullptr, outMessage);
  EXPECT_EQ(time, outMessage->header.stamp);
  EXPECT_EQ("imu", outMessage->header.frame_id);
  EXPECT_NEAR(0.361427, outMessage->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.158304, outMessage->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.149800, outMessage->magnetic_field.z, 1e-6);
}

TEST(MessageFilter, ConfigFromParams)  // NOLINT
{
  
  auto node = std::make_shared<rclcpp::Node>("test_node", rclcpp::NodeOptions().allow_undeclared_parameters(true));
  
  // const auto clk = rclcpp::Clock();

  TestInput<Field> magInput;
  TestInput<Field> magBiasInput;
  magnetometer_pipeline::BiasRemoverFilter filter(node->get_logger(), *node->get_clock(), magInput, magBiasInput);

  Field::ConstSharedPtr outMessage;
  const auto cb = [&outMessage](const message_filters::MessageEvent<Field const>& filteredMessage)
  {
    outMessage = filteredMessage.getConstMessage();
  };
  filter.registerCallback(std::function<void(const message_filters::MessageEvent<Field const>&)>(cb));

  rclcpp::Parameter param1("initial_mag_bias_x", -0.097227663);
  node->set_parameter(param1);
  rclcpp::Parameter param2("initial_mag_bias_y", -0.692264333);
  node->set_parameter(param2);
  rclcpp::Parameter param3("initial_mag_bias_z", 0.0);  // Using 0.0 for double type
  node->set_parameter(param3);

  filter.configFromParams(node);

  rclcpp::Time time(1664286802, 187375068);

  Field::SharedPtr mag(new Field);
  mag->header.stamp = time;
  mag->header.frame_id = "imu";
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag->magnetic_field.x = 0.263093;
  mag->magnetic_field.y = -0.538677;
  mag->magnetic_field.z = 0.157033;

  outMessage.reset();
  magInput.add(mag);
  ASSERT_NE(nullptr, outMessage);
  EXPECT_EQ(time, outMessage->header.stamp);
  EXPECT_EQ("imu", outMessage->header.frame_id);
  EXPECT_NEAR(0.360320, outMessage->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.153587, outMessage->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.157033, outMessage->magnetic_field.z, 1e-6);

  // New data

  time = {1664286802, 197458028};

  mag->header.stamp = time;
  // These values are exaggerated (in Gauss instead of in Tesla), but they're consistent with ethzasl_xsens_driver
  // output. To just estimate the direction, it is no problem.
  mag->magnetic_field.x = 0.264200;
  mag->magnetic_field.y = -0.533960;
  mag->magnetic_field.z = 0.149800;

  outMessage.reset();
  magInput.add(mag);
  ASSERT_NE(nullptr, outMessage);
  EXPECT_EQ(time, outMessage->header.stamp);
  EXPECT_EQ("imu", outMessage->header.frame_id);
  EXPECT_NEAR(0.361427, outMessage->magnetic_field.x, 1e-6);
  EXPECT_NEAR(0.158304, outMessage->magnetic_field.y, 1e-6);
  EXPECT_NEAR(0.149800, outMessage->magnetic_field.z, 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
