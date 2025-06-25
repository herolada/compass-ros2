#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Support for transforming compass_interfaces::msg::Azimuth messages.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <compass_conversions/compass_converter.h>
#include <compass_conversions/message_filter.h>
#include <compass_conversions/tf2_compass_msgs.h>
#include <compass_conversions/topic_names.h>
#include <compass_interfaces/msg/azimuth.hpp>
#include <compass_utils/string_utils.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <memory>
#include <message_filters/subscriber.h>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

namespace compass_conversions
{

enum class OutputType
{
  Azimuth,
  Imu,
  Pose,
  Quaternion,
};

OutputType parseOutputType(const std::string& outputType);
std::string outputTypeToString(const OutputType type);

class CompassTransformerNodelet : public rclcpp::Node
{
public:
  CompassTransformerNodelet();
  CompassTransformerNodelet(const rclcpp::NodeOptions & options);
  ~CompassTransformerNodelet() override;

  void init();
  void setBuffer(tf2_ros::Buffer::SharedPtr buffer, bool using_dedicated_thread);

protected:
  void publish(const compass_interfaces::msg::Azimuth::ConstSharedPtr& msg);
  void transformAndPublish(const compass_interfaces::msg::Azimuth::ConstSharedPtr& msg);
  void failedCb(const compass_interfaces::msg::Azimuth::ConstSharedPtr& msg,
                const tf2_ros::filter_failure_reasons::FilterFailureReason reason);

  std::shared_ptr<CompassConverter> converter;
  std::unique_ptr<UniversalAzimuthSubscriber> azimuthInput;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>> fixInput;
  std::unique_ptr<message_filters::Subscriber<std_msgs::msg::Int32>> utmZoneInput;
  std::unique_ptr<CompassFilter> compassFilter;
  std::unique_ptr<tf2_ros::MessageFilter<compass_interfaces::msg::Azimuth>> tfFilter;

  rclcpp::Publisher<compass_interfaces::msg::Azimuth>::SharedPtr pub_az;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub_quat;

  std::string targetFrame;
  std::string outFrameId;
  OutputType targetType {OutputType::Azimuth};
  tf2_ros::Buffer::SharedPtr buffer;
  std::shared_ptr<tf2_ros::TransformListener> listener;
  // std::map< std::string, std::string > remaps {};
};

}  // namespace compass_conversions
