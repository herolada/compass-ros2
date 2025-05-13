// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Convert magnetometer and IMU measurements to azimuth.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include "tl/expected.hpp"
#include <Eigen/Core>
#include <angles/angles.h>
#include <compass_interfaces/msg/azimuth.hpp>
#include <compass_utils/string_utils.hpp>
#include <compass_utils/tf2_utils.hpp>
#include <magnetometer_compass/magnetometer_compass.h>
#include <magnetometer_compass/tf2_sensor_msgs.h>
#include <map>
#include <memory>
#include <optional>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace magnetometer_compass
{
using Az = compass_interfaces::msg::Azimuth;
using Imu = sensor_msgs::msg::Imu;
using Field = sensor_msgs::msg::MagneticField;

struct MagnetometerCompassPrivate
{
  std::shared_ptr<tf2_ros::Buffer const> tf;
  std::string frame;
  std::optional<tf2::Quaternion> lastAzimuth;
  double variance {0.0};
  double initialVariance {0.0};
  double lowPassRatio {0.95};
};

MagnetometerCompass::MagnetometerCompass(
  rclcpp::Node* node, const std::string& frame, const std::shared_ptr<tf2::BufferCore>& tf) :
  MagnetometerCompass(node, frame, std::make_shared<tf2_ros::Buffer>(node->get_clock(), tf ? tf->getCacheLength() : std::chrono::duration_cast<std::chrono::nanoseconds>(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME)))
{
}

MagnetometerCompass::MagnetometerCompass(
  rclcpp::Node* node, const std::string& frame, const std::shared_ptr<tf2_ros::Buffer>& tf) :
  node(node), data(new MagnetometerCompassPrivate{})
{
  this->data->tf = tf;
  this->data->frame = frame;
}

MagnetometerCompass::~MagnetometerCompass() = default;

void MagnetometerCompass::configFromParams()
{
  if (this->node->has_parameter("initial_variance") &&
      this->node->get_parameter("initial_variance").get_value<double>() != -1.) {
    this->data->variance = this->data->initialVariance = this->node->get_parameter("initial_variance").get_value<double>();    
  }

  if (this->node->has_parameter("low_pass_ratio") &&
      this->node->get_parameter("low_pass_ratio").get_value<double>() != -1.) {
    this->data->lowPassRatio = this->node->get_parameter("low_pass_ratio").get_value<double>();
  }
}

void MagnetometerCompass::setLowPassRatio(const double ratio)
{
  this->data->lowPassRatio = ratio;
}

tl::expected<compass_interfaces::msg::Azimuth, std::string> MagnetometerCompass::computeAzimuth(
  const sensor_msgs::msg::Imu& imu, const sensor_msgs::msg::MagneticField& magUnbiased)
{
  Imu imuInBody;
  try
  {
    this->data->tf->transform(imu, imuInBody, this->data->frame, tf2::durationFromSec(0.1));
  }
  catch (const tf2::TransformException& e)
  {
    return compass_utils::make_unexpected(std::format(
      "Could not transform IMU data to frame {} because: {}", this->data->frame.c_str(), e.what()));
  }

  Field magUnbiasedInBody;
  try
  {
    this->data->tf->transform(magUnbiased, magUnbiasedInBody, this->data->frame, tf2::durationFromSec(0.1));
  }
  catch (const tf2::TransformException& e)
  {
    return compass_utils::make_unexpected(std::format(
      "Could not transform magnetometer to frame {} because: {}", this->data->frame.c_str(), e.what()));
  }

  // Compensate attitude in the magnetometer measurements

  double roll, pitch, yaw;
  compass_utils::getRPY(imuInBody.orientation, roll, pitch, yaw);

#if 1
  tf2::Quaternion rot;
  rot.setRPY(roll, pitch, 0);
  tf2::Vector3 magNoAttitude;
  tf2::convert(magUnbiasedInBody.magnetic_field, magNoAttitude);
  magNoAttitude = tf2::quatRotate(rot, magNoAttitude);

  const auto magNorth = magNoAttitude.x();
  const auto magEast = magNoAttitude.y();
#else
  // Copied from INSO, not sure where do the numbers come from
  const auto magNorth =
    magUnbiasedInBody.magnetic_field.x * cos(pitch) +
    magUnbiasedInBody.magnetic_field.y * sin(pitch) * sin(roll) +
    magUnbiasedInBody.magnetic_field.z * sin(pitch) * cos(roll);

  const auto magEast =
    magUnbiasedInBody.magnetic_field.y * cos(roll) -
    magUnbiasedInBody.magnetic_field.z * sin(roll);
#endif

  // This formula gives north-referenced clockwise-increasing azimuth
  const auto magAzimuthNow = atan2(magEast, magNorth);
  tf2::Quaternion magAzimuthNowQuat;
  magAzimuthNowQuat.setRPY(0, 0, magAzimuthNow);

  if (!this->data->lastAzimuth.has_value())
    this->data->lastAzimuth = magAzimuthNowQuat;
  else  // low-pass filter
    this->data->lastAzimuth = this->data->lastAzimuth->slerp(magAzimuthNowQuat, 1 - this->data->lowPassRatio);
  this->updateVariance();

  compass_interfaces::msg::Azimuth nedAzimuthMsg;
  nedAzimuthMsg.header.stamp = magUnbiased.header.stamp;
  nedAzimuthMsg.header.frame_id = this->data->frame;
  nedAzimuthMsg.azimuth = angles::normalize_angle_positive(compass_utils::getYaw(*this->data->lastAzimuth));
  nedAzimuthMsg.variance = this->data->variance;
  nedAzimuthMsg.unit = Az::UNIT_RAD;
  nedAzimuthMsg.orientation = Az::ORIENTATION_NED;
  nedAzimuthMsg.reference = Az::REFERENCE_MAGNETIC;

  return nedAzimuthMsg;
}

void MagnetometerCompass::reset()
{
  this->data->variance = this->data->initialVariance;
  this->data->lastAzimuth.reset();
}

void MagnetometerCompass::updateVariance()
{
  // TODO: measure consistency of IMU rotation and azimuth increase similar to
  //  https://www.sciencedirect.com/science/article/pii/S2405959519302929

  // OR sample (e.g. 8 in cube) magnetometer vectors (with imu/mag covariance) compute azimuth angles and compute their std...
}

}
