// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Convert magnetometer and IMU measurements to azimuth.
 * \author Martin Pecka
 */

#include <memory>
#include <string>
#include "tl/expected.hpp"
#include <optional>
#include <Eigen/Core>
#include <rclcpp/duration.hpp>
#include <map>

#include <angles/angles.h>
#include <compass_interfaces/msg/azimuth.hpp>
//#include <cras_cpp_common/expected.hpp>
//#include <cras_cpp_common/optional.hpp>
// #include <cras_cpp_common/param_utils.hpp>
// #include <cras_cpp_common/tf2_utils.hpp>
// #include <cras_cpp_common/tf2_utils/interruptible_buffer.h>
#include <compass_utils/tf2_utils.hpp>
#include <compass_utils/string_utils.hpp>
// #include <imu_transformer/tf2_sensor_msgs.h>
#include <magnetometer_compass/magnetometer_compass.h>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>

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
  const rclcpp::Logger& log, const rclcpp::Clock::SharedPtr clk, const std::string& frame, const std::shared_ptr<tf2::BufferCore>& tf) :
  MagnetometerCompass(log, frame, std::make_shared<tf2_ros::Buffer>(clk, tf ? tf->getCacheLength() : std::chrono::duration_cast<std::chrono::nanoseconds>(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME)))
{
}

MagnetometerCompass::MagnetometerCompass(
  const rclcpp::Logger& log, const std::string& frame, const std::shared_ptr<tf2_ros::Buffer>& tf) :
  log(log), data(new MagnetometerCompassPrivate{})
{
  this->data->tf = tf;
  this->data->frame = frame;
}

MagnetometerCompass::~MagnetometerCompass() = default;

void MagnetometerCompass::configFromParams(const rclcpp::Node::SharedPtr node)//const std::map<std::string, rclcpp::Parameter>& params)
{
  // if (params.find("initial_variance") == params.end()) {
  //   this->data->variance = this->data->initialVariance = this->data->variance;
  // } else {
  //   this->data->variance = this->data->initialVariance = params["initial_variance"];
  // }

  // if (params.find("low_pass_ratio") == params.end()) {
  //   ;
  // } else {
  //   this->data->lowPassRatio = params["low_pass_ratio"];
  // }
  this->data->variance = this->data->initialVariance = node->get_parameter_or<double>("initial_variance", this->data->variance);
  this->data->lowPassRatio = node->get_parameter_or<double>("low_pass_ratio", this->data->lowPassRatio);
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
    this->data->tf->transform(imu, imuInBody, this->data->frame, tf2::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(100))));
  }
  catch (const tf2::TransformException& e)
  {
    return compass_utils::make_unexpected(std::format(
      "Could not transform IMU data to frame {} because: {}", this->data->frame.c_str(), e.what()));
  }

  Field magUnbiasedInBody;
  try
  {
    this->data->tf->transform(magUnbiased, magUnbiasedInBody, this->data->frame, tf2::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(100))));
  }
  catch (const tf2::TransformException& e)
  {
    return compass_utils::make_unexpected(std::format(
      "Could not transform magnetometer to frame {} because: {}", this->data->frame.c_str(), e.what()));
  }

  // Compensate attitude in the magnetometer measurements

  double roll, pitch, yaw;
  compass_utils::getRPY(imuInBody.orientation, roll, pitch, yaw);

#if 0
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
}

}
