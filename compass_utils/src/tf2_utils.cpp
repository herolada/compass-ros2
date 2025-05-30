/**
 * \file
 * \brief Utilities for working with transforms for compass packages.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <compass_utils/tf2_utils.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace compass_utils
{

double getRoll(const tf2::Quaternion& quat)
{
  double roll, pitch, yaw;
  getRPY(quat, roll, pitch, yaw);
  return roll;
}

double getRoll(const geometry_msgs::msg::Quaternion& quat)
{
  double roll, pitch, yaw;
  getRPY(quat, roll, pitch, yaw);
  return roll;
}

double getPitch(const tf2::Quaternion& quat)
{
  double roll, pitch, yaw;
  getRPY(quat, roll, pitch, yaw);
  return pitch;
}

double getPitch(const geometry_msgs::msg::Quaternion& quat)
{
  double roll, pitch, yaw;
  getRPY(quat, roll, pitch, yaw);
  return pitch;
}

double getYaw(const tf2::Quaternion& quat)
{
  double roll, pitch, yaw;
  getRPY(quat, roll, pitch, yaw);
  return yaw;
}

double getYaw(const geometry_msgs::msg::Quaternion& quat)
{
  double roll, pitch, yaw;
  getRPY(quat, roll, pitch, yaw);
  return yaw;
}

void getRPY(const tf2::Quaternion& quat, double& roll, double& pitch, double& yaw)
{
  tf2::Matrix3x3 tmpMat(quat);
  tmpMat.getRPY(roll, pitch, yaw);
}

void getRPY(const geometry_msgs::msg::Quaternion& quat, double& roll, double& pitch, double& yaw)
{
  tf2::Quaternion tmpQuat;
  tf2::fromMsg(quat, tmpQuat);
  getRPY(tmpQuat, roll, pitch, yaw);
}

}