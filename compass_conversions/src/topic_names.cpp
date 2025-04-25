// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Logic for naming topics according to the type of Azimuth message they carry.
 * \author Martin Pecka
 */

#include <string>
#include <tuple>
#include <map>

// #include <boost/shared_ptr.hpp>

#include <compass_conversions/topic_names.h>
#include <compass_interfaces/msg/azimuth.hpp>
#include <optional>
#include <compass_utils/string_utils.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

using Az = compass_interfaces::msg::Azimuth;

namespace compass_conversions
{

namespace
{

std::string getAzimuthTopicSuffix(const decltype(Az::orientation) orientation, const decltype(Az::reference) reference)
{
  const std::string refStr =
    reference == Az::REFERENCE_MAGNETIC ? "mag" : (reference == Az::REFERENCE_GEOGRAPHIC ? "true" : "utm");
  const std::string orStr = orientation == Az::ORIENTATION_ENU ? "enu" : "ned";
  return refStr + "/" + orStr;
}

}

template<> std::string getAzimuthTopicSuffix<Az>(
  const decltype(Az::unit) unit,
  const decltype(Az::orientation) orientation,
  const decltype(Az::reference) reference)
{
  const auto unitStr = unit == Az::UNIT_RAD ? "rad" : "deg";
  return getAzimuthTopicSuffix(orientation, reference) + "/" + unitStr;
}

template<> std::string getAzimuthTopicSuffix<geometry_msgs::msg::QuaternionStamped>(
  const decltype(Az::unit) unit,
  const decltype(Az::orientation) orientation,
  const decltype(Az::reference) reference)
{
  return getAzimuthTopicSuffix(orientation, reference) + "/quat";
}

template<> std::string getAzimuthTopicSuffix<geometry_msgs::msg::PoseWithCovarianceStamped>(
  const decltype(Az::unit) unit,
  const decltype(Az::orientation) orientation,
  const decltype(Az::reference) reference)
{
  return getAzimuthTopicSuffix(orientation, reference) + "/pose";
}

template<> std::string getAzimuthTopicSuffix<sensor_msgs::msg::Imu>(
  const decltype(Az::unit) unit,
  const decltype(Az::orientation) orientation,
  const decltype(Az::reference) reference)
{
  return getAzimuthTopicSuffix(orientation, reference) + "/imu";
}

std::optional<std::tuple<decltype(Az::unit), decltype(Az::orientation), decltype(Az::reference)>>
parseAzimuthTopicName(const std::string& topic)
{
  const auto parts = compass_utils::split(topic, "/");

  for (const auto& part : parts) {
    printf("part %s", part.c_str());
  }
  
  if (parts.size() < 3)
    return std::nullopt;

  auto it = parts.rbegin();
  const auto unitPart = *it;
  ++it;
  const auto orPart = *it;
  ++it;
  const auto refPart = *it;

  decltype(Az::unit) unit;
  if (unitPart == "deg")
    unit = Az::UNIT_DEG;
  else if (unitPart == "rad" || unitPart == "imu" || unitPart == "pose" || unitPart == "quat")
    unit = Az::UNIT_RAD;
  else
    return std::nullopt;

  decltype(Az::orientation) orientation;
  if (orPart == "ned")
    orientation = Az::ORIENTATION_NED;
  else if (orPart == "enu")
    orientation = Az::ORIENTATION_ENU;
  else
    return std::nullopt;

  decltype(Az::reference) reference;
  if (refPart == "mag")
    reference = Az::REFERENCE_MAGNETIC;
  else if (refPart == "true")
    reference = Az::REFERENCE_GEOGRAPHIC;
  else if (refPart == "utm")
    reference = Az::REFERENCE_UTM;
  else
    return std::nullopt;

  return std::make_tuple(unit, orientation, reference);
}

std::optional<std::tuple<decltype(Az::unit), decltype(Az::orientation), decltype(Az::reference)>>
parseAzimuthTopicName(const std::shared_ptr<std::map<std::string, std::string>>& connectionHeaderPtr)
{
  if (connectionHeaderPtr != nullptr && connectionHeaderPtr->find("topic") != connectionHeaderPtr->end())
    return parseAzimuthTopicName(connectionHeaderPtr->at("topic"));
  return std::nullopt;
}

}