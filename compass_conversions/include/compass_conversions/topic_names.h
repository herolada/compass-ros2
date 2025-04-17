#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Logic for naming topics according to the type of Azimuth message they carry.
 * \author Martin Pecka
 */

#include <optional>
#include <string>
#include <type_traits>
#include <utility>

// #include <boost/shared_ptr.hpp>

#include <compass_interfaces/msg/azimuth.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace compass_conversions
{

/**
 * \brief Get the suffix of topic name that identifies the particular representation of azimuth.
 *
 * \tparam T Type of the message carrying the azimuth information.
 * \param[in] unit Angular units (only make sense for Azimuth messages).
 * \param[in] orientation ENU or NED orientation of the world.
 * \param[in] reference What North reference does the azimuth use.
 * \return The suffix.
 */
template<typename T, typename ::std::enable_if_t<
    std::is_same<T, compass_interfaces::msg::Azimuth>::value ||
    std::is_same<T, geometry_msgs::msg::PoseWithCovarianceStamped>::value ||
    std::is_same<T, geometry_msgs::msg::QuaternionStamped>::value ||
    std::is_same<T, sensor_msgs::msg::Imu>::value
  >* = nullptr>
std::string getAzimuthTopicSuffix(
  decltype(compass_interfaces::msg::Azimuth::unit) unit,
  decltype(compass_interfaces::msg::Azimuth::orientation) orientation,
  decltype(compass_interfaces::msg::Azimuth::reference) reference);

/**
 * \brief Autodetect azimuth representation from the name of the topic on which the message came.
 *
 * \param[in] topic The topic to parse.
 * \return The autodetected parameters, or nullopt if autodetection failed.
 */
std::optional<std::tuple<
  decltype(compass_interfaces::msg::Azimuth::unit),
  decltype(compass_interfaces::msg::Azimuth::orientation),
  decltype(compass_interfaces::msg::Azimuth::reference)
>> parseAzimuthTopicName(const std::string& topic);

/**
 * \brief Autodetect azimuth representation from connection header of a topic it came on.
 *
 * \param[in] connectionHeaderPtr Pointer to the connection header, should contain key "topic".
 * \return The autodetected parameters, or nullopt if autodetection failed.
 */
std::optional<std::tuple<
  decltype(compass_interfaces::msg::Azimuth::unit),
  decltype(compass_interfaces::msg::Azimuth::orientation),
  decltype(compass_interfaces::msg::Azimuth::reference)
>> parseAzimuthTopicName(const std::shared_ptr<std::map<std::string, std::string>>& connectionHeaderPtr);

}
