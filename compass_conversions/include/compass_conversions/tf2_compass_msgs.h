#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Support for transforming compass_interfaces::msg::Azimuth messages.
 * \author Martin Pecka
 */

#include <string>

#include <compass_interfaces/msg/azimuth.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/time.hpp>
#include <tf2/convert.h>

namespace tf2
{
template<> const rclcpp::Time& getTimestamp(const compass_interfaces::msg::Azimuth& t);
template<> const std::string& getFrameId(const compass_interfaces::msg::Azimuth& t);

compass_interfaces::msg::Azimuth toMsg(const compass_interfaces::msg::Azimuth& in);
void fromMsg(const compass_interfaces::msg::Azimuth& msg, compass_interfaces::msg::Azimuth& out);

template<>
void doTransform(
  const compass_interfaces::msg::Azimuth& t_in, compass_interfaces::msg::Azimuth& t_out, const geometry_msgs::msg::TransformStamped& transform);
}
