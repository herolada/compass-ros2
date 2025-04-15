#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief String utilities for compass_interfaces.
 * \author Martin Pecka
 */

#include <string>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include <optional>
#include "tl/expected.hpp"

namespace compass_utils
{
    tl::expected<rclcpp::Time, std::string> fromStructTm(const tm &time);
    int getYear(const rclcpp::Time &time);
    int getYear(const std::chrono::system_clock::time_point &time);
    int getYear(const std::chrono::steady_clock::time_point &time);
    rclcpp::Duration parseTimeZoneOffset(const std::string &s);
    rclcpp::Time parseTime(const std::string &s, const std::optional<rclcpp::Duration> &timezoneOffset = {}, const rclcpp::Time &referenceDate = rclcpp::Time());
}
