#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Time utilities for compass packages. Adapted from cras_cpp_common.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <optional>
#include <string>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>
#include "tl/expected.hpp"

namespace compass_utils
{
tl::expected<rclcpp::Time, std::string> fromStructTm(const tm &time);

/**
 * \brief Get the year represented by the given ROS time when interpreted as UTC time.
 * \param[in] time The ROS time.
 * \return The year.
 */
int getYear(const rclcpp::Time &time);

/**
 * \brief Get the year represented by the given ROS time when interpreted as UTC time.
 * \param[in] time The chrono time.
 * \return The year.
 */
int getYear(const std::chrono::system_clock::time_point &time);

// int getYear(const std::chrono::steady_clock::time_point &time); // not implemented

/**
 * \brief Parse timezone offset from the given string.
 *
 * This function accepts this format:
 * <pre>
 * {|Z|[{+|-}HH[:]MM]}
 * </pre>
 *
 * \note This function does not support textual timezones like CET.
 *
 * \param[in] s The string to parse.
 * \return The duration corresponding to the timezone offset.
 */
rclcpp::Duration parseTimeZoneOffset(const std::string &s);

/**
 * \brief Parse the given string as time.
 *
 * This function accepts this general format:
 * <pre>
 * [[YY]YY:MM:DD ]HH:MM:SS[.m][{Z|{+|-}HH[:]MM}]
 * </pre>
 *
 * However, many modifications of the format are supported:
 * - The delimiter can be any of `:-/_` (or empty string) and it doesn't need to be consistent in the string.
 * - The space delimiting date and time can be actually any of ` Tt_-`.
 * - The date part can be omitted. In that case, the date will be taken from `referenceTime`.
 * - If year is given just as two digits, `20YY` is assumed.
 * - If time zone offset is not specified in the string, it will be taken from `timezoneOffset`. But if the string
 *   contains a TZ offset, it will be used instead the one passed as argument. If neither is specified, UTC is assumed.
 * - If nonempty delimiters are used, the fields do not need to be zero-padded and can overflow their natural limit up
 *   to UINT16_MAX.
 * - A special value `now` is recognized and always returns current time. It can be any case (`NOW`, `Now` etc.).
 * - Decimal comma `,` can be used instead of decimal dot `.` .
 *
 * \param[in] s The string to parse.
 * \param[in] timezoneOffset Optional timezone offset to use if no offset is specified in the string.
 * \param[in] referenceDate If the date part is missing in the string, the date from this argument will be used.
 * \return The parsed time as seconds from epoch.
 * \throws std::invalid_argument If the string does not represent a date with time.
 */
rclcpp::Time parseTime(const std::string &s, const std::optional<rclcpp::Duration> &timezoneOffset = {}, const rclcpp::Time &referenceDate = rclcpp::Time());
}
