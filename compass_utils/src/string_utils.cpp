// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief String utilities for compass_interfaces.
 * \author Martin Pecka
 */

#include <string>
#include <format>
#include <chrono>
#include <cmath>
// #include <compass_interfaces/Azimuth.h>
#include <compass_interfaces/msg/azimuth.hpp>

#include <compass_utils/string_utils.hpp>

namespace compass_utils
{

  std::string to_pretty_string(const rclcpp::Time &value)
  {
    int32_t seconds = std::floor(value.seconds());
    // Convert seconds to system_clock::time_point
    std::chrono::system_clock::time_point tp = std::chrono::system_clock::time_point(seconds * std::chrono::seconds(1));

    // Convert to time_t for formatting
    std::time_t time = std::chrono::system_clock::to_time_t(tp);

    // Format the time into a string
    std::ostringstream oss;

    oss << std::put_time(std::gmtime(&time), "%Y-%m-%dT%H:%M:%S");

    // Append microseconds and 'Z'
    long int ns = (long int)(value.nanoseconds() - 1e9*seconds);

    if (ns != 0) {
      auto us = duration_cast<std::chrono::microseconds>(std::chrono::nanoseconds(ns));
      oss << '.' << std::setw(6) << std::setfill('0') << us.count() << 'Z';  
    } else {
      oss << 'Z';
    }

    return oss.str();
  }
  
  std::string to_pretty_string(const std::chrono::system_clock::time_point &tp)
  {

    // Convert to time_t for formatting
    std::time_t time = std::chrono::system_clock::to_time_t(tp);

    // Format the time into a string
    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&time), "%Y-%m-%dT%H:%M:%S");

    auto duration = tp.time_since_epoch();
    auto micros = std::chrono::duration_cast<std::chrono::microseconds>(duration) % 1'000'000;

    if (micros.count() != 0) {
      oss << "." << std::setw(6) << std::setfill('0') << micros.count() << 'Z';
    } else {
      oss << 'Z';
    }

    return oss.str();
  }

  // std::string to_pretty_string(const std::chrono::steady_clock::time_point &tp)
  // {

  //   // Convert to time_t for formatting
  //   std::time_t time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now() +
  //   std::chrono::duration_cast<std::chrono::system_clock::duration>(tp - std::chrono::steady_clock::now()));

  //   printf("Current time as integer: %lld\n", static_cast<long long>(time));
  //   printf("Current time as integer: %lld\n", static_cast<long long>(std::chrono::duration_cast<std::chrono::system_clock::duration>(tp - std::chrono::steady_clock::now()).count()));

  //   // Format the time into a string
  //   std::ostringstream oss;
  //   oss << std::put_time(std::gmtime(&time), "%Y-%m-%dT%H:%M:%S");

  //   auto duration = tp.time_since_epoch();
  //   auto micros = std::chrono::duration_cast<std::chrono::microseconds>(duration) % 1'000'000;

  //   if (micros.count() != 0) {
  //     oss << "." << std::setw(6) << std::setfill('0') << micros.count() << 'Z';
  //   } else {
  //     oss << 'Z';
  //   }

  //   return oss.str();
  // }

  std::string toLower(const std::string &str)
  {
    // TODO rewrite using libicu-dev
    auto result = str;
    std::transform(str.begin(), str.end(), result.begin(), [](unsigned char c)
                   { return std::tolower(c); });
    return result;
  }

  std::string unitToString(const decltype(compass_interfaces::msg::Azimuth::unit) unit)
  {
    switch (unit)
    {
    case compass_interfaces::msg::Azimuth::UNIT_RAD:
      return "rad";
    case compass_interfaces::msg::Azimuth::UNIT_DEG:
      return "deg";
    default:
      throw std::runtime_error(std::format("Invalid compass_interfaces::msg::Azimuth::unit {}", std::to_string(unit)));
    }
  }

  decltype(compass_interfaces::msg::Azimuth::unit) parseUnit(const std::string &unitStr)
  {
    const auto unit = toLower(unitStr);
    if (unit == "rad")
      return compass_interfaces::msg::Azimuth::UNIT_RAD;
    else if (unit == "deg")
      return compass_interfaces::msg::Azimuth::UNIT_DEG;
    else
      throw std::runtime_error(std::format("Invalid compass_interfaces::msg::Azimuth::unit '{}'", unitStr.c_str()));
  }

  std::string orientationToString(const decltype(compass_interfaces::msg::Azimuth::orientation) orientation)
  {
    switch (orientation)
    {
    case compass_interfaces::msg::Azimuth::ORIENTATION_ENU:
      return "ENU";
    case compass_interfaces::msg::Azimuth::ORIENTATION_NED:
      return "NED";
    default:
      throw std::runtime_error(std::format("Invalid compass_interfaces::msg::Azimuth::orientation {}", std::to_string(orientation)));
    }
  }

  decltype(compass_interfaces::msg::Azimuth::orientation) parseOrientation(const std::string &orientationStr)
  {
    const auto orientation = toLower(orientationStr);
    if (orientation == "enu")
      return compass_interfaces::msg::Azimuth::ORIENTATION_ENU;
    else if (orientation == "ned")
      return compass_interfaces::msg::Azimuth::ORIENTATION_NED;
    else
      throw std::runtime_error(std::format("Invalid compass_interfaces::msg::Azimuth::orientation '{}'", orientationStr.c_str()));
  }

  std::string referenceToString(const decltype(compass_interfaces::msg::Azimuth::reference) reference)
  {
    switch (reference)
    {
    case compass_interfaces::msg::Azimuth::REFERENCE_MAGNETIC:
      return "magnetic";
    case compass_interfaces::msg::Azimuth::REFERENCE_GEOGRAPHIC:
      return "geographic";
    case compass_interfaces::msg::Azimuth::REFERENCE_UTM:
      return "UTM";
    default:
      throw std::runtime_error(std::format("Invalid compass_interfaces::msg::Azimuth::reference {}", std::to_string(reference)));
    }
  }

  decltype(compass_interfaces::msg::Azimuth::reference) parseReference(const std::string &referenceStr)
  {
    const auto reference = toLower(referenceStr);
    if (reference == "magnetic")
      return compass_interfaces::msg::Azimuth::REFERENCE_MAGNETIC;
    else if (reference == "geographic" || reference == "true")
      return compass_interfaces::msg::Azimuth::REFERENCE_GEOGRAPHIC;
    else if (reference == "utm")
      return compass_interfaces::msg::Azimuth::REFERENCE_UTM;
    else
      throw std::runtime_error(std::format("Invalid compass_interfaces::msg::Azimuth::reference '{}'", referenceStr.c_str()));
  }

  std::string appendIfNonEmpty(const std::string& str, const std::string& suffix)
  {
    return str.empty() ? str : str + suffix;
  }

  std::vector<std::string> split(const std::string& str, const std::string& delimiter, int maxSplits)
  {
    // inspired by https://stackoverflow.com/a/46931770/1076564, CC-BY-SA 4.0
    // renamed some variables, added the maxSplits option
    size_t start{0};
    size_t end;
    size_t delimiterLength{delimiter.length()};
    std::string token;
    std::vector<std::string> result;

    while ((end = str.find(delimiter, start)) != std::string::npos && (maxSplits == -1 || result.size() < maxSplits))
    {
      token = str.substr(start, end - start);
      start = end + delimiterLength;
      result.push_back(token);
    }

    result.push_back(str.substr(start));
    return result;
  }

}
