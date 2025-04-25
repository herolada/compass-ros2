#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief String utilities for compass_interfaces.
 * \author Martin Pecka
 */

#include <string>
#include "tl/expected.hpp"
#include <rclcpp/time.hpp>

// #include <compass_interfaces/Azimuth.h>
#include <compass_interfaces/msg/azimuth.hpp>
// tartanllama expected
namespace compass_utils
{

  // taken from cpp_common_utils
  template <class E>
  tl::unexpected<typename std::decay<E>::type> make_unexpected(E &&e)
  {
    return tl::unexpected<typename std::decay<E>::type>(std::forward<E>(e));
  }

  std::string to_pretty_string(const rclcpp::Time &value);
  std::string to_pretty_string(const std::chrono::system_clock::time_point &tp);
  // std::string to_pretty_string(const std::chrono::steady_clock::time_point &tp);
  std::string toLower(const std::string &str);

  /**
   * \brief Convert the given azimuth unit to string.
   * \param[in] unit The unit.
   * \return The string.
   * \throw std::runtime_error If the unit is invalid.
   */
  std::string unitToString(decltype(compass_interfaces::msg::Azimuth::unit) unit);

  /**
   * \brief Convert the given string to azimuth unit.
   * \param[in] unitStr A string describing the unit.
   * \return The unit.
   * \throw std::runtime_error If the unit is invalid.
   */
  decltype(compass_interfaces::msg::Azimuth::unit) parseUnit(const std::string &unitStr);

  /**
   * \brief Convert the given azimuth orientation to string.
   * \param[in] orientation The orientation.
   * \return The string.
   * \throw std::runtime_error If the orientation is invalid.
   */
  std::string orientationToString(decltype(compass_interfaces::msg::Azimuth::orientation) orientation);

  /**
   * \brief Convert the given string to azimuth orientation.
   * \param[in] orientationStr A string describing the orientation.
   * \return The orientation.
   * \throw std::runtime_error If the orientation is invalid.
   */
  decltype(compass_interfaces::msg::Azimuth::orientation) parseOrientation(const std::string &orientationStr);

  /**
   * \brief Convert the given azimuth reference to string.
   * \param[in] reference The reference.
   * \return The string.
   * \throw std::runtime_error If the reference is invalid.
   */
  std::string referenceToString(decltype(compass_interfaces::msg::Azimuth::reference) reference);

  /**
   * \brief Convert the given string to azimuth reference.
   * \param[in] referenceStr A string describing the reference.
   * \return The reference.
   * \throw std::runtime_error If the reference is invalid.
   */
  decltype(compass_interfaces::msg::Azimuth::reference) parseReference(const std::string &referenceStr);

  /**
   * \brief If `str` is nonempty, returns str + suffix, otherwise empty string.
   * \param[in] str The main string.
   * \param[in] suffix The string's suffix.
   * \return The possibly suffixed string.
   */
  ::std::string appendIfNonEmpty(const ::std::string& str, const ::std::string& suffix);


  /**
   * \brief Split the given string by the given delimiter.
   * \param[in] str The string to split.
   * \param[in] delimiter The delimiter used for splitting.
   * \param[in] maxSplits If >= 0, defines the maximum number of splits.
   * \return A vector of parts of the original string.
   */
  ::std::vector<::std::string> split(const ::std::string& str, const ::std::string& delimiter, int maxSplits = -1);


  /**
   * \brief Return a string that is a concatenation of elements of `strings` delimited by `delimiter`.
   * \tparam T An iterable type (must support `size()` and foreach).
   * \param[in] strings The elements to put into a string.
   * \param[in] delimiter Delimiter put between elements.
   * \return The concatenated string.
   */
  template<typename T>
  ::std::string join(const T& strings, const ::std::string& delimiter)
  {
    const auto numStrings = strings.size();
    if (numStrings == 0)
      return "";

    ::std::stringstream ss;
    size_t i = 0;
    for (const auto& s : strings)
    {
      ss << s;
      if (i < numStrings - 1)
        ss << delimiter;
      i++;
    }
    return ss.str();
  }

}
