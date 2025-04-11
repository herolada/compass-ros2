#include <ctime>
#include <cmath>
#include <limits>
#include <regex>
#include <string>
#include <optional>
#include <charconv>
#include <format>
#include "tl/expected.hpp"
#include <chrono>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <compass_utils/string_utils.hpp>
#include <compass_utils/time_utils.hpp>

namespace compass_utils
{

  tl::expected<rclcpp::Time, std::string> fromStructTm(const tm &time)
  {
    tm t = time;
#if _DEFAULT_SOURCE
    errno = 0;
    const auto timeSecs = timegm(&t);
#else
    const auto tz = getenv("TZ");
    setenv("TZ", "", 1);
    tzset();
    const auto timeSecs = mktime(&t);
    if (tz)
      setenv("TZ", tz, 1);
    else
      unsetenv("TZ");
    tzset();
#endif
    if (timeSecs == static_cast<time_t>(-1) || errno == EOVERFLOW)
      return compass_utils::make_unexpected(std::format(
          "Cannot convert the given tm struct to ROS time (timegm failed, errno={}).", std::to_string(errno)));
    if (timeSecs < 0)
      return compass_utils::make_unexpected("Cannot convert the given tm struct to ROS time (negative seconds since 1970).");

    try
    {
      return rclcpp::Time(timeSecs, 0);
    }
    catch (const std::runtime_error &e)
    {
      return compass_utils::make_unexpected(std::format("Cannot convert the given tm struct to ROS time ({}).", e.what()));
    }
  }

  int getYear(const rclcpp::Time &time)
  {
    const auto timet = static_cast<time_t>(time.seconds());

    tm structTm{};
    const auto result = gmtime_r(&timet, &structTm);

    return structTm.tm_year + 1900;
  }

  rclcpp::Duration parseTimeoneOffset(const std::string &s)
  {
    if (s.empty() || s == "Z")
      return {0, 0};

    const std::regex zoneOffsetRegex{R"(([+-]?)(\d{1,2}):?(\d{2}))"};
    std::smatch matches;
    if (!std::regex_match(s, matches, zoneOffsetRegex))
      throw std::invalid_argument("Invalid timezone offset string.");

    uint8_t hours;
    uint8_t minutes;
    std::from_chars(matches[2].str().data(), matches[2].str().data() + matches[2].str().size(), hours);
    std::from_chars(matches[3].str().data(), matches[3].str().data() + matches[3].str().size(), minutes);

    const auto sign = (matches[1].matched && matches[1].str() == "-") ? -1 : 1;

    return {sign * (hours * 3600 + minutes * 60), 0};
  }

  rclcpp::Time parseTime(
      const std::string &s, const std::optional<rclcpp::Duration> &timezoneOffset, const rclcpp::Time &referenceDate)
  {
    // TODO find a ROS 2 alternative
    // if (s.length() == 3 && toLower(s) == "now")
    //  return this->clock.now();

    // Check if the string contains delimiters. If so, do not require zero-padding of all numbers.
    const std::regex delimitersRegex{
        R"((?:(?:(?:(\d+)[:_/-])?(\d+)[:_/-])?(\d+)[Tt _-])?(\d+)[:_/-](\d+)[:_/-](\d+)(?:[.,](\d+))?(Z|[+-]?\d{1,2}:?\d{2})?)"}; // NOLINT
    std::smatch matches;
    if (!std::regex_match(s, matches, delimitersRegex))
    {
      const std::regex noDelimsRegex{
          R"((?:((?:\d{2}){1,2})[:_/-]?([01]\d)[:_/-]?([0123]\d)[Tt _-])?([012]\d)[:_/-]?([0-6]\d)[:_/-]?([0-6]\d)(?:[.,](\d+))?(Z|[+-]?\d{1,2}:?\d{2})?)"}; // NOLINT
      if (!std::regex_match(s, matches, noDelimsRegex))
        throw std::invalid_argument("Invalid time format");
    }

    std::chrono::seconds referenceSeconds((int)referenceDate.seconds());

    std::chrono::sys_seconds tp = std::chrono::sys_seconds{referenceSeconds};

    std::chrono::year_month_day ymd = floor<std::chrono::days>(tp);

    uint16_t referenceYear = static_cast<uint16_t>(static_cast<int>(ymd.year()));
    uint16_t referenceMonth = static_cast<uint16_t>(static_cast<unsigned>(ymd.month()));
    uint16_t referenceDay = static_cast<uint16_t>(static_cast<unsigned>(ymd.day()));

    std::string yearStr;
    if (matches[1].matched)
    {
      const auto &str = matches[1].str();
      if (str.length() == 2)
        yearStr = std::string("20") + str;
      else
        yearStr = str;
    }
    uint16_t yearInt;
    std::from_chars(yearStr.data(), yearStr.data() + yearStr.size(), yearInt);
    const uint16_t year = yearStr.empty() ? referenceYear : yearInt;
    if (year < 1970)
      throw std::invalid_argument("Years before 1970 cannot be parsed to ros time.");

    uint16_t month;
    if (matches[2].matched)
    {
      std::from_chars(matches[2].str().data(), matches[2].str().data() + matches[2].str().size(), month);
    }
    else
    {
      month = referenceMonth;
    }

    uint16_t day;
    if (matches[3].matched)
    {
      std::from_chars(matches[3].str().data(), matches[3].str().data() + matches[3].str().size(), day);
    }
    else
    {
      day = referenceDay;
    }

    if (month <= 0)
      throw std::invalid_argument("Month has to be a positive number (i.e. non-zero).");

    if (day <= 0)
      throw std::invalid_argument("Day has to be a positive number (i.e. non-zero).");

    uint16_t hour;
    uint16_t minute;
    uint16_t second;
    std::from_chars(matches[4].str().data(), matches[4].str().data() + matches[4].str().size(), hour);
    std::from_chars(matches[5].str().data(), matches[5].str().data() + matches[5].str().size(), minute);
    std::from_chars(matches[6].str().data(), matches[6].str().data() + matches[6].str().size(), second);

    /* rclcpp::Duration zoneOffset{};
    if (matches[8].matched) {
      parseTimeoneOffset(matches[8].str());
    } else {
      timezoneOffset;
    } */

    rclcpp::Duration dur(0, 0);
    const auto zoneOffset = matches[8].matched ? parseTimeoneOffset(matches[8].str()) : timezoneOffset.value_or(dur);

    tm t{};
    t.tm_year = year - 1900;
    t.tm_mon = month - 1;
    t.tm_mday = day;
    t.tm_hour = hour;
    t.tm_min = minute;
    t.tm_sec = second;

    const auto maybeTime = fromStructTm(t);
    if (!maybeTime.has_value())
      throw std::invalid_argument(std::format("Invalid time format ({}).", maybeTime.error().c_str()));

    uint32_t fracNsec = 0;
    if (matches[7].matched)
    {
      auto paddedNsec = matches[7].str();
      if (paddedNsec.length() < 9)
        paddedNsec = std::format("{}{}", paddedNsec.c_str(), std::to_string(static_cast<int>(9 - paddedNsec.length())));
      else if (paddedNsec.length() > 9)
        paddedNsec = paddedNsec.substr(0, 9); // We could correctly round here, but who cares about one ns?

      std::from_chars(paddedNsec.data(), paddedNsec.data() + paddedNsec.size(), fracNsec);
    }

    return {(int)(std::round(maybeTime->seconds())) - (int)(std::round(zoneOffset.seconds())), fracNsec};
  }

}