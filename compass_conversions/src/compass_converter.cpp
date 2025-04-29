// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Convert between various compass representations.
 * \author Martin Pecka
 */

#include "tl/expected.hpp"
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <angles/angles.h>
#include <compass_conversions/compass_converter.h>
#include <compass_conversions/topic_names.h>
#include <compass_interfaces/msg/azimuth.hpp>
#include <compass_utils/string_utils.hpp>
#include <compass_utils/time_utils.hpp>
#include <compass_utils/tf2_utils.hpp>
// #include <cras_cpp_common/log_utils.h>
// #include <cras_cpp_common/string_utils.hpp>
// #include <cras_cpp_common/tf2_utils.hpp>
// #include <cras_cpp_common/time_utils.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <magnetic_model/magnetic_model.h>
#include <magnetic_model/magnetic_model_manager.h>
#include <message_filters/message_event.h>
#include <message_filters/message_traits.h>
// #include <ros/package.h>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>


namespace compass_conversions
{

using Az = compass_interfaces::msg::Azimuth;
using Imu = sensor_msgs::msg::Imu;
using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
using Quat = geometry_msgs::msg::QuaternionStamped;

/**
 * \brief Private data of CompassConverter.
 */
struct CompassConverterPrivate
{
  //! \brief Magnetic model manager.
  std::unique_ptr<magnetic_model::MagneticModelManager> magneticModelManager;

  //! \brief Cache of already initialized magnetic field models. Keys are years.
  std::map<uint32_t, std::shared_ptr<magnetic_model::MagneticModel>> magneticModels;
};

CompassConverter::CompassConverter(const rclcpp::Node* node, bool strict) :
  node(node), strict(strict), data(new CompassConverterPrivate{})
{
  this->data->magneticModelManager = std::make_unique<magnetic_model::MagneticModelManager>(node);
};

CompassConverter::~CompassConverter() = default;

void CompassConverter::configFromParams()
{
  // cras::TempLocale l(LC_ALL, "en_US.UTF-8");  // Support printing ° signs

  if (this->node->has_parameter("magnetic_declination"))
    this->forceMagneticDeclination(std::make_optional<double>(this->node->get_parameter("magnetic_declination").get_value<double>()));
  else
    this->forcedMagneticModelName = this->node->get_parameter_or<std::string>("magnetic_model", std::string());

  if (this->node->has_parameter("magnetic_models_path"))
    this->setMagneticModelPath(std::make_optional<std::string>(this->node->get_parameter("magnetic_models_path").get_value<std::string>()));

  if (this->node->has_parameter("utm_grid_convergence"))
    this->forceUTMGridConvergence(std::make_optional<double>(this->node->get_parameter("utm_grid_convergence").get_value<double>()));

  if (this->node->has_parameter("utm_zone"))
    this->forceUTMZone(std::make_optional<int>(this->node->get_parameter("utm_zone").get_value<int>()));

  this->setKeepUTMZone(this->node->get_parameter_or<bool>("keep_utm_zone", this->keepUTMZone));

  if (!this->forcedMagneticDeclination.has_value() || !this->forcedUTMGridConvergence.has_value())
  {
    if (this->node->has_parameter("initial_lat") && this->node->has_parameter("initial_lon"))
    {
      sensor_msgs::msg::NavSatFix msg;

      msg.latitude = this->node->get_parameter_or<double>("initial_lat", 0.0); 
      msg.longitude = this->node->get_parameter_or<double>("initial_lon", 0.0); 
      msg.altitude = this->node->get_parameter_or<double>("initial_alt", 0.0);

      std::list<std::string> computedValues;
      if (!this->forcedMagneticDeclination.has_value())
        computedValues.emplace_back("magnetic declination");
      if (!this->forcedUTMGridConvergence.has_value())
        computedValues.emplace_back("UTM grid convergence");

      RCLCPP_INFO(
        this->node->get_logger(), "Initial GPS coords for computation of %s are %.6f°, %.6f°, altitude %.0f m.",
        compass_utils::join(computedValues, "and").c_str(), msg.latitude, msg.longitude, msg.altitude);

      this->setNavSatPos(msg);
    }
  }
};

void CompassConverter::forceMagneticDeclination(const std::optional<double>& declination)
{
  this->forcedMagneticDeclination = declination;
};

void CompassConverter::forceUTMGridConvergence(const std::optional<double>& convergence)
{
  this->forcedUTMGridConvergence = this->lastUTMGridConvergence = convergence;
};

void CompassConverter::setMagneticModelPath(const std::optional<std::string>& modelPath)
{
  this->data->magneticModelManager->setModelPath(modelPath);
};

void CompassConverter::forceMagneticModelName(const std::string& model)
{
  this->forcedMagneticModelName = model;
};

void CompassConverter::setKeepUTMZone(const bool keep)
{
  this->keepUTMZone = keep;
};

tl::expected<double, std::string> CompassConverter::getMagneticDeclination(const rclcpp::Time& stamp) const
{
  if (this->forcedMagneticDeclination.has_value())
    return *this->forcedMagneticDeclination;

  if (!this->lastFix.has_value())
    return compass_utils::make_unexpected("Cannot determine magnetic declination without GNSS pose.");

  return this->computeMagneticDeclination(*this->lastFix, stamp);
};

tl::expected<double, std::string> CompassConverter::computeMagneticDeclination(
  const sensor_msgs::msg::NavSatFix& fix, const rclcpp::Time& stamp) const
{
  const auto year = compass_utils::getYear(stamp);
  if (this->data->magneticModels[year] == nullptr)
  {
    const auto modelName = !this->forcedMagneticModelName.empty() ?
      this->forcedMagneticModelName : this->data->magneticModelManager->getBestMagneticModelName(stamp);

    const auto model = this->data->magneticModelManager->getMagneticModel(modelName, this->strict);
    if (!model.has_value())
      return compass_utils::make_unexpected(std::format(
        "Could not create magnetic field model {} for year {} because of the following error: {}",
        modelName.c_str(), std::to_string(year), model.error().c_str()));
    this->data->magneticModels[year] = *model;
  }

  const auto& magModel = *this->data->magneticModels[year];

  const auto fieldComponents = magModel.getMagneticFieldComponents(fix, stamp);
  if (!fieldComponents.has_value())
    return compass_utils::make_unexpected(fieldComponents.error());

  return fieldComponents->values.declination;
};

tl::expected<double, std::string> CompassConverter::getUTMGridConvergence() const
{
  if (this->forcedUTMGridConvergence.has_value())
    return *this->forcedUTMGridConvergence;

  if (!this->lastUTMGridConvergence.has_value())
    return compass_utils::make_unexpected("UTM grid convergence has not yet been determined from GNSS pose.");

  return *this->lastUTMGridConvergence;
};

tl::expected<int, std::string> CompassConverter::getUTMZone() const
{
  if (this->forcedUTMZone.has_value())
    return *this->forcedUTMZone;

  if (!this->lastUTMZone.has_value())
    return compass_utils::make_unexpected("UTM zone has not yet been determined from GNSS pose.");

  return *this->lastUTMZone;
};

void CompassConverter::forceUTMZone(const std::optional<int>& zone)
{
  if (zone.has_value() && (zone < GeographicLib::UTMUPS::MINZONE || zone > GeographicLib::UTMUPS::MAXZONE))
    RCLCPP_WARN(this->node->get_logger(), "Invalid UTM zone: %d", *zone);
  else
    this->forcedUTMZone = this->lastUTMZone = zone;
};

tl::expected<std::pair<double, int>, std::string> CompassConverter::computeUTMGridConvergenceAndZone(
  const sensor_msgs::msg::NavSatFix& fix, const std::optional<int>& utmZone) const
{
  if (utmZone.has_value() && (*utmZone < GeographicLib::UTMUPS::MINZONE || *utmZone > GeographicLib::UTMUPS::MAXZONE))
    return compass_utils::make_unexpected(std::format("Invalid UTM zone: {}", std::to_string(*utmZone)));

  try
  {
    int zone;
    bool isNorthHemisphere;
    double northing, easting, utmGridConvergence, projectionScale;
    const int setzone = utmZone.value_or(GeographicLib::UTMUPS::STANDARD);

    GeographicLib::UTMUPS::Forward(fix.latitude, fix.longitude,
      zone, isNorthHemisphere, easting, northing, utmGridConvergence, projectionScale, setzone);

    return std::make_pair(angles::from_degrees(utmGridConvergence), zone);
  }
  catch (const GeographicLib::GeographicErr& e)
  {
    return compass_utils::make_unexpected(std::format("Could not get UTM grid convergence: {}", e.what()));
  }
};

tl::expected<compass_interfaces::msg::Azimuth, std::string> CompassConverter::convertAzimuth(
  const compass_interfaces::msg::Azimuth& azimuth,
  const decltype(compass_interfaces::msg::Azimuth::unit) unit,
  const decltype(compass_interfaces::msg::Azimuth::orientation) orientation,
  const decltype(compass_interfaces::msg::Azimuth::reference) reference) const
{

  // Fast track for no conversion
  if (azimuth.unit == unit && azimuth.orientation == orientation && azimuth.reference == reference)
    return azimuth;

  using Az = compass_interfaces::msg::Azimuth;

  Az result = azimuth;
  result.unit = unit;
  result.orientation = orientation;
  result.reference = reference;


  // Convert the input to NED radians
  if (azimuth.unit == Az::UNIT_DEG)
    result.azimuth = angles::from_degrees(result.azimuth);
  if (azimuth.orientation == Az::ORIENTATION_ENU)
    result.azimuth = M_PI_2 - result.azimuth;

  // When going magnetic->true, we need to add declination in NED.
  // When going true->UTM, we need to subtract grid convergence in NED.

  // Now convert between references in NED radians
  if (azimuth.reference != result.reference)
  {
    if (azimuth.reference == Az::REFERENCE_MAGNETIC)
    {
      const auto magneticDeclination = this->getMagneticDeclination(azimuth.header.stamp);
      if (!magneticDeclination.has_value())
        return compass_utils::make_unexpected(std::format(
          "Cannot convert magnetic azimuth to true without knowing magnetic declination. Error: {}",
          magneticDeclination.error().c_str()));

      result.azimuth += *magneticDeclination;

      if (result.reference == Az::REFERENCE_UTM)
      {
        const auto convergence = this->getUTMGridConvergence();
        if (!convergence.has_value())
          return compass_utils::make_unexpected(std::format(
            "Cannot convert true azimuth to UTM without knowing UTM grid convergence. Error: {}",
            convergence.error().c_str()));

        result.azimuth -= *convergence;
      }
    }
    else if (azimuth.reference == Az::REFERENCE_GEOGRAPHIC)
    {
      if (result.reference == Az::REFERENCE_MAGNETIC)
      {
        const auto magneticDeclination = this->getMagneticDeclination(azimuth.header.stamp);
        if (!magneticDeclination.has_value())
          return compass_utils::make_unexpected(std::format(
            "Cannot convert true azimuth to magnetic without knowing magnetic declination. Error: {}",
            magneticDeclination.error().c_str()));

        result.azimuth -= *magneticDeclination;
      }
      else if (result.reference == Az::REFERENCE_UTM)
      {
        const auto convergence = this->getUTMGridConvergence();
        if (!convergence.has_value())
          return compass_utils::make_unexpected(std::format(
            "Cannot convert true azimuth to UTM without knowing UTM grid convergence. Error: {}",
            convergence.error().c_str()));

        result.azimuth -= *convergence;
      }
    }
    else if (azimuth.reference == Az::REFERENCE_UTM)
    {
      const auto convergence = this->getUTMGridConvergence();
      if (!convergence.has_value())
        return compass_utils::make_unexpected(std::format(
          "Cannot convert UTM azimuth to true without knowing UTM grid convergence. Error: {}",
          convergence.error().c_str()));

      result.azimuth += *convergence;

      if (result.reference == Az::REFERENCE_MAGNETIC)
      {
        const auto magneticDeclination = this->getMagneticDeclination(azimuth.header.stamp);
        if (!magneticDeclination.has_value())
          return compass_utils::make_unexpected(std::format(
            "Cannot convert true azimuth to magnetic without knowing magnetic declination. Error: {}",
            magneticDeclination.error().c_str()));

        result.azimuth -= *magneticDeclination;
      }
    }
  }

  // Reference is correct now; convert to the output unit and orientation
  if (result.orientation == Az::ORIENTATION_ENU)
    result.azimuth = M_PI_2 - result.azimuth;
  result.azimuth = angles::normalize_angle_positive(result.azimuth);
  if (result.unit == Az::UNIT_DEG)
    result.azimuth = angles::to_degrees(result.azimuth);

  if (azimuth.unit == Az::UNIT_RAD && result.unit == Az::UNIT_DEG)
    result.variance = std::pow(angles::to_degrees(std::sqrt(azimuth.variance)), 2);
  else if (azimuth.unit == Az::UNIT_DEG && result.unit == Az::UNIT_RAD)
    result.variance = std::pow(angles::from_degrees(std::sqrt(azimuth.variance)), 2);

  return result;
};

tl::expected<compass_interfaces::msg::Azimuth, std::string> CompassConverter::convertQuaternion(
  const geometry_msgs::msg::QuaternionStamped& quat,
  const decltype(compass_interfaces::msg::Azimuth::variance) variance,
  const decltype(compass_interfaces::msg::Azimuth::unit) unit,
  const decltype(compass_interfaces::msg::Azimuth::orientation) orientation,
  const decltype(compass_interfaces::msg::Azimuth::reference) reference) const
{
  return this->convertQuaternion(quat.quaternion, quat.header, variance, unit, orientation, reference);
};

tl::expected<compass_interfaces::msg::Azimuth, std::string> CompassConverter::convertQuaternion(
  const geometry_msgs::msg::Quaternion& quat,
  const std_msgs::msg::Header& header,
  const decltype(compass_interfaces::msg::Azimuth::variance) variance,
  const decltype(compass_interfaces::msg::Azimuth::unit) unit,
  const decltype(compass_interfaces::msg::Azimuth::orientation) orientation,
  const decltype(compass_interfaces::msg::Azimuth::reference) reference) const
{
  tf2::Quaternion q;
  tf2::fromMsg(quat, q);
  if (q.length2() < 1e-6)
    return compass_utils::make_unexpected("Invalid quaternion (all zeros).");

  compass_interfaces::msg::Azimuth result;
  result.header = header;
  result.azimuth = compass_utils::getYaw(quat);
  result.variance = variance;
  if (unit == Az::UNIT_DEG)
  {
    result.azimuth = angles::to_degrees(result.azimuth);
    result.variance = std::pow(angles::to_degrees(std::sqrt(variance)), 2);
  }
  result.orientation = orientation;
  result.unit = unit;
  result.reference = reference;
  return result;
};

tl::expected<geometry_msgs::msg::QuaternionStamped, std::string> CompassConverter::convertToQuaternion(
  const compass_interfaces::msg::Azimuth& azimuth) const
{
  tf2::Stamped<tf2::Quaternion> quat;
  quat.frame_id_ = azimuth.header.frame_id;
  quat.stamp_ = tf2_ros::fromMsg(azimuth.header.stamp);
  // quat.stamp_ = tf2::TimePoint(std::chrono::nanoseconds((long long int)(azimuth.header.stamp.sec * 1e9)));
  quat.setRPY(0, 0, azimuth.azimuth * (azimuth.unit == Az::UNIT_RAD ? 1 : M_PI / 180.0));
  return tf2::toMsg(quat);
};

tl::expected<geometry_msgs::msg::PoseWithCovarianceStamped, std::string> CompassConverter::convertToPose(
  const compass_interfaces::msg::Azimuth& azimuth) const
{
  const auto maybeQuat = this->convertToQuaternion(azimuth);
  if (!maybeQuat.has_value())
    return compass_utils::make_unexpected(std::format("Could not convert azimuth to pose: {}", maybeQuat.error().c_str()));

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header = azimuth.header;
  pose.pose.pose.orientation = maybeQuat->quaternion;
  pose.pose.covariance[0 * 6 + 0] = std::numeric_limits<double>::infinity();
  pose.pose.covariance[1 * 6 + 1] = std::numeric_limits<double>::infinity();
  pose.pose.covariance[2 * 6 + 2] = std::numeric_limits<double>::infinity();
  pose.pose.covariance[3 * 6 + 3] = 4 * M_PI * M_PI;
  pose.pose.covariance[4 * 6 + 4] = 4 * M_PI * M_PI;
  pose.pose.covariance[5 * 6 + 5] = azimuth.variance;

  return pose;
};

tl::expected<sensor_msgs::msg::Imu, std::string> CompassConverter::convertToImu(const compass_interfaces::msg::Azimuth& azimuth) const
{
  const auto maybeQuat = this->convertToQuaternion(azimuth);
  if (!maybeQuat.has_value())
    return compass_utils::make_unexpected(std::format("Could not convert azimuth to pose: {}", maybeQuat.error().c_str()));

  sensor_msgs::msg::Imu imu;
  imu.header = azimuth.header;
  imu.linear_acceleration_covariance[0] = -1;
  imu.angular_velocity_covariance[0] = -1;
  imu.orientation = maybeQuat->quaternion;
  imu.orientation_covariance[0 * 3 + 0] = 4 * M_PI * M_PI;
  imu.orientation_covariance[1 * 3 + 1] = 4 * M_PI * M_PI;
  imu.orientation_covariance[2 * 3 + 2] = azimuth.variance;

  return imu;
};

tl::expected<compass_interfaces::msg::Azimuth, std::string> CompassConverter::convertQuaternionMsgEvent(
  const char* topic,
  const message_filters::MessageEvent<geometry_msgs::msg::QuaternionStamped const>& quatEvent,
  const decltype(compass_interfaces::msg::Azimuth::variance) variance,
  const decltype(compass_interfaces::msg::Azimuth::unit) unit,
  const std::optional<decltype(compass_interfaces::msg::Azimuth::orientation)>& orientation,
  const std::optional<decltype(compass_interfaces::msg::Azimuth::reference)>& reference) const
{
  auto msgOrientation = orientation;
  auto msgReference = reference;
  if (!msgOrientation.has_value() || !msgReference.has_value())
  {
    const auto maybeAzimuthParams = compass_conversions::parseAzimuthTopicName(std::string(topic));
    if (maybeAzimuthParams.has_value())
    {
      msgOrientation = std::get<1>(*maybeAzimuthParams);
      msgReference = std::get<2>(*maybeAzimuthParams);
    }
  }

  if (!msgOrientation.has_value() || !msgReference.has_value())
    return compass_utils::make_unexpected("Orientation and reference are not specified and cannot be autodetected.");

  const auto msg = quatEvent.getConstMessage();
  return this->convertQuaternion(*msg, variance, unit, *msgOrientation, *msgReference);
};

tl::expected<compass_interfaces::msg::Azimuth, std::string> CompassConverter::convertPoseMsgEvent(
  const char* topic,
  const message_filters::MessageEvent<geometry_msgs::msg::PoseWithCovarianceStamped const>& poseEvent,
  const decltype(compass_interfaces::msg::Azimuth::unit) unit,
  const std::optional<decltype(compass_interfaces::msg::Azimuth::orientation)>& orientation,
  const std::optional<decltype(compass_interfaces::msg::Azimuth::reference)>& reference) const
{
  auto msgOrientation = orientation;
  auto msgReference = reference;
  if (!msgOrientation.has_value() || !msgReference.has_value())
  {
    const auto maybeAzimuthParams = compass_conversions::parseAzimuthTopicName(std::string(topic));
    if (maybeAzimuthParams.has_value())
    {
      msgOrientation = std::get<1>(*maybeAzimuthParams);
      msgReference = std::get<2>(*maybeAzimuthParams);
    }
  }

  if (!msgOrientation.has_value() || !msgReference.has_value())
    return compass_utils::make_unexpected("Orientation and reference are not specified and cannot be autodetected.");

  const auto msg = poseEvent.getConstMessage();
  return this->convertQuaternion(
    msg->pose.pose.orientation, msg->header, msg->pose.covariance[5 * 6 + 5], unit, *msgOrientation, *msgReference);
};

tl::expected<compass_interfaces::msg::Azimuth, std::string> CompassConverter::convertImuMsgEvent(
  const char* topic,
  const message_filters::MessageEvent<sensor_msgs::msg::Imu>& imuEvent,
  const decltype(compass_interfaces::msg::Azimuth::unit) unit,
  const std::optional<decltype(compass_interfaces::msg::Azimuth::orientation)>& orientation,
  const std::optional<decltype(compass_interfaces::msg::Azimuth::reference)>& reference) const
  {
    printf("convert event\n");
    auto msgOrientation = orientation;
    auto msgReference = reference;
    if (!msgOrientation.has_value() || !msgReference.has_value())
    {
      printf("parse\n");

      const auto maybeAzimuthParams = compass_conversions::parseAzimuthTopicName(topic);
      if (maybeAzimuthParams.has_value())
      {
        msgOrientation = std::get<1>(*maybeAzimuthParams);
        msgReference = std::get<2>(*maybeAzimuthParams);
      }
    }

    // IMUs should output orientation in ENU frame
    if (!msgOrientation.has_value())
      msgOrientation = Az::ORIENTATION_ENU;

    if (!msgReference.has_value())
      return compass_utils::make_unexpected("Reference is not specified and cannot be autodetected.");

    const auto msg = imuEvent.getConstMessage();
    return this->convertQuaternion(
      msg->orientation, msg->header, msg->orientation_covariance[2 * 3 + 2], unit, *msgOrientation, *msgReference);
  };

template<class M> using ME = message_filters::MessageEvent<M>;
template<class M> using Creator = message_filters::DefaultMessageCreator<M>;

template<typename Msg>
std::shared_ptr<Msg> deserializeMessage(const std::shared_ptr<const rclcpp::SerializedMessage> msg)
{
  std::shared_ptr<Msg> deserialized_msg;
  rclcpp::Serialization<Msg> serializer;
  serializer.deserialize_message(msg, deserialized_msg);
  return deserialized_msg;
}

/* tl::expected<Az, std::string> CompassConverter::convertUniversalMsgEvent(
  const message_filters::MessageEvent<rclcpp::SerializedMessage const>& event,
  const std::string& topic,
  const decltype(Az::variance) variance,
  const decltype(Az::unit) unit,
  const std::optional<decltype(Az::orientation)>& orientation,
  const std::optional<decltype(Az::reference)>& reference) const
{
  // TODO dynamic type ... I think it is not possible nor is it advisable
  std::string type = event.getConnectionHeaderPtr()->at("type").front();
  const auto msg = event.getConstMessage();
  // const auto header = event.getConnectionHeaderPtr();
  const auto stamp = event.getReceiptTime();
  // const auto type = msg->get_message_type_support_handle().typesupport_identifier;
  namespace mt = message_filters::message_traits;

  if (type == "compass_interfaces::msg::Azimuth")
  {
    std::shared_ptr<Az> deserialized_msg = deserializeMessage<Az>(msg);
    return this->convertAzimuth(*deserialized_msg, unit, deserialized_msg->orientation, deserialized_msg->reference);
  }
  else if (type == "geometry_msgs::msg::PoseWithCovarianceStamped")
  {
    std::shared_ptr<Pose> deserialized_msg = deserializeMessage<Pose>(msg);

    const ME<Pose const> poseEvent = ME<Pose const>(deserialized_msg, stamp, false, Creator<Pose>());
    return this->convertPoseMsgEvent(topic.c_str(), poseEvent, unit, orientation, reference);
  }
  else if (type == "geometry_msgs::msg::QuaternionStamped")
  {
    std::shared_ptr<Quat> deserialized_msg = deserializeMessage<Quat>(msg);

    const auto quatEvent = ME<Quat const>(std::make_unique<Quat>(), stamp, false, Creator<Quat>());
    return this->convertQuaternionMsgEvent(topic.c_str(), quatEvent, variance, unit, orientation, reference);
  }
  else if (type == "sensor_msgs::msg::Imu")
  {
    std::shared_ptr<Imu> deserialized_msg = deserializeMessage<Imu>(msg);

    const auto imuEvent = ME<Imu const>(std::make_unique<Imu>(), stamp, false, Creator<Imu>());
    return this->convertImuMsgEvent(topic.c_str(), imuEvent, unit, orientation, reference);
  }
  else
    return compass_utils::make_unexpected(std::format("Invalid message type: {}.", type));
}; */

void CompassConverter::setNavSatPos(const sensor_msgs::msg::NavSatFix& fix)
{
  this->lastFix = fix;

  printf("setting nav sat pos\n");

  if (!this->forcedUTMGridConvergence.has_value())
  {
    const auto maybeConvergenceAndZone = this->computeUTMGridConvergenceAndZone(fix, this->forcedUTMZone);
    if (!maybeConvergenceAndZone.has_value())
    {
      RCLCPP_WARN_THROTTLE(this->node->get_logger(), *this->node->get_clock(), 10000., "Error computing UTM grid convergence: %s", maybeConvergenceAndZone.error().c_str());
    }
    else
    {
      const auto [convergence, zone] = *maybeConvergenceAndZone;

      this->lastUTMZone = zone;
      if (this->keepUTMZone && !this->forcedUTMZone.has_value())
        this->forcedUTMZone = zone;

      this->lastUTMGridConvergence = convergence;
    }
  }
};

}
