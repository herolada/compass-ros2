// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Support for transforming compass_interfaces::msg::Azimuth messages.
 * \author Martin Pecka
 */

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <compass_conversions/message_filter.h>
#include <compass_conversions/tf2_compass_msgs.h>
#include <compass_conversions/topic_names.h>
#include <compass_conversions/compass_transformer.h>
#include <compass_interfaces/msg/azimuth.hpp>
#include <compass_utils/string_utils.hpp>
// #include <cras_cpp_common/functional.hpp>
// #include <cras_cpp_common/nodelet_utils.hpp>
#include <optional>
// #include <cras_cpp_common/string_utils.hpp>
// #include <cras_cpp_common/tf2_utils/message_filter.hpp>
#include "tf2_ros/message_filter.h"
#include <message_filters/subscriber.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
// #include <nodelet/nodelet.h>
// #include <pluginlib/class_list_macros.hpp>
// #include <ros/names.h>
// #include <ros/node_handle.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/message_filter.h>
#include <rclcpp/duration.hpp>
// #include <rclcpp/publisher.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/node.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using Az = compass_interfaces::msg::Azimuth;
using Fix = sensor_msgs::msg::NavSatFix;

namespace compass_conversions
{

// enum class OutputType
// {
//   Azimuth,
//   Imu,
//   Pose,
//   Quaternion,
// };

OutputType parseOutputType(const std::string& outputType)
{
  const auto output = compass_utils::toLower(outputType);
  if (output == "azimuth")
    return OutputType::Azimuth;
  else if (output == "imu")
    return OutputType::Imu;
  else if (output == "pose")
    return OutputType::Pose;
  else if (output == "quaternion" || output == "quat")
    return OutputType::Quaternion;
  else
    throw std::runtime_error("Unknown output type: " + outputType);
}

std::string outputTypeToString(const OutputType type)
{
  switch (type)
  {
    case OutputType::Azimuth:
      return "azimuth";
    case OutputType::Imu:
      return "imu";
    case OutputType::Pose:
      return "pose";
    case OutputType::Quaternion:
      return "quaternion";
    default:
      throw std::runtime_error(std::format("Unknown output type: {}", std::to_string((static_cast<int>(type)))));
  }
}

/**
 * \brief Nodelet for transforming one type and parametrization of azimuth to another type, parametrization and
 *        TF frame.
 *
 * Subscribed topics:
 * - `~azimuth_in` (compass_interfaces/msg/Azimuth or geometry_msgs/msg/QuaternionStamped or geometry_msgs/msg/PoseWithCovarianceStamped
 *     or sensor_msgs/msg/Imu): The input azimuth. The name of the topic (if you remap it) can be used to autodetect some
 *     metadata for the conversion.
 *  - `fix` (sensor_msgs/msg/NavSatFix): GNSS fix messages that can be used to determine some parameters for the conversion.
 *  - `utm_zone` (std_msgs/msg/Int32): Optional messages with forced UTM zone.
 *  - TF (only if `~target_frame` is nonempty)
 *
 *  Published topics:
 *  - `~azimuth_out` or `~azimuth_out/SUFFIX`: The transformed azimuth. If `~target_append_suffix` is true, the variant
 *                                             with topic name suffix will be used (e.g. `~azimuth_out/mag/enu/deg`).
 *                                             The type of the published message is determined by `~target_type`.
 *
 * Parameters:
 * - `~queue_size` (int, default 10): Queue size for the subscribers and publishers.
 * - `~target_unit` (str, 'deg' or 'rad', default: 'rad'): Angular unit to be used in the transformed messages.
 * - `~target_orientation` (str, 'enu' or 'ned', default: 'enu'): ENU or NED orientation to be used in the
 *                                                                transformed messages.
 * - `~target_reference` (str, 'magnetic', 'geographic' or 'UTM', default: 'geographic'): North reference to be used in
 *                                                                                        the transformed messages.
 * - `~target_type` (str, 'azimuth', 'quaternion', 'pose' or 'imu', default 'azimuth'): The Type of output messages.
 * - `~target_append_suffix` (bool, default false): If true, the output topic will be suffixed with a metadata-based
 *                                                  string.
 * - `~target_frame` (str, default: no change): TF frame to transform the messages to. Please note that frames that are
 *                                              too "titled" from gravity will not make much sense.
 * - `~subscribe_fix` (bool, default true): Whether to subscribe `fix` topic. In some cases, you don't need it.
 * - `~subscribe_utm` (bool, default true): Whether to subscribe `utm_zone` topic. It is fully optional.
 * - `~input_orientation` (str, 'enu' or 'ned', default: unspecified): ENU or NED orientation to be used to interpret
 *                                                                     input messages (in case orientation cannot be
 *                                                                     derived either from message contents or topic
 *                                                                     name).
 * - `~input_reference` (str, 'magnetic', 'geographic' or 'UTM', default: no change): North reference to be used to
 *                                                                                    interpret input messages (in case
 *                                                                                    reference cannot be derived either
 *                                                                                    from message contents or topic
 *                                                                                    name).
 * - `~input_variance` (double, optional, rad^2): If specified, this variance will be used in the output messages
 *                                                if variance cannot be determined from the input messages (e.g. for
 *                                                `QuaternionStamped`).
 * - `~strict` (bool, default true): If true, conversions between magnetic and geographic North will fail if the used
 *                                   magnetic model is used outside its declared bounds of validity (mostly year and
 *                                   altitude).
 * - All parameters consumed by `CompassConverter` (most interesting are `initial_lat`, `initial_lon`, that can relieve
 *   this nodelet from subscribing `fix` topic, if you know the approximate coordinates in advance).
 */
// class CompassTransformerNodelet : public rclcpp::Node
// {
// public:
void CompassTransformerNodelet::setBuffer(tf2_ros::Buffer::SharedPtr buffer)
{
  this->buffer = buffer;
}
// protected:

CompassTransformerNodelet::CompassTransformerNodelet(const rclcpp::NodeOptions & options)
  : rclcpp::Node("compass_transformer_nodelet", options), buffer(std::make_shared<tf2_ros::Buffer>(this->get_clock()))
{
  onInit();
}

CompassTransformerNodelet::CompassTransformerNodelet()
  : rclcpp::Node("compass_transformer_nodelet"), buffer(std::make_shared<tf2_ros::Buffer>(this->get_clock()))
{
  onInit();
}

CompassTransformerNodelet::~CompassTransformerNodelet() = default;

void CompassTransformerNodelet::onInit()
{
  const auto queue_size = this->get_parameter_or<uint32_t>("queue_size", 10);
  const auto targetUnit = this->get_parameter_or<uint8_t>("target_unit", Az::UNIT_RAD);
  const auto targetOrientation = this->get_parameter_or<uint8_t>("target_orientation", Az::ORIENTATION_ENU);
  const auto targetReference = this->get_parameter_or<uint8_t>("target_reference", Az::REFERENCE_GEOGRAPHIC);
  this->targetType = parseOutputType(this->get_parameter_or<std::string>("target_type", outputTypeToString(this->targetType)));
  const auto targetAppendSuffix = this->get_parameter_or<bool>("target_append_suffix", false);
  this->targetFrame = this->get_parameter_or<std::string>("target_frame", std::string());
  const auto subscribeFix = this->get_parameter_or<bool>("subscribe_fix", true);
  const auto subscribeUTMZone = this->get_parameter_or<bool>("subscribe_utm", true);

  const auto log = this->get_logger();
  const auto clock = this->get_clock();
  this->converter = std::make_shared<CompassConverter>(log, *clock, this->get_parameter_or<bool>("strict", true));
  this->converter->configFromParams(shared_from_this());

  std::string outputTopicSuffix;
  std::string topicName;
  switch (this->targetType)
  {
    case OutputType::Imu:
      outputTopicSuffix = getAzimuthTopicSuffix<sensor_msgs::msg::Imu>(targetUnit, targetOrientation, targetReference);
      topicName = targetAppendSuffix ? outputTopicSuffix :  "azimuth_out";
      this->pub_imu = this->create_publisher<sensor_msgs::msg::Imu>(
        topicName, queue_size);
      break;
    case OutputType::Pose:
      outputTopicSuffix = getAzimuthTopicSuffix<geometry_msgs::msg::PoseWithCovarianceStamped>(
        targetUnit, targetOrientation, targetReference);
      topicName = targetAppendSuffix ? outputTopicSuffix :  "azimuth_out";
      this->pub_pose = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        topicName, queue_size);
      break;
    case OutputType::Quaternion:
      outputTopicSuffix = getAzimuthTopicSuffix<geometry_msgs::msg::QuaternionStamped>(
        targetUnit, targetOrientation, targetReference);
      topicName = targetAppendSuffix ? outputTopicSuffix :  "azimuth_out";
      this->pub_quat = this->create_publisher<geometry_msgs::msg::QuaternionStamped>(
        topicName, queue_size);
      break;
    default:
      outputTopicSuffix = getAzimuthTopicSuffix<Az>(targetUnit, targetOrientation, targetReference);
      topicName = targetAppendSuffix ? outputTopicSuffix :  "azimuth_out";
      this->pub_az = this->create_publisher<Az>(
        topicName, queue_size);
      break;
  }

  this->azimuthInput = std::make_unique<UniversalAzimuthSubscriber>(shared_from_this(), "azimuth_in", queue_size);
  this->azimuthInput->configFromParams(shared_from_this());

  this->compassFilter = std::make_unique<CompassFilter>(
    log, *clock, this->converter, *this->azimuthInput, targetUnit, targetOrientation, targetReference);

  if (subscribeFix)
  {
    this->fixInput = std::make_unique<message_filters::Subscriber<Fix>>(this, "fix");
    this->compassFilter->connectFixInput(*this->fixInput);
  }

  if (subscribeUTMZone)
  {
    this->utmZoneInput = std::make_unique<message_filters::Subscriber<std_msgs::msg::Int32>>(this, "utm_zone");
    this->compassFilter->connectUTMZoneInput(*this->utmZoneInput);
  }

  if (targetFrame.empty())
  {
    this->compassFilter->registerCallback(&CompassTransformerNodelet::publish, this);
  }
  else
  {
    this->tfFilter = std::make_unique<tf2_ros::MessageFilter<Az>>(
      *this->compassFilter, *this->buffer, targetFrame, queue_size, shared_from_this());
    this->tfFilter->registerCallback(&CompassTransformerNodelet::transformAndPublish, this);
    // registerFailureCallback IS CURRENTLY DISABLED IN TF2_ROS FOR "UNKNOWN REASONS" ...
    // this->tfFilter->registerFailureCallback(std::bind_front(&CompassTransformerNodelet::failedCb, this));
  }

  RCLCPP_INFO(log, "Publishing azimuth to topic %s (type %s).",
    topicName.c_str(), outputTypeToString(this->targetType).c_str());
}
  /* void onInit() override
  {
    cras::Nodelet::onInit();

    const auto params = this->privateParams();

    // Start reading params

    const auto queue_size = params->getParam("queue_size", 10_sz, "messages");
    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    const auto targetUnit = params->getParam<decltype(Az::unit), std::string>("target_unit", Az::UNIT_RAD, "");
      // cras::GetParamConvertingOptions<decltype(Az::unit), std::string>(
      //   &compass_interfaces::msg::unitToString, &compass_interfaces::msg::parseUnit));

    const auto targetOrientation = params->getParam<decltype(Az::orientation), std::string>(
      "target_orientation", Az::ORIENTATION_ENU, "");
      // cras::GetParamConvertingOptions<decltype(Az::orientation), std::string>(
      //   &compass_interfaces::msg::orientationToString, &compass_interfaces::msg::parseOrientation));

    const auto targetReference = params->getParam<decltype(Az::reference), std::string>(
      "target_reference", Az::REFERENCE_GEOGRAPHIC, "");
      // cras::GetParamConvertingOptions<decltype(Az::reference), std::string>(
      //  &compass_interfaces::msg::referenceToString, &compass_interfaces::msg::parseReference));

    this->targetType = params->getParam<OutputType, std::string>("target_type", this->targetType, "");
      // cras::GetParamConvertingOptions<OutputType, std::string>(&outputTypeToString, &parseOutputType));

    const auto targetAppendSuffix = params->getParam("target_append_suffix", false);

    this->targetFrame = params->getParam("target_frame", std::string());

    const auto subscribeFix = params->getParam("subscribe_fix", true);
    const auto subscribeUTMZone = params->getParam("subscribe_utm", true);

    // End reading params

    const auto log = this->getLogger();

    this->converter = std::make_shared<CompassConverter>(log, params->getParam("strict", true));
    this->converter->configFromParams(*params);

    auto outputNh = targetAppendSuffix ? ros::NodeHandle(pnh, "azimuth_out") : pnh;

    std::string outputTopicSuffix;
    switch (this->targetType)
    {
      case OutputType::Imu:
        outputTopicSuffix = getAzimuthTopicSuffix<sensor_msgs::msg::Imu>(targetUnit, targetOrientation, targetReference);
        this->pub = outputNh.advertise<sensor_msgs::msg::Imu>(
          targetAppendSuffix ? outputTopicSuffix :  "azimuth_out", queue_size);
        break;
      case OutputType::Pose:
        outputTopicSuffix = getAzimuthTopicSuffix<geometry_msgs::msg::PoseWithCovarianceStamped>(
          targetUnit, targetOrientation, targetReference);
        this->pub = outputNh.advertise<geometry_msgs::msg::PoseWithCovarianceStamped>(
          targetAppendSuffix ? outputTopicSuffix :  "azimuth_out", queue_size);
        break;
      case OutputType::Quaternion:
        outputTopicSuffix = getAzimuthTopicSuffix<geometry_msgs::msg::QuaternionStamped>(
          targetUnit, targetOrientation, targetReference);
        this->pub = outputNh.advertise<geometry_msgs::msg::QuaternionStamped>(
          targetAppendSuffix ? outputTopicSuffix :  "azimuth_out", queue_size);
        break;
      default:
        outputTopicSuffix = getAzimuthTopicSuffix<Az>(targetUnit, targetOrientation, targetReference);
        this->pub = outputNh.advertise<Az>(
          targetAppendSuffix ? outputTopicSuffix :  "azimuth_out", queue_size);
        break;
    }

    this->azimuthInput = std::make_unique<UniversalAzimuthSubscriber>(this->get_logger(), pnh, "azimuth_in", queue_size);
    this->azimuthInput->configFromParams(*params);

    this->compassFilter = std::make_unique<CompassFilter>(
      log, this->converter, *this->azimuthInput, targetUnit, targetOrientation, targetReference);

    if (subscribeFix)
    {
      this->fixInput = std::make_unique<message_filters::Subscriber<Fix>>(nh, "fix", queue_size);
      this->compassFilter->connectFixInput(*this->fixInput);
    }

    if (subscribeUTMZone)
    {
      this->utmZoneInput = std::make_unique<message_filters::Subscriber<std_msgs::msg::Int32>>(nh, "utm_zone", queue_size);
      this->compassFilter->connectUTMZoneInput(*this->utmZoneInput);
    }

    if (targetFrame.empty())
    {
      this->compassFilter->registerCallback(&CompassTransformerNodelet::publish, this);
    }
    else
    {
      this->tfFilter = std::make_unique<tf2_ros::MessageFilter<Az>>(
        log, *this->compassFilter, this->getBuffer().getRawBuffer(), targetFrame, queue_size, nh);
      this->tfFilter->registerCallback(&CompassTransformerNodelet::transformAndPublish, this);
      this->tfFilter->registerFailureCallback(std::bind_front(&CompassTransformerNodelet::failedCb, this));
    }

    RCLCPP_INFO(log, "Publishing azimuth to topic %s (type %s).",
      ros::names::resolve(this->pub.getTopic()).c_str(), outputTypeToString(this->targetType).c_str());
  } */

void CompassTransformerNodelet::publish(const Az::ConstSharedPtr& msg)
{
  switch (this->targetType)
  {
    case OutputType::Imu:
    {
      const auto maybeImu = this->converter->convertToImu(*msg);
      if (maybeImu.has_value())
        this->pub_imu->publish<sensor_msgs::msg::Imu>(maybeImu.value());
      else
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1.0, "%s", maybeImu.error().c_str());
      break;
    }
    case OutputType::Pose:
    {
      const auto maybePose = this->converter->convertToPose(*msg);
      if (maybePose.has_value())
        this->pub_pose->publish(*maybePose);
      else
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1.0, "%s", maybePose.error().c_str());
      break;
    }
    case OutputType::Quaternion:
    {
      const auto maybeQuat = this->converter->convertToQuaternion(*msg);
      if (maybeQuat.has_value())
        this->pub_quat->publish(*maybeQuat);
      else
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1.0, "%s", maybeQuat.error().c_str());
      break;
    }
    default:
      this->pub_az->publish(*msg);
      break;
  }
}

void CompassTransformerNodelet::transformAndPublish(const Az::ConstSharedPtr& msg)
{
  try
  {
    Az::SharedPtr outMsg(new Az{});
    *outMsg = this->buffer->transform(*msg, this->targetFrame, tf2::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(100))));
    this->publish(outMsg);
  }
  catch (const tf2::TransformException& e)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1.0, "Azimuth transformation failed: %s", e.what());
  }
}

void CompassTransformerNodelet::failedCb(const Az::ConstSharedPtr& /*msg*/, const tf2_ros::filter_failure_reasons::FilterFailureReason reason)
{
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1.0, "Can't transform incoming Azimuth data to frame %s. Reason %d",
    this->targetFrame.c_str(), reason);
}

// std::shared_ptr<CompassConverter> converter;
// std::unique_ptr<UniversalAzimuthSubscriber> azimuthInput;
// std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>> fixInput;
// std::unique_ptr<message_filters::Subscriber<std_msgs::msg::Int32>> utmZoneInput;
// std::unique_ptr<CompassFilter> compassFilter;
// std::unique_ptr<tf2_ros::MessageFilter<compass_interfaces::msg::Azimuth>> tfFilter;
// //rclcpp::PublisherBase::SharedPtr pub;
// rclcpp::Publisher<Az>::SharedPtr pub_az;
// rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
// rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose;
// rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub_quat;
// std::string targetFrame;
// OutputType targetType {OutputType::Azimuth};
// tf2_ros::Buffer::SharedPtr buffer;// {std::make_shared<tf2_ros::Buffer>(this->get_clock())};

}


RCLCPP_COMPONENTS_REGISTER_NODE(compass_conversions::CompassTransformerNodelet)