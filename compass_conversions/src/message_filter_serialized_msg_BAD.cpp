// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Message filter to convert between various compass representations.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

// #include <boost/bind.hpp>
// #include <boost/make_shared.hpp>

#include <compass_conversions/message_filter.h>
#include <compass_utils/string_utils.hpp>
#include <compass_interfaces/msg/azimuth.hpp>
// #include <cras_cpp_common/log_utils.h>
#include <optional>
// #include <ros/callback_queue_interface.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/rclcpp.hpp>
// #include <ros/transport_hints.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <message_filters/message_event.h>
#include <message_filters/subscriber.h>

using Az = compass_interfaces::msg::Azimuth;

namespace compass_conversions
{


UniversalAzimuthSubscriber::UniversalAzimuthSubscriber(rclcpp::Node* node,
  std::string topic, const uint32_t queueSize
  //, const ros::TransportHints& transportHints, ros::CallbackQueueInterface* callbackQueue
  ): node(node), converter(node->get_logger(), *node->get_clock(), true), topic(topic), queueSize(queueSize),
  sub(message_filters::Subscriber<rclcpp::SerializedMessage>())
{
  this->subscribe(node, topic, queueSize);//, transportHints, callbackQueue);
}

UniversalAzimuthSubscriber::~UniversalAzimuthSubscriber()
{
  this->unsubscribe();
}

void UniversalAzimuthSubscriber::subscribe(rclcpp::Node* node, std::string topic, const uint32_t queueSize)
  // const ros::TransportHints& transportHints, ros::CallbackQueueInterface* callbackQueue)
{
  this->unsubscribe();

  if (!topic.empty())
  {

    this->node = node;
    this->topic = topic;
    this->queueSize = queueSize;

    this->sub.subscribe(node, topic);
    this->sub.registerCallback(
      // [this](const AzimuthEventType msg) {
      //   this->cb(msg);
      // std::bind(&cb, this, std::placeholders::_1)
      std::function<void(const EventType&)>(std::bind_front(&UniversalAzimuthSubscriber::cb, this)));
  }
}

void UniversalAzimuthSubscriber::subscribe()
{
  this->unsubscribe();

  if (!this->topic.empty())
  {
    // // this->sub = this->node.subscribe(this->subscribeOps);
    // this->sub = this->node->create_subscription<Az>(this->topic, this->queueSize,
    //   [this](const EventType& msg) {
    //     this->cb(msg);
    // });

    this->sub.subscribe(this->node, this->topic);
    this->sub.registerCallback(
      // [this](const AzimuthEventType msg) {
      //   this->cb(msg);
      std::function<void(const EventType&)>(std::bind_front(&UniversalAzimuthSubscriber::cb, this)));

  }
}

void UniversalAzimuthSubscriber::unsubscribe()
{
  this->sub.unsubscribe();
}

void UniversalAzimuthSubscriber::setInputDefaults(
  const std::optional<decltype(compass_interfaces::msg::Azimuth::orientation)>& orientation,
  const std::optional<decltype(compass_interfaces::msg::Azimuth::reference)>& reference,
  const std::optional<decltype(compass_interfaces::msg::Azimuth::variance)>& variance)
{
  this->inputOrientation = orientation;
  this->inputReference = reference;
  this->inputVariance = variance;
}

void UniversalAzimuthSubscriber::configFromParams(const rclcpp::Node* node)
{
  std::optional<decltype(Az::orientation)> inputOrientation;
  if (node->has_parameter("input_orientation"))
    inputOrientation = std::make_optional<uint8_t>(node->get_parameter("input_orientation").get_value<uint8_t>());
      // cras::GetParamConvertingOptions<decltype(Az::orientation), std::string>(
      //   &compass_interfaces::msg::orientationToString, &compass_interfaces::msg::parseOrientation));

  std::optional<decltype(Az::reference)> inputReference;
  if (node->has_parameter("input_reference"))
    inputReference = std::make_optional<uint8_t>(node->get_parameter("input_reference").get_value<uint8_t>());
      // cras::GetParamConvertingOptions<decltype(Az::reference), std::string>(
      //   &compass_interfaces::msg::referenceToString, &compass_interfaces::msg::parseReference));

  std::optional<decltype(Az::variance)> inputVariance;
  if (node->has_parameter("input_variance"))
    inputVariance = std::make_optional<double>(node->get_parameter("input_variance").get_value<double>());

  this->setInputDefaults(inputOrientation, inputReference, inputVariance);
}

std::string UniversalAzimuthSubscriber::getTopic() const
{
  return this->sub.getTopic();
}

/* const message_filters::Subscriber<compass_interfaces::msg::Azimuth>& UniversalAzimuthSubscriber::getSubscriber() const
{
  if (this->messageType == "compass_interfaces::msg::Azimuth")
    {
      return this->sub;
    }
    else if (this->messageType == "geometry_msgs::msg::PoseWithCovarianceStamped")
    {
      return this->poseSub;
    }
    else if (this->messageType == "geometry_msgs::msg::QuaternionStamped")
    {
      return this->quatSub;
    }
    else if (this->messageType == "sensor_msgs::msg::Imu")
    {
      return this->imuSub;
    }
    else
    {
      RCLCPP_ERROR(this->node->get_logger(), "Invalid message type: %s. Returning Azimuth subscriber by default.", this->messageType.c_str());
      return this->sub;
    } 
} */
const message_filters::Subscriber<rclcpp::SerializedMessage>& UniversalAzimuthSubscriber::getSubscriber() const
{return this->sub;}



// void UniversalAzimuthSubscriber::add(const EventType&)
// {
// }

void UniversalAzimuthSubscriber::cb(const EventType& event) {
  const compass_interfaces::msg::Azimuth az = Az();
  auto msg_type = this->node->get_topic_names_and_types();
  const auto maybeAzimuth = this->converter.convertUniversalMsgEvent(event, this->topic, this->inputVariance.value_or(0.0), Az::UNIT_RAD, az.orientation, az.reference);
  if (!maybeAzimuth.has_value())
  {
    RCLCPP_ERROR_THROTTLE(this->node->get_logger(), *this->node->get_clock(), 10000., "Error converting message to Azimuth: %s", maybeAzimuth.error().c_str());
    return;
  }
  // const auto header = event.getConnectionHeaderPtr();
  const auto stamp = event.getReceiptTime();
  this->signalMessage(message_filters::MessageEvent<Az const>(
    std::make_shared<Az const>(*maybeAzimuth), stamp, false, message_filters::DefaultMessageCreator<Az>()));
}

/* void UniversalAzimuthSubscriber::cb(const EventType& event)
{
  const auto maybeAzimuth = this->converter.convertUniversalMsgEvent(
    event, this->inputVariance.value_or(0.0), Az::UNIT_RAD, this->inputOrientation, this->inputReference);
  if (!maybeAzimuth.has_value())
  {
    RCLCPP_ERROR_THROTTLE(this->node->get_logger(), *this->node->get_clock(), 10000., "Error converting message to Azimuth: %s", maybeAzimuth.error().c_str());
    return;
  }

  // const auto header = event.getConnectionHeaderPtr();
  const auto stamp = event.getReceiptTime();
  this->signalMessage(message_filters::MessageEvent<Az const>(
    std::make_shared<Az>(*maybeAzimuth), stamp, false, message_filters::DefaultMessageCreator<Az>()));
} */


CompassFilter::~CompassFilter() = default;

void CompassFilter::cbAzimuth(const AzimuthEventType& azimuthEvent)
{
  const auto& msg = azimuthEvent.getConstMessage();
  const auto output = this->converter->convertAzimuth(
    *msg, this->unit, this->orientation, this->reference.value_or(msg->reference));
  if (!output.has_value())
  {
    RCLCPP_ERROR_THROTTLE(this->log, this->clock, 10000.,
      "Azimuth conversion failed%s: %s", fixReceived ? "" : "(no fix message received yet)", output.error().c_str());
    return;
  }
  this->signalMessage(AzimuthEventType(
    std::make_shared<Az const>(*output), //azimuthEvent.getConnectionHeaderPtr(),
    azimuthEvent.getReceiptTime(), false, message_filters::DefaultMessageCreator<Az>()));
}

void CompassFilter::cbFix(const FixEventType& fixEvent)
{
  this->fixReceived = true;
  this->converter->setNavSatPos(*fixEvent.getConstMessage());
}

void CompassFilter::cbUTMZone(const UTMZoneEventType& utmZoneEvent)
{
  this->converter->forceUTMZone(utmZoneEvent.getConstMessage()->data);
}

}
