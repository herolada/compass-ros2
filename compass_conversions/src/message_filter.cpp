// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Message filter to convert between various compass representations.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

#include <compass_conversions/message_filter.h>
#include <compass_utils/string_utils.hpp>
#include <compass_interfaces/msg/azimuth.hpp>
// #include <cras_cpp_common/log_utils.h>
#include <optional>
// #include <ros/callback_queue_interface.h>
#include <message_filters/message_event.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/rclcpp.hpp>
// #include <ros/transport_hints.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <message_filters/message_event.h>

using Az = compass_interfaces::msg::Azimuth;

namespace compass_conversions
{

UniversalAzimuthSubscriber::UniversalAzimuthSubscriber(const rclcpp::Logger& log, const rclcpp::Clock& clock, const rclcpp::Node* node,
  const std::string& topic, const uint32_t queueSize
  //, const ros::TransportHints& transportHints, ros::CallbackQueueInterface* callbackQueue
  ): log(log), clock(clock), node(node), converter(log, true), topic(topic), queueSize(queueSize)
{
  this->subscribe(node, topic, queueSize);//, transportHints, callbackQueue);
}

UniversalAzimuthSubscriber::~UniversalAzimuthSubscriber()
{
  this->unsubscribe();
}

void UniversalAzimuthSubscriber::subscribe(const rclcpp::Node* node, const std::string& topic, const uint32_t queueSize)
  // const ros::TransportHints& transportHints, ros::CallbackQueueInterface* callbackQueue)
{
  this->unsubscribe();

  if (!topic.empty())
  {
    // this->subscribeOps.template initByFullCallbackType<const EventType&>(
      // topic, queueSize, boost::bind(&UniversalAzimuthSubscriber::cb, this, boost::placeholders::_1));
    // this->subscribeOps.callback_queue = callbackQueue;
    // this->subscribeOps.transport_hints = transportHints;
    // this->sub = node.subscribe();// this->subscribeOps);
    this->sub = node.create_subscription(topic, queueSize, this->cb);
    this->node = node;
    this->topic = topic;
    this->queueSize = queueSize;
  }
}

void UniversalAzimuthSubscriber::subscribe()
{
  this->unsubscribe();

  if (!this->topic.empty())
    // this->sub = this->node.subscribe(this->subscribeOps);
    this->sub = this->node.create_subscription(this->topic, this->queueSize, this->cb)

}

void UniversalAzimuthSubscriber::unsubscribe()
{
  this->sub.reset();
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
    inputOrientation = node->get_parameter_or<std::optional<uint8_t>>("input_orientation", std::nullopt);
      // cras::GetParamConvertingOptions<decltype(Az::orientation), std::string>(
      //   &compass_interfaces::msg::orientationToString, &compass_interfaces::msg::parseOrientation));

  std::optional<decltype(Az::reference)> inputReference;
  if (node->has_parameter("input_reference"))
    inputReference = node->get_parameter_or<std::optional<uint8_t>>("input_reference", std::nullopt);
      // cras::GetParamConvertingOptions<decltype(Az::reference), std::string>(
      //   &compass_interfaces::msg::referenceToString, &compass_interfaces::msg::parseReference));

  std::optional<decltype(Az::variance)> inputVariance;
  if (node->has_parameter("input_variance"))
    inputVariance = node->get_parameter_or<std::optional<double>>("input_variance", std::nullopt);

  this->setInputDefaults(inputOrientation, inputReference, inputVariance);
}

std::string UniversalAzimuthSubscriber::getTopic() const
{
  return this->topic;
}

const rclcpp::Subscription<compass_interfaces::msg::Azimuth>& UniversalAzimuthSubscriber::getSubscriber() const
{
  return this->sub;
}

void UniversalAzimuthSubscriber::add(const EventType&)
{
}

void UniversalAzimuthSubscriber::cb(const EventType& event)
{
  const auto maybeAzimuth = this->converter.convertUniversalMsgEvent(
    event, this->inputVariance.value_or(0.0), Az::UNIT_RAD, this->inputOrientation, this->inputReference);
  if (!maybeAzimuth.has_value())
  {
    RCLCPP_ERROR_THROTTLE(this->log, this->clock, 10.0, "Error converting message to Azimuth: %s", maybeAzimuth.error().c_str());
    return;
  }

  // const auto header = event.getConnectionHeaderPtr();
  const auto stamp = event.getReceiptTime();
  this->signalMessage(message_filters::MessageEvent<Az const>(
    std::make_shared<Az>(*maybeAzimuth), stamp, false, message_filters::DefaultMessageCreator<Az>()));
}


CompassFilter::~CompassFilter() = default;

void CompassFilter::cbAzimuth(const AzimuthEventType& azimuthEvent)
{
  const auto& msg = azimuthEvent.getConstMessage();
  const auto output = this->converter->convertAzimuth(
    *msg, this->unit, this->orientation, this->reference.value_or(msg->reference));
  if (!output.has_value())
  {
    RCLCPP_ERROR_THROTTLE(this->log, this->clock, 10.0,
      "Azimuth conversion failed%s: %s", fixReceived ? "" : "(no fix message received yet)", output.error().c_str());
    return;
  }
  this->signalMessage(AzimuthEventType(
    std::make_shared<Az>(*output), //azimuthEvent.getConnectionHeaderPtr(),
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
