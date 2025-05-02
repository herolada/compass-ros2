// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Message filter to convert between various compass representations.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
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


namespace compass_conversions
{
using Az = compass_interfaces::msg::Azimuth;
using Imu = sensor_msgs::msg::Imu;
using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
using Quat = geometry_msgs::msg::QuaternionStamped;

UniversalAzimuthSubscriber::UniversalAzimuthSubscriber(rclcpp::Node* node,
  std::string topic, const uint32_t queueSize
  //, const ros::TransportHints& transportHints, ros::CallbackQueueInterface* callbackQueue
  ): node(node), converter(node, true), topic(topic), queueSize(queueSize),
  azSub(message_filters::Subscriber<Az>()),
  poseSub(message_filters::Subscriber<Pose>()),
  quatSub(message_filters::Subscriber<Quat>()),
  imuSub(message_filters::Subscriber<Imu>())
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

    this->azSub.subscribe(node, topic);
    this->poseSub.subscribe(node, topic+"/pose");
    this->quatSub.subscribe(node, topic+"/quat");
    this->imuSub.subscribe(node, topic+"/imu");
    this->azSub.registerCallback(
      std::function<void(const AzimuthEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::azCb, this)));
    this->poseSub.registerCallback(
      std::function<void(const PoseEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::poseCb, this)));
    this->quatSub.registerCallback(
      std::function<void(const QuatEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::quatCb, this)));
    this->imuSub.registerCallback(
      std::function<void(const ImuEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::imuCb, this)));

    RCLCPP_INFO(node->get_logger(), "Listening for azimuth at topics %s, %s, %s, %s.\n",
      (topic).c_str(), (topic+"/pose").c_str(), (topic+"/quat").c_str(), (topic+"/imu").c_str());

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

    this->azSub.subscribe(this->node, this->topic);
    this->poseSub.subscribe(this->node, this->topic+"/pose");
    this->quatSub.subscribe(this->node, this->topic+"/quat");
    this->imuSub.subscribe(this->node, this->topic+"/imu");
    this->azSub.registerCallback(
      std::function<void(const AzimuthEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::azCb, this)));
    this->poseSub.registerCallback(
      std::function<void(const PoseEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::poseCb, this)));
    this->quatSub.registerCallback(
      std::function<void(const QuatEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::quatCb, this)));
    this->imuSub.registerCallback(
      std::function<void(const ImuEventType&)>(std::bind_front(&UniversalAzimuthSubscriber::imuCb, this)));

      RCLCPP_INFO(this->node->get_logger(), "Listening for azimuth at topics %s, %s, %s, %s.\n",
        (topic).c_str(), (topic+"/pose").c_str(), (topic+"/quat").c_str(), (topic+"/imu").c_str());

  }
}

void UniversalAzimuthSubscriber::unsubscribe()
{
  this->azSub.unsubscribe();
  this->poseSub.unsubscribe();
  this->quatSub.unsubscribe();
  this->imuSub.unsubscribe();
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
  if (node->has_parameter("input_orientation") && 
      !node->get_parameter("input_orientation").get_value<std::string>().empty())
    inputOrientation = std::make_optional<uint8_t>(compass_utils::parseOrientation(node->get_parameter("input_orientation").get_value<std::string>()));
      // cras::GetParamConvertingOptions<decltype(Az::orientation), std::string>(
      //   &compass_interfaces::msg::orientationToString, &compass_interfaces::msg::parseOrientation));

  std::optional<decltype(Az::reference)> inputReference;
  if (node->has_parameter("input_reference") && 
      !node->get_parameter("input_reference").get_value<std::string>().empty())
    inputReference = std::make_optional<uint8_t>(compass_utils::parseReference(node->get_parameter("input_reference").get_value<std::string>()));

      // cras::GetParamConvertingOptions<decltype(Az::reference), std::string>(
      //   &compass_interfaces::msg::referenceToString, &compass_interfaces::msg::parseReference));

  std::optional<decltype(Az::variance)> inputVariance;
  if (node->has_parameter("input_variance") &&
      node->get_parameter("input_variance").get_value<double>() != -1.)
    inputVariance = std::make_optional<double>(node->get_parameter("input_variance").get_value<double>());

  this->setInputDefaults(inputOrientation, inputReference, inputVariance);
}

std::string UniversalAzimuthSubscriber::getTopic() const
{
  return this->topic;
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
// const message_filters::Subscriber<rclcpp::SerializedMessage>& UniversalAzimuthSubscriber::getSubscriber() const
// {return this->sub;}

  // Defined in header....
  // const message_filters::Subscriber<message_filters::MessageEvent<compass_interfaces::msg::Azimuth const>>& UniversalAzimuthSubscriber::getAzSubscriber() const
  //   {return this->azSub;}
  // const message_filters::Subscriber<message_filters::MessageEvent<geometry_msgs::msg::PoseWithCovarianceStamped const>>& UniversalAzimuthSubscriber::getPoseSubscriber() const
  //   {return this->poseSub;}
  // const message_filters::Subscriber<message_filters::MessageEvent<geometry_msgs::msg::QuaternionStamped const>>& UniversalAzimuthSubscriber::getQuatSubscriber() const
  //   {return this->quatSub;}
  // const message_filters::Subscriber<message_filters::MessageEvent<sensor_msgs::msg::Imu const>>& UniversalAzimuthSubscriber::getImuSubscriber() const
  //   {return this->imuSub;}


// void UniversalAzimuthSubscriber::add(const EventType&)
// {
// }

void UniversalAzimuthSubscriber::azCb(const AzimuthEventType& event) {
  const auto msg = event.getConstMessage();
  const auto stamp = event.getReceiptTime();
  const auto maybeAzimuth = this->converter.convertAzimuth(*msg, Az::UNIT_RAD, msg->orientation, msg->reference);

  if (!maybeAzimuth.has_value())
  {
    RCLCPP_ERROR_THROTTLE(this->node->get_logger(), *this->node->get_clock(), 10000., "Error converting message to Azimuth: %s", maybeAzimuth.error().c_str());
    return;
  }
  // const auto header = event.getConnectionHeaderPtr();

  this->signalMessage(message_filters::MessageEvent<Az const>(
    std::make_shared<Az const>(*maybeAzimuth), stamp, false, message_filters::DefaultMessageCreator<Az>()));
}

void UniversalAzimuthSubscriber::poseCb(const PoseEventType& event) {
  const auto msg = event.getConstMessage();
  const auto stamp = event.getReceiptTime();
  const auto maybeAzimuth = this->converter.convertPoseMsgEvent(this->topic.c_str(), event, Az::UNIT_RAD, this->inputOrientation, this->inputReference);

  if (!maybeAzimuth.has_value())
  {
    RCLCPP_ERROR_THROTTLE(this->node->get_logger(), *this->node->get_clock(), 10000., "Error converting message to Azimuth: %s", maybeAzimuth.error().c_str());
    return;
  }
  // const auto header = event.getConnectionHeaderPtr();
  this->signalMessage(message_filters::MessageEvent<Az const>(
    std::make_shared<Az const>(*maybeAzimuth), stamp, false, message_filters::DefaultMessageCreator<Az>()));
}

void UniversalAzimuthSubscriber::quatCb(const QuatEventType& event) {
  const auto msg = event.getConstMessage();
  const auto stamp = event.getReceiptTime();
  const auto maybeAzimuth = this->converter.convertQuaternionMsgEvent(
    this->topic.c_str(), event, this->inputVariance.value_or(0.0), Az::UNIT_RAD, this->inputOrientation, this->inputReference);

  if (!maybeAzimuth.has_value())
  {
    RCLCPP_ERROR_THROTTLE(this->node->get_logger(), *this->node->get_clock(), 10000., "Error converting message to Azimuth: %s", maybeAzimuth.error().c_str());
    return;
  }
  // const auto header = event.getConnectionHeaderPtr();
  this->signalMessage(message_filters::MessageEvent<Az const>(
    std::make_shared<Az const>(*maybeAzimuth), stamp, false, message_filters::DefaultMessageCreator<Az>()));
}

void UniversalAzimuthSubscriber::imuCb(const ImuEventType& event) {
  const auto msg = event.getConstMessage();
  const auto stamp = event.getReceiptTime();
  const auto maybeAzimuth = this->converter.convertImuMsgEvent(this->topic.c_str(), event, Az::UNIT_RAD, this->inputOrientation, this->inputReference);

    if (!maybeAzimuth.has_value())
  {
    RCLCPP_ERROR_THROTTLE(this->node->get_logger(), *this->node->get_clock(), 10000., "Error converting message to Azimuth: %s", maybeAzimuth.error().c_str());
    return;
  }
  // const auto header = event.getConnectionHeaderPtr();
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
    RCLCPP_ERROR_THROTTLE(this->node->get_logger(), *this->node->get_clock(), 10000.,
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
