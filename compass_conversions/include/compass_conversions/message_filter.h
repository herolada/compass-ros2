#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Message filter to convert between various compass representations.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <compass_conversions/compass_converter.h>
#include <compass_interfaces/msg/azimuth.hpp>
// #include <cras_cpp_common/functional.hpp>
// #include <cras_cpp_common/log_utils.h>
// #include <cras_cpp_common/param_utils/bound_param_helper.hpp>
#include <optional>
#include <message_filters/connection.h>
#include <message_filters/simple_filter.h>
#include <message_filters/subscriber.h>
// #include <ros/callback_queue_interface.h>
// #include <ros/message_event.h>
// #include <ros/node_handle.h>
// #include <ros/subscribe_options.h>
// #include <ros/subscriber.h>
// #include <ros/transport_hints.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int32.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/clock.hpp>
#include <message_filters/message_event.h>

namespace compass_conversions
{

/**
 * \brief message_filters subscriber that can subscribe to various topic types and convert them all to an Azimuth
 *        message.
 *
 * Currently supported types are: compass_interfaces::msg::Azimuth, geometry_msgs::msg::PoseWithCovarianceStamped,
 * geometry_msgs::msg::QuaternionStamped, sensor_msgs::msg::Imu.
 */
class UniversalAzimuthSubscriber :
  public message_filters::SimpleFilter<compass_interfaces::msg::Azimuth>, public message_filters::SubscriberBase<compass_interfaces::msg::Azimuth>
{
public:
  typedef message_filters::MessageEvent<rclcpp::GenericSubscription const> EventType;

  /**
   * \brief Constructor
   *
   * \param log Logger.
   * \param nh The ros::NodeHandle to use for subscribing.
   * \param topic The topic to subscribe to.
   * \param queueSize Queue size of the subscription.
   * \param transportHints The transport hints to pass to the subscriber.
   * \param callbackQueue The callback queue to attach to.
   */
  UniversalAzimuthSubscriber(rclcpp::Node* node, const std::string& topic,
    const uint32_t queueSize
    //  , const ros::TransportHints& transportHints = {}, ros::CallbackQueueInterface* callbackQueue = nullptr
    );

  ~UniversalAzimuthSubscriber() override;

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param nh The ros::NodeHandle to use for subscribing.
   * \param topic The topic to subscribe to.
   * \param queueSize Queue size of the subscription.
   * \param transportHints The transport hints to pass to the subscriber.
   * \param callbackQueue The callback queue to attach to.
   */
  void subscribe(rclcpp::Node* node, const std::string& topic, const uint32_t queueSize
    //,  const ros::TransportHints& transportHints, ros::CallbackQueueInterface* callbackQueue
    );  // WARN there used to be override here

  /**
   * \brief Re-subscribe to a topic.
   */
  void subscribe() override;

  /**
   * \brief Unsubscribe from the topic.
   */
  void unsubscribe() override;

  /**
   * \brief Set defaults for inputs which do not support autodetection of various azimuth properties.
   *
   * \param[in] orientation The default orientation used if it cannot be detected.
   * \param[in] reference The reference used if it cannot be detected.
   * \param[in] variance Default variance used for topics which cannot automatically discover it.
   */
  void setInputDefaults(
    const std::optional<decltype(compass_interfaces::msg::Azimuth::orientation)>& orientation,
    const std::optional<decltype(compass_interfaces::msg::Azimuth::reference)>& reference,
    const std::optional<decltype(compass_interfaces::msg::Azimuth::variance)>& variance);

  /**
   * \brief Configure the subscriber from ROS parameters.
   * \param[in] params ROS parameters.
   *
   * Supported parameters:
   * - `~input_orientation` (str, 'enu' or 'ned', default: unspecified): ENU or NED orientation to be used to interpret
   *                                                                     input messages (in case orientation cannot be
   *                                                                     derived either from message contents or topic
   *                                                                     name).
   * - `~input_reference` (str, 'magnetic', 'geographic' or 'UTM', default: no change): North reference to be used to
   *                                                                                    interpret input messages (in
   *                                                                                    case reference cannot be derived
   *                                                                                    either from message contents or
   *                                                                                    topic name).
   * - `~input_variance` (double, optional, rad^2): If specified, this variance will be used in the output messages
   *                                                if variance cannot be determined from the input messages (e.g. for
   *                                                `QuaternionStamped`).
   */
  void configFromParams(const rclcpp::Node* node);

  /**
   * \brief Get the name of the subscribed topic.
   * \return The topic name.
   */
  std::string getTopic() const;

  /**
   * \brief Returns the internal rclcpp::Subscription.
   */
  const rclcpp::Subscription<compass_interfaces::msg::Azimuth>& getSubscriber() const;
  // if it doesnt work maybe use GenericSubscription? (does not require specifying the msg template)

  template<typename F>
  void connectInput(F& f)
  {
  }

  void add(const EventType& event);

protected:
  void cb(const EventType& event);

  rclcpp::Subscription<compass_interfaces::msg::Azimuth> sub;  //!< The ROS subscriber.
  //rclcpp::SubscriptionOptions subscribeOps;  //!< Options for recreating the subscriber.
  rclcpp::Node* node;  //!< The nodehandle to use for subscribing,
  CompassConverter converter;  //!< The azimuth message converter.

  //! Orientation of the input azimuth (in case it is a data type which does not tell orientation explicitly).
  std::optional<decltype(compass_interfaces::msg::Azimuth::orientation)> inputOrientation;

  //! Reference of the input azimuth (in case it is a data type which does not tell reference explicitly).
  std::optional<decltype(compass_interfaces::msg::Azimuth::reference)> inputReference;

  //! Variance of the input azimuth (in case it is a data type which does not tell reference explicitly).
  std::optional<decltype(compass_interfaces::msg::Azimuth::variance)> inputVariance;
  const std::string& topic {std::string()};
  const uint32_t queueSize {10};
};









/**
 * \brief Message filter to convert between various compass representations.
 *
 * \sa https://wiki.ros.org/message_filters
 *
 * Example usage:
 * \code{.cpp}
 * message_filters::UniversalAzimuthSubscriber azimuthInput(...);
 * message_filters::Subscriber<sensor_msgs::msg::NavSatFix> fixInput(...);
 * auto converter = std::make_shared<compass_conversions::CompassConverter>(log, true);
 * // converter->configFromParams(params);
 * compass_conversions::CompassFilter filter(log, converter, azimuthInput, fixInput,
 *   compass_interfaces::msg::Azimuth::UNIT_RAD, compass_interfaces::msg::Azimuth::ORIENTATION_ENU,
 *   compass_interfaces::msg::Azimuth::REFERENCE_GEOGRAPHIC);
 * filter.registerCallback([](const compass_interfaces::msg::AzimuthConstPtr& msg) {
 *   ...  // Handle the data
 * });
 * \endcode
 */
class CompassFilter : public message_filters::SimpleFilter<compass_interfaces::msg::Azimuth>
{
public:
  typedef message_filters::MessageEvent<compass_interfaces::msg::Azimuth const> AzimuthEventType;
  typedef message_filters::MessageEvent<sensor_msgs::msg::NavSatFix const> FixEventType;
  typedef message_filters::MessageEvent<std_msgs::msg::Int32 const> UTMZoneEventType;

  /**
   * \brief Construct azimuth filter that can convert all parameters.
   *
   * \tparam AzimuthInput The type of the input filter.
   * \tparam FixInput The type of the navsat fix filter.
   * \tparam UTMZoneInput The type of the UTM Zone filter.
   * \param[in] log Logger.
   * \param[in] converter The azimuth converter instance. If nullptr, a default converter is constructed.
   * \param[in] azimuthInput The message filter producing azimuth messages.
   * \param[in] fixInput The message filter producing fix messages.
   * \param[in] utmZoneInput The message filter producing UTM zone messages.
   * \param[in] unit The output azimuth unit.
   * \param[in] orientation The output azimuth orientation.
   * \param[in] reference The output azimuth reference.
   */
  template<class AzimuthInput, class FixInput, class UTMZoneInput>
  CompassFilter(const rclcpp::Logger& log, const rclcpp::Clock& clock, const std::shared_ptr<CompassConverter>& converter,
    AzimuthInput& azimuthInput, FixInput& fixInput, UTMZoneInput& utmZoneInput,
    decltype(compass_interfaces::msg::Azimuth::unit) unit, decltype(compass_interfaces::msg::Azimuth::orientation) orientation,
    decltype(compass_interfaces::msg::Azimuth::reference) reference):
    log(log), converter(converter), unit(unit), orientation(orientation), reference(reference)
  {
    if (this->converter == nullptr)
      this->converter = std::make_shared<CompassConverter>(log, true);
    this->connectAzimuthInput(azimuthInput);
    this->connectFixInput(fixInput);
    this->connectUTMZoneInput(utmZoneInput);
  }

  /**
   * \brief Construct azimuth filter that can convert all parameters.
   *
   * \tparam AzimuthInput The type of the input filter.
   * \tparam FixInput The type of the navsat fix filter.
   * \param[in] log Logger.
   * \param[in] converter The azimuth converter instance. If nullptr, a default converter is constructed.
   * \param[in] azimuthInput The message filter producing azimuth messages.
   * \param[in] fixInput The message filter producing fix messages.
   * \param[in] unit The output azimuth unit.
   * \param[in] orientation The output azimuth orientation.
   * \param[in] reference The output azimuth reference.
   */
  template<class AzimuthInput, class FixInput>
  CompassFilter(const rclcpp::Logger& log, const rclcpp::Clock& clock, const std::shared_ptr<CompassConverter>& converter,
    AzimuthInput& azimuthInput, FixInput& fixInput,
    decltype(compass_interfaces::msg::Azimuth::unit) unit, decltype(compass_interfaces::msg::Azimuth::orientation) orientation,
    decltype(compass_interfaces::msg::Azimuth::reference) reference):
    log(log), converter(converter), unit(unit), orientation(orientation), reference(reference)
  {
    if (this->converter == nullptr)
      this->converter = std::make_shared<CompassConverter>(log, true);
    this->connectAzimuthInput(azimuthInput);
    this->connectFixInput(fixInput);
  }

  /**
   * \brief Construct azimuth filter that can only convert units and orientation.
   *
   * \tparam AzimuthInput The type of the input filter.
   * \param[in] log Logger.
   * \param[in] converter The azimuth converter instance. If nullptr, a default converter is constructed.
   * \param[in] azimuthInput The message filter producing azimuth messages.
   * \param[in] unit The output azimuth unit.
   * \param[in] orientation The output azimuth orientation.
   * \param[in] reference The output azimuth reference.
   */
  template<class AzimuthInput>
  CompassFilter(const rclcpp::Logger& log, const rclcpp::Clock& clock, const std::shared_ptr<CompassConverter>& converter,
    AzimuthInput& azimuthInput,
    decltype(compass_interfaces::msg::Azimuth::unit) unit, decltype(compass_interfaces::msg::Azimuth::orientation) orientation,
    decltype(compass_interfaces::msg::Azimuth::reference) reference):
    log(log), converter(converter), unit(unit), orientation(orientation), reference(reference)
  {
    if (this->converter == nullptr)
      this->converter = std::make_shared<CompassConverter>(log, true);
    this->connectAzimuthInput(azimuthInput);
  }

  virtual ~CompassFilter();

  template<class AzimuthInput>
  void connectAzimuthInput(AzimuthInput& f)
  {
    this->azimuthConnection.disconnect();
    // The explicit cast to boost:function is needed to retain the message event metadata
    this->azimuthConnection = f.registerCallback(
      boost::function<void(const AzimuthEventType&)>(std::bind_front(&CompassFilter::cbAzimuth, this)));
  }

  template<class FixInput>
  void connectFixInput(FixInput& f)
  {
    this->fixConnection.disconnect();
    // The explicit cast to boost:function is needed to retain the message event metadata
    this->fixConnection = f.registerCallback(
      boost::function<void(const FixEventType&)>(std::bind_front(&CompassFilter::cbFix, this)));
  }

  template<class UTMZoneInput>
  void connectUTMZoneInput(UTMZoneInput& f)
  {
    this->utmZoneConnection.disconnect();
    // The explicit cast to boost:function is needed to retain the message event metadata
    this->utmZoneConnection = f.registerCallback(
      boost::function<void(const UTMZoneEventType&)>(std::bind_front(&CompassFilter::cbUTMZone, this)));
  }

protected:
  virtual void cbAzimuth(const AzimuthEventType& azimuthEvent);
  virtual void cbFix(const FixEventType& fixEvent);
  virtual void cbUTMZone(const UTMZoneEventType& utmZoneEvent);

  message_filters::Connection azimuthConnection;  //!< Connection to the azimuth input.
  message_filters::Connection fixConnection;  //!< Connection to the navsat fix input.
  message_filters::Connection utmZoneConnection;  //!< Connection to the UTM zone input.

  std::shared_ptr<CompassConverter> converter;  //!< The compass converter instance.
  bool fixReceived {false};  //!< Whether at least one navsat fix message has been received.

  decltype(compass_interfaces::msg::Azimuth::unit) unit;  //!< The target azimuth unit.
  decltype(compass_interfaces::msg::Azimuth::orientation) orientation;  //!< The target azimuth orientation.

  //! The target azimuth reference (unchanged if empty).
  std::optional<decltype(compass_interfaces::msg::Azimuth::reference)> reference;
  const rclcpp::Logger& log;
  const rclcpp::Clock& clock;
  //const rclcpp::Node* node;
  //const std::string& topic {std::string()};
  //const uint32_t queueSize {10};
};

}
