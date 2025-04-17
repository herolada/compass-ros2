// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Visualize Azimuth as a PoseWithCovarianceStamped pointing to North.
 * \author Martin Pecka
 */

#include <memory>
#include <string>
#include <optional>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <compass_conversions/compass_converter.h>
#include <compass_conversions/message_filter.h>
#include <compass_interfaces/msg/azimuth.hpp>
#include <compass_utils/rate_limiter.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.h>
#include <message_filters/subscriber.h>
// #include <pluginlib/class_list_macros.hpp>
//#include <ros/ros.h>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp_components/register_node_macro.hpp"

namespace magnetometer_compass
{
using Az = compass_interfaces::msg::Azimuth;
using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
using Fix = sensor_msgs::msg::NavSatFix;
using Zone = std_msgs::msg::Int32;

/**
 * \brief Compute various azimuth values based on a magnetometer, IMU orientation and possibly also GPS coordinates.
 *
 * Subscribed topics:
 * - `~azimuth` (multiple types supported): The azimuth to visualize. Any type supported by compass_conversions package
 *                                          can be used: `compass_interfaces/Azimuth`, `geometry_msgs/Quaternion`,
 *                                          `geometry_msgs/PoseWithCovarianceStamped` or `sensor_msgs/Imu`. If other
 *                                          types than `compass_interfaces/Azimuth` are used, either the resolved topic name
 *                                          must contain the azimuth type identification (e.g. end with `mag/enu/imu`),
 *                                          or you must provide parameters `~input_reference` and `~input_orientation`.
 * - `gps/fix` (`sensor_msgs/NavSatFix`, optional): GPS fix messages from which the latitude, longitude, altitude and
 *                                                  current year can be read. These are further used to compute
 *                                                  magnetic declination and UTM grid convergence factor.
 * - `utm_zone` (`std_msgs/Int32`, optional): Optional UTM zone updates.
 *
 * Published topics (see above for explanation):
 * - `~azimuth_vis` (`sensor_msgs/MagneticField`, enabled by param `~publish_mag_unbiased`, off by default):
 *     The magnetic field measurement with bias removed.
 * 
 * Parameters:
 * - `max_rate` (double, optional): If specified, visualization messages frequency will be at most this value [Hz].
 * - `magnetic_declination` (double, radians, optional): If set, forces this value of magnetic declination.
 * - `utm_grid_convergence` (double, radians, optional): If set, forces this value of UTM grid convergence.
 * - `magnetic_models_path` (string, default "$PACKAGE/data/magnetic"): Path where WMM magnetic models can be found.
 *     If set to empty string, the models will be searched in a default folder of GeographicLib. Environment variables
 *     `GEOGRAPHICLIB_MAGNETIC_PATH` or `GEOGRAPHICLIB_DATA` influence the location of this folder.
 * - `magnetic_model` (string, optional): If set, forces using the given WMM model instead of determining the proper
 *                                        one by year. Example value is "wmm2020".
 * - `utm_zone` (int, optional): If set, forces using this UTM zone instead of determining the proper one.
 * - `keep_utm_zone` (bool, default true): If true, the first determined UTM zone will be used for all future
 *                                         conversions.
 * - `initial_lat` (double, degrees, optional): If set, use this latitude before the first navsat pose is received.
 * - `initial_lon` (double, degrees, optional): If set, use this longitude before the first navsat pose is received.
 * - `initial_alt` (double, meters, optional): If set, use this altitude before the first navsat pose is received.
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
 */
class VisualizeAzimuthNodelet : public rclcpp::Node
{
public:
  VisualizeAzimuthNodelet();
  VisualizeAzimuthNodelet(const rclcpp::NodeOptions & options);
  ~VisualizeAzimuthNodelet() override;

protected:

  void onInit();
  /**
   * \brief Callback Azimuth messages are received.
   * \param[in] azimuth The Azimuth converted to pose.
   */
  void azimuthCb(const Az& azimuth);

  std::unique_ptr<compass_utils::RateLimiter> rateLimiter;
  std::shared_ptr<compass_conversions::CompassConverter> converter;
  std::unique_ptr<compass_conversions::UniversalAzimuthSubscriber> azSub;
  std::unique_ptr<message_filters::Subscriber<Fix>> fixSub;
  std::unique_ptr<message_filters::Subscriber<Zone>> zoneSub;
  std::unique_ptr<compass_conversions::CompassFilter> filter;

  rclcpp::Publisher<Pose>::SharedPtr visPub;
};

VisualizeAzimuthNodelet::VisualizeAzimuthNodelet() : rclcpp::Node("visualize_azimuth_nodelet") {
  onInit();
};
VisualizeAzimuthNodelet::VisualizeAzimuthNodelet(const rclcpp::NodeOptions & options) : rclcpp::Node("visualize_azimuth_nodelet", options) {
  onInit();
};

VisualizeAzimuthNodelet::~VisualizeAzimuthNodelet() = default;

void VisualizeAzimuthNodelet::onInit()
{
  //auto nh = this->get_node_base_interface();
  //auto node = std::make_shared<rclcpp::Node>("handle_for_message_filter");
  
  // set rateLimiter
  double rate;
  if (this->has_parameter("max_rate"))
    this->get_parameter<double>("max_rate", rate);
    this->rateLimiter = std::make_unique<compass_utils::TokenBucketLimiter>(this->get_clock(), rclcpp::Rate(rate));

  // set compass converter
  this->converter = std::make_shared<compass_conversions::CompassConverter>(this->get_logger(), *this->get_clock(), true);
  this->converter->configFromParams(shared_from_this());

  // publisher
  this->visPub = this->create_publisher<Pose>("azimuth_vis", 10);

  // subscribe azimuth, gps fix, utm_zone
  this->azSub = std::make_unique<compass_conversions::UniversalAzimuthSubscriber>(shared_from_this(), "azimuth", 100);
  this->azSub->configFromParams(shared_from_this());

  this->fixSub = std::make_unique<message_filters::Subscriber<Fix>>(shared_from_this(), "gps/fix");//, 10);
  this->zoneSub = std::make_unique<message_filters::Subscriber<Zone>>(shared_from_this(), "utm_zone");//, 10);

  // set compass filter
  this->filter = std::make_unique<compass_conversions::CompassFilter>(
    this->get_logger(), *this->get_clock(), this->converter, *this->azSub, *this->fixSub, *this->zoneSub,
    Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  this->filter->registerCallback(&VisualizeAzimuthNodelet::azimuthCb, this);

  RCLCPP_INFO(this->get_logger(), "Visualizing azimuth messages from [%s] on topic [%s]",
    this->azSub->getTopic().c_str(), this->visPub->get_topic_name());
  
}

/* void VisualizeAzimuthNodelet::onInit()
{  
  cras::Nodelet::onInit();

  auto nh = this->getNodeHandle();
  auto pnh = this->getPrivateNodeHandle();

  const auto params = this->privateParams();

  // set rateLimiter
   if (params->hasParam("max_rate"))
    this->rateLimiter = std::make_unique<compass_utils::TokenBucketLimiter>(
      params->getParam<rclcpp::Rate>("max_rate", std::nullopt));

  // set compass converter
  this->converter = std::make_shared<compass_conversions::CompassConverter>(this->getLogger(), true);
  this->converter->configFromParams(*params);

  // publisher
  this->visPub = pnh.advertise<Pose>("azimuth_vis", 10);

  // subscribe azimuth, gps fix, utm_zone
  this->azSub = std::make_unique<compass_conversions::UniversalAzimuthSubscriber>(this->get_logger(), pnh, "azimuth", 100);
  this->azSub->configFromParams(*params);
  this->fixSub = std::make_unique<message_filters::Subscriber<Fix>>(nh, "gps/fix", 10);
  this->zoneSub = std::make_unique<message_filters::Subscriber<Zone>>(nh, "utm_zone", 10);

  // set compass filter
  this->filter = std::make_unique<compass_conversions::CompassFilter>(
      this->get_logger(), this->converter, *this->azSub, *this->fixSub, *this->zoneSub,
      Az::UNIT_RAD, Az::ORIENTATION_ENU, Az::REFERENCE_UTM);
  this->filter->registerCallback(&VisualizeAzimuthNodelet::azimuthCb, this);

  RCLCPP_INFO(this->get_logger(), "Visualizing azimuth messages from [%s] on topic [%s]",
    this->azSub->getTopic().c_str(), this->visPub.getTopic().c_str());
} */

void VisualizeAzimuthNodelet::azimuthCb(const Az& azimuthEast)
{
  if (this->rateLimiter != nullptr && !this->rateLimiter->shouldPublish(azimuthEast.header.stamp))
    return;

  auto azimuthNorth = azimuthEast;
  azimuthNorth.azimuth -= M_PI / 2;

  const auto maybePose = this->converter->convertToPose(azimuthNorth);
  if (!maybePose.has_value())
  {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 10.0, "Visualizing azimuth failed: %s", maybePose.error().c_str());
    return;
  }

  auto pose = *maybePose;
  // Invert the orientation. Normally, azimuth tells us the direction of local X axis from North. However, for
  // visualization, we want to get a pose in local frame that points to North.
  tf2::Quaternion q;
  tf2::convert(pose.pose.pose.orientation, q);
  tf2::convert(q.inverse(), pose.pose.pose.orientation);
  // The infinite covariances yielded by converter do not play well with RViz
  pose.pose.covariance[0 * 6 + 0] = 0;
  pose.pose.covariance[1 * 6 + 1] = 0;
  pose.pose.covariance[2 * 6 + 2] = 0;
  this->visPub->publish(pose);
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(magnetometer_compass::VisualizeAzimuthNodelet)