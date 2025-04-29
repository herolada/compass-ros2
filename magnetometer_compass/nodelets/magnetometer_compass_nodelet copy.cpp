// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Compute various azimuth values based on a magnetometer, IMU orientation and possibly also GPS coordinates.
 * \author Martin Pecka
 */

#include <memory>
#include <string>

#include <compass_conversions/compass_converter.h>
#include <compass_conversions/topic_names.h>
#include <compass_interfaces/msg/azimuth.hpp>
#include <compass_utils/string_utils.hpp>
#include <compass_utils/tf2_utils.hpp>
//#include <cras_cpp_common/nodelet_utils.hpp>
//#include <cras_cpp_common/tf2_utils.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
// #include <imu_transformer/tf2_sensor_msgs.h>
#include <magnetometer_compass/tf2_sensor_msgs.h>
#include <magnetometer_compass/magnetometer_compass.h>
#include <magnetometer_compass/magnetometer_compass_nodelet.hpp>
#include <magnetometer_pipeline/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// #include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
// #include <rclcpp/rclcpp.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include <tf2_ros/buffer.h>

namespace magnetometer_compass
{

using Az = compass_interfaces::msg::Azimuth;
using Quat = geometry_msgs::msg::QuaternionStamped;
using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
using Imu = sensor_msgs::msg::Imu;
using Field = sensor_msgs::msg::MagneticField;

typedef message_filters::sync_policies::ApproximateTime<Imu, Field> SyncPolicy;


MagnetometerCompassNodelet::MagnetometerCompassNodelet() : rclcpp::Node("magnetometer_compass_nodelet"),
  magPublishers(), truePublishers(), utmPublishers(), buffer(std::make_shared<tf2_ros::Buffer>(this->get_clock()))
{
}

MagnetometerCompassNodelet::MagnetometerCompassNodelet(const rclcpp::NodeOptions & options) : rclcpp::Node("magnetometer_compass_nodelet", options),
  magPublishers(), truePublishers(), utmPublishers(), buffer(std::make_shared<tf2_ros::Buffer>(this->get_clock()))
{
}

void MagnetometerCompassNodelet::setBuffer(tf2_ros::Buffer::SharedPtr buffer)
{
  this->buffer = buffer;
}


void MagnetometerCompassNodelet::init()
{
  //  auto nh = this->getNodeHandle(); //topics bez prefixu
  //  auto pnh = this->getPrivateNodeHandle(); //parameters
  //auto node = std::make_shared<rclcpp::Node>("handle_for_message_filter"); // nahradit this nebo reference original objektu
  //  'subnode' pro sub name space(?)

  //  auto params = this->privateParams();

  //  this->frame = params->getParam("frame", "base_link");

  rclcpp::Node::SharedPtr imuNh = this->create_sub_node("imu");
  rclcpp::Node::SharedPtr compassNh = this->create_sub_node("compass");

  this->frame = this->get_parameter_or<std::string>("frame", "base_link");

  const auto strict = this->get_parameter_or<bool>("strict", true);
  this->converter = std::make_shared<compass_conversions::CompassConverter>(this, strict);
  this->converter->configFromParams();

  // this->buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  this->compass = std::make_shared<MagnetometerCompass>(this, this->frame, this->buffer);
  this->compass->configFromParams();

  this->publishMagUnbiased = this->get_parameter_or<bool>("publish_mag_unbiased", this->publishMagUnbiased);
  this->subscribeMagUnbiased = this->get_parameter_or<bool>("subscribe_mag_unbiased", this->subscribeMagUnbiased);

  if (this->publishMagUnbiased && this->subscribeMagUnbiased)
    throw std::runtime_error("Cannot simultaneously subscribe and publish unbiased magnetometer.");

  // Set default publishers
  this->magPublishers.ned.publishDeg = true;

  bool publish = this->publishMagUnbiased;

  // ros::NodeHandle compassNh(nh, "compass");
  // auto compass_node = std::make_shared<rclcpp::Node>("compass");
  // Read the publish_* parameters and set up the selected publishers
  // TODO better deal with the nh X pnh ...
  this->magPublishers.init(compassNh, this, this->converter, "publish", "", Az::REFERENCE_MAGNETIC, "mag");
  publish |= this->magPublishers.publish;
  this->truePublishers.init(compassNh, this, this->converter, "publish", "", Az::REFERENCE_GEOGRAPHIC, "true");
  publish |= this->truePublishers.publish;
  this->utmPublishers.init(compassNh, this, this->converter, "publish", "", Az::REFERENCE_UTM, "utm");
  publish |= this->utmPublishers.publish;

  if (!publish)
    RCLCPP_WARN(this->get_logger(), "No publishers have been requested. Please, set one of the publish_* parameters to true.");

  // ros::NodeHandle imuNh(nh, "imu");
  // auto compass_node = std::make_shared<rclcpp::Node>("imu");

  if (this->publishMagUnbiased)
    this->magUnbiasedPub = imuNh->create_publisher<Field>("mag_unbiased", 10);

  this->imuSub = std::make_unique<message_filters::Subscriber<Imu>>(imuNh, "data");//, 100);

  // Check if we should try to unbias the magnetometer ourselves or if you already got it unbiased on the input.
  if (this->subscribeMagUnbiased)
  {
    this->magSub = std::make_unique<message_filters::Subscriber<Field>>(imuNh, "mag_unbiased");//, 100);
    this->syncSub = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(200), *this->imuSub, *this->magSub);
  }
  else
  {
    this->magSub = std::make_unique<message_filters::Subscriber<Field>>(imuNh, "mag");//, 100);
    this->magBiasSub = std::make_unique<message_filters::Subscriber<Field>>(imuNh, "mag_bias");//, 10);
    this->magBiasRemoverFilter = std::make_unique<magnetometer_pipeline::BiasRemoverFilter>(
      this, *this->magSub, *this->magBiasSub);
    this->magBiasRemoverFilter->configFromParams();

    this->syncSub = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(200), *this->imuSub, *this->magBiasRemoverFilter);
  }
  this->syncSub->registerCallback(&MagnetometerCompassNodelet::imuMagCb, this);

  // this->fixSub = this->subscribe("gps/fix", 10, &MagnetometerCompassNodelet::fixCb, this);
  this->fixSub = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps/fix", 10, [this] (const sensor_msgs::msg::NavSatFix& msg) {this->fixCb(msg);});
};

MagnetometerCompassNodelet::~MagnetometerCompassNodelet() = default;

/* void MagnetometerCompassNodelet::onInit()
{
  cras::Nodelet::onInit();

  auto nh = this->getNodeHandle();
  auto pnh = this->getPrivateNodeHandle();

  auto params = this->privateParams();

  this->frame = params->getParam("frame", "base_link");

  const auto strict = params->getParam("strict", true);
  this->converter = std::make_shared<compass_conversions::CompassConverter>(this->getLogger(), strict);
  this->converter->configFromParams(*params);

  this->compass = std::make_shared<MagnetometerCompass>(this->get_logger(), this->frame, this->getBufferPtr());
  this->compass->configFromParams(*params);

  this->publishMagUnbiased = params->getParam("publish_mag_unbiased", this->publishMagUnbiased);
  this->subscribeMagUnbiased = params->getParam("subscribe_mag_unbiased", this->subscribeMagUnbiased);

  if (this->publishMagUnbiased && this->subscribeMagUnbiased)
    throw std::runtime_error("Cannot simultaneously subscribe and publish unbiased magnetometer.");

  // Set default publishers
  this->magPublishers.ned.publishDeg = true;

  bool publish = this->publishMagUnbiased;

  ros::NodeHandle compassNh(nh, "compass");
  // Read the publish_* parameters and set up the selected publishers
  this->magPublishers.init(compassNh, pnh, this->converter, "publish", "", Az::REFERENCE_MAGNETIC, "mag");
  publish |= this->magPublishers.publish;
  this->truePublishers.init(compassNh, pnh, this->converter, "publish", "", Az::REFERENCE_GEOGRAPHIC, "true");
  publish |= this->truePublishers.publish;
  this->utmPublishers.init(compassNh, pnh, this->converter, "publish", "", Az::REFERENCE_UTM, "utm");
  publish |= this->utmPublishers.publish;

  if (!publish)
    RCLCPP_WARN(this->get_logger(), "No publishers have been requested. Please, set one of the publish_* parameters to true.");

  ros::NodeHandle imuNh(nh, "imu");

  if (this->publishMagUnbiased)
    this->magUnbiasedPub = imuNh.advertise<Field>("mag_unbiased", 10);

  this->imuSub = std::make_unique<message_filters::Subscriber<Imu>>(imuNh, "data", 100);

  // Check if we should try to unbias the magnetometer ourselves or if you already got it unbiased on the input.
  if (this->subscribeMagUnbiased)
  {
    this->magSub = std::make_unique<message_filters::Subscriber<Field>>(imuNh, "mag_unbiased", 100);
    this->syncSub = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(200), *this->imuSub, *this->magSub);
  }
  else
  {
    this->magSub = std::make_unique<message_filters::Subscriber<Field>>(imuNh, "mag", 100);
    this->magBiasSub = std::make_unique<message_filters::Subscriber<Field>>(imuNh, "mag_bias", 10);
    this->magBiasRemoverFilter = std::make_unique<magnetometer_pipeline::BiasRemoverFilter>(
      this->getLogger(), *this->magSub, *this->magBiasSub);
    this->magBiasRemoverFilter->configFromParams(*params);

    this->syncSub = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(200), *this->imuSub, *this->magBiasRemoverFilter);
  }
  this->syncSub->registerCallback(&MagnetometerCompassNodelet::imuMagCb, this);

  this->fixSub = nh.subscribe("gps/fix", 10, &MagnetometerCompassNodelet::fixCb, this);
} */

AzimuthPublishersConfigForOrientation::AzimuthPublishersConfigForOrientation()
{
};

void AzimuthPublishersConfigForOrientation::init(
  rclcpp::Node::SharedPtr namespace_node, rclcpp::Node* param_node, 
  const std::shared_ptr<compass_conversions::CompassConverter>& converter,
  const std::string& paramPrefix, const std::string& topicPrefix, const uint8_t reference, const uint8_t orientation,
  const std::string& referenceStr, const std::string& orientationStr)
{
  this->namespace_node = namespace_node;
  this->param_node = param_node;
  this->converter = converter;

  auto prefix = paramPrefix + "_" + referenceStr + "_azimuth_" + orientationStr + "_";

  this->publishQuat = param_node->get_parameter_or<bool>(prefix + "quat", this->publishQuat);
  this->publishImu = param_node->get_parameter_or<bool>(prefix + "imu", this->publishImu);
  this->publishPose = param_node->get_parameter_or<bool>(prefix + "pose", this->publishPose);
  this->publishRad = param_node->get_parameter_or<bool>(prefix + "rad", this->publishRad);
  this->publishDeg = param_node->get_parameter_or<bool>(prefix + "deg", this->publishDeg);

  this->publish = this->publishQuat || this->publishImu || this->publishPose || this->publishDeg || this->publishRad;

  using compass_conversions::getAzimuthTopicSuffix;

  prefix = compass_utils::appendIfNonEmpty(topicPrefix, "/");

  if (this->publishQuat)
    // this->quatPub = nh.advertise<Quat>(prefix + getAzimuthTopicSuffix<Quat>(Az::UNIT_RAD, orientation, reference), 10);
    this->quatPub = namespace_node->create_publisher<Quat>(prefix + getAzimuthTopicSuffix<Quat>(Az::UNIT_RAD, orientation, reference), 10);
  if (this->publishImu)
    this->imuPub = namespace_node->create_publisher<Imu>(prefix + getAzimuthTopicSuffix<Imu>(Az::UNIT_RAD, orientation, reference), 10);
  if (this->publishPose)
    this->posePub = namespace_node->create_publisher<Pose>(prefix + getAzimuthTopicSuffix<Pose>(Az::UNIT_RAD, orientation, reference), 10);
  if (this->publishRad) {
    this->radPub = namespace_node->create_publisher<Az>(prefix + getAzimuthTopicSuffix<Az>(Az::UNIT_RAD, orientation, reference), 10);
  }
  if (this->publishDeg) {
    this->degPub = namespace_node->create_publisher<Az>(prefix + getAzimuthTopicSuffix<Az>(Az::UNIT_DEG, orientation, reference), 10);
  }
};

AzimuthPublishersConfig::AzimuthPublishersConfig() :
  ned(), enu()
{
};

void AzimuthPublishersConfig::init(
  rclcpp::Node::SharedPtr namespace_node, rclcpp::Node* param_node, 
  const std::shared_ptr<compass_conversions::CompassConverter>& converter,
  const std::string& paramPrefix, const std::string& topicPrefix,
  const uint8_t reference, const std::string& referenceStr)
{
  this->namespace_node = namespace_node;
  this->param_node = param_node;
  this->converter = converter;
  this->ned.init(namespace_node, param_node, converter, paramPrefix, topicPrefix, reference, Az::ORIENTATION_NED, referenceStr, "ned");
  this->enu.init(namespace_node, param_node, converter, paramPrefix, topicPrefix, reference, Az::ORIENTATION_ENU, referenceStr, "enu");
  this->publish = this->ned.publish || this->enu.publish;
};

void MagnetometerCompassNodelet::imuMagCb(const Imu& imu, const Field& magUnbiased)
{
  if (this->publishMagUnbiased) {
    this->magUnbiasedPub->publish(magUnbiased);
  }
  const auto maybeAzimuth = this->compass->computeAzimuth(imu, magUnbiased);
  if (!maybeAzimuth.has_value())
  {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 1000., "%s", maybeAzimuth.error().c_str());
    return;
  }

  Imu imuInBody;
  try
  {
    // No timeout because computeAzimuth() has already waited for this exact transform
    this->buffer->transform(imu, imuInBody, this->frame);
  }
  catch (const tf2::TransformException& e)
  {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 1000.,
      "Could not transform IMU data to frame %s because: %s", this->frame.c_str(), e.what());
    return;
  }

  const auto& nedAzimuthMsg = *maybeAzimuth;
  this->magPublishers.publishAzimuths(nedAzimuthMsg, imuInBody);

  if (this->truePublishers.publish)
  {
    const auto maybeTrueNedAzimuthMsg = this->converter->convertAzimuth(
      nedAzimuthMsg, nedAzimuthMsg.unit, nedAzimuthMsg.orientation, Az::REFERENCE_GEOGRAPHIC);
    if (maybeTrueNedAzimuthMsg) {
      this->truePublishers.publishAzimuths(*maybeTrueNedAzimuthMsg, imuInBody);
    }
    else {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 1000., "%s", maybeTrueNedAzimuthMsg.error().c_str());
    }
  }

  if (this->utmPublishers.publish)
  {
    const auto maybeUTMNedAzimuthMsg = this->converter->convertAzimuth(
      nedAzimuthMsg, nedAzimuthMsg.unit, nedAzimuthMsg.orientation, Az::REFERENCE_UTM);
    if (maybeUTMNedAzimuthMsg.has_value()) {
      this->utmPublishers.publishAzimuths(*maybeUTMNedAzimuthMsg, imuInBody);
    }
    else {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(this->get_logger(), *this->get_clock(), 1000., "%s", maybeUTMNedAzimuthMsg.error().c_str());
    }
  }
};

void AzimuthPublishersConfig::publishAzimuths(const Az& nedAzimuth, const Imu& imuInBody)
{
  if (!this->publish)
    return;

  if (this->ned.publish)
  {
    auto imuNed = imuInBody;  // If IMU message should not be published, we fake it here with the ENU-referenced one
    if (this->ned.publishImu)
    {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = imuInBody.header.stamp;
      tf.header.frame_id = imuInBody.header.frame_id + "_ned";
      tf2::convert(this->enuToNed, tf.transform.rotation);
      tf2::doTransform(imuInBody, imuNed, tf);
    }
    this->ned.publishAzimuths(nedAzimuth, imuNed);
  }

  if (this->enu.publish)
  {
    // Rotate to ENU
    auto maybeEnuAzimuth = this->converter->convertAzimuth(
      nedAzimuth, nedAzimuth.unit, Az::ORIENTATION_ENU, nedAzimuth.reference);

    if (maybeEnuAzimuth.has_value())
      this->enu.publishAzimuths(*maybeEnuAzimuth, imuInBody);
    else
      RCLCPP_ERROR_THROTTLE(namespace_node->get_logger(), *namespace_node->get_clock(), 1000., "Could not convert from NED to ENU: %s", maybeEnuAzimuth.error().c_str());
  }
};

void AzimuthPublishersConfigForOrientation::publishAzimuths(const Az& azimuthRad, const Imu& imuInBody)
{
  if (this->publishQuat)
  {
    const auto maybeQuat = this->converter->convertToQuaternion(azimuthRad);
    if (maybeQuat.has_value())
      this->quatPub->publish(*maybeQuat);
    else
      RCLCPP_ERROR_THROTTLE(namespace_node->get_logger(), *namespace_node->get_clock(), 1000., "%s", maybeQuat.error().c_str());
  }

  if (this->publishImu)
  {
    const auto maybeQuat = this->converter->convertToQuaternion(azimuthRad);
    if (!maybeQuat.has_value())
    {
      RCLCPP_ERROR_THROTTLE(namespace_node->get_logger(), *namespace_node->get_clock(), 1000., "%s", maybeQuat.error().c_str());
    }
    else
    {
      // The IMU message comes in an arbitrarily-referenced frame, and we adjust its yaw to become georeferenced.
      double azimuthYaw = compass_utils::getYaw(maybeQuat->quaternion);

      tf2::Quaternion imuRot;
      tf2::convert(imuInBody.orientation, imuRot);
      double roll, pitch, yaw;
      compass_utils::getRPY(imuRot, roll, pitch, yaw);

      tf2::Quaternion desiredRot;
      desiredRot.setRPY(roll, pitch, azimuthYaw);

      const auto diffRot = desiredRot.inverse() * imuRot;

      sensor_msgs::msg::Imu imuMsg;

      geometry_msgs::msg::TransformStamped tf;
      tf.header = imuInBody.header;
      tf2::convert(diffRot, tf.transform.rotation);
      tf2::doTransform(imuInBody, imuMsg, tf);

      imuMsg.orientation_covariance[8] = azimuthRad.variance;

      this->imuPub->publish(imuMsg);
    }
  }

  if (this->publishPose)
  {
    const auto maybePose = this->converter->convertToPose(azimuthRad);
    if (maybePose.has_value())
      this->posePub->publish(*maybePose);
    else
      RCLCPP_ERROR_THROTTLE(namespace_node->get_logger(), *namespace_node->get_clock(), 1000., "%s", maybePose.error().c_str());
  }

  if (this->publishRad)
  {
    this->radPub->publish(azimuthRad);
  }

  if (this->publishDeg)
  {
    const auto maybeAzimuthDeg = this->converter->convertAzimuth(
      azimuthRad, Az::UNIT_DEG, azimuthRad.orientation, azimuthRad.reference);
    if (maybeAzimuthDeg.has_value())
      this->degPub->publish(*maybeAzimuthDeg);
    else
      RCLCPP_ERROR_THROTTLE(namespace_node->get_logger(), *namespace_node->get_clock(), 1000., "%s", maybeAzimuthDeg.error().c_str());
  }
};

void MagnetometerCompassNodelet::fixCb(const sensor_msgs::msg::NavSatFix& fix)
{
  this->converter->setNavSatPos(fix);
};

}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(magnetometer_compass::MagnetometerCompassNodelet, rclcpp::Node)
RCLCPP_COMPONENTS_REGISTER_NODE(magnetometer_compass::MagnetometerCompassNodelet)