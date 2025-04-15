#ifndef COMPASS_CONVERSIONS__COMPASS_TRANSFORMER_NODELET_HPP_
#define COMPASS_CONVERSIONS__COMPASS_TRANSFORMER_NODELET_HPP_

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/buffer.h>
#include <message_filters/subscriber.h>

#include <compass_conversions/message_filter.h>
#include <compass_conversions/tf2_compass_msgs.h>
#include <compass_conversions/topic_names.h>
#include <compass_utils/string_utils.hpp>
#include <compass_interfaces/msg/azimuth.hpp>

namespace compass_conversions
{

enum class OutputType
{
  Azimuth,
  Imu,
  Pose,
  Quaternion,
};

OutputType parseOutputType(const std::string& outputType);
std::string outputTypeToString(const OutputType type);

class CompassTransformerNodelet : public rclcpp::Node
{
public:
  CompassTransformerNodelet();

  void setBuffer(tf2_ros::Buffer::SharedPtr buffer);

protected:
  void publish(const compass_interfaces::msg::Azimuth::ConstPtr& msg);
  void transformAndPublish(const compass_interfaces::msg::Azimuth::ConstPtr& msg);
  void failedCb(const compass_interfaces::msg::Azimuth::ConstPtr& msg,
                const tf2_ros::filter_failure_reasons::FilterFailureReason reason);

  std::shared_ptr<CompassConverter> converter;
  std::unique_ptr<UniversalAzimuthSubscriber> azimuthInput;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>> fixInput;
  std::unique_ptr<message_filters::Subscriber<std_msgs::msg::Int32>> utmZoneInput;
  std::unique_ptr<CompassFilter> compassFilter;
  std::unique_ptr<tf2_ros::MessageFilter<compass_interfaces::msg::Azimuth>> tfFilter;

  rclcpp::Publisher<compass_interfaces::msg::Azimuth>::SharedPtr pub_az;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub_quat;

  std::string targetFrame;
  OutputType targetType {OutputType::Azimuth};
  tf2_ros::Buffer::SharedPtr buffer;
  // std::map< std::string, std::string > remaps {};
};

}  // namespace compass_conversions

#endif  // COMPASS_CONVERSIONS__COMPASS_TRANSFORMER_NODELET_HPP_
