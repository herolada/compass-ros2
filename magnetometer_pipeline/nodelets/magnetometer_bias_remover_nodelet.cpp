// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Remove known bias from 3-axis magnetometer.
 * \author Martin Pecka
 */

#include <memory>
#include <functional>
// #include <cras_cpp_common/nodelet_utils.hpp>
#include <magnetometer_pipeline/message_filter.h>
#include <magnetometer_pipeline/magnetometer_bias_remover_nodelet.hpp>
#include <message_filters/subscriber.h>
// #include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

#include <rclcpp_components/register_node_macro.hpp>


namespace magnetometer_pipeline
{
using Field = sensor_msgs::msg::MagneticField;

MagnetometerBiasRemoverNodelet::MagnetometerBiasRemoverNodelet() : rclcpp::Node("magnetometer_bias_remover_nodelet") {
  onInit();  
};

MagnetometerBiasRemoverNodelet::MagnetometerBiasRemoverNodelet(const rclcpp::NodeOptions & options) 
  : rclcpp::Node("magnetometer_bias_remover_nodelet", options) {
    onInit();
};

void MagnetometerBiasRemoverNodelet::onInit() {
  this->declare_parameter<double>("initial_mag_bias_x", 0.0);
  this->declare_parameter<double>("initial_mag_bias_y", 0.0);
  this->declare_parameter<double>("initial_mag_bias_z", 0.0);
  this->declare_parameter<std::vector<double>>("initial_mag_scaling_matrix", std::vector<double>(9, 0.0));

  rclcpp::Node::SharedPtr topicNh = this->create_sub_node("imu");

  this->magUnbiasedPub = topicNh->create_publisher<Field>("mag_unbiased", 10);

  this->magSub = std::make_unique<message_filters::Subscriber<Field>>(topicNh, "mag");//, 100);
  this->magBiasSub = std::make_unique<message_filters::Subscriber<Field>>(topicNh, "mag_bias");//, 10);

  this->remover = std::make_unique<BiasRemoverFilter>(this, *this->magSub, *this->magBiasSub);
  this->remover->configFromParams();
  this->remover->registerCallback(//[this](const Field& msg) {this->magUnbiasedPub->publish(msg);});
    std::function<void(const Field&)>(std::bind_front(&MagnetometerBiasRemoverNodelet::cb, this)));
};

void MagnetometerBiasRemoverNodelet::cb (const Field& msg) {
  this->magUnbiasedPub->publish(msg);
};

MagnetometerBiasRemoverNodelet::~MagnetometerBiasRemoverNodelet() = default;

}
RCLCPP_COMPONENTS_REGISTER_NODE(magnetometer_pipeline::MagnetometerBiasRemoverNodelet)


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<magnetometer_pipeline::MagnetometerBiasRemoverNodelet>());
  rclcpp::shutdown();
  return 0;
}