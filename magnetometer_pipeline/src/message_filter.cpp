// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Message filter to remove bias from 3-axis magnetometer measurements.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

// #include <boost/make_shared.hpp>

// #include <cras_cpp_common/log_utils.h>
#include <magnetometer_pipeline/message_filter.h>
#include <message_filters/message_event.h>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rcutils/logging_macros.h"
#include "rcutils/error_handling.h"
#include "rcutils/logging.h"

namespace magnetometer_pipeline
{

using Field = sensor_msgs::msg::MagneticField;

BiasRemoverFilter::~BiasRemoverFilter() = default;

void BiasRemoverFilter::configFromParams()
{
  this->remover->configFromParams(this->node);
}

void BiasRemoverFilter::cbMag(const message_filters::MessageEvent<Field const>& event)
{
  const auto maybeMagUnbiased = this->remover->removeBias(*event.getConstMessage());
  if (!maybeMagUnbiased.has_value())
  {
    // auto log = rclcpp::get_logger("BiasRemoverFilter");
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(this->node->get_logger(), *this->node->get_clock(), 10000., "Bias remover cannot work: %s. Waiting...", maybeMagUnbiased.error().c_str());
    return;
  }

  // const auto header = event.getConstMessage()->header;
  const auto stamp = event.getReceiptTime();
  this->signalMessage(message_filters::MessageEvent<Field const>(
    std::make_shared<Field const>(*maybeMagUnbiased), stamp, false, message_filters::DefaultMessageCreator<Field>()));
}

void BiasRemoverFilter::cbBias(const message_filters::MessageEvent<Field const>& event)
{
  this->remover->setBias(*event.getConstMessage());
}

}
