// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Remove known bias from 3-axis magnetometer.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include "tl/expected.hpp"
#include <Eigen/Core>
#include <compass_conversions/tf2_compass_msgs.h>
#include <compass_utils/string_utils.hpp>
#include <magnetometer_pipeline/bias_remover.h>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>

namespace magnetometer_pipeline
{
using Field = sensor_msgs::msg::MagneticField;

struct MagnetometerBiasRemoverPrivate
{
  //! \brief Whether magnetometer bias has already been established either from subscriber or initial parameters.
  bool hasBias{false};

  //! \brief Whether magnetometer scale has already been established either from subscriber or initial parameters and
  //!        it is different from identity.
  bool hasScale{false};

  //! \brief Last received value of magnetometer bias.
  Eigen::Vector3d lastBias{0, 0, 0};

  //! \brief Scaling factor of magnetometer measurements (optional).
  Eigen::Matrix3d lastScale{Eigen::Matrix3d::Identity()};
};

MagnetometerBiasRemover::MagnetometerBiasRemover() :
  data(new MagnetometerBiasRemoverPrivate{})
{
}

MagnetometerBiasRemover::~MagnetometerBiasRemover() = default;

void MagnetometerBiasRemover::configFromParams(const rclcpp::Node* node)
{
  if ((node->has_parameter("initial_mag_bias_x") && node->get_parameter("initial_mag_bias_x").get_value<double>() != -1.) ||
      (node->has_parameter("initial_mag_bias_y") && node->get_parameter("initial_mag_bias_y").get_value<double>() != -1.) ||
      (node->has_parameter("initial_mag_bias_z") && node->get_parameter("initial_mag_bias_z").get_value<double>() != -1.))
  {
    sensor_msgs::msg::MagneticField msg;
    msg.magnetic_field.x = node->get_parameter_or<double>("initial_mag_bias_x", 0.0);
    msg.magnetic_field.y = node->get_parameter_or<double>("initial_mag_bias_y", 0.0);
    msg.magnetic_field.z = node->get_parameter_or<double>("initial_mag_bias_z", 0.0);

    std::vector<double> scaling_vec;
    if (node->has_parameter("initial_mag_scaling_matrix") &&
        node->get_parameter("initial_mag_scaling_matrix").get_value<std::vector<double>>().size() != 1) {
      node->get_parameter("initial_mag_scaling_matrix", scaling_vec);
    } else {
      scaling_vec = std::vector<double>(this->data->lastScale.data(), this->data->lastScale.data() + 9);
    }
    
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> scalingMatrix(msg.magnetic_field_covariance.data());
    if (scaling_vec.size() == 9) {
        scalingMatrix = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(scaling_vec.data());
    } else {
        scalingMatrix = this->data->lastScale;
    }

    this->setBias(msg);

    RCLCPP_INFO(node->get_logger(), "Initial magnetometer bias is %0.3f %0.3f %0.3f %s scaling factor",
      msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z, this->hasScale() ? "with" : "without");
  }
}

bool MagnetometerBiasRemover::hasBias() const
{
  return this->data->hasBias;
}

bool MagnetometerBiasRemover::hasScale() const
{
  return this->data->hasScale;
}

tl::expected<Field, std::string> MagnetometerBiasRemover::removeBias(const Field& mag)
{
  if (!this->data->hasBias)
    return compass_utils::make_unexpected("Magnetometer bias not available.");

  Eigen::Vector3d field;
  tf2::fromMsg(mag.magnetic_field, field);
  field = this->data->lastScale * (field - this->data->lastBias);

  Field magUnbiased = mag;
  magUnbiased.magnetic_field.x = field.x();
  magUnbiased.magnetic_field.y = field.y();
  magUnbiased.magnetic_field.z = field.z();

  return magUnbiased;
}

void MagnetometerBiasRemover::setBias(const Field& bias)
{
  tf2::fromMsg(bias.magnetic_field, this->data->lastBias);
  this->data->hasBias = true;

  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> covMatrix(bias.magnetic_field_covariance.data());
  if (covMatrix.cwiseAbs().sum() > 1e-10)
    this->data->lastScale = covMatrix;
  this->data->hasScale = (this->data->lastScale - Eigen::Matrix3d::Identity()).cwiseAbs().sum() > 1e-10;
}

}
