#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Manager of magnetic field models.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include <memory>
#include <string>

#include "tl/expected.hpp"
#include <optional>
#include <magnetic_model/magnetic_model.h>
#include <rclcpp/time.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

namespace magnetic_model
{

struct MagneticModelManagerPrivate;

/**
 * \brief Earth magnetic model.
 */
class MagneticModelManager
{
public:

  /**
   * \brief Create magnetic model manager.
   *
   * \param[in] log The logger.
   * \param[in] modelPath Path to the folder with stored models. If nullopt, the default data distributed with this
   *                      package will be used. If empty string, a default system location will be used. The default
   *                      system location is determined by GeographicLib and can be influenced by setting environment
   *                      variables `GEOGRAPHICLIB_MAGNETIC_PATH` or `GEOGRAPHICLIB_DATA`.
   */
  explicit MagneticModelManager(const rclcpp::Node* node, const std::optional<std::string>& modelPath = {});
  virtual ~MagneticModelManager();

  /**
   * \brief Get the model path.
   * \return Path to the folder with stored models. If empty, a default system location will be used.
   */
  std::string getModelPath() const;

  /**
   * \brief Set the path to the folder with stored models.
   * \param[in] modelPath Path to the folder with stored models. If nullopt, the default data distributed with this
   *                      package will be used. If empty string, a default system location will be used. The default
   *                      system location is determined by GeographicLib and can be influenced by setting environment
   *                      variables `GEOGRAPHICLIB_MAGNETIC_PATH` or `GEOGRAPHICLIB_DATA`.
   */
  void setModelPath(const std::optional<std::string>& modelPath);

  /**
   * \brief Get the best magnetic model for the given date.
   * \param[in] date The date in question.
   * \return Name of the best magnetic model. If forcedMagneticModelName is non-empty, this value is always returned.
   */
  virtual std::string getBestMagneticModelName(const rclcpp::Time& date) const;

  /**
   * \brief Get the most suitable magnetic model for the given year.
   * \param[in] stamp The time for which model should be returned.
   * \param[in] strict Whether the returned model should fail if data outside its validity range are queried.
   * \return The magnetic model or error if there is no suitable model.
   */
  virtual tl::expected<std::shared_ptr<MagneticModel>, std::string> getMagneticModel(
    const rclcpp::Time& stamp, bool strict) const;

  /**
   * \brief Get the magnetic model with the given name.
   * \param[in] name Name of the model. Check MagneticModel constants to see possible values.
   * \param[in] strict Whether the returned model should fail if data outside its validity range are queried.
   * \return The magnetic model or error if it cannot be found.
   */
  virtual tl::expected<std::shared_ptr<MagneticModel>, std::string> getMagneticModel(
    const std::string& name, bool strict) const;

protected:
  //! \brief PIMPL data
  std::unique_ptr<MagneticModelManagerPrivate> data;
  const rclcpp::Node* node;
};

}
