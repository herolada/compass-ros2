// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Earth magnetic field model.
 * \author Martin Pecka
 */

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <format>

#include <GeographicLib/MagneticModel.hpp>

#include "tl/expected.hpp"
#include <optional>
#include <magnetic_model/magnetic_model.h>
#include <magnetic_model/magnetic_model_manager.h>
//#include <ros/package.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/logger.hpp>
#include "rclcpp/rclcpp.hpp"
//#include "rcutils/error_handling.h"
#include <compass_utils/string_utils.hpp>
#include <compass_utils/time_utils.hpp>

namespace magnetic_model
{

/**
 * \brief Private data of MagneticModelManager.
 */
struct MagneticModelManagerPrivate
{
  //! \brief Cache of already initialized magnetic field models. Keys are model names/strictness.
  std::map<std::pair<std::string, bool>, std::shared_ptr<MagneticModel>> magneticModels;

  //! \brief Path to the models on disk. Empty means system default.
  std::string modelPath;
};

MagneticModelManager::MagneticModelManager(const rclcpp::Logger& log, const rclcpp::Clock& clock, const std::optional<std::string>& modelPath):
  log(log), clock(clock), data(new MagneticModelManagerPrivate{}) //logger(std::make_unique<rclcpp::Logger>())
{
  this->setModelPath(modelPath);
}

MagneticModelManager::~MagneticModelManager() = default;

std::string MagneticModelManager::getModelPath() const
{
  return this->data->modelPath;
}

void MagneticModelManager::setModelPath(const std::optional<std::string>& modelPath)
{
  if (modelPath.has_value())
  {
    if (modelPath->empty())
      this->data->modelPath = GeographicLib::MagneticModel::DefaultMagneticPath();
    else
      this->data->modelPath = *modelPath;
  }
  else
  {
    //const auto packagePath = ros::package::getPath("magnetic_model");
    const auto packagePath = ament_index_cpp::get_package_share_directory("magnetic_model");
    if (!packagePath.empty())
    {
      this->data->modelPath = packagePath + "/data/magnetic";
    }
    else
    {
      RCLCPP_ERROR(this->log, "Could not resolve package magnetic_model. Is the workspace properly sourced?");
      this->data->modelPath = GeographicLib::MagneticModel::DefaultMagneticPath();
    }
  }

  this->data->magneticModels.clear();

  RCLCPP_INFO(this->log, "Using WMM models from directory %s.", this->data->modelPath.c_str());
}

std::string MagneticModelManager::getBestMagneticModelName(const rclcpp::Time& date) const
{
  const auto year = compass_utils::getYear(date);  // If the conversion failed, year would be 0, thus triggering the last branch.
  if (year >= 2025)
    return MagneticModel::WMM2025;
  else if (year >= 2020)
    return MagneticModel::WMM2020;
  else if (year >= 2015)
    return MagneticModel::WMM2015;
  else
    return MagneticModel::WMM2010;
}

tl::expected<std::shared_ptr<MagneticModel>, std::string> MagneticModelManager::getMagneticModel(
  const rclcpp::Time& stamp, const bool strict) const
{
  const auto name = this->getBestMagneticModelName(stamp);
  const auto model = this->getMagneticModel(name, strict);
  if (!model.has_value())
    return compass_utils::make_unexpected(model.error());
  if (strict && !model.value()->isValid(stamp))
    return compass_utils::make_unexpected(std::format(
      "The best magnetic model {} is not valid at time {}.", name.c_str(), compass_utils::to_pretty_string(stamp).c_str()));
  return *model;
}

tl::expected<std::shared_ptr<MagneticModel>, std::string> MagneticModelManager::getMagneticModel(
  const std::string& name, const bool strict) const
{
  const auto key = std::make_pair(name, strict);
  if (this->data->magneticModels.find(key) == this->data->magneticModels.end())
  {
    try
    {
      this->data->magneticModels[key] = std::make_shared<MagneticModel>(this->log, name, this->data->modelPath, strict, this->clock);
    }
    catch (const std::invalid_argument& e)
    {
      return compass_utils::make_unexpected(e.what());
    }
  }

  return this->data->magneticModels[key];
}

}
