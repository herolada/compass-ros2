// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for testing string utilities for compass_interfaces.
 * \author Martin Pecka, Adam Herold (ROS2 transcription)
 */

#include "gtest/gtest.h"

#include <string>

#include <compass_utils/string_utils.hpp>
#include <compass_interfaces/msg/azimuth.hpp>

using Az = compass_interfaces::msg::Azimuth;

TEST(CompassMsgs, Unit) // NOLINT
{
  EXPECT_EQ(Az::UNIT_RAD, compass_utils::parseUnit("rad"));
  EXPECT_EQ(Az::UNIT_RAD, compass_utils::parseUnit("RAD"));
  EXPECT_EQ(Az::UNIT_RAD, compass_utils::parseUnit("Rad"));

  EXPECT_EQ(Az::UNIT_DEG, compass_utils::parseUnit("deg"));
  EXPECT_EQ(Az::UNIT_DEG, compass_utils::parseUnit("DEG"));
  EXPECT_EQ(Az::UNIT_DEG, compass_utils::parseUnit("Deg"));

  EXPECT_THROW(compass_utils::parseUnit("foo"), std::runtime_error);

  EXPECT_EQ("rad", compass_utils::unitToString(Az::UNIT_RAD));
  EXPECT_EQ("deg", compass_utils::unitToString(Az::UNIT_DEG));
  EXPECT_THROW(compass_utils::unitToString(10), std::runtime_error);
}

TEST(CompassMsgs, Orientation) // NOLINT
{
  EXPECT_EQ(Az::ORIENTATION_ENU, compass_utils::parseOrientation("enu"));
  EXPECT_EQ(Az::ORIENTATION_ENU, compass_utils::parseOrientation("ENU"));
  EXPECT_EQ(Az::ORIENTATION_ENU, compass_utils::parseOrientation("Enu"));

  EXPECT_EQ(Az::ORIENTATION_NED, compass_utils::parseOrientation("ned"));
  EXPECT_EQ(Az::ORIENTATION_NED, compass_utils::parseOrientation("NED"));
  EXPECT_EQ(Az::ORIENTATION_NED, compass_utils::parseOrientation("Ned"));

  EXPECT_THROW(compass_utils::parseOrientation("foo"), std::runtime_error);

  EXPECT_EQ("ENU", compass_utils::orientationToString(Az::ORIENTATION_ENU));
  EXPECT_EQ("NED", compass_utils::orientationToString(Az::ORIENTATION_NED));
  EXPECT_THROW(compass_utils::orientationToString(10), std::runtime_error);
}

TEST(CompassMsgs, Reference) // NOLINT
{
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, compass_utils::parseReference("magnetic"));
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, compass_utils::parseReference("MAGNETIC"));
  EXPECT_EQ(Az::REFERENCE_MAGNETIC, compass_utils::parseReference("Magnetic"));

  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_utils::parseReference("geographic"));
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_utils::parseReference("GEOGRAPHIC"));
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_utils::parseReference("Geographic"));
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_utils::parseReference("true"));
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_utils::parseReference("TRUE"));
  EXPECT_EQ(Az::REFERENCE_GEOGRAPHIC, compass_utils::parseReference("True"));

  EXPECT_EQ(Az::REFERENCE_UTM, compass_utils::parseReference("utm"));
  EXPECT_EQ(Az::REFERENCE_UTM, compass_utils::parseReference("UTM"));
  EXPECT_EQ(Az::REFERENCE_UTM, compass_utils::parseReference("Utm"));

  EXPECT_THROW(compass_utils::parseReference("foo"), std::runtime_error);

  EXPECT_EQ("magnetic", compass_utils::referenceToString(Az::REFERENCE_MAGNETIC));
  EXPECT_EQ("geographic", compass_utils::referenceToString(Az::REFERENCE_GEOGRAPHIC));
  EXPECT_EQ("UTM", compass_utils::referenceToString(Az::REFERENCE_UTM));
  EXPECT_THROW(compass_utils::referenceToString(10), std::runtime_error);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}