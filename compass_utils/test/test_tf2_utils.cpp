// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Unit test for tf2_utils.h
 * \author Martin Pecka
 */

// Test TfMessageFilter is mostly taken from
// https://github.com/rclcpp/geometry2/blob/noetic-devel/tf2_ros/test/message_filter_test.cpp
// on commit 589caf083cae9d8fae7effdb910454b4681b9ec1 . Slight modifications were done to increase reliability of
// the test (circumventing real topic pub/sub) and to utilize the customized class TfMessageFilter.
// For that part, the following copyright notice is valid:

/*
 * Copyright (c) 2014, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "gtest/gtest.h"

#include <compass_utils/tf2_utils.hpp>

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/**
 * \brief Test conversion of tf2 quaternion to roll, pitch and yaw.
 */
TEST(TF2Utils, TF2GetRPY)  // NOLINT
{
  tf2::Matrix3x3 m;
  tf2::Quaternion q;
  double roll, pitch, yaw;

  m.setRPY(0, 0, 0); m.getRotation(q); compass_utils::getRPY(q, roll, pitch, yaw);
  EXPECT_EQ(0, roll); EXPECT_EQ(0, pitch); EXPECT_EQ(0, yaw);
  EXPECT_EQ(roll, compass_utils::getRoll(q)); EXPECT_EQ(pitch, compass_utils::getPitch(q)); EXPECT_EQ(yaw, compass_utils::getYaw(q));

  m.setRPY(M_PI_2, 0, 0); m.getRotation(q); compass_utils::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(M_PI_2, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(q)); EXPECT_EQ(pitch, compass_utils::getPitch(q)); EXPECT_EQ(yaw, compass_utils::getYaw(q));

  m.setRPY(0, M_PI_2, 0); m.getRotation(q); compass_utils::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0, roll, 1e-4); EXPECT_NEAR(M_PI_2, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(q)); EXPECT_EQ(pitch, compass_utils::getPitch(q)); EXPECT_EQ(yaw, compass_utils::getYaw(q));

  m.setRPY(0, 0, M_PI_2); m.getRotation(q); compass_utils::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(M_PI_2, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(q)); EXPECT_EQ(pitch, compass_utils::getPitch(q)); EXPECT_EQ(yaw, compass_utils::getYaw(q));

  m.setRPY(0.1, 0.1, 0); m.getRotation(q); compass_utils::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0.1, roll, 1e-4); EXPECT_NEAR(0.1, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(q)); EXPECT_EQ(pitch, compass_utils::getPitch(q)); EXPECT_EQ(yaw, compass_utils::getYaw(q));

  m.setRPY(0.5, 0.5, 0); m.getRotation(q); compass_utils::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0.5, roll, 1e-4); EXPECT_NEAR(0.5, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(q)); EXPECT_EQ(pitch, compass_utils::getPitch(q)); EXPECT_EQ(yaw, compass_utils::getYaw(q));

  m.setRPY(1.0, 1.0, 0); m.getRotation(q); compass_utils::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(1.0, roll, 1e-4); EXPECT_NEAR(1.0, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(q)); EXPECT_EQ(pitch, compass_utils::getPitch(q)); EXPECT_EQ(yaw, compass_utils::getYaw(q));

  m.setRPY(0, 0.1, 0.1); m.getRotation(q); compass_utils::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0, roll, 1e-4); EXPECT_NEAR(0.1, pitch, 1e-4); EXPECT_NEAR(0.1, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(q)); EXPECT_EQ(pitch, compass_utils::getPitch(q)); EXPECT_EQ(yaw, compass_utils::getYaw(q));

  m.setRPY(0, 0.5, 0.5); m.getRotation(q); compass_utils::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0, roll, 1e-4); EXPECT_NEAR(0.5, pitch, 1e-4); EXPECT_NEAR(0.5, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(q)); EXPECT_EQ(pitch, compass_utils::getPitch(q)); EXPECT_EQ(yaw, compass_utils::getYaw(q));

  m.setRPY(0, 1.0, 1.0); m.getRotation(q); compass_utils::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0.0, roll, 1e-4); EXPECT_NEAR(1.0, pitch, 1e-4); EXPECT_NEAR(1.0, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(q)); EXPECT_EQ(pitch, compass_utils::getPitch(q)); EXPECT_EQ(yaw, compass_utils::getYaw(q));

  m.setRPY(0.1, 0, 0.1); m.getRotation(q); compass_utils::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0.1, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(0.1, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(q)); EXPECT_EQ(pitch, compass_utils::getPitch(q)); EXPECT_EQ(yaw, compass_utils::getYaw(q));

  m.setRPY(0.5, 0, 0.5); m.getRotation(q); compass_utils::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0.5, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(0.5, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(q)); EXPECT_EQ(pitch, compass_utils::getPitch(q)); EXPECT_EQ(yaw, compass_utils::getYaw(q));

  m.setRPY(1.0, 0, 1.0); m.getRotation(q); compass_utils::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(1.0, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(1.0, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(q)); EXPECT_EQ(pitch, compass_utils::getPitch(q)); EXPECT_EQ(yaw, compass_utils::getYaw(q));

  m.setRPY(0.1, 0.1, 0.1); m.getRotation(q); compass_utils::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0.1, roll, 1e-4); EXPECT_NEAR(0.1, pitch, 1e-4); EXPECT_NEAR(0.1, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(q)); EXPECT_EQ(pitch, compass_utils::getPitch(q)); EXPECT_EQ(yaw, compass_utils::getYaw(q));

  m.setRPY(0.5, 0.5, 0.5); m.getRotation(q); compass_utils::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(0.5, roll, 1e-4); EXPECT_NEAR(0.5, pitch, 1e-4); EXPECT_NEAR(0.5, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(q)); EXPECT_EQ(pitch, compass_utils::getPitch(q)); EXPECT_EQ(yaw, compass_utils::getYaw(q));

  m.setRPY(1.0, 1.0, 1.0); m.getRotation(q); compass_utils::getRPY(q, roll, pitch, yaw);
  EXPECT_NEAR(1.0, roll, 1e-4); EXPECT_NEAR(1.0, pitch, 1e-4); EXPECT_NEAR(1.0, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(q)); EXPECT_EQ(pitch, compass_utils::getPitch(q)); EXPECT_EQ(yaw, compass_utils::getYaw(q));
}

/**
 * \brief Test conversion of geometry_msgs quaternion to roll, pitch and yaw.
 */
TEST(TF2Utils, GeometryGetRPY)  // NOLINT
{
  tf2::Matrix3x3 m;
  tf2::Quaternion q;
  geometry_msgs::msg::Quaternion gq;
  double roll, pitch, yaw;

  m.setRPY(0, 0, 0); m.getRotation(q); gq = tf2::toMsg(q); compass_utils::getRPY(gq, roll, pitch, yaw);
  EXPECT_EQ(0, roll); EXPECT_EQ(0, pitch); EXPECT_EQ(0, yaw);
  EXPECT_EQ(roll, compass_utils::getRoll(gq)); EXPECT_EQ(pitch, compass_utils::getPitch(gq)); EXPECT_EQ(yaw, compass_utils::getYaw(gq));

  m.setRPY(M_PI_2, 0, 0); m.getRotation(q); gq = tf2::toMsg(q); compass_utils::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(M_PI_2, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(gq)); EXPECT_EQ(pitch, compass_utils::getPitch(gq)); EXPECT_EQ(yaw, compass_utils::getYaw(gq));

  m.setRPY(0, M_PI_2, 0); m.getRotation(q); gq = tf2::toMsg(q); compass_utils::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0, roll, 1e-4); EXPECT_NEAR(M_PI_2, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(gq)); EXPECT_EQ(pitch, compass_utils::getPitch(gq)); EXPECT_EQ(yaw, compass_utils::getYaw(gq));

  m.setRPY(0, 0, M_PI_2); m.getRotation(q); gq = tf2::toMsg(q); compass_utils::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(M_PI_2, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(gq)); EXPECT_EQ(pitch, compass_utils::getPitch(gq)); EXPECT_EQ(yaw, compass_utils::getYaw(gq));

  m.setRPY(0.1, 0.1, 0); m.getRotation(q); gq = tf2::toMsg(q); compass_utils::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0.1, roll, 1e-4); EXPECT_NEAR(0.1, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(gq)); EXPECT_EQ(pitch, compass_utils::getPitch(gq)); EXPECT_EQ(yaw, compass_utils::getYaw(gq));

  m.setRPY(0.5, 0.5, 0); m.getRotation(q); gq = tf2::toMsg(q); compass_utils::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0.5, roll, 1e-4); EXPECT_NEAR(0.5, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(gq)); EXPECT_EQ(pitch, compass_utils::getPitch(gq)); EXPECT_EQ(yaw, compass_utils::getYaw(gq));

  m.setRPY(1.0, 1.0, 0); m.getRotation(q); gq = tf2::toMsg(q); compass_utils::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(1.0, roll, 1e-4); EXPECT_NEAR(1.0, pitch, 1e-4); EXPECT_NEAR(0, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(gq)); EXPECT_EQ(pitch, compass_utils::getPitch(gq)); EXPECT_EQ(yaw, compass_utils::getYaw(gq));

  m.setRPY(0, 0.1, 0.1); m.getRotation(q); gq = tf2::toMsg(q); compass_utils::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0, roll, 1e-4); EXPECT_NEAR(0.1, pitch, 1e-4); EXPECT_NEAR(0.1, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(gq)); EXPECT_EQ(pitch, compass_utils::getPitch(gq)); EXPECT_EQ(yaw, compass_utils::getYaw(gq));

  m.setRPY(0, 0.5, 0.5); m.getRotation(q); gq = tf2::toMsg(q); compass_utils::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0, roll, 1e-4); EXPECT_NEAR(0.5, pitch, 1e-4); EXPECT_NEAR(0.5, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(gq)); EXPECT_EQ(pitch, compass_utils::getPitch(gq)); EXPECT_EQ(yaw, compass_utils::getYaw(gq));

  m.setRPY(0, 1.0, 1.0); m.getRotation(q); gq = tf2::toMsg(q); compass_utils::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0.0, roll, 1e-4); EXPECT_NEAR(1.0, pitch, 1e-4); EXPECT_NEAR(1.0, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(gq)); EXPECT_EQ(pitch, compass_utils::getPitch(gq)); EXPECT_EQ(yaw, compass_utils::getYaw(gq));

  m.setRPY(0.1, 0, 0.1); m.getRotation(q); gq = tf2::toMsg(q); compass_utils::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0.1, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(0.1, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(gq)); EXPECT_EQ(pitch, compass_utils::getPitch(gq)); EXPECT_EQ(yaw, compass_utils::getYaw(gq));

  m.setRPY(0.5, 0, 0.5); m.getRotation(q); gq = tf2::toMsg(q); compass_utils::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0.5, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(0.5, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(gq)); EXPECT_EQ(pitch, compass_utils::getPitch(gq)); EXPECT_EQ(yaw, compass_utils::getYaw(gq));

  m.setRPY(1.0, 0, 1.0); m.getRotation(q); gq = tf2::toMsg(q); compass_utils::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(1.0, roll, 1e-4); EXPECT_NEAR(0, pitch, 1e-4); EXPECT_NEAR(1.0, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(gq)); EXPECT_EQ(pitch, compass_utils::getPitch(gq)); EXPECT_EQ(yaw, compass_utils::getYaw(gq));

  m.setRPY(0.1, 0.1, 0.1); m.getRotation(q); gq = tf2::toMsg(q); compass_utils::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0.1, roll, 1e-4); EXPECT_NEAR(0.1, pitch, 1e-4); EXPECT_NEAR(0.1, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(gq)); EXPECT_EQ(pitch, compass_utils::getPitch(gq)); EXPECT_EQ(yaw, compass_utils::getYaw(gq));

  m.setRPY(0.5, 0.5, 0.5); m.getRotation(q); gq = tf2::toMsg(q); compass_utils::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(0.5, roll, 1e-4); EXPECT_NEAR(0.5, pitch, 1e-4); EXPECT_NEAR(0.5, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(gq)); EXPECT_EQ(pitch, compass_utils::getPitch(gq)); EXPECT_EQ(yaw, compass_utils::getYaw(gq));

  m.setRPY(1.0, 1.0, 1.0); m.getRotation(q); gq = tf2::toMsg(q); compass_utils::getRPY(gq, roll, pitch, yaw);
  EXPECT_NEAR(1.0, roll, 1e-4); EXPECT_NEAR(1.0, pitch, 1e-4); EXPECT_NEAR(1.0, yaw, 1e-4);
  EXPECT_EQ(roll, compass_utils::getRoll(gq)); EXPECT_EQ(pitch, compass_utils::getPitch(gq)); EXPECT_EQ(yaw, compass_utils::getYaw(gq));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  // rclcpp::init(argc, argv, "test_tf2_utils");
  // rclcpp::start();
  return RUN_ALL_TESTS();
}
