/**
 * \file
 * \brief Object implementing an `ok()` method that can interrupt pending sleeps when it returns false.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/utilities.hpp>

#include <compass_utils/interruptible_sleep_interface.h>
#include <compass_utils/semaphore.hpp>

namespace compass_utils
{

struct InterruptibleSleepInterfacePrivate
{
  //! \brief This semaphore prevents the object to be destructed before pending sleeps finish.
  mutable ReverseSemaphore semaphore;
};

InterruptibleSleepInterface::InterruptibleSleepInterface(const rclcpp::Clock& clock) :
  data(new InterruptibleSleepInterfacePrivate), clock(clock)
{
}

InterruptibleSleepInterface::~InterruptibleSleepInterface()
{
  this->data->semaphore.disable();
  this->data->semaphore.waitZero();
}

bool InterruptibleSleepInterface::ok() const
{
  // This should only be called when the object is partially destroyed. Normally, the override method is called.
  return this->data->semaphore.isEnabled();
}

bool InterruptibleSleepInterface::sleep(const rclcpp::Duration& duration) const
{
  if (duration.seconds() == 0 && duration.nanoseconds() == 0)
    return true;

  // Fast-track exit in case ok() is false to prevent additional locks of the mutex that is needed for object
  // destruction.
  if (!this->ok() || !rclcpp::ok())
    return false;

  SemaphoreGuard<ReverseSemaphore> guard(this->data->semaphore);

  // code heavily inspired by BSD-licensed https://github.com/ros/roscpp_core/blob/noetic-devel/rostime/src/time.cpp
  // what is added is the this->ok() check in the while loop and making the system time sleep also interruptible

  // if (rclcpp::Time::useSystemTime())
  // {
  //   const auto start = ros::WallTime::now();
  //   const auto wallDuration = ros::WallDuration(duration.sec, duration.nsec);
  //   const auto pollWallDuration = wallDuration * 0.01;
  //   const auto end = start + wallDuration;

  //   bool rc = ros::WallTime::now() >= end;  // if the duration was veery short, we might already have finished now
  //   while (rclcpp::ok() && this->ok() && (ros::WallTime::now() < end))
  //   {
  //     pollWallDuration.sleep();
  //     rc = true;
  //   }

  //   return rc && rclcpp::ok() && this->ok();
  // }

  auto start = this->clock->now();
  auto end = start + duration;
  if (start.seconds() == 0 && start.nanoseconds() == 0)
    end = rclcpp::Time::max();

  bool rc = this->clock->now() >= end;  // if the duration was veery short, we might already have finished now
  while (rclcpp::ok() && this->ok() && (this->clock->now() < end))
  {
    //this->pollDuration.sleep();
    rclcpp::sleep_for(std::chrono::nanoseconds(pollDuration.nanoseconds()));
    rc = true;

    // If we started at time 0 wait for the first actual time to arrive before starting the timer on
    // our sleep
    if (start.seconds() == 0 && start.nanoseconds() == 0)
    {
      start = this->clock->now();
      end = start + duration;
    }

    // If time jumped backwards from when we started sleeping, return immediately
    if (this->clock->now() < start)
      return false;
  }
  return rc && rclcpp::ok() && this->ok();
}

}
