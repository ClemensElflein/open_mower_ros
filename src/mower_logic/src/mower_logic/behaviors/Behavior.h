// Created by Clemens Elflein on 2/21/22.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//
#ifndef SRC_BEHAVIOR_H
#define SRC_BEHAVIOR_H

#include <actionlib/client/simple_action_client.h>

#include <atomic>
#include <memory>

#include "mower_logic/MowerLogicConfig.h"
#include "mower_msgs/HighLevelStatus.h"
#include "ros/ros.h"

enum eAutoMode { MANUAL = 0, SEMIAUTO = 1, AUTO = 2 };

enum pauseType { PAUSE_MANUAL = 0b1, PAUSE_EMERGENCY = 0b10 };

struct sSharedState {
  // True, if the semiautomatic task is still in progress
  bool active_semiautomatic_task;
};

/**
 * Behavior definition
 */
class Behavior {
 private:
  ros::Time startTime;

 protected:
  std::atomic<bool> aborted;
  std::atomic<bool> paused;

  std::atomic<u_int8_t> requested_pause_flag;

  std::atomic<bool> isGPSGood;
  std::atomic<uint8_t> sub_state;

  double time_in_state() {
    return (ros::Time::now() - startTime).toSec();
  }

  mower_logic::MowerLogicConfig config;
  std::shared_ptr<sSharedState> shared_state;

  /**
   * Called ONCE on state enter.
   */
  virtual void enter() = 0;

 public:
  virtual std::string state_name() = 0;

  virtual std::string sub_state_name() {
    return "";
  }

  bool hasGoodGPS() {
    return isGPSGood;
  }

  void setGoodGPS(bool isGood) {
    isGPSGood = isGood;
  }

  void requestContinue(pauseType reason = pauseType::PAUSE_MANUAL) {
    requested_pause_flag &= ~reason;
  }

  void requestPause(pauseType reason = pauseType::PAUSE_MANUAL) {
    requested_pause_flag |= reason;
  }

  void start(mower_logic::MowerLogicConfig& c, std::shared_ptr<sSharedState> s) {
    ROS_INFO_STREAM("");
    ROS_INFO_STREAM("");
    ROS_INFO_STREAM("--------------------------------------");
    ROS_INFO_STREAM("- Entered state: " << state_name());
    ROS_INFO_STREAM("--------------------------------------");
    aborted = false;
    paused = false;
    requested_pause_flag = 0;
    this->config = c;
    this->shared_state = std::move(s);
    startTime = ros::Time::now();
    isGPSGood = false;
    sub_state = 0;
    enter();
  }

  template <typename ActionSpec>
  actionlib::SimpleClientGoalState sendGoalAndWaitUnlessAborted(
      actionlib::SimpleActionClient<ActionSpec>* client, const typename ActionSpec::_action_goal_type::_goal_type& goal,
      double poll_rate = 10) {
    ros::Rate rate(poll_rate);
    client->sendGoal(goal);

    while (true) {
      rate.sleep();

      auto state = client->getState();
      if (aborted) {
        client->cancelGoal();
        return state;
      }
      if (state.isDone()) {
        return state;
      }
    }
  }

  /**
   * Execute the behavior. This call should block until the behavior is executed fully.
   * @returns the pointer to the next behavior (can return itself).
   */
  virtual Behavior* execute() = 0;

  /**
   * Called ONCE before state exits
   */
  virtual void exit() = 0;

  /**
   * Reset the internal state of the behavior.
   */
  virtual void reset() = 0;

  /**
   * If called, save state internally and return the execute() method asap.
   * Execution should resume on the next execute() call.
   */
  void abort() {
    if (!aborted) {
      ROS_INFO_STREAM("- Behaviour.h: abort() called");
    }
    aborted = true;
  }

  // Return true, if this state needs absolute positioning.
  // The state will be aborted if GPS is lost and resumed at some later point in time.
  virtual bool needs_gps() = 0;

  // return true, if the mower motor should currently be running.
  virtual bool mower_enabled() = 0;

  // return true to redirect joystick speeds to the controller
  virtual bool redirect_joystick() = 0;

  virtual void command_home() = 0;
  virtual void command_start() = 0;
  virtual void command_s1() = 0;
  virtual void command_s2() = 0;

  virtual uint8_t get_sub_state() = 0;
  virtual uint8_t get_state() = 0;

  virtual void handle_action(std::string action) = 0;
};

#endif  // SRC_BEHAVIOR_H
