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
#ifndef SRC_UNDOCKINGBEHAVIOR_H
#define SRC_UNDOCKINGBEHAVIOR_H

#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/ExePathAction.h>
#include <tf2/LinearMath/Transform.h>

#include "Behavior.h"
#include "DockingBehavior.h"
#include "IdleBehavior.h"
#include "MowingBehavior.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "xbot_msgs/AbsolutePose.h"
#include "xbot_msgs/ActionInfo.h"

class UndockingBehavior : public Behavior {
 public:
  static UndockingBehavior INSTANCE;
  static UndockingBehavior RETRY_INSTANCE;

  UndockingBehavior(Behavior* nextBehavior);

 private:
  std::vector<xbot_msgs::ActionInfo> actions;

  Behavior* nextBehavior;
  geometry_msgs::PoseStamped docking_pose_stamped;
  bool gpsRequired;

  bool waitForGPS();

 public:
  UndockingBehavior();

  std::string state_name() override;

  Behavior* execute() override;

  void enter() override;

  void exit() override;

  void reset() override;

  bool needs_gps() override;

  bool mower_enabled() override;

  void command_home() override;

  void command_start() override;

  void command_s1() override;

  void command_s2() override;

  bool redirect_joystick() override;

  uint8_t get_sub_state() override;

  uint8_t get_state() override;

  void handle_action(std::string action) override;
};

#endif  // SRC_UNDOCKINGBEHAVIOR_H
