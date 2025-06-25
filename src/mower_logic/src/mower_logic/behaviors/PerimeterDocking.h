// Created by Bodo Pfelzer on 28/10/23.
// Copyright (c) 2023 Clemens Elflein and OpenMower contributors. All rights reserved.
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
#ifndef SRC_PERIMETER_DOCKING_H
#define SRC_PERIMETER_DOCKING_H

#include "Behavior.h"

/*
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mower_map/GetDockingPointSrv.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Transform.h>

#include "mower_msgs/Status.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
*/

class PerimeterBase : public Behavior {
 public:
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

 protected:
  int setupConnections();
};

class PerimeterFollowBehavior : public PerimeterBase {
 public:
  Behavior *execute() override;

 protected:
  /**
   * @brief distance travelled.
   */
  double travelled;
  /**
   * @brief We arrived and should continue with the returned Behavior.
   */
  virtual Behavior *arrived() = 0;
};

class PerimeterDockingBehavior : public PerimeterFollowBehavior {
 private:
  int chargeSeen;

 public:
  static PerimeterDockingBehavior INSTANCE;
  std::string state_name() override;

 protected:
  Behavior *arrived() override;
};

class PerimeterSearchBehavior : public PerimeterBase {
 public:
  static PerimeterSearchBehavior INSTANCE;
  /**
   * Is usage configured?
   */
  static int configured(const mower_logic::MowerLogicConfig &config);

  std::string state_name() override;
  Behavior *execute() override;
};

class PerimeterUndockingBehavior : public PerimeterBase {
 public:
  static PerimeterUndockingBehavior INSTANCE;
  /**
   * Is usage configured?
   */
  static int configured(const mower_logic::MowerLogicConfig &config);

  std::string state_name() override;
  Behavior *execute() override;
};

class PerimeterMoveToGpsBehavior : public PerimeterFollowBehavior {
 public:
  static PerimeterMoveToGpsBehavior INSTANCE;
  std::string state_name() override;
  void enter() override;

 protected:
  Behavior *arrived() override;
};

#endif  // SRC_PERIMETER_DOCKING_H
