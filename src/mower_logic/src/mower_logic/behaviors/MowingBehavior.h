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
#ifndef SRC_MOWINGBEHAVIOR_H
#define SRC_MOWINGBEHAVIOR_H

#include <atomic>

#include "Behavior.h"
#include "UndockingBehavior.h"
#include "ftc_local_planner/PlannerGetProgress.h"
#include "mower_map/MapArea.h"
#include "slic3r_coverage_planner/Path.h"
#include "slic3r_coverage_planner/PlanPath.h"
#include "xbot_msgs/ActionInfo.h"

class MowingBehavior : public Behavior {
 private:
  std::vector<xbot_msgs::ActionInfo> actions;

  bool skip_area;
  bool skip_path;
  bool create_mowing_plan(int area_index);

  bool execute_mowing_plan();

  /// @return true if spinup succeeded or spinup is disabled, false if we should abort/exit
  bool wait_for_mower_spinup();

  // Progress
  bool mowerEnabled = false;
  std::vector<slic3r_coverage_planner::Path> currentMowingPaths;

  ros::Time last_checkpoint;
  int currentMowingPath;
  int currentMowingArea;
  int currentMowingPathIndex;
  std::string currentMowingAreaId;
  std::string currentMowingAreaName;
  std::string currentMowingPlanDigest;
  // atomic: the mowing.plan rpc reads it from its own thread
  std::atomic<double> currentMowingAngleIncrementSum{0};

 public:
  MowingBehavior();

  static MowingBehavior INSTANCE;

  // Plans an area with the angle and settings mowing would use (offset, increment, overrides). Also used by the
  // mowing.plan RPC, so it must not touch the state of a running job.
  bool plan_area(const mower_map::MapArea& area, const mower_logic::MowerLogicConfig& cfg, ros::ServiceClient& planner,
                 std::vector<slic3r_coverage_planner::Path>& paths, double& angle);

  std::string state_name() override;

  Behavior* execute() override;

  void enter() override;

  void exit() override;

  void reset() override;

  // whether the checkpoint has a job that got interrupted part way
  bool has_unfinished_job();

  // drops the progress of an interrupted job, the next start mows from the beginning. only while not mowing
  void reset_job();

  bool needs_gps() override;

  bool mower_enabled() override;

  void command_home() override;

  void command_start() override;

  void command_s1() override;

  void command_s2() override;

  bool redirect_joystick() override;

  uint8_t get_sub_state() override;

  uint8_t get_state() override;

  int16_t get_current_area() const;

  std::string get_current_area_id() const;

  std::string get_current_area_name() const;

  int16_t get_current_path();

  int16_t get_current_path_index();

  void handle_action(std::string action) override;

  void update_actions();

  void checkpoint();

  bool restore_checkpoint();
};

#endif  // SRC_MOWINGBEHAVIOR_H
