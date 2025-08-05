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
#include "IdleBehavior.h"

#include <mower_logic/PowerConfig.h>
#include <mower_msgs/Power.h>

#include "PerimeterDocking.h"

extern void stopMoving();
extern void stopBlade();
extern void setEmergencyMode(bool emergency);
extern void setGPS(bool enabled);
extern void setRobotPose(geometry_msgs::Pose &pose);
extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo> &actions);
extern ros::Time rain_resume;

extern ros::ServiceClient dockingPointClient;
extern mower_msgs::Status getStatus();
extern mower_msgs::Power getPower();
extern mower_logic::MowerLogicConfig getConfig();
extern void setConfig(mower_logic::MowerLogicConfig);
extern ll::PowerConfig getPowerConfig();
extern dynamic_reconfigure::Server<mower_logic::MowerLogicConfig> *reconfigServer;

extern ros::ServiceClient mapClient;
extern ros::ServiceClient dockingPointClient;

IdleBehavior IdleBehavior::INSTANCE(false);
IdleBehavior IdleBehavior::DOCKED_INSTANCE(true);

std::string IdleBehavior::state_name() {
  return "IDLE";
}

Behavior *IdleBehavior::execute() {
  // Check, if we have a configured map. If not, print info and go to area recorder
  mower_map::GetMowingAreaSrv mapSrv;
  mapSrv.request.index = 0;
  if (!mapClient.call(mapSrv)) {
    ROS_WARN("We don't have a map configured. Starting Area Recorder!");
    return &AreaRecordingBehavior::INSTANCE;
  }

  // Check, if we have a docking position. If not, print info and go to area recorder
  mower_map::GetDockingPointSrv get_docking_point_srv;
  if (!dockingPointClient.call(get_docking_point_srv)) {
    ROS_WARN("We don't have a docking point configured. Starting Area Recorder!");
    return &AreaRecordingBehavior::INSTANCE;
  }

  setGPS(false);
  geometry_msgs::PoseStamped docking_pose_stamped;
  docking_pose_stamped.pose = get_docking_point_srv.response.docking_pose;
  docking_pose_stamped.header.frame_id = "map";
  docking_pose_stamped.header.stamp = ros::Time::now();

  ros::Rate r(25);
  while (ros::ok()) {
    stopMoving();
    stopBlade();
    const auto last_config = getConfig();
    const auto last_power_config = getPowerConfig();
    const auto last_status = getStatus();
    const auto last_power = getPower();

    const bool automatic_mode = last_config.automatic_mode == eAutoMode::AUTO;
    const bool active_semiautomatic_task =
        last_config.automatic_mode == eAutoMode::SEMIAUTO && shared_state->active_semiautomatic_task;
    const bool rain_delay = last_config.rain_mode == 2 && ros::Time::now() < rain_resume;
    if (rain_delay) {
      ROS_INFO_STREAM_THROTTLE(300, "Rain delay: " << int((rain_resume - ros::Time::now()).toSec() / 60) << " minutes");
    }
    const bool mower_ready = last_power.v_battery > last_power_config.battery_full_voltage &&
                             last_status.mower_motor_temperature < last_config.motor_cold_temperature &&
                             !last_config.manual_pause_mowing && !rain_delay;

    if (manual_start_mowing || ((automatic_mode || active_semiautomatic_task) && mower_ready)) {
      // set the robot's position to the dock if we're actually docked
      if (last_power.v_charge > 5.0) {
        if (PerimeterUndockingBehavior::configured(config)) return &PerimeterUndockingBehavior::INSTANCE;
        ROS_INFO_STREAM("Currently inside the docking station, we set the robot's pose to the docks pose.");
        setRobotPose(docking_pose_stamped.pose);
        return &UndockingBehavior::INSTANCE;
      }
      // Not docked, so just mow
      setGPS(true);
      return &MowingBehavior::INSTANCE;
    }

    if (start_area_recorder) {
      return &AreaRecordingBehavior::INSTANCE;
    }

    // This gets called if we need to refresh, e.g. on clearing maps
    if (aborted) {
      return &IdleBehavior::INSTANCE;
    }

    if (last_config.docking_redock && stay_docked && last_power.v_charge < 5.0) {
      ROS_WARN("We docked but seem to have lost contact with the charger.  Undocking and trying again!");
      return &UndockingBehavior::RETRY_INSTANCE;
    }

    r.sleep();
  }

  return nullptr;
}

void IdleBehavior::enter() {
  start_area_recorder = false;
  // Reset the docking behavior, to allow docking
  DockingBehavior::INSTANCE.reset();

  // disable it, so that we don't start mowing immediately
  manual_start_mowing = false;

  for (auto &a : actions) {
    a.enabled = true;
  }
  registerActions("mower_logic:idle", actions);
}

void IdleBehavior::exit() {
  for (auto &a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:idle", actions);
}

void IdleBehavior::reset() {
}

bool IdleBehavior::needs_gps() {
  return false;
}

bool IdleBehavior::mower_enabled() {
  return false;
}

void IdleBehavior::command_home() {
  // IdleBehavior == docked, don't do anything.
}

void IdleBehavior::command_start() {
  // We got start, so we can reset the last manual pause
  auto config = getConfig();
  config.manual_pause_mowing = false;
  setConfig(config);

  manual_start_mowing = true;
}

void IdleBehavior::command_s1() {
  start_area_recorder = true;
}

void IdleBehavior::command_s2() {
}

bool IdleBehavior::redirect_joystick() {
  return false;
}

uint8_t IdleBehavior::get_sub_state() {
  return 0;
}

uint8_t IdleBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_IDLE;
}

IdleBehavior::IdleBehavior(bool stayDocked) {
  this->stay_docked = stayDocked;

  xbot_msgs::ActionInfo start_mowing_action;
  start_mowing_action.action_id = "start_mowing";
  start_mowing_action.enabled = false;
  start_mowing_action.action_name = "Start Mowing";

  xbot_msgs::ActionInfo start_area_recording_action;
  start_area_recording_action.action_id = "start_area_recording";
  start_area_recording_action.enabled = false;
  start_area_recording_action.action_name = "Start Area Recording";

  actions.clear();
  actions.push_back(start_mowing_action);
  actions.push_back(start_area_recording_action);
}

void IdleBehavior::handle_action(std::string action) {
  if (action == "mower_logic:idle/start_mowing") {
    ROS_INFO_STREAM("Got start_mowing command");
    command_start();
  } else if (action == "mower_logic:idle/start_area_recording") {
    ROS_INFO_STREAM("Got start_area_recording command");
    command_s1();
  }
}
