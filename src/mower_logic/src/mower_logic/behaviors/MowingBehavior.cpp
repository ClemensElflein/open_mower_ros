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
#include "MowingBehavior.h"

#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Path.h>

#include "mower_map/ClearNavPointSrv.h"
#include "mower_map/SetNavPointSrv.h"
#include "mower_msgs/MowPathsAction.h"

extern ros::ServiceClient mapClient;
extern ros::ServiceClient pathClient;
extern ros::ServiceClient pathProgressClient;
extern ros::ServiceClient setNavPointClient;
extern ros::ServiceClient clearNavPointClient;

extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> *mbfClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfClientExePath;
extern mower_logic::MowerLogicConfig getConfig();
extern void setConfig(mower_logic::MowerLogicConfig);

extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo> &actions);

extern actionlib::SimpleActionServer<mower_msgs::MowPathsAction> *mowPathsServer;
extern void acceptGoal();
extern std::vector<slic3r_coverage_planner::Path> currentMowingPaths;
extern int currentMowingPath;
extern int currentMowingPathIndex;
extern bool expectMoreGoals;

MowingBehavior MowingBehavior::INSTANCE;

std::string MowingBehavior::state_name() {
  if (paused) {
    return "PAUSED";
  }
  return "MOWING";
}

Behavior *MowingBehavior::execute() {
  ros::Rate waitForNewGoalRate(0.2);
  while (ros::ok() && !aborted && mowPathsServer->isActive()) {
    bool finished = execute_mowing_plan();
    if (finished) {
      mowPathsServer->setSucceeded();
      if (expectMoreGoals) {
        while (!mowPathsServer->isNewGoalAvailable()) {
          waitForNewGoalRate.sleep();
        }
        acceptGoal();
      }
    }
  }

  if (!ros::ok()) {
    // something went wrong
    return nullptr;
  }

  return &IdleBehavior::INSTANCE;
}

void MowingBehavior::enter() {
  skip_area = false;
  skip_path = false;
  paused = aborted = false;

  for (auto &a : actions) {
    a.enabled = true;
  }
  registerActions("mower_logic:mowing", actions);
}

void MowingBehavior::exit() {
  for (auto &a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:mowing", actions);
}

void MowingBehavior::reset() {
}

bool MowingBehavior::needs_gps() {
  return true;
}

bool MowingBehavior::mower_enabled() {
  return mowerEnabled;
}

void MowingBehavior::update_actions() {
  for (auto &a : actions) {
    a.enabled = true;
  }

  // pause / resume switch. other actions are always available
  actions[0].enabled = !(requested_pause_flag & pauseType::PAUSE_MANUAL);
  actions[1].enabled = requested_pause_flag & pauseType::PAUSE_MANUAL;

  registerActions("mower_logic:mowing", actions);
}

int getCurrentMowPathIndex() {
  ftc_local_planner::PlannerGetProgress progressSrv;
  int currentIndex = -1;
  if (pathProgressClient.call(progressSrv)) {
    currentIndex = progressSrv.response.index;
  } else {
    ROS_ERROR("MowingBehavior: getMowIndex() - Error getting progress from FTC planner");
  }
  return (currentIndex);
}

void printNavState(int state) {
  switch (state) {
    case actionlib::SimpleClientGoalState::PENDING: ROS_INFO(">>> State: Pending <<<"); break;
    case actionlib::SimpleClientGoalState::ACTIVE: ROS_INFO(">>> State: Active <<<"); break;
    case actionlib::SimpleClientGoalState::RECALLED: ROS_INFO(">>> State: Recalled <<<"); break;
    case actionlib::SimpleClientGoalState::REJECTED: ROS_INFO(">>> State: Rejected <<<"); break;
    case actionlib::SimpleClientGoalState::PREEMPTED: ROS_INFO(">>> State: Preempted <<<"); break;
    case actionlib::SimpleClientGoalState::ABORTED: ROS_INFO(">>> State: Aborted <<<"); break;
    case actionlib::SimpleClientGoalState::SUCCEEDED: ROS_INFO(">>> State: Succeeded <<<"); break;
    case actionlib::SimpleClientGoalState::LOST: ROS_INFO(">>> State: Lost <<<"); break;
    default: ROS_INFO(">>> State: Unknown Hu ? <<<"); break;
  }
}

bool MowingBehavior::execute_mowing_plan() {
  int first_point_attempt_counter = 0;
  int first_point_trim_counter = 0;
  ros::Time paused_time(0.0);

  const int currentMowingArea = 0;  // FIXME

  // loop through all mowingPaths to execute the plan fully.
  while (currentMowingPath < currentMowingPaths.size() && ros::ok() && !aborted) {
    ////////////////////////////////////////////////
    // PAUSE HANDLING
    ////////////////////////////////////////////////
    if (requested_pause_flag) {  // pause was requested
      paused = true;
      mowerEnabled = false;
      u_int8_t last_requested_pause_flags = 0;
      while (requested_pause_flag && !aborted)  // while emergency and/or manual pause not asked to continue, we wait
      {
        if (last_requested_pause_flags != requested_pause_flag) {
          update_actions();
        }
        last_requested_pause_flags = requested_pause_flag;

        std::string pause_reason = "";
        if (requested_pause_flag & pauseType::PAUSE_EMERGENCY) {
          pause_reason += "on EMERGENCY";
          if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
            pause_reason += " and ";
          }
        }
        if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
          pause_reason += "waiting for CONTINUE";
        }
        ROS_INFO_STREAM_THROTTLE(30, "MowingBehavior: PAUSED (" << pause_reason << ")");
        ros::Rate r(1.0);
        r.sleep();
        // TODO: Don't we need to check for aborted here?
      }
      // we will drop into paused, thus will also wait for GPS to be valid again
    }
    if (paused) {
      paused_time = ros::Time::now();
      while (!this->hasGoodGPS() && !aborted)  // while no good GPS we wait
      {
        ROS_INFO_STREAM("MowingBehavior: PAUSED (" << (ros::Time::now() - paused_time).toSec()
                                                   << "s) (waiting for GPS)");
        ros::Rate r(1.0);
        r.sleep();
        // TODO: Don't we need to check for aborted here?
      }
      ROS_INFO_STREAM("MowingBehavior: CONTINUING");
      paused = false;
      update_actions();
    }

    auto &path = currentMowingPaths[currentMowingPath];
    ROS_INFO_STREAM("MowingBehavior: Path segment length: " << path.path.poses.size() << " poses.");

    // Check if path is empty. If so, directly skip it
    if (currentMowingPathIndex >= path.path.poses.size()) {
      ROS_INFO_STREAM("MowingBehavior: Skipping empty path.");
      currentMowingPath++;
      currentMowingPathIndex = 0;
      continue;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DRIVE TO THE FIRST POINT OF THE MOW PATH
    //
    // * we have n attempts, if we fail we go to pause() mode because most likely it was GPS problems that
    //   prevented us from reaching the inital pose
    // * after n attempts, we fail the mow area and skip to the next one
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
      ROS_INFO_STREAM("MowingBehavior: (FIRST POINT)  Moving to path segment starting point");
      if (path.is_outline && getConfig().add_fake_obstacle) {
        mower_map::SetNavPointSrv set_nav_point_srv;
        set_nav_point_srv.request.nav_pose = path.path.poses[currentMowingPathIndex].pose;
        setNavPointClient.call(set_nav_point_srv);
        sleep(1);
      }

      mbf_msgs::MoveBaseGoal moveBaseGoal;
      moveBaseGoal.target_pose = path.path.poses[currentMowingPathIndex];
      moveBaseGoal.controller = "FTCPlanner";
      mbfClient->sendGoal(moveBaseGoal);
      sleep(1);
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);
      ros::Rate r(10);

      // wait for path execution to finish
      while (ros::ok()) {
        current_status = mbfClient->getState();
        if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
            current_status.state_ == actionlib::SimpleClientGoalState::PENDING) {
          // path is being executed, everything seems fine.
          // check if we should pause or abort mowing
          /* // FIXME
          if (skip_area) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) SKIP AREA was requested.");
            // remove all paths in current area and return true
            mowerEnabled = false;
            mbfClientExePath->cancelAllGoals();
            currentMowingPaths.clear();
            skip_area = false;
            return true;
          }
          if (skip_path) {
            skip_path = false;
            currentMowingPath++;
            currentMowingPathIndex = 0;
            return false;
          }
          */
          if (aborted) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) ABORT was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            return false;
          }
          if (requested_pause_flag) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) PAUSE was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            return false;
          }
        } else {
          ROS_INFO_STREAM("MowingBehavior: (FIRST POINT)  Got status "
                          << current_status.state_ << " from MBF/FTCPlanner -> Stopping path execution.");
          // we're done, break out of the loop
          break;
        }
        r.sleep();
      }

      first_point_attempt_counter++;
      if (current_status.state_ != actionlib::SimpleClientGoalState::SUCCEEDED) {
        // we cannot reach the start point
        ROS_ERROR_STREAM("MowingBehavior: (FIRST POINT) - Could not reach goal (first point). Planner Status was: "
                         << current_status.state_);
        // we have 3 attempts to get to the start pose of the mowing area
        if (first_point_attempt_counter < config.max_first_point_attempts) {
          ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) - Attempt " << first_point_attempt_counter << " / "
                                                                     << config.max_first_point_attempts
                                                                     << " Making a little pause ...");
          paused = true;
          update_actions();
        } else {
          // We failed to reach the first point in the mow path by simply repeating the drive to process
          // So now we will trim the path by removing the first pose
          if (first_point_trim_counter < config.max_first_point_trim_attempts) {
            // We try now to remove the first point so the 2nd, 3rd etc point becomes our target
            // mow path points are offset by 10cm
            ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) - Attempt "
                            << first_point_trim_counter << " / " << config.max_first_point_trim_attempts
                            << " Trimming first point off the beginning of the mow path.");
            currentMowingPathIndex++;
            first_point_trim_counter++;
            first_point_attempt_counter = 0;  // give it another <config.max_first_point_attempts> attempts
            paused = true;
            update_actions();
          } else {
            // Unable to reach the start of the mow path (we tried multiple attempts for the same point, and we skipped
            // points which also didnt work, time to give up)
            ROS_ERROR_STREAM(
                "MowingBehavior: (FIRST POINT) Max retries reached, we are unable to reach any of the first points - "
                "aborting at index: "
                << currentMowingPathIndex << " path: " << currentMowingPath << " area: " << currentMowingArea);
            this->abort();
          }
        }
        continue;
      }

      mower_map::ClearNavPointSrv clear_nav_point_srv;
      clearNavPointClient.call(clear_nav_point_srv);

      // we have reached the start pose of the mow area, reset error handling values
      first_point_attempt_counter = 0;
      first_point_trim_counter = 0;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Execute the path segment and either drop it if we finished it successfully or trim it if we were aborted
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
      // enable mower (only when we reach the start not on the way to mowing already)
      mowerEnabled = true;

      mbf_msgs::ExePathGoal exePathGoal;
      nav_msgs::Path exePath;
      exePath.header = path.path.header;
      exePath.poses = std::vector<geometry_msgs::PoseStamped>(path.path.poses.begin() + currentMowingPathIndex,
                                                              path.path.poses.end());
      int exePathStartIndex = currentMowingPathIndex;
      exePathGoal.path = exePath;
      exePathGoal.angle_tolerance = 5.0 * (M_PI / 180.0);
      exePathGoal.dist_tolerance = 0.2;
      exePathGoal.tolerance_from_action = true;
      exePathGoal.controller = "FTCPlanner";

      ROS_INFO_STREAM("MowingBehavior: (MOW) First point reached - Executing mow path with "
                      << path.path.poses.size() << " poses, from index " << exePathStartIndex);
      mbfClientExePath->sendGoal(exePathGoal);
      sleep(1);
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);
      ros::Rate r(10);

      // wait for path execution to finish
      while (ros::ok()) {
        current_status = mbfClientExePath->getState();
        if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
            current_status.state_ == actionlib::SimpleClientGoalState::PENDING) {
          // path is being executed, everything seems fine.
          // check if we should pause or abort mowing
          /* // FIXME
          if (skip_area) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) SKIP AREA was requested.");
            // remove all paths in current area and return true
            mowerEnabled = false;
            currentMowingPaths.clear();
            skip_area = false;
            return true;
          }
          if (skip_path) {
            skip_path = false;
            currentMowingPath++;
            currentMowingPathIndex = 0;
            return false;
          }
          */
          if (aborted) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) ABORT was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            break;  // Trim path
          }
          if (requested_pause_flag) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) PAUSE was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            break;  // Trim path
          }
          if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE) {
            // show progress
            int currentIndex = getCurrentMowPathIndex();
            if (currentIndex != -1) {
              currentMowingPathIndex = exePathStartIndex + currentIndex;
            }
            ROS_INFO_STREAM_THROTTLE(
                5, "MowingBehavior: (MOW) Progress: " << currentMowingPathIndex << "/" << path.path.poses.size());
            // if ( ros::Time::now() - last_checkpoint > ros::Duration(30.0) ) checkpoint();

            // TODO: Do we need to check whether the goal is still the same (and active)?
            mower_msgs::MowPathsFeedback feedback;
            feedback.current_path = currentMowingPath;
            feedback.current_point = currentMowingPathIndex;
            mowPathsServer->publishFeedback(feedback);
          }
        } else {
          ROS_INFO_STREAM("MowingBehavior: (MOW)  Got status " << current_status.state_
                                                               << " from MBF/FTCPlanner -> Stopping path execution.");
          // we're done, break out of the loop
          break;
        }
        r.sleep();
      }

      // Only skip/trim if goal execution began
      if (current_status.state_ != actionlib::SimpleClientGoalState::PENDING &&
          current_status.state_ != actionlib::SimpleClientGoalState::RECALLED) {
        ROS_INFO_STREAM(">> MowingBehavior: (MOW) PlannerGetProgress currentMowingPathIndex = "
                        << currentMowingPathIndex << " of " << path.path.poses.size());
        printNavState(current_status.state_);
        // if we have fully processed the segment or we have encountered an error, drop the path segment
        /* TODO: we can not trust the SUCCEEDED state because the planner sometimes says suceeded with
            the currentIndex far from the size of the poses ! (BUG in planner ?)
            instead we trust only the currentIndex vs. poses.size() */
        if (currentMowingPathIndex >= path.path.poses.size() ||
            (path.path.poses.size() - currentMowingPathIndex) < 5)  // fully mowed the path ?
        {
          ROS_INFO_STREAM("MowingBehavior: (MOW) Mow path finished, skipping to next mow path.");
          currentMowingPath++;
          currentMowingPathIndex = 0;
          // continue with next segment
        } else {
          // we didnt drive all points in the mow path, so we go into pause mode
          // TODO: we should figure out the likely reason for our failure to complete the path
          // if GPS -> PAUSE
          // if something else -> Recovery Behaviour ?

          // currentMowingPathIndex might be 0 if we never consumed one of the points, we advance at least 1 point
          if (currentMowingPathIndex == 0) currentMowingPathIndex++;
          if (!requested_pause_flag) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) PAUSED due to MBF Error at " << currentMowingPathIndex);
            paused = true;
            update_actions();
          }
        }
      }
    }
  }

  mowerEnabled = false;

  // true, if we have executed all paths
  return currentMowingPath >= currentMowingPaths.size();
}

void MowingBehavior::command_home() {
  ROS_INFO_STREAM("MowingBehavior: HOME");

  // Require explicit approval to start mowing again
  ROS_INFO_STREAM("Preventing auto-start");
  auto config = getConfig();
  config.manual_pause_mowing = true;
  setConfig(config);

  if (paused) {
    // Request continue to wait for odom
    this->requestContinue();
    // Then instantly abort i.e. go to dock.
  }
  this->abort();
}

void MowingBehavior::command_start() {
  ROS_INFO_STREAM("MowingBehavior: MANUAL CONTINUE");

  auto config = getConfig();
  if (config.manual_pause_mowing) {
    // We are paused, user wants to resume, so store that immediately.
    // This way, once we are docked the mower will continue as soon as all other conditions are g2g
    ROS_INFO_STREAM("Re-enabling auto-start");
    config.manual_pause_mowing = false;
    setConfig(config);
  }

  this->requestContinue();
}

void MowingBehavior::command_s1() {
  ROS_INFO_STREAM("MowingBehavior: MANUAL PAUSED");
  this->requestPause();
}

void MowingBehavior::command_s2() {
  skip_area = true;
}

bool MowingBehavior::redirect_joystick() {
  return false;
}

uint8_t MowingBehavior::get_sub_state() {
  return 0;
}

uint8_t MowingBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

int16_t MowingBehavior::get_current_area() {
  return 0;  // FIXME
}

int16_t MowingBehavior::get_current_path() {
  return currentMowingPath;
}

int16_t MowingBehavior::get_current_path_index() {
  return currentMowingPathIndex;
}

MowingBehavior::MowingBehavior() {
  last_checkpoint = ros::Time(0.0);
  xbot_msgs::ActionInfo pause_action;
  pause_action.action_id = "mower_logic:mowing/pause";
  pause_action.enabled = false;
  pause_action.action_name = "Pause Mowing";

  xbot_msgs::ActionInfo continue_action;
  continue_action.action_id = "mower_logic:mowing/continue";
  continue_action.enabled = false;
  continue_action.action_name = "Continue Mowing";

  xbot_msgs::ActionInfo abort_mowing_action;
  abort_mowing_action.action_id = "mower_logic:mowing/abort_mowing";
  abort_mowing_action.enabled = false;
  abort_mowing_action.action_name = "Stop Mowing";

  xbot_msgs::ActionInfo skip_area_action;
  skip_area_action.action_id = "mower_logic:mowing/skip_area";
  skip_area_action.enabled = false;
  skip_area_action.action_name = "Skip Area";

  xbot_msgs::ActionInfo skip_path_action;
  skip_path_action.action_id = "mower_logic:mowing/skip_path";
  skip_path_action.enabled = false;
  skip_path_action.action_name = "Skip Path";

  actions.clear();
  actions.push_back(pause_action);
  actions.push_back(continue_action);
  actions.push_back(abort_mowing_action);
  actions.push_back(skip_area_action);
  actions.push_back(skip_path_action);
  // restore_checkpoint();
}

bool MowingBehavior::handle_action(const std::string &action, const std::string &payload, std::string &response) {
  if (action == "mower_logic:mowing/pause") {
    ROS_INFO_STREAM("got pause command");
    this->requestPause();
  } else if (action == "mower_logic:mowing/continue") {
    ROS_INFO_STREAM("got continue command");
    this->requestContinue();
  } else if (action == "mower_logic:mowing/abort_mowing") {
    ROS_INFO_STREAM("got abort mowing command");
    command_home();
  } else if (action == "mower_logic:mowing/skip_area") {
    ROS_INFO_STREAM("got skip_area command");
    skip_area = true;
  } else if (action == "mower_logic:mowing/skip_path") {
    ROS_INFO_STREAM("got skip_path command");
    skip_path = true;
  } else {
    return false;
  }
  update_actions();
  return true;
}
