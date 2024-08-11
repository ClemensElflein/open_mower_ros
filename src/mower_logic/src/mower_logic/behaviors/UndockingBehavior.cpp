// Created by Clemens Elflein on 2/21/22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based
// on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
#include "UndockingBehavior.h"

extern ros::ServiceClient dockingPointClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfClientExePath;
extern xbot_msgs::AbsolutePose getPose();
extern mower_msgs::Status getStatus();

extern void setRobotPose(geometry_msgs::Pose &pose);
extern void stopMoving();
extern bool isGpsGood();
extern bool setGPS(bool enabled);

extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo> &actions);

UndockingBehavior UndockingBehavior::INSTANCE(&MowingBehavior::INSTANCE);
UndockingBehavior UndockingBehavior::RETRY_INSTANCE(&DockingBehavior::INSTANCE);

UndockingBehavior::UndockingBehavior() {
  xbot_msgs::ActionInfo abort_docking_action;
  abort_docking_action.action_id = "abort_undocking";
  abort_docking_action.enabled = true;
  abort_docking_action.action_name = "Stop Undocking";

  actions.clear();
  actions.push_back(abort_docking_action);
}

std::string UndockingBehavior::state_name() {
  return "UNDOCKING";
}

Behavior *UndockingBehavior::execute() {
  // get robot's current pose from odometry.
  xbot_msgs::AbsolutePose pose = getPose();
  tf2::Quaternion quat;
  tf2::fromMsg(pose.pose.pose.orientation, quat);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  mbf_msgs::ExePathGoal exePathGoal;

  nav_msgs::Path path;

  int undock_point_count = config.undock_distance * 10.0;
  for (int i = 0; i < undock_point_count; i++) {
    geometry_msgs::PoseStamped docking_pose_stamped_front;
    docking_pose_stamped_front.pose = pose.pose.pose;
    docking_pose_stamped_front.header = pose.header;
    docking_pose_stamped_front.pose.position.x -= cos(yaw) * (i / 10.0);
    docking_pose_stamped_front.pose.position.y -= sin(yaw) * (i / 10.0);
    path.poses.push_back(docking_pose_stamped_front);
  }

  exePathGoal.path = path;
  exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
  exePathGoal.dist_tolerance = 0.1;
  exePathGoal.tolerance_from_action = true;
  exePathGoal.controller = "DockingFTCPlanner";

  mbfClientExePath->sendGoal(exePathGoal);

  ros::Rate r(10);
  bool waitingForResult = true;
  bool success = false;

  while (waitingForResult) {
    r.sleep();
    if (aborted) {
      ROS_INFO_STREAM("Undocking aborted.");
      mbfClientExePath->cancelGoal();
      stopMoving();
      return &IdleBehavior::INSTANCE;
    }

    switch (mbfClientExePath->getState().state_) {
      case actionlib::SimpleClientGoalState::ACTIVE:
      case actionlib::SimpleClientGoalState::PENDING: break;
      case actionlib::SimpleClientGoalState::SUCCEEDED:
        waitingForResult = false;
        success = true;
        break;
      default: waitingForResult = false; break;
    }
  }

  // stop the bot for now
  stopMoving();

  if (!success) {
    ROS_ERROR_STREAM("Error during undock");
    return &IdleBehavior::INSTANCE;
  }

  ROS_INFO_STREAM("Undock success. Waiting for GPS.");
  bool hasGps = waitForGPS();

  if (!hasGps) {
    ROS_ERROR_STREAM("Could not get GPS.");
    return &IdleBehavior::INSTANCE;
  }

  // TODO return mow area
  return nextBehavior;
}

void UndockingBehavior::enter() {
  reset();
  paused = aborted = false;

  // Get the docking pose in map
  mower_map::GetDockingPointSrv get_docking_point_srv;
  dockingPointClient.call(get_docking_point_srv);
  docking_pose_stamped.pose = get_docking_point_srv.response.docking_pose;
  docking_pose_stamped.header.frame_id = "map";
  docking_pose_stamped.header.stamp = ros::Time::now();

  // set the robot's position to the dock if we're actually docked
  if (getStatus().v_charge > 5.0) {
    ROS_INFO_STREAM("Currently inside the docking station, we set the robot's pose to the docks pose.");
    setRobotPose(docking_pose_stamped.pose);
  }

  for (auto &a : actions) {
    a.enabled = true;
  }
  registerActions("mower_logic:undocking", actions);
}

void UndockingBehavior::exit() {
  for (auto &a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:undocking", actions);
}

void UndockingBehavior::reset() {
  gpsRequired = false;
}

bool UndockingBehavior::needs_gps() {
  return gpsRequired;
}

bool UndockingBehavior::mower_enabled() {
  // No mower during docking
  return false;
}

bool UndockingBehavior::waitForGPS() {
  gpsRequired = false;
  setGPS(true);
  ros::Rate odom_rate(1.0);
  while (ros::ok() && !aborted) {
    if (isGpsGood()) {
      ROS_INFO("Got good gps, let's go");
      break;
    } else {
      ROS_INFO_STREAM("waiting for gps. current accuracy: " << getPose().position_accuracy);
      odom_rate.sleep();
    }
  }
  if (!ros::ok() || aborted) {
    return false;
  }

  // wait additional time for odometry filters to converge
  ros::Rate r(ros::Duration(config.gps_wait_time, 0));
  r.sleep();

  gpsRequired = true;

  return true;
}

UndockingBehavior::UndockingBehavior(Behavior *next) {
  this->nextBehavior = next;
}

void UndockingBehavior::command_home() {
  this->abort();
}

void UndockingBehavior::command_start() {
}

void UndockingBehavior::command_s1() {
}

void UndockingBehavior::command_s2() {
}

bool UndockingBehavior::redirect_joystick() {
  return false;
}

uint8_t UndockingBehavior::get_sub_state() {
  return 2;
}
uint8_t UndockingBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

void UndockingBehavior::handle_action(std::string action) {
  if (action == "mower_logic:undocking/abort_undocking") {
    ROS_INFO_STREAM("Got abort undocking command");
    command_home();
  }
}
