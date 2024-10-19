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

#include "tf2_eigen/tf2_eigen.h"

extern ros::ServiceClient dockingPointClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction> *mbfClientExePath;
extern xbot_msgs::AbsolutePose getPose();
extern mower_msgs::Status getStatus();

extern void setRobotPose(geometry_msgs::Pose &pose);
extern void stopMoving();
extern bool isGpsGood();
extern bool setGPS(bool enabled);

UndockingBehavior UndockingBehavior::INSTANCE(&MowingBehavior::INSTANCE);
UndockingBehavior UndockingBehavior::RETRY_INSTANCE(&DockingBehavior::INSTANCE);

std::string UndockingBehavior::state_name() {
  return "UNDOCKING";
}

Behavior *UndockingBehavior::execute() {
  static bool rng_seeding_required = true;

  // get robot's current pose from odometry.
  xbot_msgs::AbsolutePose pose = getPose();
  tf2::Quaternion quat;
  tf2::fromMsg(pose.pose.pose.orientation, quat);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  mbf_msgs::ExePathGoal exePathGoal;

  nav_msgs::Path path;

  geometry_msgs::PoseStamped docking_pose_stamped_front;
  docking_pose_stamped_front.pose = pose.pose.pose;
  docking_pose_stamped_front.header = pose.header;

  const int straight_undock_point_count = 3;  // The FTC planner requires at least 3 points to work
  double incremental_distance = config.undock_distance / straight_undock_point_count;
  for (int i = 0; i < straight_undock_point_count; i++) {
    docking_pose_stamped_front.pose.position.x -= cos(yaw) * incremental_distance;
    docking_pose_stamped_front.pose.position.y -= sin(yaw) * incremental_distance;
    path.poses.push_back(docking_pose_stamped_front);
  }

  double angle;
  if (config.undock_fixed_angle) {
    angle = config.undock_angle * M_PI / 180.0;
    ROS_INFO_STREAM("Fixed angle undock: " << config.undock_angle);
  } else {
    // seed based on first undock time rather than boot so should be ok even without RTC
    if (rng_seeding_required) {
      srand(ros::Time::now().toSec());
      ROS_INFO_STREAM("Random angle undock: Seeded rand()");
      rng_seeding_required = false;
    }
    double random_number = ((double)rand() / RAND_MAX) * 2.0 - 1.0;
    double random_angle_deg = abs(config.undock_angle) * random_number;
    ROS_INFO_STREAM("Random angle undock: " << random_angle_deg);
    angle = random_angle_deg * M_PI / 180.0;
  }

  const int angled_undock_point_count = 10;
  incremental_distance = config.undock_angled_distance / angled_undock_point_count;
  for (int i = 0; i < angled_undock_point_count; i++) {
    double orientation = yaw + angle * (config.undock_use_curve ? ((i + 1.0) / angled_undock_point_count) : 1);

    docking_pose_stamped_front.pose.position.x -= cos(orientation) * incremental_distance;
    docking_pose_stamped_front.pose.position.y -= sin(orientation) * incremental_distance;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, orientation);
    docking_pose_stamped_front.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(docking_pose_stamped_front);
  }

  exePathGoal.path = path;
  exePathGoal.angle_tolerance = 1.0 * (M_PI / 180.0);
  exePathGoal.dist_tolerance = 0.1;
  exePathGoal.tolerance_from_action = true;
  exePathGoal.controller = "DockingFTCPlanner";

  auto result = mbfClientExePath->sendGoalAndWait(exePathGoal);

  bool success = result.state_ == actionlib::SimpleClientGoalState::SUCCEEDED;

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
}

void UndockingBehavior::exit() {
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
}
