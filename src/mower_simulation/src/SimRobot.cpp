//
// Created by clemens on 29.11.24.
//

#include "SimRobot.h"

#include <nav_msgs/Odometry.h>
#include <spdlog/spdlog.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <xbot_msgs/AbsolutePose.h>

#include <algorithm>
#include <boost/thread/pthread/thread_data.hpp>

constexpr double SimRobot::BATTERY_VOLTS_MIN;
constexpr double SimRobot::BATTERY_VOLTS_MAX;
constexpr double SimRobot::CHARGE_CURRENT;
constexpr double SimRobot::CHARGE_VOLTS;

SimRobot::SimRobot(ros::NodeHandle& nh) : nh_{nh} {
  nh_.param("publish_tf", publish_tf_, true);
}

void SimRobot::Start() {
  std::lock_guard<std::mutex> lk{state_mutex_};
  if (started_) {
    return;
  }
  started_ = true;
  gps_service_ = nh_.advertiseService("/xbot_positioning/set_gps_state", &SimRobot::OnSetGpsState, this);
  pose_service_ = nh_.advertiseService("/xbot_positioning/set_robot_pose", &SimRobot::OnSetPose, this);
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_out", 50);
  xbot_absolute_pose_pub_ = nh_.advertise<xbot_msgs::AbsolutePose>("xb_pose_out", 50);
  joy_vel_sub_ = nh_.subscribe("/joy_vel", 1, &SimRobot::OnJoyVel, this, ros::TransportHints().tcpNoDelay(true));
  // Keep in sync with DiffDriveService::tick_schedule_
  timer_ = nh_.createTimer(ros::Duration(0.02), &SimRobot::SimulationStep, this);
  timer_.start();
}

bool SimRobot::OnSetGpsState(xbot_positioning::GPSControlSrvRequest& req,
                             xbot_positioning::GPSControlSrvResponse& res) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  gps_good_ = req.gps_enabled;
  return true;
}

bool SimRobot::OnSetPose(xbot_positioning::SetPoseSrvRequest& req, xbot_positioning::SetPoseSrvResponse& res) {
  // Ignored, because calling this service won't move a real mower either, it just improves the estimation.
  return true;
}

void SimRobot::GetTwist(double& vx, double& vr) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  vx = last_noisy_vx_;
  vr = last_noisy_vr_;
}

void SimRobot::ApplyEmergencyUpdate(uint16_t add, uint16_t clear) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  emergency_reasons_ = (emergency_reasons_ & ~clear) | add;
}

void SimRobot::TriggerEmergency() {
  std::lock_guard<std::mutex> lk{state_mutex_};
  // Latch it: LATCH is never cleared by the high level's heartbeat (which only clears
  // HIGH_LEVEL), so the emergency stays until the high level explicitly resets it.
  emergency_reasons_ |= EmergencyReason::HIGH_LEVEL | EmergencyReason::LATCH;
}

void SimRobot::ClearEmergency() {
  std::lock_guard<std::mutex> lk{state_mutex_};
  emergency_reasons_ = 0;
}

void SimRobot::GetEmergencyState(bool& active, bool& latch, uint16_t& reason) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  reason = emergency_reasons_;
  active = emergency_reasons_ != 0;
  latch = (emergency_reasons_ & EmergencyReason::LATCH) != 0;
}

void SimRobot::SetMovementAllowed(bool allowed) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  movement_allowed_ = allowed;
}

void SimRobot::SetGpsGood(bool good) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  gps_good_ = good;
}

bool SimRobot::IsGpsGood() {
  std::lock_guard<std::mutex> lk{state_mutex_};
  return gps_good_;
}

void SimRobot::SetBatteryFull(bool full) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  battery_volts_ = full ? BATTERY_VOLTS_MAX : BATTERY_VOLTS_MIN;
}

void SimRobot::SetBatteryVolts(double volts) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  // Allow over/under-voltage for fault simulation; only guard against negatives.
  battery_volts_ = std::max(0.0, volts);
}

SimRobot::SimControlState SimRobot::GetSimControlState() {
  std::lock_guard<std::mutex> lk{state_mutex_};
  SimControlState state{};
  state.emergency_active = emergency_reasons_ != 0;
  state.emergency_latch = (emergency_reasons_ & EmergencyReason::LATCH) != 0;
  state.emergency_reason = emergency_reasons_;
  state.movement_allowed = movement_allowed_;
  state.gps_good = gps_good_;
  state.battery_voltage = battery_volts_;
  state.battery_percentage =
      std::max(0.0, std::min(1.0, (battery_volts_ - BATTERY_VOLTS_MIN) / (BATTERY_VOLTS_MAX - BATTERY_VOLTS_MIN)));
  state.charging = is_charging_;
  state.joy_override = joy_override_;
  return state;
}

void SimRobot::SetControlTwist(double linear, double angular) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  if (joy_override_) {
    return;
  }
  vx_ = linear;
  vr_ = angular;
}

void SimRobot::SetJoyOverride(bool enabled) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  joy_override_ = enabled;
  if (!enabled) {
    vx_ = 0.0;
    vr_ = 0.0;
  }
}

void SimRobot::OnJoyVel(const geometry_msgs::Twist::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  if (!joy_override_) {
    return;
  }
  vx_ = msg->linear.x;
  vr_ = msg->angular.z;
}

void SimRobot::Displace(double dx, double dy, double dheading) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  pos_x_ += dx;
  pos_y_ += dy;
  pos_heading_ = fmod(pos_heading_ + dheading, M_PI * 2.0);
  if (pos_heading_ < 0) {
    pos_heading_ += M_PI * 2.0;
  }
}

void SimRobot::MoveToDock() {
  std::lock_guard<std::mutex> lk{state_mutex_};
  pos_x_ = docking_pos_x_;
  pos_y_ = docking_pos_y_;
  pos_heading_ = docking_pos_heading_;
  is_charging_ = true;
  charging_started_time = ros::Time::now();
}

void SimRobot::GetPosition(double& x, double& y, double& heading) {
  std::lock_guard<std::mutex> lk{state_mutex_};

  x = pos_x_;
  y = pos_y_;
  heading = pos_heading_;

  x += position_noise(generator);
  y += position_noise(generator);
  heading += heading_noise(generator);
  heading = fmod(heading, M_PI * 2.0);
  while (heading < 0) {
    heading += M_PI * 2.0;
  }
}

void SimRobot::SetPosition(const double x, const double y, const double heading) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  pos_x_ = x;
  pos_y_ = y;
  pos_heading_ = heading;
}

void SimRobot::SetDockingPose(const double x, const double y, const double heading) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  docking_pos_x_ = x;
  docking_pos_y_ = y;
  docking_pos_heading_ = heading;
}

void SimRobot::GetIsCharging(bool& charging, double& seconds_since_start, std::string& charging_status,
                             double& charger_volts, double& battery_volts, double& charging_current) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  charging = is_charging_;
  seconds_since_start = (ros::Time::now() - charging_started_time).toSec();
  charging_status = charger_state_;
  charger_volts = charger_volts_;
  charging_current = charge_current_;
  battery_volts = battery_volts_;
}

void SimRobot::SimulationStep(const ros::TimerEvent& te) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  const auto now = ros::Time::now();
  if (last_update_.isZero()) {
    last_update_ = now;
    PublishPosition();
    return;
  }
  // Update Position if not in emergency mode. Any emergency reason (latch, timeout, ...)
  // stops the robot, mirroring the firmware.
  if (emergency_reasons_ != 0) {
    last_noisy_vx_ = 0.0;
    last_noisy_vr_ = 0.0;
  } else {
    double time_diff_s = (now - last_update_).toSec();

    // Ground truth integrates the commanded twist, not a noisy one. Noise belongs on
    // the *reported* sensor values (wheel odometry / gyro) below, not fed back into the
    // robot's actual simulated trajectory - otherwise sensor noise becomes a genuine
    // random walk in the real path, which is what made the sim "drive like on rubber".
    //
    // When movement is disallowed the robot is "stuck": the ground-truth position is
    // frozen (so GPS reports no motion) while the reported wheel odometry / gyro below
    // still follows the commanded twist (wheels keep turning).
    if (movement_allowed_) {
      if (fabs(vr_) > 1e-6) {
        double r = vx_ / vr_;
        pos_x_ += r * (sin(pos_heading_ + vr_ * time_diff_s) - sin(pos_heading_));
        pos_y_ -= r * (cos(pos_heading_ + vr_ * time_diff_s) - cos(pos_heading_));
        pos_heading_ += vr_ * time_diff_s;
      } else {
        pos_x_ += vx_ * cos(pos_heading_) * time_diff_s;
        pos_y_ += vx_ * sin(pos_heading_) * time_diff_s;
      }
      pos_heading_ = fmod(pos_heading_, M_PI * 2.0);
      if (pos_heading_ < 0) {
        pos_heading_ += M_PI * 2.0;
      }
    }

    // Skip noise when the robot is commanded to rest; a stationary mower does not
    // random-walk its reported odometry either.
    const bool at_rest = (vx_ == 0.0 && vr_ == 0.0);
    last_noisy_vx_ = at_rest ? 0.0 : vx_ + linear_speed_noise(generator);
    last_noisy_vr_ = at_rest ? 0.0 : vr_ + angular_speed_noise(generator);
  }

  // Update Charger Status
  // Hysteresis: engage when closer than 0.02 m, disengage only when farther than 0.03 m,
  // preventing oscillation caused by noise in the integrated position.
  const double dock_dist = sqrt((docking_pos_x_ - pos_x_) * (docking_pos_x_ - pos_x_) +
                                (docking_pos_y_ - pos_y_) * (docking_pos_y_ - pos_y_));
  if (!is_charging_ && dock_dist < 0.02) {
    spdlog::info("Charging");
    is_charging_ = true;
    charging_started_time = ros::Time::now();
    // Snap to the exact docking pose to eliminate any pre-latch drift so the
    // disengage threshold cannot be tripped while the robot sits on the dock.
    pos_x_ = docking_pos_x_;
    pos_y_ = docking_pos_y_;
  } else if (is_charging_ && dock_dist > 0.03) {
    spdlog::info("Stopped Charging");
    is_charging_ = false;
  }

  if (is_charging_) {
    if (battery_volts_ < BATTERY_VOLTS_MAX) {
      charger_state_ = "CC";
      battery_volts_ += 0.05;
      if (battery_volts_ > BATTERY_VOLTS_MAX) {
        battery_volts_ = BATTERY_VOLTS_MAX;
      }
      charger_volts_ = CHARGE_VOLTS;
      charge_current_ = CHARGE_CURRENT;
    } else if (charge_current_ > 0.2) {
      charger_state_ = "CV";
      battery_volts_ = BATTERY_VOLTS_MAX;
      charger_volts_ = CHARGE_VOLTS;
      charge_current_ = charge_current_ * 0.99;
    } else {
      charger_state_ = "Done";
      battery_volts_ = BATTERY_VOLTS_MAX;
      charger_volts_ = CHARGE_VOLTS;
      charge_current_ = 0;
    }
  } else {
    charger_state_ = "Not Charging";
    battery_volts_ = std::max(BATTERY_VOLTS_MIN, battery_volts_ - 0.001);
    charger_volts_ = 0.0;
    charge_current_ = 0.0;
  }

  PublishPosition();

  last_update_ = now;
}

void SimRobot::PublishPosition() {
  nav_msgs::Odometry odometry;
  geometry_msgs::TransformStamped odom_trans;
  xbot_msgs::AbsolutePose xb_absolute_pose_msg;
  static tf2_ros::TransformBroadcaster transform_broadcaster;

  odometry.header.seq++;
  odometry.header.stamp = ros::Time::now();
  odometry.header.frame_id = "map";
  odometry.child_frame_id = "base_link";
  odometry.pose.pose.position.x = pos_x_;
  odometry.pose.pose.position.y = pos_y_;
  tf2::Quaternion q_mag;
  q_mag.setRPY(0.0, 0.0, pos_heading_);
  odometry.pose.pose.orientation = tf2::toMsg(q_mag);

  odom_trans.header = odometry.header;
  odom_trans.child_frame_id = odometry.child_frame_id;
  odom_trans.transform.translation.x = odometry.pose.pose.position.x;
  odom_trans.transform.translation.y = odometry.pose.pose.position.y;
  odom_trans.transform.translation.z = odometry.pose.pose.position.z;
  odom_trans.transform.rotation = odometry.pose.pose.orientation;

  xb_absolute_pose_msg.header = odometry.header;
  xb_absolute_pose_msg.sensor_stamp = 0;
  xb_absolute_pose_msg.received_stamp = 0;
  xb_absolute_pose_msg.source = xbot_msgs::AbsolutePose::SOURCE_SENSOR_FUSION;
  xb_absolute_pose_msg.flags =
      xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE |
      xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_DEAD_RECKONING | xbot_msgs::AbsolutePose::FLAG_GPS_RTK |
      (gps_good_ ? xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED : xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT);
  xb_absolute_pose_msg.orientation_valid = true;
  xb_absolute_pose_msg.motion_vector_valid = false;
  // Good GPS: RTK fix, ~2 cm. Bad GPS: no RTK fix, ~1 m.
  xb_absolute_pose_msg.position_accuracy = gps_good_ ? 0.02 : 1.0;
  xb_absolute_pose_msg.orientation_accuracy = 0.01;
  xb_absolute_pose_msg.pose = odometry.pose;
  xb_absolute_pose_msg.vehicle_heading = pos_heading_;
  xb_absolute_pose_msg.motion_heading = pos_heading_;

  odometry_pub_.publish(odometry);
  if (publish_tf_) {
    transform_broadcaster.sendTransform(odom_trans);
  }
  xbot_absolute_pose_pub_.publish(xb_absolute_pose_msg);

  spdlog::debug("Position: x:{}, y:{}, heading:{}", pos_x_, pos_y_, pos_heading_);
}
