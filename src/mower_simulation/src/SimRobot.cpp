//
// Created by clemens on 29.11.24.
//

#include "SimRobot.h"

#include <nav_msgs/Odometry.h>
#include <spdlog/spdlog.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/thread/pthread/thread_data.hpp>

constexpr double SimRobot::BATTERY_VOLTS_MIN;
constexpr double SimRobot::BATTERY_VOLTS_MAX;
constexpr double SimRobot::CHARGE_CURRENT;
constexpr double SimRobot::CHARGE_VOLTS;

SimRobot::SimRobot(ros::NodeHandle& nh) : nh_{nh} {
}

void SimRobot::Start() {
  std::lock_guard<std::mutex> lk{state_mutex_};
  if (started_) {
    return;
  }
  started_ = true;
  actual_position_publisher_ = nh_.advertise<nav_msgs::Odometry>("actual_position", 50);
  timer_ = nh_.createTimer(ros::Duration(0.1), &SimRobot::SimulationStep, this);
  timer_.start();
}

void SimRobot::GetTwist(double& vx, double& vr) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  vx = vx_;
  vr = vr_;
  vx += linear_speed_noise(generator);
  vr += angular_speed_noise(generator);
}

void SimRobot::ResetEmergency() {
  std::lock_guard<std::mutex> lk{state_mutex_};
  emergency_active_ = false;
  emergency_latch_ = false;
}

void SimRobot::SetEmergency(bool active, const std::string& reason) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  emergency_active_ = active;
  emergency_latch_ |= active;
  emergency_reason_ = reason;
}

void SimRobot::GetEmergencyState(bool& active, bool& latch, std::string& reason) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  active = emergency_active_;
  latch = emergency_latch_;
  reason = emergency_reason_;
}

void SimRobot::SetControlTwist(double linear, double angular) {
  std::lock_guard<std::mutex> lk{state_mutex_};
  vx_ = linear;
  vr_ = angular;
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
  // Update Position if not in emergency mode
  if (!emergency_latch_) {
    double time_diff_s = (now - last_update_).toSec();
    double delta_x = (vx_ * cos(pos_heading_)) * time_diff_s;
    double delta_y = (vx_ * sin(pos_heading_)) * time_diff_s;
    double delta_th = vr_ * time_diff_s;
    pos_x_ += delta_x;
    pos_y_ += delta_y;
    pos_heading_ += delta_th;
    pos_heading_ = fmod(pos_heading_, M_PI * 2.0);
    if (pos_heading_ < 0) {
      pos_heading_ += M_PI * 2.0;
    }
  }

  // Update Charger Status
  if (sqrt((docking_pos_x_ - pos_x_) * (docking_pos_x_ - pos_x_) +
           (docking_pos_y_ - pos_y_) * (docking_pos_y_ - pos_y_)) < 0.5) {
    if (!is_charging_) {
      spdlog::info("Charging");
      is_charging_ = true;
      charging_started_time = ros::Time::now();
    }
  } else {
    if (is_charging_) {
      spdlog::info("Stopped Charging");
      is_charging_ = false;
    }
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

  last_update_ = now;

  {
    actual_position_.header.frame_id = "map";
    actual_position_.header.seq++;
    actual_position_.header.stamp = ros::Time::now();
    actual_position_.pose.pose.position.x = pos_x_;
    actual_position_.pose.pose.position.y = pos_y_;
    tf2::Quaternion q_mag(0.0, 0.0, pos_heading_);
    actual_position_.pose.pose.orientation = tf2::toMsg(q_mag);
    actual_position_publisher_.publish(actual_position_);
  }

  spdlog::debug("Position: x:{}, y:{}, heading:{}", pos_x_, pos_y_, pos_heading_);
}
