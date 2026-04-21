// Stall-breaker: detects a wheel that is commanded to move but isn't turning
// (sensorless VESC below sl_min_erpm with low commanded duty) and briefly
// overrides the duty cycle with a hard pulse to kick the motor past the
// sensorless-control threshold.
//
// Header-only, ROS-free (only uses ros::Time so tests can inject a fake clock).

#ifndef MOWER_COMMS_V1_STALL_BREAKER_H
#define MOWER_COMMS_V1_STALL_BREAKER_H

#include <ros/time.h>

#include <cmath>
#include <string>

struct StallBreakerConfig {
  bool enabled = false;
  double min_cmd_duty = 0.05;
  int rpm_threshold = 50;
  int detect_time_ms = 300;
  double pulse_duty = 0.8;
  int pulse_time_ms = 500;
  int cooldown_time_ms = 1000;
  int max_consecutive_pulses = 5;
  // Sustained dwell above rpm_threshold that counts as "moving freely again"
  // and resets consecutive_pulses. Kept short so normal mowing passes reset.
  int motion_reset_ms = 200;
};

class StallBreaker {
 public:
  enum class State { IDLE, DETECTING, PULSING, COOLDOWN, UNRECOVERABLE };

  StallBreaker() = default;
  explicit StallBreaker(std::string name) : name_(std::move(name)) {}

  void setConfig(const StallBreakerConfig& cfg) {
    cfg_ = cfg;
  }
  const StallBreakerConfig& config() const {
    return cfg_;
  }
  State state() const {
    return state_;
  }
  int consecutive_pulses() const {
    return consecutive_pulses_;
  }
  const std::string& name() const {
    return name_;
  }

  // Call every control tick. Returns the duty cycle to actually send to the ESC.
  double update(double commanded_duty, int measured_rpm, const ros::Time& now) {
    if (!cfg_.enabled) {
      state_ = State::IDLE;
      consecutive_pulses_ = 0;
      return commanded_duty;
    }

    const double abs_cmd = std::fabs(commanded_duty);
    const int abs_rpm = std::abs(measured_rpm);
    const bool commanding = abs_cmd > cfg_.min_cmd_duty;
    const bool stalled = abs_rpm < cfg_.rpm_threshold;
    const bool moving_enough = abs_rpm >= cfg_.rpm_threshold;

    // Track sustained motion to reset consecutive_pulses after a real recovery.
    if (moving_enough) {
      if (motion_start_.isZero()) {
        motion_start_ = now;
      } else if ((now - motion_start_).toSec() * 1000.0 >= cfg_.motion_reset_ms) {
        consecutive_pulses_ = 0;
        if (state_ == State::UNRECOVERABLE) state_ = State::IDLE;
      }
    } else {
      motion_start_ = ros::Time();
    }

    switch (state_) {
      case State::IDLE:
        if (commanding && stalled) {
          state_ = State::DETECTING;
          detect_start_ = now;
        }
        return commanded_duty;

      case State::DETECTING:
        if (!commanding || !stalled) {
          state_ = State::IDLE;
          return commanded_duty;
        }
        if ((now - detect_start_).toSec() * 1000.0 >= cfg_.detect_time_ms) {
          if (consecutive_pulses_ >= cfg_.max_consecutive_pulses) {
            state_ = State::UNRECOVERABLE;
            return commanded_duty;
          }
          state_ = State::PULSING;
          pulse_start_ = now;
          pulse_sign_ = commanded_duty < 0 ? -1.0 : 1.0;
          consecutive_pulses_++;
          return pulse_sign_ * cfg_.pulse_duty;
        }
        return commanded_duty;

      case State::PULSING:
        if ((now - pulse_start_).toSec() * 1000.0 >= cfg_.pulse_time_ms) {
          state_ = State::COOLDOWN;
          cooldown_start_ = now;
          return commanded_duty;
        }
        return pulse_sign_ * cfg_.pulse_duty;

      case State::COOLDOWN:
        if ((now - cooldown_start_).toSec() * 1000.0 >= cfg_.cooldown_time_ms) {
          state_ = State::IDLE;
        }
        return commanded_duty;

      case State::UNRECOVERABLE:
        // Stay here until sustained motion resets (handled above).
        return commanded_duty;
    }
    return commanded_duty;
  }

 private:
  StallBreakerConfig cfg_{};
  std::string name_{};
  State state_ = State::IDLE;
  int consecutive_pulses_ = 0;
  double pulse_sign_ = 1.0;
  ros::Time detect_start_{};
  ros::Time pulse_start_{};
  ros::Time cooldown_start_{};
  ros::Time motion_start_{};
};

#endif  // MOWER_COMMS_V1_STALL_BREAKER_H
