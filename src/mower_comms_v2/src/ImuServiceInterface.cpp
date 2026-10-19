//
// Created by clemens on 26.07.24.
//

#include "ImuServiceInterface.h"

#include <algorithm>
#include <cmath>

namespace {
// Gyro rates arrive quantised to 1 deg/s, so quantisation dominates the noise:
// (1 deg/s)^2 / 12 = 2.54e-5 (rad/s)^2.
constexpr double kAngularVelocityVariance = 2.54e-5;
// Nothing currently consumes the accelerometer; this is a deliberately
// conservative placeholder rather than a measured figure.
constexpr double kLinearAccelerationVariance = 1e-2;
}  // namespace

void ImuServiceInterface::OnTransactionStart(uint64_t timestamp) {
  transaction_timestamp_ = timestamp;
}

ros::Time ImuServiceInterface::stampFromFirmware(uint64_t fw_micros) {
  const double now = ros::Time::now().toSec();
  if (fw_micros == 0) {
    return ros::Time(now);
  }
  const double fw_s = static_cast<double>(fw_micros) * 1e-6;
  const double observed = now - fw_s;

  // The offset between the firmware clock and ROS time is the transit delay we cannot see. 
  // Tracking the minimum rejects scheduling jitter instead of baking it into every stamp.
  if (!have_offset_ || std::abs(observed - fw_to_ros_offset_) > 5.0) {
    // First packet, or the firmware rebooted and its clock restarted.
    fw_to_ros_offset_ = observed;
    have_offset_ = true;
  } else {
    // Leak upward slowly (~100 ppm) so the estimate can follow crystal drift
    // rather than latching onto one unusually early packet forever.
    constexpr double kLeakPerSample = 2e-6;
    fw_to_ros_offset_ = std::min(observed, fw_to_ros_offset_ + kLeakPerSample);
  }

  // Never hand out a stamp from the future - consumers differentiate these.
  return ros::Time(std::min(fw_s + fw_to_ros_offset_, now));
}

void ImuServiceInterface::OnAxesChanged(const double* new_value, uint32_t length) {
  if (length < 6) {
    return;
  }
  // Stamp from the firmware clock, not from arrival time. 
  // Stamping on arrival feeds that jitter straight into any integrated heading.
  imu_msg.header.stamp = stampFromFirmware(transaction_timestamp_);
  imu_msg.header.seq++;
  imu_msg.header.frame_id = "base_link";
  imu_msg.linear_acceleration.x = new_value[0];
  imu_msg.linear_acceleration.y = new_value[1];
  imu_msg.linear_acceleration.z = new_value[2];
  imu_msg.angular_velocity.x = new_value[3];
  imu_msg.angular_velocity.y = new_value[4];
  imu_msg.angular_velocity.z = new_value[5];
  
  // -1 in the first covariance element is the sensor_msgs convention for "no estimate".
  imu_msg.orientation_covariance[0] = -1.0;

  imu_msg.angular_velocity_covariance[0] = kAngularVelocityVariance;
  imu_msg.angular_velocity_covariance[4] = kAngularVelocityVariance;
  imu_msg.angular_velocity_covariance[8] = kAngularVelocityVariance;
  imu_msg.linear_acceleration_covariance[0] = kLinearAccelerationVariance;
  imu_msg.linear_acceleration_covariance[4] = kLinearAccelerationVariance;
  imu_msg.linear_acceleration_covariance[8] = kLinearAccelerationVariance;

  imu_publisher_.publish(imu_msg);
}

bool ImuServiceInterface::validateAxisConfig() {
  if (axis_config_.size() != 6) {
    ROS_ERROR_STREAM("Invalid IMU axis config length of '" << axis_config_ << "': " << axis_config_.size() << " != 6");
    return false;
  }

  // Validate syntax: (+|-)(X|Y|Z)
  for (size_t i = 0; i < 6; i += 2) {
    if ((axis_config_[i] != '+' && axis_config_[i] != '-') ||
        (axis_config_[i + 1] != 'X' && axis_config_[i + 1] != 'Y' && axis_config_[i + 1] != 'Z')) {
      ROS_ERROR_STREAM("Invalid IMU axis config syntax ((+|-)(X|Y|Z)){3}: " << axis_config_);
      return false;
    }
  }

  // Validate on double axis use
  if (axis_config_[1] == axis_config_[3] || axis_config_[1] == axis_config_[5] || axis_config_[3] == axis_config_[5]) {
    ROS_ERROR_STREAM("Invalid IMU axis config '" << axis_config_ << "'! Double axis usage");
    return false;
  }

  return true;
}

bool ImuServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  std::array<int8_t, 3> axis_remap = {1, -2, -3};  // Default (YardForce mainboard) mapping: X=1, Y=-2, Z=-3

  StartTransaction(true);

  if (validateAxisConfig()) {
    // Parse config string into axis_remap[] and axis_sign[]
    for (int i = 0; i < 3; ++i) {
      const size_t pos = i * 2;                                 // Position within config string (0, 2, 4)
      const int8_t sign = (axis_config_[pos] == '+') ? 1 : -1;  // Axis sign

      // ASCII-based axis calculation
      const char axis = axis_config_[pos + 1];
      const int8_t axis_num = static_cast<int8_t>((axis - 'X') + 1);  // X->1, Y->2, Z->3
      assert(axis_num >= 1 && axis_num <= 3);

      axis_remap[i] = sign * axis_num;
    }
  } else {
    ROS_ERROR_STREAM("Invalid IMU axis config: " << axis_config_ << "! Using default YardForce +X-Y-Z mapping.");
  }
  SetRegisterAxisRemap(axis_remap.data(), axis_remap.size());
  CommitTransaction();

  return true;
}
