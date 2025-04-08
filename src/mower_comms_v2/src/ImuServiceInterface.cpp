//
// Created by clemens on 26.07.24.
//

#include "ImuServiceInterface.h"

void ImuServiceInterface::OnAxesChanged(const double* new_value, uint32_t length) {
  if (length < 6) {
    return;
  }
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.seq++;
  imu_msg.header.frame_id = "base_link";
  imu_msg.linear_acceleration.x = new_value[0];
  imu_msg.linear_acceleration.y = new_value[1];
  imu_msg.linear_acceleration.z = new_value[2];
  imu_msg.angular_velocity.x = new_value[3];
  imu_msg.angular_velocity.y = new_value[4];
  imu_msg.angular_velocity.z = new_value[5];
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
  std::array<uint8_t, 3> axis_remap = {0, 1, 2};  // Default (YardForce mainboard) mapping: X->0, Y->1, Z->2
  std::array<int8_t, 3> axis_sign = {1, -1, -1};  // Default (YardForce mainboard) signs

  StartTransaction(true);

  if (validateAxisConfig()) {
    // Parse config string into axis_remap[] and axis_sign[]
    for (int i = 0; i < 3; ++i) {
      size_t pos = i * 2;                                  // Position within config string (0, 2, 4)
      axis_sign[i] = (axis_config_[pos] == '+') ? 1 : -1;  // Assign sign
      axis_remap[i] = axis_config_[pos + 1] - 'X';         // Assign mapping (via ASCII calc where 'X'->0, 'Y'->1, ...)
    }
  } else {
    ROS_ERROR_STREAM("Invalid IMU axis config: " << axis_config_ << "! Use YardForce +X-Y-Z mapping.");
  }
  SetRegisterAxisRemap(axis_remap.data(), axis_remap.size());
  SetRegisterAxisSign(axis_sign.data(), axis_sign.size());

  CommitTransaction();
  return true;
}