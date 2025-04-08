//
// Created by clemens on 25.07.24.
//

#include "DiffDriveServiceInterface.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

bool DiffDriveServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  StartTransaction(true);
  SetRegisterWheelDistance(wheel_distance_);
  SetRegisterWheelTicksPerMeter(ticks_per_meter_);
  CommitTransaction();
  return true;
}

void DiffDriveServiceInterface::SendTwist(const geometry_msgs::TwistConstPtr& msg) {
  // Convert from ROS and publish
  double data[6]{};
  data[0] = msg->linear.x;
  data[1] = msg->linear.y;
  data[2] = msg->linear.z;
  data[3] = msg->angular.x;
  data[4] = msg->angular.y;
  data[5] = msg->angular.z;
  SendControlTwist(data, 6);
}

void DiffDriveServiceInterface::OnActualTwistChanged(const double* new_value, uint32_t length) {
  // 3 linear, 3 angular
  if (length == 6) {
    // Convert to ROS and publish
    geometry_msgs::TwistStamped twist;
    twist.header.frame_id = "base_link";
    twist.header.stamp = ros::Time::now();
    twist.header.seq = seq++;
    twist.twist.linear.x = new_value[0];
    twist.twist.linear.y = new_value[1];
    twist.twist.linear.z = new_value[2];
    twist.twist.angular.x = new_value[3];
    twist.twist.angular.y = new_value[4];
    twist.twist.angular.z = new_value[5];

    actual_twist_publisher_.publish(twist);
  }
}

void DiffDriveServiceInterface::OnLeftESCCurrentChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.current = new_value;
}

void DiffDriveServiceInterface::OnRightESCTemperatureChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  right_esc_state_.temperature_pcb = new_value;
}

void DiffDriveServiceInterface::OnRightESCCurrentChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  right_esc_state_.current = new_value;
}

void DiffDriveServiceInterface::OnWheelTicksChanged(const uint32_t* new_value, uint32_t length) {
  if (length != 2) return;
  left_esc_state_.tacho = new_value[0];
  right_esc_state_.tacho = new_value[1];
}

void DiffDriveServiceInterface::OnLeftESCTemperatureChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.temperature_pcb = new_value;
}

void DiffDriveServiceInterface::OnServiceConnected(uint16_t service_id) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
  right_esc_state_.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
}

void DiffDriveServiceInterface::OnLeftESCStatusChanged(const uint8_t& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.status = new_value;
}

void DiffDriveServiceInterface::OnRightESCStatusChanged(const uint8_t& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  right_esc_state_.status = new_value;
}

void DiffDriveServiceInterface::OnServiceDisconnected(uint16_t service_id) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
  right_esc_state_.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
}

void DiffDriveServiceInterface::OnTransactionEnd() {
  // Publish values to ROS
  left_esc_status_publisher_.publish(left_esc_state_);
  right_esc_status_publisher_.publish(right_esc_state_);
}
