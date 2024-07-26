//
// Created by clemens on 25.07.24.
//

#include "DiffDriveServiceInterface.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

bool DiffDriveServiceInterface::OnConfigurationRequested(const std::string& uid) {
  StartTransaction(true);
  SetRegisterWheelDistance(wheel_distance_);
  SetRegisterWheelTicksPerMeter(ticks_per_meter_);
  CommitTransaction();
  return true;
}

void DiffDriveServiceInterface::SendTwist(const geometry_msgs::TwistConstPtr& msg) {
  double data[6]{};
  data[0] = msg->linear.x;
  data[1] = msg->linear.y;
  data[2] = msg->linear.z;
  data[3] = msg->angular.x;
  data[4] = msg->angular.y;
  data[5] = msg->angular.z;
  SendControlTwist(data, 6);
}
void DiffDriveServiceInterface::GetEscInfo(bool& left_error, float& left_esc_temperature, float& left_esc_current,
                                           bool& right_error, float& right_esc_temperature, float& right_esc_current) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_error = left_esc_state_.error;
  left_esc_temperature = left_esc_state_.temperature;
  left_esc_current = left_esc_state_.current;
  right_error = right_esc_state_.error;
  right_esc_temperature = right_esc_state_.temperature;
  right_esc_current = right_esc_state_.current;
}
void DiffDriveServiceInterface::OnActualTwistChanged(const double* new_value, uint32_t length) {
  // 3 linear, 3 angular
  if (length == 6) {
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
  right_esc_state_.temperature = new_value;
}
void DiffDriveServiceInterface::OnRightESCCurrentChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  right_esc_state_.current = new_value;
}
void DiffDriveServiceInterface::OnLeftESCTemperatureChanged(const float& new_value) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.temperature = new_value;
}
void DiffDriveServiceInterface::OnServiceConnected(const std::string& uid) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.error = false;
  right_esc_state_.error = false;
}
void DiffDriveServiceInterface::OnTransactionStart(uint64_t timestamp) {
}
void DiffDriveServiceInterface::OnTransactionEnd() {
}
void DiffDriveServiceInterface::OnServiceDisconnected(const std::string& uid) {
  std::unique_lock<std::mutex> lk{state_mutex_};
  left_esc_state_.error = true;
  right_esc_state_.error = true;
}
