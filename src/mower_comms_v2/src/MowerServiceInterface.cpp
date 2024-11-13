//
// Created by clemens on 25.07.24.
//

#include "MowerServiceInterface.h"

bool MowerServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  // No configuration
  return true;
}
void MowerServiceInterface::SetMowerEnabled(bool enabled) {
  SendMowerEnabled(enabled);
}
void MowerServiceInterface::OnMowerStatusChanged(const uint8_t& new_value) {
  status_msg_.mower_status = new_value;
}
void MowerServiceInterface::OnRainDetectedChanged(const uint8_t& new_value) {
  status_msg_.rain_detected = new_value;
}
void MowerServiceInterface::OnMowerRunningChanged(const uint8_t& new_value) {
  status_msg_.mow_enabled = new_value;
}
void MowerServiceInterface::OnMowerESCTemperatureChanged(const float& new_value) {
  status_msg_.mower_esc_temperature = new_value;
}
void MowerServiceInterface::OnMowerMotorTemperatureChanged(const float& new_value) {
  status_msg_.mower_motor_temperature = new_value;
}
void MowerServiceInterface::OnMowerMotorCurrentChanged(const float& new_value) {
  status_msg_.mower_esc_current = new_value;
}
void MowerServiceInterface::OnMowerMotorRPMChanged(const float& new_value) {
  status_msg_.mower_motor_rpm = new_value;
}
void MowerServiceInterface::OnServiceConnected(uint16_t service_id) {
  status_msg_ = {};
}
void MowerServiceInterface::OnTransactionStart(uint64_t timestamp) {
  status_msg_.stamp = ros::Time::now();
}
void MowerServiceInterface::OnTransactionEnd() {
  status_publisher_.publish(status_msg_);
}
void MowerServiceInterface::OnServiceDisconnected(uint16_t service_id) {
}
