//
// Created by clemens on 25.07.24.
//

#include "MowerServiceInterface.h"

void MowerServiceInterface::Tick() {
  SendMowerEnabled(status_msg_.mow_enabled);
}

void MowerServiceInterface::SetMowerEnabled(bool enabled) {
  SendMowerEnabled(enabled);
  status_msg_.mow_enabled = enabled;
  status_publisher_.publish(status_msg_);
}

void MowerServiceInterface::OnMowerStatusChanged(const uint8_t &new_value) {
  status_msg_.mower_status = new_value;
}

void MowerServiceInterface::OnRainDetectedChanged(const uint8_t &new_value) {
  status_msg_.rain_detected = new_value;
}

void MowerServiceInterface::OnMowerRunningChanged(const uint8_t &new_value) {
  // TODO: set a flag, if the mower is actually running or not.
}

void MowerServiceInterface::OnMowerESCTemperatureChanged(const float &new_value) {
  status_msg_.mower_esc_temperature = new_value;
}

void MowerServiceInterface::OnMowerMotorTemperatureChanged(const float &new_value) {
  status_msg_.mower_motor_temperature = new_value;
}

void MowerServiceInterface::OnMowerMotorCurrentChanged(const float &new_value) {
  status_msg_.mower_esc_current = new_value;
}

void MowerServiceInterface::OnMowerMotorRPMChanged(const float &new_value) {
  status_msg_.mower_motor_rpm = new_value;
}

void MowerServiceInterface::OnServiceConnected(uint16_t service_id) {
  status_msg_ = {};
  SendMowerEnabled(false);
}

void MowerServiceInterface::OnTransactionStart(uint64_t timestamp) {
  status_msg_.stamp = ros::Time::now();
}

void MowerServiceInterface::OnTransactionEnd() {
  status_publisher_.publish(status_msg_);
}
