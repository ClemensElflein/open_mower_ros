//
// Created by clemens on 25.07.24.
//

#include "MowerServiceInterface.h"

bool MowerServiceInterface::OnConfigurationRequested(const std::string& uid) {
  // No configuration
  return true;
}
void MowerServiceInterface::OnMowerStatusChanged(const uint8_t& new_value) {
  status_msg_.mower_status = new_value;
}
void MowerServiceInterface::OnRaspberryPiPowerChanged(const uint8_t& new_value) {
  status_msg_.raspberry_pi_power = new_value;
}
void MowerServiceInterface::OnGPSPowerChanged(const uint8_t& new_value) {
  status_msg_.gps_power = new_value;
}
void MowerServiceInterface::OnESCPowerChanged(const uint8_t& new_value) {
  status_msg_.esc_power = new_value;
}
void MowerServiceInterface::OnRainDetectedChanged(const uint8_t& new_value) {
  status_msg_.rain_detected = new_value;
}
void MowerServiceInterface::OnChargeVoltageChanged(const float& new_value) {
  status_msg_.v_charge = new_value;
}
void MowerServiceInterface::OnBatteryVoltageChanged(const float& new_value) {
  status_msg_.v_battery = new_value;
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
void MowerServiceInterface::OnServiceConnected(const std::string& uid) {
  status_msg_ = {};
}
void MowerServiceInterface::OnTransactionStart(uint64_t timestamp) {
  status_msg_.stamp = ros::Time::now();
}
void MowerServiceInterface::OnTransactionEnd() {
  status_poblisher_.publish(status_msg_);
}
void MowerServiceInterface::OnServiceDisconnected(const std::string& uid) {
}
