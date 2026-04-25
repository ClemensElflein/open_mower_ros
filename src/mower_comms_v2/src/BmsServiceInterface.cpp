/*
 * OpenMower
 * Part of open_mower_ros (https://github.com/ClemensElflein/open_mower_ros)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "BmsServiceInterface.h"

void BmsServiceInterface::OnVoltageChanged(const float& new_value) {
  bms_msg_.voltage = new_value;
}

void BmsServiceInterface::OnCurrentChanged(const float& new_value) {
  bms_msg_.current = new_value;
}

void BmsServiceInterface::OnRelativeStateOfChargeChanged(const float& new_value) {
  bms_msg_.relative_state_of_charge = new_value;
}

void BmsServiceInterface::OnRemainingCapacityChanged(const float& new_value) {
  bms_msg_.remaining_capacity = new_value;
}

void BmsServiceInterface::OnFullChargeCapacityChanged(const float& new_value) {
  bms_msg_.full_charge_capacity = new_value;
}

void BmsServiceInterface::OnCycleCountChanged(const uint16_t& new_value) {
  bms_msg_.cycle_count = new_value;
}

void BmsServiceInterface::OnTemperatureChanged(const float& new_value) {
  bms_msg_.temperature = new_value;
}

void BmsServiceInterface::OnBatteryStatusChanged(const uint16_t& new_value) {
  // Convert status bits to readable string
  std::string status_str;
  for (const auto& entry : kBatteryStatusBits) {
    if (new_value & entry.bit) {
      if (!status_str.empty()) status_str += ", ";
      status_str += entry.name;
    }
  }
  bms_msg_.battery_status = status_str;
}

void BmsServiceInterface::OnExtraDataChanged(const char* new_value, uint32_t length) {
  bms_msg_.extra_data = std::string(new_value, length);
}

bool BmsServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  // BMS service has no registers to configure
  return true;
}

void BmsServiceInterface::OnTransactionStart(uint64_t timestamp) {
  bms_msg_ = {};
  bms_msg_.stamp = ros::Time::now();
}

void BmsServiceInterface::OnTransactionEnd() {
  bms_publisher_.publish(bms_msg_);
}
