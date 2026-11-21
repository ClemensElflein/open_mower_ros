/*
 * OpenMower
 * Part of open_mower_ros (https://github.com/ClemensElflein/open_mower_ros)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "BmsServiceInterface.h"

#include <algorithm>
#include <exception>
#include <nlohmann/json.hpp>
#include <string>

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

void BmsServiceInterface::OnBatteryStatusRawChanged(const uint16_t& new_value) {
  bms_msg_.battery_status_raw = new_value;
}

void BmsServiceInterface::OnCellVoltagesChanged(const float* new_value, uint32_t length) {
  constexpr uint32_t kMaxCells = 8;  // fixed_array<float, 8> in Bms.msg
  const uint32_t count = std::min<uint32_t>(length, kMaxCells);
  for (uint32_t i = 0; i < count; i++) {
    bms_msg_.cell_voltages[i] = new_value[i];
  }
  for (uint32_t i = count; i < kMaxCells; i++) {
    bms_msg_.cell_voltages[i] = 0.0f;
  }
}

void BmsServiceInterface::PollDeviceInfo() {
  const auto generation = pack_generation_.load();
  if (published_generation_ == generation) {
    return;
  }

  const auto now = ros::SteadyTime::now();
  if (now < next_info_attempt_) {
    return;
  }
  next_info_attempt_ = now + ros::WallDuration(5.0);

  std::string json;
  if (!CallGetDeviceInfo(json, 1000) || json.empty()) {
    return;
  }

  try {
    const auto info = nlohmann::json::parse(json);
    bms_info_msg_ = {};
    bms_info_msg_.stamp = ros::Time::now();
    bms_info_msg_.mfr_name = info.value("mfr_name", std::string());
    bms_info_msg_.mfr_date = info.value("mfr_date", std::string());
    bms_info_msg_.dev_name = info.value("dev_name", std::string());
    bms_info_msg_.dev_version = info.value("dev_version", std::string());
    bms_info_msg_.dev_chemistry = info.value("dev_chemistry", std::string());
    bms_info_msg_.serial_number = info.value("serial_number", 0);
    bms_info_msg_.design_capacity_ah = info.value("design_capacity_ah", 0.0f);
    bms_info_msg_.design_voltage_v = info.value("design_voltage_v", 0.0f);
    bms_info_msg_.cell_count = info.value("cell_count", 0);
  } catch (const std::exception& e) {
    ROS_WARN("BMS: could not parse the device info: %s", e.what());
    return;
  }

  // A reconnect during the RPC leaves the new generation pending for the next poll.
  if (generation == pack_generation_.load()) {
    bms_info_publisher_.publish(bms_info_msg_);
    published_generation_ = generation;
  }
}

bool BmsServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  // BMS service has no registers to configure
  return true;
}

void BmsServiceInterface::OnServiceConnected(uint16_t service_id) {
  if (service_id == service_id_) {
    // A reboot can trigger a fresh claim before the previous connection times out.
    last_telemetry_ = ros::SteadyTime::now();
    ++pack_generation_;
  }
}

void BmsServiceInterface::OnTransactionStart(uint64_t timestamp) {
  const auto now = ros::SteadyTime::now();
  if (now - last_telemetry_ >= ros::WallDuration(5.0)) {
    ++pack_generation_;
  }
  last_telemetry_ = now;
  bms_msg_ = {};
  bms_msg_.stamp = ros::Time::now();
}

void BmsServiceInterface::OnTransactionEnd() {
  bms_publisher_.publish(bms_msg_);
}
