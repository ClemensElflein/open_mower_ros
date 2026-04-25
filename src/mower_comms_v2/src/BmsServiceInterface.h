/*
 * OpenMower
 * Part of open_mower_ros (https://github.com/ClemensElflein/open_mower_ros)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef BMSSERVICEINTERFACE_H
#define BMSSERVICEINTERFACE_H

#include <mower_msgs/Bms.h>
#include <ros/publisher.h>

#include <BmsServiceInterfaceBase.hpp>

class BmsServiceInterface : public BmsServiceInterfaceBase {
 public:
  BmsServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx, const ros::Publisher& bms_publisher)
      : BmsServiceInterfaceBase(service_id, ctx), bms_publisher_(bms_publisher) {
  }

 protected:
  void OnVoltageChanged(const float& new_value) override;
  void OnCurrentChanged(const float& new_value) override;
  void OnRelativeStateOfChargeChanged(const float& new_value) override;
  void OnRemainingCapacityChanged(const float& new_value) override;
  void OnFullChargeCapacityChanged(const float& new_value) override;
  void OnCycleCountChanged(const uint16_t& new_value) override;
  void OnTemperatureChanged(const float& new_value) override;
  void OnBatteryStatusChanged(const uint16_t& new_value) override;
  void OnExtraDataChanged(const char* new_value, uint32_t length) override;

  bool OnConfigurationRequested(uint16_t service_id) override;

 private:
  void OnTransactionStart(uint64_t timestamp) override;
  void OnTransactionEnd() override;
  mower_msgs::Bms bms_msg_{};
  const ros::Publisher& bms_publisher_;
};

struct BatteryStatusBitName {
  uint16_t bit;
  const char* name;
};

inline constexpr std::array<BatteryStatusBitName, 10> kBatteryStatusBits{{
    {BatteryStatusBit::STATUS_FULLY_DISCHARGED, "Fully discharged"},
    {BatteryStatusBit::STATUS_FULLY_CHARGED, "Fully charged"},
    {BatteryStatusBit::STATUS_DISCHARGING, "Discharging"},
    {BatteryStatusBit::STATUS_INITIALIZED, "Initialized"},
    {BatteryStatusBit::ALARM_REMAINING_TIME, "ALARM: Remaining time"},
    {BatteryStatusBit::ALARM_REMAINING_CAPACITY, "ALARM: Remaining capacity"},
    {BatteryStatusBit::ALARM_TERMINATE_DISCHARGE, "ALARM: Terminate discharge"},
    {BatteryStatusBit::ALARM_OVER_TEMPERATURE, "ALARM: Over temperature"},
    {BatteryStatusBit::ALARM_TERMINATE_CHARGE, "ALARM: Terminate charge"},
    {BatteryStatusBit::ALARM_OVER_CHARGED, "ALARM: Over charged"},
}};

#endif  // BMSSERVICEINTERFACE_H
