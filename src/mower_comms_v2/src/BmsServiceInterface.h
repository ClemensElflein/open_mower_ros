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
#include <mower_msgs/BmsInfo.h>
#include <ros/publisher.h>

#include <BmsServiceInterfaceBase.hpp>
#include <atomic>

class BmsServiceInterface : public BmsServiceInterfaceBase {
 public:
  BmsServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx, const ros::Publisher& bms_publisher,
                      const ros::Publisher& bms_info_publisher)
      : BmsServiceInterfaceBase(service_id, ctx),
        bms_publisher_(bms_publisher),
        bms_info_publisher_(bms_info_publisher) {
  }

  /**
   * @brief Fetch the static device info once the pack answers.
   *
   * Called from the node's timer, not from a service callback: the call blocks until the
   * low level answers. Retry every 5 s until successful and refresh after a disconnect
   * or a gap in telemetry, when a different pack may have been installed.
   */
  void PollDeviceInfo();

 protected:
  void OnVoltageChanged(const float& new_value) override;
  void OnCurrentChanged(const float& new_value) override;
  void OnRelativeStateOfChargeChanged(const float& new_value) override;
  void OnRemainingCapacityChanged(const float& new_value) override;
  void OnFullChargeCapacityChanged(const float& new_value) override;
  void OnCycleCountChanged(const uint16_t& new_value) override;
  void OnTemperatureChanged(const float& new_value) override;
  void OnBatteryStatusChanged(const uint16_t& new_value) override;
  void OnBatteryStatusRawChanged(const uint16_t& new_value) override;
  void OnCellVoltagesChanged(const float* new_value, uint32_t length) override;

  bool OnConfigurationRequested(uint16_t service_id) override;
  void OnServiceDisconnected(uint16_t service_id) override;

 private:
  void OnTransactionStart(uint64_t timestamp) override;
  void OnTransactionEnd() override;
  mower_msgs::Bms bms_msg_{};
  mower_msgs::BmsInfo bms_info_msg_{};
  const ros::Publisher& bms_publisher_;
  const ros::Publisher& bms_info_publisher_;
  // The IO callbacks advance the generation. Only the polling thread records a successful fetch.
  std::atomic<uint64_t> pack_generation_{1};
  uint64_t published_generation_{0};
  ros::SteadyTime last_telemetry_{ros::SteadyTime::now()};  // IO thread
  ros::SteadyTime next_info_attempt_;                       // polling thread
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
