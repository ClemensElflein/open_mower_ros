/*
 * OpenMower
 * Part of open_mower_ros (https://github.com/ClemensElflein/open_mower_ros)
 *
 * Created by clemens on 09.09.24.
 * Copyright (C) 2024, 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef POWERSERVICEINTERFACE_H
#define POWERSERVICEINTERFACE_H

#include <mower_msgs/Power.h>
#include <ros/publisher.h>

#include <PowerServiceInterfaceBase.hpp>
#include <array>
#include <limits>

class PowerServiceInterface : public PowerServiceInterfaceBase {
 public:
  PowerServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                        const ros::Publisher& status_publisher, float battery_full_voltage, float battery_empty_voltage,
                        float battery_critical_voltage, float battery_critical_high_voltage, float charge_voltage,
                        float charge_current, float charge_termination_current, float charge_precharge_current,
                        int charge_recharge_voltage, float system_current)
      : PowerServiceInterfaceBase(service_id, ctx),
        status_publisher_(status_publisher),
        battery_full_voltage_(battery_full_voltage),
        battery_empty_voltage_(battery_empty_voltage),
        battery_critical_voltage_(battery_critical_voltage),
        battery_critical_high_voltage_(battery_critical_high_voltage),
        charge_voltage_(charge_voltage),
        charge_current_(charge_current),
        charge_termination_current_(charge_termination_current),
        charge_precharge_current_(charge_precharge_current),
        charge_recharge_voltage_(charge_recharge_voltage),
        system_current_(system_current) {
  }

 protected:
  void OnChargeVoltageChanged(const float& new_value) override;
  void OnChargeCurrentChanged(const float& new_value) override;
  void OnBatteryVoltageChanged(const float& new_value) override;
  void OnChargingStatusChanged(const char* new_value, uint32_t length) override;
  void OnChargerEnabledChanged(const uint8_t& new_value) override;
  void OnBatteryPercentageChanged(const float& new_value) override;
  void OnChargeVoltageADCChanged(const float& new_value) override;
  void OnBatteryVoltageADCChanged(const float& new_value) override;
  void OnDCDCInputCurrentChanged(const float& new_value) override;
  void OnChargerInputCurrentChanged(const float& new_value) override;

  bool OnConfigurationRequested(uint16_t service_id) override;

 private:
  void OnTransactionStart(uint64_t timestamp) override;
  void OnTransactionEnd() override;
  mower_msgs::Power power_msg_{};
  const ros::Publisher& status_publisher_;

  float battery_full_voltage_;
  float battery_empty_voltage_;
  float battery_critical_voltage_;
  float battery_critical_high_voltage_;
  float charge_voltage_;
  float charge_current_;
  float charge_termination_current_;
  float charge_precharge_current_;
  int charge_recharge_voltage_;
  float system_current_;
};

#endif  // POWERSERVICEINTERFACE_H
