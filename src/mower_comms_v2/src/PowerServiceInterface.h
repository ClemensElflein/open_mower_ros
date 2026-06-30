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
#include <ros/ros.h>

#include <PowerServiceInterfaceBase.hpp>
#include <array>
#include <limits>

class PowerServiceInterface : public PowerServiceInterfaceBase {
 public:
  PowerServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                        const ros::Publisher& status_publisher, const ros::NodeHandle& param_nh)
      : PowerServiceInterfaceBase(service_id, ctx), status_publisher_(status_publisher) {
    // Mainly for monitoring and informational purposes
    if (!param_nh.getParam("services/power/battery_full_voltage", battery_full_voltage_)) {
      ROS_ERROR("Need to set param: services/power/battery_full_voltage");
      param_error_ = 1;
      return;
    }
    if (!param_nh.getParam("services/power/battery_empty_voltage", battery_empty_voltage_)) {
      ROS_ERROR("Need to set param: services/power/battery_empty_voltage");
      param_error_ = 1;
      return;
    }
    if (!param_nh.getParam("services/power/battery_critical_voltage", battery_critical_voltage_)) {
      ROS_ERROR("Need to set param: services/power/battery_critical_voltage");
      param_error_ = 1;
      return;
    }
    if (!param_nh.getParam("services/power/battery_critical_high_voltage", battery_critical_high_voltage_)) {
      ROS_ERROR("Need to set param: services/power/battery_critical_high_voltage");
      param_error_ = 1;
      return;
    }

    // Optional charger configuration
    param_nh.getParam("services/power/charge_voltage", charge_voltage_);
    param_nh.getParam("services/power/charge_current", charge_current_);
    param_nh.getParam("services/power/charge_termination_current", charge_termination_current_);
    param_nh.getParam("services/power/charge_pre_charge_current", charge_precharge_current_);
    param_nh.getParam("services/power/charge_re_charge_voltage", charge_recharge_voltage_);

    // Optional settings also required for charger DPM (dynamic power management)
    param_nh.getParam("services/power/system_current", system_current_);
    param_nh.getParam("services/power/dangerously_override_hardware_charge_current_limit",
                      override_hardware_charge_current_limit_);
    param_nh.getParam("services/power/log_debug", log_debug_);
  }

  /**
   * Exit code for missing required parameters (0 = parameters valid).
   */
  int GetParamError() const {
    return param_error_;
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

  float battery_full_voltage_ = 0.0f;
  float battery_empty_voltage_ = 0.0f;
  float battery_critical_voltage_ = 0.0f;
  float battery_critical_high_voltage_ = 0.0f;
  float charge_voltage_ = -1.0f;
  float charge_current_ = -1.0f;
  float charge_termination_current_ = -1.0f;
  float charge_precharge_current_ = -1.0f;
  int charge_recharge_voltage_ = -1;
  float system_current_ = -1.0f;  // Max. current allowed to be drawn from wall AC/DC
  bool override_hardware_charge_current_limit_ = false;
  bool log_debug_ = false;
  int param_error_ = 0;
};

#endif  // POWERSERVICEINTERFACE_H
