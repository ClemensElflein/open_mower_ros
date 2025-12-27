//
// Created by clemens on 09.09.24.
//

#ifndef POWERSERVICEINTERFACE_H
#define POWERSERVICEINTERFACE_H

#include <mower_msgs/Power.h>
#include <ros/publisher.h>

#include <PowerServiceInterfaceBase.hpp>
#include <array>

class PowerServiceInterface : public PowerServiceInterfaceBase {
 public:
  PowerServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                        const ros::Publisher& status_publisher, float battery_full_voltage, float battery_empty_voltage,
                        float battery_critical_voltage, float battery_critical_high_voltage,
                        float battery_charge_current)
      : PowerServiceInterfaceBase(service_id, ctx),
        status_publisher_(status_publisher),
        battery_full_voltage_(battery_full_voltage),
        battery_empty_voltage_(battery_empty_voltage),
        battery_critical_voltage_(battery_critical_voltage),
        battery_critical_high_voltage_(battery_critical_high_voltage),
        battery_charge_current_(battery_charge_current) {
  }

 protected:
  void OnChargeVoltageChanged(const float& new_value) override;
  void OnChargeCurrentChanged(const float& new_value) override;
  void OnBatteryVoltageChanged(const float& new_value) override;
  void OnChargingStatusChanged(const char* new_value, uint32_t length) override;
  void OnChargerEnabledChanged(const uint8_t& new_value) override;
  bool OnConfigurationRequested(uint16_t service_id) override;

  void OnBatteryPercentageChanged(const float& new_value) override;
  void OnBatteryVoltageBMSChanged(const float& new_value) override;
  void OnBatteryCurrentChanged(const float& new_value) override;
  void OnBatterySoCChanged(const float& new_value) override;
  void OnBatteryTemperatureChanged(const float& new_value) override;
  void OnBatteryStatusChanged(const uint16_t& new_value) override;
  void OnBMSExtraDataChanged(const char* new_value, uint32_t length) override;

 private:
  void OnTransactionStart(uint64_t timestamp) override;
  void OnTransactionEnd() override;
  mower_msgs::Power power_msg_{};
  const ros::Publisher& status_publisher_;

  float battery_full_voltage_;
  float battery_empty_voltage_;
  float battery_critical_voltage_;
  float battery_critical_high_voltage_;
  float battery_charge_current_;
};

struct BatteryStatusBitName {
  uint16_t bit;
  const char* name;
};

inline constexpr std::array<BatteryStatusBitName, 12> kBatteryStatusBits{{
    {BatteryStatusBit::STATUS_FULLY_DISCHARGED, "Fully discharged"},
    {BatteryStatusBit::STATUS_FULLY_CHARGED, "Fully charged"},
    {BatteryStatusBit::STATUS_DISCHARGING, "Discharging"},
    {BatteryStatusBit::STATUS_INITIALIZED, "Initialized"},
    {BatteryStatusBit::ALARM_REMAINING_TIME, "ALARM: Remaining time"},
    {BatteryStatusBit::ALARM_REMAINING_CAPACITY, "ALARM: Remaining capacity"},
    {BatteryStatusBit::ALARM_RESERVED1, "ALARM: Reserved 1"},
    {BatteryStatusBit::ALARM_TERMINATE_DISCHARGE, "ALARM: Terminate discharge"},
    {BatteryStatusBit::ALARM_OVER_TEMPERATURE, "ALARM: Over temperature"},
    {BatteryStatusBit::ALARM_RESERVED2, "ALARM: Reserved 2"},
    {BatteryStatusBit::ALARM_TERMINATE_CHARGE, "ALARM: Terminate charge"},
    {BatteryStatusBit::ALARM_OVER_CHARGED, "ALARM: Over charged"},
}};

#endif  // POWERSERVICEINTERFACE_H
