//
// Created by clemens on 09.09.24.
//

#include "PowerServiceInterface.h"

void PowerServiceInterface::OnChargeVoltageCHGChanged(const float& new_value) {
  power_msg_.charge_voltage_chg = new_value;
}

void PowerServiceInterface::OnChargeCurrentChanged(const float& new_value) {
  power_msg_.charge_current = new_value;
}

void PowerServiceInterface::OnBatteryVoltageCHGChanged(const float& new_value) {
  power_msg_.battery_voltage_chg = new_value;
}

void PowerServiceInterface::OnChargingStatusChanged(const char* new_value, uint32_t length) {
  power_msg_.charger_status = std::string(new_value, length);
}

void PowerServiceInterface::OnChargerEnabledChanged(const uint8_t& new_value) {
  power_msg_.charger_enabled = new_value;
}

void PowerServiceInterface::OnBatteryVoltageBMSChanged(const float& new_value) {
  power_msg_.battery_voltage_bms = new_value;
}

void PowerServiceInterface::OnBatteryCurrentChanged(const float& new_value) {
  power_msg_.battery_current = new_value;
}

void PowerServiceInterface::OnBatteryPercentageChanged(const float& new_value) {
  power_msg_.battery_pct = new_value;
}

void PowerServiceInterface::OnBatterySoCChanged(const float& new_value) {
  power_msg_.battery_soc = new_value;
}

void PowerServiceInterface::OnBatteryTemperatureChanged(const float& new_value) {
  power_msg_.battery_temp = new_value;
}

void PowerServiceInterface::OnBatteryStatusChanged(const uint16_t& new_value) {
  for (const auto& entry : kBatteryStatusBits) {
    if (new_value & entry.bit) {
      if (!power_msg_.bms_status.empty()) power_msg_.bms_status += ", ";
      power_msg_.bms_status += entry.name;
    }
  }
}

void PowerServiceInterface::OnBMSExtraDataChanged(const char* new_value, uint32_t length) {
  power_msg_.bms_extra_data = std::string(new_value, length);
}

void PowerServiceInterface::OnChargeVoltageADCChanged(const float& new_value) {
  power_msg_.charge_voltage_adc = new_value;
}

void PowerServiceInterface::OnBatteryVoltageADCChanged(const float& new_value) {
  power_msg_.battery_voltage_adc = new_value;
}

void PowerServiceInterface::OnDCDCInputCurrentChanged(const float& new_value) {
  power_msg_.dcdc_input_current = new_value;
}

void PowerServiceInterface::OnChargerInputCurrentChanged(const float& new_value) {
  power_msg_.charger_input_current = new_value;
}

bool PowerServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  StartTransaction(true);
  SetRegisterBatteryFullVoltage(battery_full_voltage_);
  SetRegisterBatteryEmptyVoltage(battery_empty_voltage_);
  SetRegisterCriticalBatteryLowVoltage(battery_critical_voltage_);
  SetRegisterCriticalBatteryHighVoltage(battery_critical_high_voltage_);
  SetRegisterChargeCurrent(battery_charge_current_);
  SetRegisterSystemCurrentLimit(system_current_);
  CommitTransaction();
  return true;
}

void PowerServiceInterface::OnTransactionStart(uint64_t timestamp) {
  power_msg_ = {};
  power_msg_.stamp = ros::Time::now();
}

void PowerServiceInterface::OnTransactionEnd() {
  status_publisher_.publish(power_msg_);
}
