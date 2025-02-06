//
// Created by clemens on 09.09.24.
//

#include "PowerServiceInterface.h"

void PowerServiceInterface::OnChargeVoltageChanged(const float& new_value) {
  power_msg_.v_charge = new_value;
}
void PowerServiceInterface::OnChargeCurrentChanged(const float& new_value) {
  power_msg_.charge_current = new_value;
}
void PowerServiceInterface::OnBatteryVoltageChanged(const float& new_value) {
  power_msg_.v_battery = new_value;
}
void PowerServiceInterface::OnChargingStatusChanged(const char* new_value, uint32_t length) {
  power_msg_.charger_status = std::string(new_value, length);
}
void PowerServiceInterface::OnChargerEnabledChanged(const uint8_t& new_value) {
  power_msg_.charger_enabled = new_value;
}
void PowerServiceInterface::OnTransactionStart(uint64_t timestamp) {
  power_msg_ = {};
  power_msg_.stamp = ros::Time::now();
}

void PowerServiceInterface::OnTransactionEnd() {
  status_publisher_.publish(power_msg_);
}
