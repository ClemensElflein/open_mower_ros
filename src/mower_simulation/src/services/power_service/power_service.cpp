//
// Created by clemens on 09.09.24.
//

#include "power_service.hpp"


PowerService::PowerService(uint16_t service_id, SimRobot &robot)
    : PowerServiceBase(service_id, 200000), robot_(robot) {}
bool PowerService::Configure() { return true; }

void PowerService::OnStart() {  }
void PowerService::OnCreate() {}
void PowerService::OnStop() {}

void PowerService::tick() {
  bool is_charging;
  double charging_time, charge_volts, battery_volts, charge_current;
  std::string charge_state;
  robot_.GetIsCharging(is_charging, charging_time, charge_state, charge_volts, battery_volts, charge_current);


  // Send the sensor values
  StartTransaction();
  SendBatteryVoltage(battery_volts);
  SendChargeVoltage(charge_volts);
  SendChargeCurrent(charge_current);
  SendChargerEnabled(true);
  SendChargingStatus(charge_state.c_str(), charge_state.length());
  CommitTransaction();
}
bool PowerService::OnChargingAllowedChanged(const uint8_t& new_value) {
  (void)new_value;
  return true;
}