//
// Created by clemens on 09.09.24.
//

#ifndef POWER_SERVICE_HPP
#define POWER_SERVICE_HPP

#include <PowerServiceBase.hpp>

#include "../../SimRobot.h"

class PowerService : public PowerServiceBase {
 public:
  explicit PowerService(uint16_t service_id, SimRobot &robot);

 private:
  static constexpr auto CHARGE_STATUS_ERROR = "Error";
  static constexpr auto CHARGE_STATUS_FAULT = "Error (Fault)";
  static constexpr auto CHARGE_STATUS_CHARGER_NOT_FOUND = "Charger Comms Error";
  static constexpr auto CHARGE_STATUS_NOT_CHARGING = "Not Charging";
  static constexpr auto CHARGE_STATUS_PRE_CHARGE = "Pre Charge";
  static constexpr auto CHARGE_STATUS_TRICKLE = "Trickle Charge";
  static constexpr auto CHARGE_STATUS_CC = "Fast Charge (CC)";
  static constexpr auto CHARGE_STATUS_CV = "Taper Charge (CV)";
  static constexpr auto CHARGE_STATUS_TOP_OFF = "Top Off";
  static constexpr auto CHARGE_STATUS_DONE = "Done";
  SimRobot &robot_;

  void tick() override;

 protected:
  bool OnChargingAllowedChanged(const uint8_t &new_value) override;
};

#endif  // POWER_SERVICE_HPP
