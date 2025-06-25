#ifndef POWER_SERVICE_HPP
#define POWER_SERVICE_HPP

#include <PowerServiceBase.hpp>

using namespace xbot::service;

class PowerService : public PowerServiceBase {
 public:
  explicit PowerService(uint16_t service_id) : PowerServiceBase(service_id) {
  }

 protected:
  void OnChargingAllowedChanged(const uint8_t &new_value) override;

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

  void tick();
  ServiceSchedule tick_schedule_{*this, 200'000, XBOT_FUNCTION_FOR_METHOD(PowerService, &PowerService::tick, this)};
};

#endif  // POWER_SERVICE_HPP
