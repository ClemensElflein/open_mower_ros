//
// Created by clemens on 26.07.24.
//

#ifndef EMERGENCY_SERVICE_HPP
#define EMERGENCY_SERVICE_HPP

#include <ros/time.h>

#include <EmergencyServiceBase.hpp>

#include "../../SimRobot.h"

using namespace xbot::service;

class EmergencyService : public EmergencyServiceBase {
 private:
 public:
  explicit EmergencyService(uint16_t service_id, SimRobot &robot) : EmergencyServiceBase(service_id), robot_(robot) {
  }

 protected:
  bool OnStart() override;
  void OnStop() override;

 private:
  void tick();
  ManagedSchedule tick_schedule_{scheduler_, IsRunning(), 100'000,
                                 XBOT_FUNCTION_FOR_METHOD(EmergencyService, &EmergencyService::tick, this)};

  SimRobot &robot_;

  ros::Time last_clear_emergency_message_{0};
  std::string emergency_reason{"Boot"};

 protected:
  void OnSetEmergencyChanged(const uint8_t &new_value) override;
};

#endif  // EMERGENCY_SERVICE_HPP
