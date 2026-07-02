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
  explicit EmergencyService(uint16_t service_id, SimRobot& robot) : EmergencyServiceBase(service_id), robot_(robot) {
  }

 protected:
  void OnStop() override;

 private:
  void tick();
  ManagedSchedule tick_schedule_{scheduler_, IsRunning(), 100'000,
                                 XBOT_FUNCTION_FOR_METHOD(EmergencyService, &EmergencyService::tick, this)};

  SimRobot& robot_;

  // Time the last high-level emergency message was received. Used to raise
  // TIMEOUT_HIGH_LEVEL when the high level goes quiet, and clear it once it is back.
  ros::Time last_high_level_emergency_message_{0};

 protected:
  void OnHighLevelEmergencyChanged(const uint16_t* new_value, uint32_t length) override;
};

#endif  // EMERGENCY_SERVICE_HPP
