//
// Created by clemens on 02.08.24.
//

#ifndef GPS_SERVICE_HPP
#define GPS_SERVICE_HPP
#include <GpsServiceBase.hpp>

#include "../../SimRobot.h"

using namespace xbot::service;

class GpsService : public GpsServiceBase {
 public:
  explicit GpsService(uint16_t service_id, SimRobot &robot) : GpsServiceBase(service_id), robot_(robot) {
  }

 private:
  void tick();
  ManagedSchedule tick_schedule_{scheduler_, IsRunning(), 200'000,
                                 XBOT_FUNCTION_FOR_METHOD(GpsService, &GpsService::tick, this)};

 private:
  SimRobot &robot_;
};

#endif  // GPS_SERVICE_HPP
