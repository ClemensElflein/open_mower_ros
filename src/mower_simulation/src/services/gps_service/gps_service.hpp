//
// Created by clemens on 02.08.24.
//

#ifndef GPS_SERVICE_HPP
#define GPS_SERVICE_HPP
#include <GpsServiceBase.hpp>

#include "../../SimRobot.h"

class GpsService : public GpsServiceBase {
 public:
  explicit GpsService(uint16_t service_id, SimRobot &robot) : GpsServiceBase(service_id, 200000), robot_(robot) {
  }

 private:
  void tick() override;

 private:
  SimRobot &robot_;
};

#endif  // GPS_SERVICE_HPP
