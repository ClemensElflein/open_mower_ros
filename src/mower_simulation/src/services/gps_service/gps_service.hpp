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

 protected:
  bool Configure() override;
  void OnStart() override;
  void OnCreate() override;
  void OnStop() override;

 private:
  void tick() override;

 protected:
  bool OnRTCMChanged(const uint8_t *new_value, uint32_t length) override;

 private:
  SimRobot &robot_;
};

#endif  // GPS_SERVICE_HPP
