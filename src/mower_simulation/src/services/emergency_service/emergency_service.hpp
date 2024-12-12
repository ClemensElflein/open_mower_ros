//
// Created by clemens on 26.07.24.
//

#ifndef EMERGENCY_SERVICE_HPP
#define EMERGENCY_SERVICE_HPP

#include <ros/time.h>

#include <EmergencyServiceBase.hpp>

#include "../../SimRobot.h"
class EmergencyService : public EmergencyServiceBase {
 private:
 public:
  explicit EmergencyService(uint16_t service_id, SimRobot &robot)
      : EmergencyServiceBase(service_id, 100000), robot_(robot) {
  }

 protected:
  bool Configure() override;
  void OnStart() override;
  void OnStop() override;
  void OnCreate() override;

 private:
  void tick() override;

  SimRobot &robot_;

  ros::Time last_clear_emergency_message_{0};
  std::string emergency_reason{"Boot"};

 protected:
  bool OnSetEmergencyChanged(const uint8_t &new_value) override;
};

#endif  // EMERGENCY_SERVICE_HPP
