//
// Created by clemens on 31.07.24.
//

#ifndef IMU_SERVICE_HPP
#define IMU_SERVICE_HPP

#include <ImuServiceBase.hpp>

#include "../../SimRobot.h"

class ImuService : public ImuServiceBase {
 public:
  explicit ImuService(const uint16_t service_id, SimRobot &robot) : ImuServiceBase(service_id, 10000), robot_(robot) {
  }

 protected:
  bool Configure() override;
  void OnStart() override;
  void OnStop() override;
  void OnCreate() override;

 private:
  SimRobot &robot_;
  void tick() override;
};

#endif  // IMU_SERVICE_HPP
