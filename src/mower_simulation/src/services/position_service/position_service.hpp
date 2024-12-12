//
// Created by clemens on 02.08.24.
//

#ifndef POSITION_SERVICE_HPP
#define POSITION_SERVICE_HPP
#include <PositionServiceBase.hpp>
#include "../../SimRobot.h"

class PositionService : public PositionServiceBase {
public:
 explicit PositionService(uint16_t service_id, SimRobot &robot)
   : PositionServiceBase(service_id, 200000), robot_(robot) {
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

#endif  // POSITION_SERVICE_HPP
