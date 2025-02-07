//
// Created by clemens on 31.07.24.
//

#ifndef MOWER_SERVICE_HPP
#define MOWER_SERVICE_HPP

#include <MowerServiceBase.hpp>

#include "../../SimRobot.h"

class MowerService : public MowerServiceBase {
 public:
  explicit MowerService(const uint16_t service_id, SimRobot &robot_);

 private:
 SimRobot &robot_;
 bool mower_running_ = false;
  void tick() override;

 protected:
  void OnStart() override;
  bool OnMowerEnabledChanged(const uint8_t &new_value) override;
};

#endif  // MOWER_SERVICE_HPP
