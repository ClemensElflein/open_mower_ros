//
// Created by clemens on 31.07.24.
//

#include "mower_service.hpp"

void MowerService::tick() {
}

bool MowerService::OnMowerEnabledChanged(const uint8_t& new_value) {
  return true;
}

MowerService::MowerService(const uint16_t service_id, SimRobot& robot)
    : MowerServiceBase(service_id, 100000000), robot_(robot) {
}
