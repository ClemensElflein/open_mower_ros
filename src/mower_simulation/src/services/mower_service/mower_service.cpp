//
// Created by clemens on 31.07.24.
//

#include "mower_service.hpp"

bool MowerService::OnStart() {
  mower_running_ = false;
  return true;
}

void MowerService::tick() {
  StartTransaction();
  bool emergency;
  bool unused1;
  std::string unused2;
  robot_.GetEmergencyState(unused1, emergency, unused2);
  bool running = !emergency && mower_running_;
  SendMowerRunning(running);
  SendRainDetected(false);
  SendMowerMotorCurrent(running ? 1.0 : 0.0);
  SendMowerMotorRPM(running ? 4500.0 : 0);
  SendMowerStatus(200);
  SendMowerMotorTemperature(25.0);
  SendMowerESCTemperature(35.0);
  CommitTransaction();
}

void MowerService::OnMowerEnabledChanged(const uint8_t& new_value) {
  mower_running_ = new_value;
}
