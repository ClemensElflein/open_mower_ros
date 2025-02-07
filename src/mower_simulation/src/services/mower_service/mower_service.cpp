//
// Created by clemens on 31.07.24.
//

#include "mower_service.hpp"


bool MowerService::Configure() {
  // No configuration needed
  return true;
}
void MowerService::OnCreate() {
}
void MowerService::OnStart() {
  mower_running_ = false;
}
void MowerService::OnStop() { }

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


bool MowerService::OnMowerEnabledChanged(const uint8_t& new_value) {
  mower_running_ = new_value;
  return true;
}

MowerService::MowerService(const uint16_t service_id, SimRobot &robot)
    : MowerServiceBase(service_id, 1000000), robot_(robot) {
}
