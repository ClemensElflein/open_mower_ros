//
// Created by clemens on 26.07.24.
//

#include "diff_drive_service.hpp"

void DiffDriveService::OnStop() {
  robot_.SetControlTwist(0, 0);
}

void DiffDriveService::tick() {
  StartTransaction();
  SendLeftESCStatus(200u);
  SendRightESCStatus(200u);
  double twist[6]{0};
  robot_.GetTwist(twist[0], twist[5]);
  SendActualTwist(twist, sizeof(twist) / sizeof(double));
  CommitTransaction();
}

void DiffDriveService::OnControlTwistChanged(const double* new_value, uint32_t length) {
  if (length != 6) return;
  // we can only do forward and rotation around one axis
  const auto linear = static_cast<float>(new_value[0]);
  const auto angular = static_cast<float>(new_value[5]);

  robot_.SetControlTwist(linear, angular);
}
bool DiffDriveService::OnStart() {
  // Check, if configuration is valid, if not retry
  if (WheelDistance.value == 0 || WheelTicksPerMeter.value == 0.0) {
    return false;
  }
  return true;
}
