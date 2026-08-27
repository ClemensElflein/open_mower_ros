//
// Created by openmower on 09.08.26.
//

#include "MetaServiceInterface.h"

bool MetaServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  (void)service_id;

  StartTransaction(true);
  SetRegisterRobotFirmware(firmware_name_.c_str(), firmware_name_.size());
  CommitTransaction();

  // (Re-)start polling the firmware version.
  // Runs until StopFirmwareCheck() is called or the service disconnects.
  firmware_check_timer_.start();

  return true;
}

void MetaServiceInterface::OnServiceDisconnected(uint16_t service_id) {
  (void)service_id;
  firmware_check_timer_.stop();
  // The firmware is gone (offline or rebooting for an update).
  // Report an unknown version (major == 0) so motion stays gated off.
  version_callback_(FirmwareInfo{});
}

void MetaServiceInterface::CheckFirmwareVersion() {
  FirmwareInfo info;

  uint16_t major_version = 0;
  if (CallGetMajorVersion(major_version)) {
    info.major = major_version;

    char fw_version[50] = {};
    uint16_t result_length = sizeof(fw_version);
    if (CallGetFirmwareVersion(fw_version, result_length)) {
      info.version = fw_version;
    }
  }

  version_callback_(info);
}
