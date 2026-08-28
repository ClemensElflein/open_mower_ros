//
// Created by openmower on 09.08.26.
//

#include "MetaServiceInterface.h"

bool MetaServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  (void)service_id;

  StartTransaction(true);
  SetRegisterRobotFirmware(firmware_name_.c_str(), firmware_name_.size());
  CommitTransaction();

  configured_ = true;  // MetaService connected/configured => start reading the firmware version via RPC
  firmware_check_timer_.start();

  return true;
}

void MetaServiceInterface::OnServiceDisconnected(uint16_t service_id) {
  (void)service_id;

  configured_ = false;

  // The firmware is gone (offline or rebooting for an update). Report an unknown
  // version (major == 0) so motion stays gated off
  version_callback_(FirmwareInfo{});
  // Keep timer running so we recover automatically once the service reconnects.
  // Also important to log an old non-unified FW (without MetaService)
  firmware_check_timer_.start();
}

void MetaServiceInterface::CheckFirmwareVersion() {
  FirmwareInfo info;
  info.connected = configured_.load();

  if (info.connected) {
    uint16_t major_version = 0;
    if (CallGetMajorVersion(major_version)) {
      info.major = major_version;

      char fw_version[50] = {};
      uint16_t result_length = sizeof(fw_version);
      if (CallGetFirmwareVersion(fw_version, result_length)) {
        info.version = fw_version;
      }
    }
  }

  version_callback_(info);
}
