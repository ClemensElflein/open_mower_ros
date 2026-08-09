//
// Created by openmower on 09.08.26.
//

#include "MetaServiceInterface.h"

#include <ros/console.h>

void MetaServiceInterface::OnServiceConnected(uint16_t service_id) {
  (void)service_id;

  char fw_version[50] = {};
  uint16_t result_length = sizeof(fw_version);
  if (CallGetFirmwareVersion(fw_version, result_length)) {
    ROS_INFO_STREAM("Firmware version: " << fw_version);
  } else {
    ROS_WARN("Failed to get firmware version from MetaService");
  }
}

bool MetaServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  (void)service_id;

  if (firmware_name_.empty()) {
    return false;  // No firmware name configured — skip config
  }

  uint16_t major_version = 0;
  if (!CallGetMajorVersion(major_version)) {
    ROS_WARN("Failed to get major version from MetaService - skipping firmware configuration");
    return false;
  }
  if (major_version != 1) {
    ROS_ERROR_STREAM("Firmware major version mismatch: expected 1, got " << major_version
                                                                         << " - refusing to configure, FW stays in "
                                                                            "Stage-2 wait loop");
    return false;
  }

  ROS_INFO_STREAM("Configuring robot firmware: " << firmware_name_);
  StartTransaction(true);
  SetRegisterRobotFirmware(firmware_name_.c_str(), firmware_name_.size());
  CommitTransaction();
  return true;
}
