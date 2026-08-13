//
// Created by openmower on 09.08.26.
//

#include "MetaServiceInterface.h"

#include <ros/console.h>

bool MetaServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  (void)service_id;

  StartTransaction(true);
  SetRegisterRobotFirmware(firmware_name_.c_str(), firmware_name_.size());
  CommitTransaction();

  // Re-arm the one-shot timer on every configuration request, so the firmware
  // info is logged again after a firmware update (service reboot/reconnect).
  post_config_timer_.start();

  return true;
}

void MetaServiceInterface::OnServiceDisconnected(uint16_t service_id) {
  (void)service_id;
  post_config_timer_.stop();
}

void MetaServiceInterface::OnPostConfigurationTimer(const ros::TimerEvent&) {
  LogFirmwareInfo();
}

void MetaServiceInterface::LogFirmwareInfo() {
  uint16_t major_version = 0;
  if (CallGetMajorVersion(major_version)) {
    ROS_INFO_STREAM("Firmware major version: " << major_version);
  } else {
    ROS_WARN("Failed to get major version from MetaService");
  }

  char fw_version[50] = {};
  uint16_t result_length = sizeof(fw_version);
  if (CallGetFirmwareVersion(fw_version, result_length)) {
    ROS_INFO_STREAM("Firmware version: " << fw_version);
  } else {
    ROS_WARN("Failed to get firmware version from MetaService");
  }
}
