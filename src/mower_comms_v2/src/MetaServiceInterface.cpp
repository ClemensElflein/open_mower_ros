//
// Created by openmower on 09.08.26.
//

#include "MetaServiceInterface.h"

#include <ros/console.h>
#include <unistd.h>

namespace {

/// Retry an RPC call up to @p max_attempts times with @p delay_ms between attempts.
/// Returns true on first success.
template <typename Callable>
bool RetryRpc(const char* name, int max_attempts, int delay_ms, Callable&& call) {
  for (int attempt = 1; attempt <= max_attempts; ++attempt) {
    if (call()) {
      return true;
    }
    if (attempt < max_attempts) {
      ROS_INFO_STREAM("Retry " << name << " in " << delay_ms << " ms (attempt " << attempt << "/" << max_attempts
                               << ")");
      usleep(delay_ms * 1000);
    }
  }
  return false;
}

}  // namespace

void MetaServiceInterface::OnServiceConnected(uint16_t service_id) {
  (void)service_id;

  // Log firmware version for diagnostics
  {
    char fw_version[50] = {};
    uint16_t result_length = sizeof(fw_version);
    if (RetryRpc("GetFirmwareVersion", 3, 100, [&] { return CallGetFirmwareVersion(fw_version, result_length); })) {
      ROS_INFO_STREAM("Firmware version: " << fw_version);
    } else {
      ROS_WARN("Failed to get firmware version from MetaService after retries");
    }
  }

  // If no ll/board is set, we are most likely on a Stage-1 only robot (Sabo/xBot).
  // The FW starts without configuration (all registers are optional), so there is
  // nothing to configure.
  if (firmware_name_.empty()) {
    ROS_INFO("No ll/board set, skipping Stage-2 firmware configuration");
    return;
  }

  // Stage-2 robot: validate major version before pushing the robot name.
  {
    uint16_t major_version = 0;
    if (!RetryRpc("GetMajorVersion", 3, 100, [&] { return CallGetMajorVersion(major_version); })) {
      ROS_WARN("Failed to get major version from MetaService after retries! Skipping firmware configuration");
      return;
    }
    if (major_version != 1) {
      ROS_ERROR_STREAM("Firmware major version mismatch: expected 1, got " << major_version
                                                                           << ", refusing to configure, FW stays in "
                                                                              "Stage-2 wait loop");
      return;
    }
  }

  // Push RobotFirmware register via configuration transaction.
  ROS_INFO_STREAM("Configuring robot firmware: " << firmware_name_);
  StartTransaction(true);
  SetRegisterRobotFirmware(firmware_name_.c_str(), firmware_name_.size());
  CommitTransaction();
}
