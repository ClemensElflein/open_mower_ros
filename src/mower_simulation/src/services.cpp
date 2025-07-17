#include "services.hpp"

#include "../../../services/service_ids.h"

SimRobot robot{};

EmergencyService emergency_service{xbot::service_ids::EMERGENCY};
DiffDriveService diff_drive{xbot::service_ids::DIFF_DRIVE};
MowerService mower_service{xbot::service_ids::MOWER};
ImuService imu_service{xbot::service_ids::IMU};
PowerService power_service{xbot::service_ids::POWER};
GpsService gps_service{xbot::service_ids::GPS};

void StartServices() {
  EmergencyService emergency_service{xbot::service_ids::EMERGENCY};
  DiffDriveService diff_drive_service{xbot::service_ids::DIFF_DRIVE};
  MowerService mower_service{xbot::service_ids::MOWER};
  ImuService imu_service{xbot::service_ids::IMU};
  PowerService power_service{xbot::service_ids::POWER};
  GpsService gps_service{xbot::service_ids::GPS};
}
