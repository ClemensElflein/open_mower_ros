#ifndef SERVICES_HPP
#define SERVICES_HPP

#include "SimRobot.h"
#include "services/diff_drive_service/diff_drive_service.hpp"
#include "services/emergency_service/emergency_service.hpp"
#include "services/gps_service/gps_service.hpp"
#include "services/imu_service/imu_service.hpp"
#include "services/mower_service/mower_service.hpp"
#include "services/power_service/power_service.hpp"

extern SimRobot robot;

extern EmergencyService emergency_service;
extern DiffDriveService diff_drive;
extern MowerService mower_service;
extern ImuService imu_service;
extern PowerService power_service;
extern GpsService gps_service;

void StartServices();

#endif  // SERVICES_HPP
