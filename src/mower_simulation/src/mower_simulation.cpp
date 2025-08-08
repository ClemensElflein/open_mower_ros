// Created by Clemens Elflein on 2/18/22, 5:37 PM.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//
#include "ros/ros.h"

// Include messages for mower control
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/Emergency.h>
#include <mower_msgs/Power.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <xbot-service/Io.hpp>
#include <xbot-service/portable/system.hpp>

#include "../../../services/service_ids.h"
#include "SimRobot.h"
#include "dynamic_reconfigure/server.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "mower_map/GetDockingPointSrv.h"
#include "mower_msgs/EmergencyStopSrv.h"
#include "mower_msgs/MowerControlSrv.h"
#include "mower_msgs/Status.h"
#include "mower_simulation/MowerSimulationConfig.h"
#include "nav_msgs/Odometry.h"
#include "services/diff_drive_service/diff_drive_service.hpp"
#include "services/emergency_service/emergency_service.hpp"
#include "services/gps_service/gps_service.hpp"
#include "services/imu_service/imu_service.hpp"
#include "services/mower_service/mower_service.hpp"
#include "services/power_service/power_service.hpp"
#include "xbot_msgs/AbsolutePose.h"
#include "xbot_positioning/GPSControlSrv.h"
#include "xbot_positioning/SetPoseSrv.h"

ros::Publisher status_pub;
ros::Publisher cmd_vel_pub;
ros::Publisher pose_pub;
ros::Publisher initial_pose_publisher;
ros::ServiceClient docking_point_client;

dynamic_reconfigure::Server<mower_simulation::MowerSimulationConfig> *reconfig_server;

int main(int argc, char **argv) {
  ros::init(argc, argv, "mower_simulation");

  ros::NodeHandle n;
  ros::NodeHandle paramNh("~");

  reconfig_server = new dynamic_reconfigure::Server<mower_simulation::MowerSimulationConfig>(paramNh);
  // reconfig_server->setCallback(reconfigureCB);

  docking_point_client = n.serviceClient<mower_map::GetDockingPointSrv>("mower_map_service/get_docking_point");

  xbot::service::system::initSystem();
  xbot::service::Io::start();

  SimRobot robot{paramNh};

  // Move the robot to the docking station.
  // TODO: Use a better way to make sure that the docking position is loaded.
  sleep(3);
  mower_map::GetDockingPointSrv get_docking_point_srv;
  docking_point_client.call(get_docking_point_srv);
  const auto &docking_pose = get_docking_point_srv.response.docking_pose;
  tf2::Quaternion quat;
  tf2::fromMsg(docking_pose.orientation, quat);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  robot.SetDockingPose(docking_pose.position.x, docking_pose.position.y, yaw);
  robot.SetPosition(docking_pose.position.x, docking_pose.position.y, yaw);

  EmergencyService emergency_service{xbot::service_ids::EMERGENCY, robot};
  DiffDriveService diff_drive_service{xbot::service_ids::DIFF_DRIVE, robot};
  MowerService mower_service{xbot::service_ids::MOWER, robot};
  ImuService imu_service{xbot::service_ids::IMU, robot};
  PowerService power_service{xbot::service_ids::POWER, robot};
  GpsService gps_service{xbot::service_ids::GPS, robot};

  emergency_service.start();
  diff_drive_service.start();
  mower_service.start();
  imu_service.start();
  power_service.start();
  gps_service.start();

  robot.Start();

  ros::spin();
  delete (reconfig_server);
  return 0;
}
