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
#include <ros/ros.h>

// Include messages for mower control
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <mower_map/GetDockingPointSrv.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/Emergency.h>
#include <mower_msgs/EmergencyStopSrv.h>
#include <mower_msgs/MowerControlSrv.h>
#include <mower_msgs/Power.h>
#include <mower_msgs/Status.h>
#include <mower_simulation/MowerSimulationConfig.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <xbot_msgs/AbsolutePose.h>
#include <xbot_positioning/GPSControlSrv.h>
#include <xbot_positioning/SetPoseSrv.h>

#include <xbot-service/Io.hpp>
#include <xbot-service/portable/system.hpp>

#include "services.hpp"

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

  StartServices();
  robot.Start(paramNh);

  ros::spin();
  delete (reconfig_server);
  return 0;
}
