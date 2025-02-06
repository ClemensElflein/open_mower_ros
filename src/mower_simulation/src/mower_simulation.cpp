// Created by Clemens Elflein on 2/18/22, 5:37 PM.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based
// on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
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
#include "services/imu_service/imu_service.hpp"
#include "services/mower_service/mower_service.hpp"
#include "services/gps_service/gps_service.hpp"
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

  EmergencyService emergency_service{1, robot};
  DiffDriveService diff_drive_service{2, robot};
  MowerService mower_service{3, robot};
  ImuService imu_service{4, robot};
  PowerService power_service{5, robot};
  GpsService gps_service{6, robot};

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
