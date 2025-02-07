//
// Created by Clemens Elflein on 15.03.22.
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
#include <geometry_msgs/TwistStamped.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/Emergency.h>
#include <mower_msgs/EmergencyStopSrv.h>
#include <mower_msgs/HighLevelControlSrv.h>
#include <mower_msgs/MowerControlSrv.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "DiffDriveServiceInterface.h"
#include "DockingSensorServiceInterface.h"
#include "EmergencyServiceInterface.h"
#include "GpsServiceInterface.h"
#include "ImuServiceInterface.h"
#include "LidarServiceInterface.h"
#include "MowerServiceInterface.h"
#include "PowerServiceInterface.h"

ros::Publisher status_pub;
ros::Publisher power_pub;
ros::Publisher gps_position_pub;
ros::Publisher status_left_esc_pub;
ros::Publisher status_right_esc_pub;
ros::Publisher emergency_pub;
ros::Publisher actual_twist_pub;

ros::Publisher sensor_imu_pub;
ros::Publisher sensor_lidar_pub;
ros::Publisher sensor_dock_pub;

ros::ServiceClient highLevelClient;

std::unique_ptr<EmergencyServiceInterface> emergency_service = nullptr;
std::unique_ptr<DiffDriveServiceInterface> diff_drive_service = nullptr;
std::unique_ptr<MowerServiceInterface> mower_service = nullptr;
std::unique_ptr<ImuServiceInterface> imu_service = nullptr;
std::unique_ptr<LidarServiceInterface> lidar_service = nullptr;
std::unique_ptr<PowerServiceInterface> power_service = nullptr;
std::unique_ptr<DockingSensorServiceInterface> docking_sensor_service = nullptr;
std::unique_ptr<GpsServiceInterface> gps_service = nullptr;

xbot::serviceif::Context ctx{};

bool setEmergencyStop(mower_msgs::EmergencyStopSrvRequest &req, mower_msgs::EmergencyStopSrvResponse &res) {
  // This should never be the case, also this is no race condition, because callback will only be called
  // after initialization whereas the service is created during intialization
  if (!emergency_service) return false;

  emergency_service->SetEmergency(req.emergency);
  return true;
}

void velReceived(const geometry_msgs::Twist::ConstPtr &msg) {
  if (!diff_drive_service) return;
  diff_drive_service->SendTwist(msg);
}

void sendEmergencyHeartbeatTimerTask(const ros::TimerEvent &) {
  emergency_service->Heartbeat();
}

bool setMowEnabled(mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvRequest &res) {
  mower_service->SetMowerEnabled(req.mow_enabled);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mower_comms_v2");

  ros::NodeHandle n;
  ros::NodeHandle paramNh("~");

  highLevelClient = n.serviceClient<mower_msgs::HighLevelControlSrv>("mower_service/high_level_control");

  ros::ServiceServer mow_service = n.advertiseService("ll/_service/mow_enabled", setMowEnabled);
  ros::ServiceServer ros_emergency_service = n.advertiseService("ll/_service/emergency", setEmergencyStop);
  ros::Subscriber cmd_vel_sub = n.subscribe("ll/cmd_vel", 0, velReceived, ros::TransportHints().tcpNoDelay(true));
  // ros::Subscriber high_level_status_sub = n.subscribe("/mower_logic/current_state", 0, highLevelStatusReceived);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.5), sendEmergencyHeartbeatTimerTask);

  std::string bind_ip = "0.0.0.0";
  paramNh.getParam("bind_ip", bind_ip);
  ROS_INFO_STREAM("Bind IP (Robot Internal): " << bind_ip);
  ctx = xbot::serviceif::Start(true, bind_ip);

  // Emergency service
  emergency_pub = n.advertise<mower_msgs::Emergency>("ll/emergency", 1);
  emergency_service = std::make_unique<EmergencyServiceInterface>(1, ctx, emergency_pub);
  emergency_service->Start();

  // Diff drive service
  actual_twist_pub = n.advertise<geometry_msgs::TwistStamped>("ll/diff_drive/measured_twist", 1);
  status_left_esc_pub = n.advertise<mower_msgs::ESCStatus>("ll/diff_drive/left_esc_status", 1);
  status_right_esc_pub = n.advertise<mower_msgs::ESCStatus>("ll/diff_drive/right_esc_status", 1);
  double wheel_ticks_per_m = 0.0;
  double wheel_distance_m = 0.0;
  paramNh.getParam("wheel_ticks_per_m", wheel_ticks_per_m);
  paramNh.getParam("wheel_distance_m", wheel_distance_m);
  ROS_INFO_STREAM("Wheel ticks [1/m]: " << wheel_ticks_per_m);
  ROS_INFO_STREAM("Wheel distance [m]: " << wheel_distance_m);
  diff_drive_service = std::make_unique<DiffDriveServiceInterface>(
      2, ctx, actual_twist_pub, status_left_esc_pub, status_right_esc_pub, wheel_ticks_per_m, wheel_distance_m);
  diff_drive_service->Start();

  // Mower service
  status_pub = n.advertise<mower_msgs::Status>("ll/mower_status", 1);
  mower_service = std::make_unique<MowerServiceInterface>(3, ctx, status_pub);
  mower_service->Start();

  // IMU service
  sensor_imu_pub = n.advertise<sensor_msgs::Imu>("ll/imu/data_raw", 1);
  imu_service = std::make_unique<ImuServiceInterface>(4, ctx, sensor_imu_pub);
  imu_service->Start();

  // Power service
  power_pub = n.advertise<mower_msgs::Power>("ll/power", 1);
  power_service = std::make_unique<PowerServiceInterface>(5, ctx, power_pub);
  power_service->Start();

  // GPS service
  double datum_lat, datum_long, datum_height;
  bool has_datum = true;
  has_datum &= paramNh.getParam("datum_lat", datum_lat);
  has_datum &= paramNh.getParam("datum_long", datum_long);
  has_datum &= paramNh.getParam("datum_height", datum_height);
  if (!has_datum) {
    ROS_ERROR_STREAM("You need to provide datum_lat and datum_long and datum_height in order to use the absolute mode");
    return 2;
  }
  ROS_INFO_STREAM("Datum: " << datum_lat << ", " << datum_long << ", " << datum_height);
  gps_position_pub = n.advertise<xbot_msgs::AbsolutePose>("ll/position/gps", 1);
  gps_service = std::make_unique<GpsServiceInterface>(6, ctx, gps_position_pub, datum_lat, datum_long, datum_height);
  gps_service->Start();

  // Lidar service
  sensor_lidar_pub = n.advertise<sensor_msgs::LaserScan>("ll/lidar", 1);
  lidar_service = std::make_unique<LidarServiceInterface>(42, ctx, sensor_lidar_pub);
  lidar_service->Start();

  // Docking sensor service
  sensor_dock_pub = n.advertise<mower_msgs::DockingSensor>("ll/dock_sensor", 1);
  docking_sensor_service = std::make_unique<DockingSensorServiceInterface>(43, ctx, sensor_dock_pub);
  docking_sensor_service->Start();

  ros::spin();

  return 0;
}
