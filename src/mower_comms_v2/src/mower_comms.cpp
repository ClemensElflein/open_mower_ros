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
#include <mower_msgs/Emergency.h>
#include <mower_msgs/EmergencyStopSrv.h>
#include <mower_msgs/HighLevelControlSrv.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "DiffDriveServiceInterface.h"
#include "EmergencyServiceInterface.h"
#include "ImuServiceInterface.h"
#include "MowerServiceInterface.h"

ros::Publisher status_pub;
ros::Publisher emergency_pub;
ros::Publisher actual_twist_pub;

ros::Publisher sensor_imu_pub;

ros::ServiceClient highLevelClient;

std::unique_ptr<EmergencyServiceInterface> emergency_service = nullptr;
std::unique_ptr<DiffDriveServiceInterface> diff_drive_service = nullptr;
std::unique_ptr<MowerServiceInterface> mower_service = nullptr;
std::unique_ptr<ImuServiceInterface> imu_service = nullptr;

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

int main(int argc, char **argv) {
  ros::init(argc, argv, "mower_comms_v2");

  ros::NodeHandle n;
  ros::NodeHandle paramNh("~");

  highLevelClient = n.serviceClient<mower_msgs::HighLevelControlSrv>("mower_service/high_level_control");

  // Ticks / m and wheel distance for this robot
  double wheel_ticks_per_m = 0.0;
  double wheel_distance_m = 0.0;

  paramNh.getParam("wheel_ticks_per_m", wheel_ticks_per_m);
  paramNh.getParam("wheel_distance_m", wheel_distance_m);

  ROS_INFO_STREAM("Wheel ticks [1/m]: " << wheel_ticks_per_m);
  ROS_INFO_STREAM("Wheel distance [m]: " << wheel_distance_m);

  status_pub = n.advertise<mower_msgs::Status>("mower/status", 1);
  emergency_pub = n.advertise<mower_msgs::Emergency>("mower/emergency", 1);
  actual_twist_pub = n.advertise<geometry_msgs::TwistStamped>("mower/actual_twist", 1);

  sensor_imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
  // ros::ServiceServer mow_service = n.advertiseService("mower_service/mow_enabled", setMowEnabled);
  ros::ServiceServer ros_emergency_service = n.advertiseService("mower_service/emergency", setEmergencyStop);
  ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 0, velReceived, ros::TransportHints().tcpNoDelay(true));
  // ros::Subscriber high_level_status_sub = n.subscribe("/mower_logic/current_state", 0, highLevelStatusReceived);
  // ros::Timer publish_timer = n.createTimer(ros::Duration(0.02), publishActuatorsTimerTask);

  ctx = xbot::serviceif::Start();

  emergency_service = std::make_unique<EmergencyServiceInterface>(1, ctx, emergency_pub);
  diff_drive_service =
      std::make_unique<DiffDriveServiceInterface>(2, ctx, actual_twist_pub, wheel_ticks_per_m, wheel_distance_m);
  mower_service = std::make_unique<MowerServiceInterface>(3, ctx, status_pub);
  imu_service = std::make_unique<ImuServiceInterface>(4, ctx, sensor_imu_pub);

  emergency_service->Start();
  diff_drive_service->Start();
  mower_service->Start();
  imu_service->Start();

  while (ctx.io->OK() && ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
