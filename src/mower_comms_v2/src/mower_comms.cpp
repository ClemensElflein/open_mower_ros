//
// Created by Clemens Elflein on 15.03.22.
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
#include <geometry_msgs/TwistStamped.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/Emergency.h>
#include <mower_msgs/EmergencyStopSrv.h>
#include <mower_msgs/HighLevelControlSrv.h>
#include <mower_msgs/MowerControlSrv.h>
#include <nmea_msgs/Sentence.h>
#include <ros/ros.h>
#include <rtcm_msgs/Message.h>
#include <sensor_msgs/Imu.h>
#include <spdlog/sinks/callback_sink.h>
#include <spdlog/spdlog.h>
#include <std_msgs/String.h>

#include "../../../services/service_ids.h"
#include "BmsServiceInterface.h"
#include "DiffDriveServiceInterface.h"
#include "EmergencyServiceInterface.h"
#include "GpsServiceInterface.h"
#include "HighLevelServiceInterface.h"
#include "ImuServiceInterface.h"
#include "InputServiceInterface.h"
#include "MowerServiceInterface.h"
#include "PowerServiceInterface.h"

ros::Publisher status_pub;
ros::Publisher nmea_pub;
ros::Publisher power_pub;
ros::Publisher bms_pub;
ros::Publisher gps_position_pub;
ros::Publisher status_left_esc_pub;
ros::Publisher status_right_esc_pub;
ros::Publisher emergency_pub;
ros::Publisher actual_twist_pub;
ros::Publisher action_pub;

ros::Publisher sensor_imu_pub;

ros::ServiceClient highLevelClient;

std::unique_ptr<EmergencyServiceInterface> emergency_service = nullptr;
std::unique_ptr<DiffDriveServiceInterface> diff_drive_service = nullptr;
std::unique_ptr<MowerServiceInterface> mower_service = nullptr;
std::unique_ptr<ImuServiceInterface> imu_service = nullptr;
std::unique_ptr<PowerServiceInterface> power_service = nullptr;
std::unique_ptr<BmsServiceInterface> bms_service = nullptr;
std::unique_ptr<GpsServiceInterface> gps_service = nullptr;
std::unique_ptr<InputServiceInterface> input_service = nullptr;
std::unique_ptr<HighLevelServiceInterface> high_level_service = nullptr;

xbot::serviceif::Context ctx{};

bool setEmergencyStop(mower_msgs::EmergencyStopSrvRequest& req, mower_msgs::EmergencyStopSrvResponse& res) {
  emergency_service->SetHighLevelEmergency(req.emergency);
  return true;
}

void velReceived(const geometry_msgs::Twist::ConstPtr& msg) {
  diff_drive_service->SendTwist(msg);
}

void rtcmReceived(const rtcm_msgs::Message& msg) {
  static std::vector<uint8_t> rtcm_buffer{};
  static ros::Time last_time_sent{0};
  ros::Time now = ros::Time::now();
  // Append the bytes to the buffer
  rtcm_buffer.insert(rtcm_buffer.end(), msg.message.begin(), msg.message.end());
  // In order to not spam after each received byte, limit packets to 5Hz and to max 1k of size
  if (rtcm_buffer.size() < 1000 && (now - last_time_sent).toSec() < 0.2) return;
  last_time_sent = now;
  gps_service->SendRTCM(rtcm_buffer.data(), rtcm_buffer.size());
  rtcm_buffer.clear();
}

void sendEmergencyHeartbeatTimerTask(const ros::TimerEvent&) {
  emergency_service->Heartbeat();
}

void actionReceived(const std_msgs::String::ConstPtr& action) {
  input_service->OnAction(action->data);
}

void sendMowerEnabledTimerTask(const ros::TimerEvent& e) {
  mower_service->Tick();
}

bool setMowEnabled(mower_msgs::MowerControlSrvRequest& req, mower_msgs::MowerControlSrvResponse& res) {
  mower_service->SetMowerEnabled(req.mow_enabled);
  return true;
}

static void spdlog_cb(const spdlog::details::log_msg& msg) {
  ros::console::Level level = ros::console::Level::Info;
  switch (msg.level) {
    case spdlog::level::level_enum::trace:
    case spdlog::level::level_enum::debug: level = ros::console::Level::Debug; break;
    case spdlog::level::level_enum::info: break;
    case spdlog::level::level_enum::warn: level = ros::console::Level::Warn; break;
    case spdlog::level::level_enum::err: level = ros::console::Level::Error; break;
    case spdlog::level::level_enum::critical: level = ros::console::Level::Fatal; break;
    case spdlog::level::level_enum::off: return;
  }
  ROS_LOG(level, ROSCONSOLE_DEFAULT_NAME, "%.*s", static_cast<int>(msg.payload.size()), msg.payload.data());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mower_comms_v2");

  {
    auto sink = std::make_shared<spdlog::sinks::callback_sink_mt>(spdlog_cb);
    auto logger = std::make_shared<spdlog::logger>("", std::move(sink));
    spdlog::set_default_logger(logger);
  }

  ros::NodeHandle n;
  ros::NodeHandle paramNh("/ll");

  highLevelClient = n.serviceClient<mower_msgs::HighLevelControlSrv>("mower_service/high_level_control");
  action_pub = n.advertise<std_msgs::String>("xbot/action", 1);

  std::string bind_ip = "0.0.0.0";
  paramNh.getParam("bind_ip", bind_ip);
  ROS_INFO_STREAM("Bind IP (Robot Internal): " << bind_ip);
  xbot::serviceif::SetShutdownCallback([] { ros::requestShutdown(); });
  ctx = xbot::serviceif::Start(true, bind_ip);

  // Emergency service
  emergency_pub = n.advertise<mower_msgs::Emergency>("ll/emergency", 1);
  emergency_service = std::make_unique<EmergencyServiceInterface>(xbot::service_ids::EMERGENCY, ctx, emergency_pub);
  emergency_service->Start();

  // Diff drive service
  actual_twist_pub = n.advertise<geometry_msgs::TwistStamped>("ll/diff_drive/measured_twist", 1);
  status_left_esc_pub = n.advertise<mower_msgs::ESCStatus>("ll/diff_drive/left_esc_status", 1);
  status_right_esc_pub = n.advertise<mower_msgs::ESCStatus>("ll/diff_drive/right_esc_status", 1);
  diff_drive_service = std::make_unique<DiffDriveServiceInterface>(xbot::service_ids::DIFF_DRIVE, ctx, actual_twist_pub,
                                                                   status_left_esc_pub, status_right_esc_pub, paramNh);
  if (int err = diff_drive_service->GetParamError()) return err;
  diff_drive_service->Start();

  // Mower service
  status_pub = n.advertise<mower_msgs::Status>("ll/mower_status", 1);
  mower_service = std::make_unique<MowerServiceInterface>(xbot::service_ids::MOWER, ctx, status_pub);
  mower_service->Start();

  // IMU service
  sensor_imu_pub = n.advertise<sensor_msgs::Imu>("ll/imu/data_raw", 1);
  imu_service = std::make_unique<ImuServiceInterface>(xbot::service_ids::IMU, ctx, sensor_imu_pub, paramNh);
  imu_service->Start();

  // Power service
  power_pub = n.advertise<mower_msgs::Power>("ll/power", 1);
  power_service = std::make_unique<PowerServiceInterface>(xbot::service_ids::POWER, ctx, power_pub, paramNh);
  if (int err = power_service->GetParamError()) return err;
  power_service->Start();

  // BMS service
  bms_pub = n.advertise<mower_msgs::Bms>("ll/bms", 1);
  bms_service = std::make_unique<BmsServiceInterface>(xbot::service_ids::BMS, ctx, bms_pub);
  bms_service->Start();

  // GPS service
  gps_position_pub = n.advertise<xbot_msgs::AbsolutePose>("ll/position/gps", 1);
  nmea_pub = n.advertise<nmea_msgs::Sentence>("ll/position/gps/nmea", 1);
  gps_service = std::make_unique<GpsServiceInterface>(xbot::service_ids::GPS, ctx, gps_position_pub, nmea_pub, paramNh);
  if (int err = gps_service->GetParamError()) return err;
  gps_service->Start();

  // Input service
  input_service = std::make_unique<InputServiceInterface>(xbot::service_ids::INPUT, ctx, paramNh, action_pub);
  input_service->Start();

  // HighLevel service
  high_level_service = std::make_unique<HighLevelServiceInterface>(xbot::service_ids::HIGH_LEVEL, ctx);
  high_level_service->Start();

  // All subscriptions, timers and service servers are registered after all service interfaces are
  // fully constructed, so callbacks can never fire on null pointers.
  ros::ServiceServer mow_service = n.advertiseService("ll/_service/mow_enabled", setMowEnabled);
  ros::ServiceServer ros_emergency_service = n.advertiseService("ll/_service/emergency", setEmergencyStop);
  ros::Subscriber cmd_vel_sub = n.subscribe("ll/cmd_vel", 0, velReceived, ros::TransportHints().tcpNoDelay(true));
  ros::Subscriber rtcm_sub = n.subscribe("ll/position/gps/rtcm", 0, rtcmReceived);
  // ros::Subscriber high_level_status_sub = n.subscribe("/mower_logic/current_state", 0, highLevelStatusReceived);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.5), sendEmergencyHeartbeatTimerTask);
  ros::Timer publish_timer_2 = n.createTimer(ros::Duration(5.0), sendMowerEnabledTimerTask);
  ros::Subscriber action_sub = n.subscribe("xbot/action", 0, actionReceived, ros::TransportHints().tcpNoDelay(true));

  ROS_INFO("All mower_comms_v2 services started");

  ros::spin();
  xbot::serviceif::Stop();

  return 0;
}
