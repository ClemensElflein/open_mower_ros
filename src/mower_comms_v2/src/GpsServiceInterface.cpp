//
// Created by clemens on 30.11.24.
//

#include "GpsServiceInterface.h"

#include <nmea_msgs/Sentence.h>

#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include "GeographicLib/DMS.hpp"
#include "robot_localization/navsat_conversions.h"

GpsServiceInterface::GpsServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                                         const ros::Publisher& absolute_pose_publisher,
                                         const ros::Publisher& nmea_publisher, const ros::NodeHandle& param_nh)
    : GpsServiceInterfaceBase(service_id, ctx),
      absolute_pose_publisher_(absolute_pose_publisher),
      nmea_publisher_(nmea_publisher) {
  int baud_rate = 0;
  param_nh.getParam("services/gps/baud_rate", baud_rate);
  param_nh.getParam("services/gps/protocol", protocol_);
  int port_index = 0;
  param_nh.getParam("services/gps/port_index", port_index);
  if (baud_rate == 0 || protocol_.empty()) {
    ROS_ERROR("Need to specify GPS protocol and baud rate!");
    param_error_ = 1;
    return;
  }
  baud_rate_ = baud_rate;
  port_index_ = port_index;
  ROS_INFO_STREAM("GPS protocol: " << protocol_ << ", baud rate: " << baud_rate_
                                   << ", gps port index:" << static_cast<int>(port_index_));

  double datum_lat, datum_long, datum_height;
  bool has_datum = true;
  has_datum &= param_nh.getParam("services/gps/datum_lat", datum_lat);
  has_datum &= param_nh.getParam("services/gps/datum_long", datum_long);
  has_datum &= param_nh.getParam("services/gps/datum_height", datum_height);
  if (!has_datum) {
    ROS_ERROR_STREAM("You need to provide datum_lat and datum_long and datum_height in order to use the absolute mode");
    param_error_ = 2;
    return;
  }
  ROS_INFO_STREAM("Datum: " << datum_lat << ", " << datum_long << ", " << datum_height);

  param_nh.getParam("services/gps/absolute_coords", absolute_coords_);

  RobotLocalization::NavsatConversions::LLtoUTM(datum_lat, datum_long, datum_n_, datum_e_, datum_zone_);
  datum_u_ = datum_height;
}

void GpsServiceInterface::SendNMEA(double lat_in, double lon_in) {
  using namespace GeographicLib;
  // only send every 10 seconds, this will be more than needed
  static ros::Time last_vrs_feedback(0.0);
  static nmea_msgs::Sentence vrs_msg{};
  if ((ros::Time::now() - last_vrs_feedback).toSec() < 10.0) {
    return;
  }
  last_vrs_feedback = ros::Time::now();

  auto lat = GeographicLib::DMS::Encode(lat_in, GeographicLib::DMS::component::MINUTE, 4,
                                        GeographicLib::DMS::flag::LATITUDE, ';');
  auto lon = GeographicLib::DMS::Encode(lon_in, GeographicLib::DMS::component::MINUTE, 4,
                                        GeographicLib::DMS::flag::LONGITUDE, ';');

  // remove separator char
  boost::erase_all(lat, ";");
  boost::erase_all(lon, ";");

  auto lat_hemisphere = lat.substr(lat.length() - 1, 1);
  auto lon_hemisphere = lon.substr(lon.length() - 1, 1);

  std::stringstream message_ss;
  auto time_facet = new boost::posix_time::time_facet("%H%M%s");

  message_ss.imbue(std::locale(message_ss.getloc(), time_facet));
  message_ss << "GPGGA," << ros::Time::now().toBoost() << "," << lat.substr(0, lat.length() - 1) << ","
             << lat_hemisphere << "," << lon.substr(0, lon.length() - 1) << "," << lon_hemisphere
             << ",1,0,0,0,M,0,M,0000,";

  std::string message_content = message_ss.str();

  uint8_t checksum = 0;
  for (const auto c : message_content) {
    checksum ^= c;
  }

  std::stringstream final_message_ss;
  final_message_ss << "$" << message_content << "*" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                   << (int)checksum;

  vrs_msg.header.frame_id = "gps";
  vrs_msg.header.seq++;
  vrs_msg.header.stamp = ros::Time::now();
  vrs_msg.sentence = final_message_ss.str();
  nmea_publisher_.publish(vrs_msg);
}

bool GpsServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  StartTransaction(true);
  SetRegisterBaudrate(baud_rate_);
  if (protocol_ == "UBX") {
    SetRegisterProtocol(ProtocolType::UBX);
  } else if (protocol_ == "NMEA") {
    SetRegisterProtocol(ProtocolType::NMEA);
  } else {
    ROS_ERROR_STREAM("Invalid Protocol: " << protocol_);
  }
  if (port_index_ > 0) {
    SetRegisterUart(port_index_);
  }
  CommitTransaction();
  return true;
}

void GpsServiceInterface::OnTransactionStart(uint64_t timestamp) {
  pose_msg_.header.frame_id = "gps";
  pose_msg_.header.stamp = ros::Time::now();
  pose_msg_.header.seq++;
  pose_msg_.motion_vector_valid = false;
  pose_msg_.orientation_valid = false;
  pose_msg_.sensor_stamp = timestamp / 1000;
  pose_msg_.flags = 0;
}

void GpsServiceInterface::OnPositionChanged(const double* new_value, uint32_t length) {
  if (length != 3) {
    ROS_INFO_STREAM("OnPositionChanged called with length " << length);
    return;
  }
  if (absolute_coords_) {
    SendNMEA(new_value[0], new_value[1]);
    double e, n;
    std::string zone;
    RobotLocalization::NavsatConversions::LLtoUTM(new_value[0], new_value[1], n, e, zone);
    pose_msg_.pose.pose.position.x = e - datum_e_;
    pose_msg_.pose.pose.position.y = n - datum_n_;
    pose_msg_.pose.pose.position.z = new_value[2] - datum_u_;
  } else {
    double n = new_value[1] + datum_n_;
    double e = new_value[0] + datum_e_;
    double lat, lng;
    RobotLocalization::NavsatConversions::UTMtoLL(n, e, datum_zone_, lat, lng);
    SendNMEA(lat, lng);
    pose_msg_.pose.pose.position.x = new_value[0];
    pose_msg_.pose.pose.position.y = new_value[1];
    pose_msg_.pose.pose.position.z = new_value[2];
  }
}

void GpsServiceInterface::OnPositionHorizontalAccuracyChanged(const double& new_value) {
  pose_msg_.position_accuracy = static_cast<float>(new_value);
}

void GpsServiceInterface::OnFixTypeChanged(const char* new_value, uint32_t length) {
  pose_msg_.flags |= xbot_msgs::AbsolutePose::FLAG_GPS_RTK;
  std::string type(new_value, length);
  if (type == "FIX") {
    pose_msg_.flags |= xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED;
  } else if (type == "FLOAT") {
    pose_msg_.flags |= xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT;
  }
}

void GpsServiceInterface::OnMotionVectorENUChanged(const double* new_value, uint32_t length) {
  if (length != 3) {
    ROS_INFO_STREAM("OnMotionVectorENUChanged called with length " << length);
    return;
  }
  pose_msg_.motion_vector_valid = true;
  pose_msg_.motion_vector.x = new_value[0];
  pose_msg_.motion_vector.y = new_value[1];
  pose_msg_.motion_vector.z = new_value[2];
}

void GpsServiceInterface::OnMotionHeadingAndAccuracyChanged(const double* new_value, uint32_t length) {
  if (length != 2) {
    ROS_INFO_STREAM("OnMotionHeadingAndAccuracyChanged called with length " << length);
    return;
  }
  pose_msg_.motion_heading = new_value[0];
  pose_msg_.motion_vector_valid = true;
}

void GpsServiceInterface::OnVehicleHeadingAndAccuracyChanged(const double* new_value, uint32_t length) {
  if (length != 2) {
    ROS_INFO_STREAM("OnVehicleHeadingAndAccuracyChanged called with length " << length);
    return;
  }
  pose_msg_.vehicle_heading = new_value[0];
  pose_msg_.orientation_accuracy = new_value[1];
  pose_msg_.orientation_valid = true;
}

void GpsServiceInterface::OnTransactionEnd() {
  absolute_pose_publisher_.publish(pose_msg_);
}
