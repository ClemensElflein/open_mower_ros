//
// Created by clemens on 30.11.24.
//

#include "GpsServiceInterface.h"

#include "robot_localization/navsat_conversions.h"

GpsServiceInterface::GpsServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                                         const ros::Publisher& imu_publisher,
                                         double datum_lat, double datum_long, double datum_height)
    : GpsServiceInterfaceBase(service_id, ctx), absolute_pose_publisher_(imu_publisher) {
  RobotLocalization::NavsatConversions::LLtoUTM(datum_lat, datum_long, datum_n_, datum_e_, datum_zone_);
  datum_u_ = datum_height;
}

bool GpsServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  StartTransaction(true);
  SetRegisterBaudrate(921600);
  SetRegisterProtocol("UBX", 3);
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
  double e, n;
  std::string zone;
  RobotLocalization::NavsatConversions::LLtoUTM(new_value[0], new_value[1], n, e, zone);
  pose_msg_.pose.pose.position.x = e - datum_e_;
  pose_msg_.pose.pose.position.y = n - datum_n_;
  pose_msg_.pose.pose.position.z = new_value[2] - datum_u_;
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
