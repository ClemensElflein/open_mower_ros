//
// Created by clemens on 30.11.24.
//

#include "GpsPositionServiceInterface.h"
bool GpsPositionServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  StartTransaction(true);
  SetRegisterBaudrate(921600);
  SetRegisterProtocol("UBX", 3);
  double llh[3]{};
  SetRegisterDatumLatLonHeight(llh,3);
  CommitTransaction();
  return true;
}


void GpsPositionServiceInterface::OnTransactionStart(uint64_t timestamp) {
  pose_msg_.header.frame_id = "gps";
  pose_msg_.header.stamp = ros::Time::now();
  pose_msg_.header.seq++;
  pose_msg_.motion_vector_valid = false;
  pose_msg_.orientation_valid = false;
  pose_msg_.sensor_stamp = timestamp / 1000;
  pose_msg_.flags = 0;
}


void GpsPositionServiceInterface::OnPositionXYZChanged(const double* new_value, uint32_t length) {
  if(length != 3) {
    ROS_INFO_STREAM("OnPositionXYZChanged called with length " << length);
    return;
  }
  pose_msg_.pose.pose.position.x = new_value[0];
  pose_msg_.pose.pose.position.y = new_value[1];
  pose_msg_.pose.pose.position.z = new_value[2];
}
void GpsPositionServiceInterface::OnPositionAccuracyChanged(const double& new_value) {
  pose_msg_.position_accuracy = static_cast<float>(new_value);
}
void GpsPositionServiceInterface::OnFixTypeChanged(const char* new_value, uint32_t length) {
  pose_msg_.flags |= xbot_msgs::AbsolutePose::FLAG_GPS_RTK;
  std::string type(new_value, length);
  if(type == "FIX") {
    pose_msg_.flags |= xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED;
  } else if(type == "FLOAT") {
    pose_msg_.flags |= xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT;
  }
}
void GpsPositionServiceInterface::OnMotionVectorXYZChanged(const double* new_value, uint32_t length) {
  if(length != 3) {
    ROS_INFO_STREAM("OnMotionVectorXYZChanged called with length " << length);
    return;
  }
  pose_msg_.motion_vector_valid = true;
  pose_msg_.motion_vector.x = new_value[0];
  pose_msg_.motion_vector.y = new_value[1];
  pose_msg_.motion_vector.z = new_value[2];
}
void GpsPositionServiceInterface::OnMotionHeadingAndAccuracyChanged(const double* new_value, uint32_t length) {
  if(length != 2) {
    ROS_INFO_STREAM("OnMotionHeadingAndAccuracyChanged called with length " << length);
    return;
  }
  pose_msg_.motion_heading = new_value[0];
  pose_msg_.motion_vector_valid = true;
}
void GpsPositionServiceInterface::OnVehicleHeadingAndAccuracyChanged(const double* new_value, uint32_t length) {
  if(length != 2) {
    ROS_INFO_STREAM("OnVehicleHeadingAndAccuracyChanged called with length " << length);
    return;
  }
  pose_msg_.vehicle_heading = new_value[0];
  pose_msg_.orientation_accuracy = new_value[1];
  pose_msg_.orientation_valid = true;
}
void GpsPositionServiceInterface::OnNMEAChanged(const char* new_value, uint32_t length) {
  (void)new_value;
  (void)length;
}
void GpsPositionServiceInterface::OnServiceConnected(uint16_t service_id) {
}
void GpsPositionServiceInterface::OnTransactionEnd() {
  absolute_pose_publisher_.publish(pose_msg_);
}
void GpsPositionServiceInterface::OnServiceDisconnected(uint16_t service_id) {
}
