//
// Created by clemens on 28.08.24.
//

#include "LidarServiceInterface.h"
bool LidarServiceInterface::OnConfigurationRequested(const std::string& uid) {
  return true;
}
void LidarServiceInterface::OnAngleMinRadChanged(const double& new_value) {
  laser_scan_msg_.angle_min = new_value;
}
void LidarServiceInterface::OnAngleMaxRadChanged(const double& new_value) {
  laser_scan_msg_.angle_max = new_value;
}
void LidarServiceInterface::OnTimeIncrementSecondsChanged(const float& new_value) {
  laser_scan_msg_.time_increment = new_value;
}
void LidarServiceInterface::OnScanTimeSecondsChanged(const float& new_value) {
  laser_scan_msg_.scan_time = new_value;
}
void LidarServiceInterface::OnRangeMinMChanged(const float& new_value) {
  laser_scan_msg_.range_min = new_value;
}
void LidarServiceInterface::OnRangeMaxMChanged(const float& new_value) {
  laser_scan_msg_.range_max = new_value;
}
void LidarServiceInterface::OnRangesChanged(const float* new_value, uint32_t length) {
  laser_scan_msg_.ranges.clear();
  laser_scan_msg_.ranges.insert(laser_scan_msg_.ranges.end(), new_value, new_value + length);
}
void LidarServiceInterface::OnIntensitiesChanged(const float* new_value, uint32_t length) {
  laser_scan_msg_.intensities.clear();
  laser_scan_msg_.intensities.insert(laser_scan_msg_.intensities.end(), new_value, new_value + length);
}
void LidarServiceInterface::OnTransactionEnd() {
  laser_scan_msg_.header.frame_id = "lidar";
  laser_scan_msg_.header.seq++;
  laser_scan_msg_.header.stamp = ros::Time::now();
  laser_scan_msg_.range_min = 0.0;
  laser_scan_msg_.range_max = 20.0;
  laser_scan_msg_.angle_increment =
      fmod(laser_scan_msg_.angle_max - laser_scan_msg_.angle_min + 2.0 * M_PI, 2.0 * M_PI) /
      laser_scan_msg_.ranges.size();

  lidar_publisher_.publish(laser_scan_msg_);
}
