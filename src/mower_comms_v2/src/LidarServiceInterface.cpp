//
// Created by clemens on 28.08.24.
//

#include "LidarServiceInterface.h"

bool LidarServiceInterface::OnConfigurationRequested(const std::string &uid) {
  return true;
}

void LidarServiceInterface::OnAngleMinRadChanged(const double &new_value) {
  angle_min = new_value;
}

void LidarServiceInterface::OnAngleMaxRadChanged(const double &new_value) {
  angle_max = new_value;
}

void LidarServiceInterface::OnTimeIncrementSecondsChanged(const float &new_value) {
  time_increment = new_value;
}

void LidarServiceInterface::OnScanTimeSecondsChanged(const float &new_value) {
  scan_time = new_value;
}

void LidarServiceInterface::OnRangeMinMChanged(const float &new_value) {
  min_m = std::min(min_m, new_value);
}

void LidarServiceInterface::OnRangeMaxMChanged(const float &new_value) {
  max_m = std::max(max_m, new_value);
}

void LidarServiceInterface::OnRangesChanged(const float *new_value, uint32_t length) {
  ranges.clear();
  ranges.insert(ranges.end(), new_value, new_value + length);
}

void LidarServiceInterface::OnIntensitiesChanged(const float *new_value, uint32_t length) {
}

void LidarServiceInterface::OnNewScanChanged(const uint8_t &new_value) {
  new_scan = true;
}

void LidarServiceInterface::OnTransactionEnd() {
  if (new_scan) {
    // Send the gathered data
    if (laser_scan_msg_.ranges.size() > 0) {
      laser_scan_msg_.header.frame_id = "base_link";
      laser_scan_msg_.header.seq++;
      laser_scan_msg_.header.stamp = ros::Time::now();
      laser_scan_msg_.range_min = 0.0;
      laser_scan_msg_.range_max = 20.0;
      laser_scan_msg_.angle_increment =
          fmod(laser_scan_msg_.angle_max - laser_scan_msg_.angle_min + 2.0 * M_PI, 2.0 * M_PI) /
          laser_scan_msg_.ranges.size();

      lidar_publisher_.publish(laser_scan_msg_);
    }
    new_scan = false;

    // Set the data from the last transaction as a start
    laser_scan_msg_.angle_max = angle_max;
    laser_scan_msg_.angle_min = angle_min;
    laser_scan_msg_.ranges = ranges;
    laser_scan_msg_.time_increment = time_increment;
    laser_scan_msg_.scan_time = scan_time;
    laser_scan_msg_.range_max = max_m;
    laser_scan_msg_.range_min = min_m;
  } else {
    // append to the scan
    laser_scan_msg_.range_max = std::max(laser_scan_msg_.range_max, max_m);
    laser_scan_msg_.range_min = std::min(laser_scan_msg_.range_min, min_m);

    laser_scan_msg_.angle_max = std::max(laser_scan_msg_.angle_max, angle_max);
    laser_scan_msg_.angle_min = std::min(laser_scan_msg_.angle_min, angle_min);
    laser_scan_msg_.ranges.clear();
    laser_scan_msg_.ranges.insert(laser_scan_msg_.ranges.end(), ranges.begin(), ranges.end());
  }
}
