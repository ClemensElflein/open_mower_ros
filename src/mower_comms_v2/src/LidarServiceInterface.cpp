//
// Created by clemens on 28.08.24.
//

#include "LidarServiceInterface.h"

#include "spdlog/spdlog.h"

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
  new_scan = new_value;
}

void LidarServiceInterface::OnTransactionEnd() {
  if (new_scan) {
    // append ranges from the old rotation
    {
      float current_angle_increment = fmodf(angle_max - angle_min + 2.0 * M_PI, 2.0 * M_PI) / ranges.size();
      while (angle_min > M_PI && !ranges.empty()) {
        if (laser_scan_msg_.ranges.size() < 400) {
          laser_scan_msg_.ranges.push_back(ranges.front());
        }
        ranges.pop_front();
        laser_scan_msg_.angle_max = angle_min;
        angle_min = fmodf(angle_min + current_angle_increment, M_PI * 2.0);
      }
    }

    // Send the gathered data
    const auto now = ros::Time::now();
    if (!laser_scan_msg_.ranges.empty()) {
      if (last_full_scan_time_.is_zero()) {
        last_full_scan_time_ = now;
      } else {
        {
          // some SLAM algorithm want a constant amount of points, otherwise it rejects the laser scan....
          // our laser theoretically gives 400 pts, but sometimes its +- a few, so yea..
          while (laser_scan_msg_.ranges.size() > 400) {
            laser_scan_msg_.ranges.pop_back();
          }
          while (laser_scan_msg_.ranges.size() < 400) {
            laser_scan_msg_.ranges.push_back(0);
          }
        }

        laser_scan_msg_.header.frame_id = "lidar";
        laser_scan_msg_.header.seq++;
        laser_scan_msg_.range_min = 0.0;
        laser_scan_msg_.range_max = 20.0;
        laser_scan_msg_.scan_time = (now - last_full_scan_time_).toSec();
        laser_scan_msg_.time_increment = laser_scan_msg_.scan_time / laser_scan_msg_.ranges.size();

        // our laser scanner does sometimes report weird start/end angles, so we set min-max angles manually
        // TODO: either fix, or make configurable
        laser_scan_msg_.angle_max = 2.0 * M_PI - M_PI / 800;
        laser_scan_msg_.angle_min = 0;
        // END: fix

        laser_scan_msg_.angle_increment =
            fmodf(laser_scan_msg_.angle_max - laser_scan_msg_.angle_min + 2.0 * M_PI, 2.0 * M_PI) /
            laser_scan_msg_.ranges.size();

        last_full_scan_time_ = now;

        // TODO: copy needed?
        sensor_msgs::LaserScan copy{laser_scan_msg_};
        lidar_publisher_.publish(copy);
      }
    }
    new_scan = false;

    // Set the data from the last transaction as a start
    laser_scan_msg_.header.stamp = now - ros::Duration().fromNSec(time_offset_micros * 1000);
    laser_scan_msg_.angle_min = angle_min;
    laser_scan_msg_.angle_max = angle_max;
    laser_scan_msg_.ranges.clear();
    laser_scan_msg_.ranges.insert(laser_scan_msg_.ranges.end(), ranges.begin(), ranges.end());
    laser_scan_msg_.range_max = max_m;
    laser_scan_msg_.range_min = min_m;
  } else {
    // append to the scan
    laser_scan_msg_.range_max = std::max(laser_scan_msg_.range_max, max_m);
    laser_scan_msg_.range_min = std::min(laser_scan_msg_.range_min, min_m);

    laser_scan_msg_.angle_max = angle_max;
    // keep it to a sane value in case the new_scan flag is not sent correctly
    if (laser_scan_msg_.ranges.size() < 400) {
      laser_scan_msg_.ranges.insert(laser_scan_msg_.ranges.end(), ranges.begin(), ranges.end());
    }
  }
}

void LidarServiceInterface::OnTransactionStart(uint64_t timestamp) {
  time_offset_micros = timestamp;
}
