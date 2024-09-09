//
// Created by clemens on 28.08.24.
//

#ifndef LIDARSERVICEINTERFACE_H
#define LIDARSERVICEINTERFACE_H

#include <ros/publisher.h>
#include <sensor_msgs/LaserScan.h>

#include <LidarServiceInterfaceBase.hpp>

class LidarServiceInterface : public LidarServiceInterfaceBase {
 public:
  LidarServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx, const ros::Publisher& lidar_publisher)
      : LidarServiceInterfaceBase(service_id, ctx), lidar_publisher_(lidar_publisher) {
  }
  bool OnConfigurationRequested(const std::string& uid) override;

 protected:
  void OnAngleMinRadChanged(const double& new_value) override;
  void OnAngleMaxRadChanged(const double& new_value) override;
  void OnTimeIncrementSecondsChanged(const float& new_value) override;
  void OnScanTimeSecondsChanged(const float& new_value) override;
  void OnRangeMinMChanged(const float& new_value) override;
  void OnRangeMaxMChanged(const float& new_value) override;
  void OnRangesChanged(const float* new_value, uint32_t length) override;
  void OnIntensitiesChanged(const float* new_value, uint32_t length) override;

 private:
  void OnTransactionEnd() override;

  const ros::Publisher& lidar_publisher_;
  sensor_msgs::LaserScan laser_scan_msg_{};
};

#endif  // LIDARSERVICEINTERFACE_H
