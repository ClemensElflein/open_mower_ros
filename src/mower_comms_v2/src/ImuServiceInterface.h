//
// Created by clemens on 26.07.24.
//

#ifndef IMUSERVICEINTERFACE_H
#define IMUSERVICEINTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <ImuServiceInterfaceBase.hpp>

class ImuServiceInterface : public ImuServiceInterfaceBase {
 public:
  ImuServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx, const ros::Publisher& imu_publisher,
                      const ros::NodeHandle& param_nh)
      : ImuServiceInterfaceBase(service_id, ctx), imu_publisher_(imu_publisher) {
    param_nh.getParam("services/imu/axis_config", axis_config_);
    ROS_INFO_STREAM("IMU axis config: " << axis_config_);
  }

  bool OnConfigurationRequested(uint16_t service_id) override;

 protected:
  void OnAxesChanged(const double* new_value, uint32_t length) override;
  void OnTransactionStart(uint64_t timestamp) override;

 private:
  const ros::Publisher& imu_publisher_;
  std::string axis_config_;

  sensor_msgs::Imu imu_msg{};
  bool validateAxisConfig();

  // Firmware timestamp (microseconds since firmware boot) of the packet
  // currently being decoded, and the running estimate of the offset that maps
  // it onto ROS time.
  uint64_t transaction_timestamp_ = 0;
  double fw_to_ros_offset_ = 0.0;
  bool have_offset_ = false;

  ros::Time stampFromFirmware(uint64_t fw_micros);
};

#endif  // IMUSERVICEINTERFACE_H
