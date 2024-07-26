//
// Created by clemens on 26.07.24.
//

#ifndef IMUSERVICEINTERFACE_H
#define IMUSERVICEINTERFACE_H

#include <ros/publisher.h>
#include <sensor_msgs/Imu.h>

#include <IMUServiceInterfaceBase.hpp>

class ImuServiceInterface : public IMUServiceInterfaceBase {
 public:
  ImuServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx, const ros::Publisher& imu_publisher)
      : IMUServiceInterfaceBase(service_id, ctx), imu_publisher_(imu_publisher) {
  }
  bool OnConfigurationRequested(const std::string& uid) override;

 protected:
  void OnAxesChanged(const double* new_value, uint32_t length) override;

 private:
  void OnServiceConnected(const std::string& uid) override;
  void OnTransactionStart(uint64_t timestamp) override;
  void OnTransactionEnd() override;
  void OnServiceDisconnected(const std::string& uid) override;
  const ros::Publisher& imu_publisher_;

  sensor_msgs::Imu imu_msg{};
};

#endif  // IMUSERVICEINTERFACE_H
