//
// Created by clemens on 25.07.24.
//

#ifndef MOWERSERVICEINTERFACE_H
#define MOWERSERVICEINTERFACE_H

#include <mower_msgs/Status.h>
#include <ros/publisher.h>

#include <MowerServiceInterfaceBase.hpp>

class MowerServiceInterface : public MowerServiceInterfaceBase {
 public:
  MowerServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                        const ros::Publisher& status_poblisher)
      : MowerServiceInterfaceBase(service_id, ctx), status_poblisher_(status_poblisher) {
  }

  bool OnConfigurationRequested(const std::string& uid) override;

 protected:
  void OnMowerStatusChanged(const uint8_t& new_value) override;
  void OnRaspberryPiPowerChanged(const uint8_t& new_value) override;
  void OnGPSPowerChanged(const uint8_t& new_value) override;
  void OnESCPowerChanged(const uint8_t& new_value) override;
  void OnRainDetectedChanged(const uint8_t& new_value) override;
  void OnChargeVoltageChanged(const float& new_value) override;
  void OnBatteryVoltageChanged(const float& new_value) override;
  void OnMowerRunningChanged(const uint8_t& new_value) override;
  void OnMowerESCTemperatureChanged(const float& new_value) override;
  void OnMowerMotorTemperatureChanged(const float& new_value) override;
  void OnMowerMotorCurrentChanged(const float& new_value) override;
  void OnMowerMotorRPMChanged(const float& new_value) override;

 private:
  void OnServiceConnected(const std::string& uid) override;
  void OnTransactionStart(uint64_t timestamp) override;
  void OnTransactionEnd() override;
  void OnServiceDisconnected(const std::string& uid) override;
  mower_msgs::Status status_msg_{};
  const ros::Publisher& status_poblisher_;
};

#endif  // MOWERSERVICEINTERFACE_H
