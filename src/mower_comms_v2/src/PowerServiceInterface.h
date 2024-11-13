//
// Created by clemens on 09.09.24.
//

#ifndef POWERSERVICEINTERFACE_H
#define POWERSERVICEINTERFACE_H

#include <mower_msgs/Power.h>
#include <ros/publisher.h>

#include <PowerServiceInterfaceBase.hpp>

class PowerServiceInterface : public PowerServiceInterfaceBase {
 public:
  PowerServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                        const ros::Publisher& status_publisher)
      : PowerServiceInterfaceBase(service_id, ctx), status_publisher_(status_publisher) {
  }
  bool OnConfigurationRequested(uint16_t service_id) override;

 protected:
  void OnChargeVoltageChanged(const float& new_value) override;
  void OnChargeCurrentChanged(const float& new_value) override;
  void OnBatteryVoltageChanged(const float& new_value) override;
  void OnChargingStatusChanged(const char* new_value, uint32_t length) override;
  void OnChargerEnabledChanged(const uint8_t& new_value) override;

 private:
  void OnTransactionStart(uint64_t timestamp) override;
  void OnTransactionEnd() override;
  mower_msgs::Power power_msg_{};
  const ros::Publisher& status_publisher_;
};

#endif  // POWERSERVICEINTERFACE_H
