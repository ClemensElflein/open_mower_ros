//
// Created by clemens on 09.09.24.
//

#ifndef DOCKINGSENSORSERVICEINTERFACE_H
#define DOCKINGSENSORSERVICEINTERFACE_H

#include <mower_msgs/DockingSensor.h>
#include <ros/publisher.h>

#include <DockingSensorServiceInterfaceBase.hpp>

class DockingSensorServiceInterface : public DockingSensorServiceInterfaceBase {
 public:
  DockingSensorServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                                const ros::Publisher& sensor_publisher)
      : DockingSensorServiceInterfaceBase(service_id, ctx), sensor_publisher_(sensor_publisher) {
  }
  bool OnConfigurationRequested(uint16_t service_id) override;

 protected:
  void OnDetectedSensorsLeftChanged(const uint8_t& new_value) override;
  void OnDetectedSensorsRightChanged(const uint8_t& new_value) override;

 private:
  void OnServiceConnected(uint16_t service_id) override;
  void OnTransactionStart(uint64_t timestamp) override;
  void OnTransactionEnd() override;
  const ros::Publisher& sensor_publisher_;
  mower_msgs::DockingSensor msg{};
};

#endif  // DOCKINGSENSORSERVICEINTERFACE_H
