//
// Created by clemens on 25.07.24.
//

#ifndef EMERGENCYSERVICEINTERFACE_H
#define EMERGENCYSERVICEINTERFACE_H

#include <ros/node_handle.h>

#include <EmergencyServiceInterfaceBase.hpp>

namespace sc = std::chrono;

class EmergencyServiceInterface : public EmergencyServiceInterfaceBase {
 public:
  EmergencyServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx, const ros::Publisher& publisher)
      : EmergencyServiceInterfaceBase(service_id, ctx), publisher(publisher) {
  }

  bool SetEmergency(bool new_value);
  void Heartbeat();

 protected:
  void OnEmergencyActiveChanged(const uint8_t& new_value) override;
  void OnEmergencyLatchChanged(const uint8_t& new_value) override;
  void OnEmergencyReasonChanged(const char* new_value, uint32_t length) override;

 private:
  void OnServiceConnected(uint16_t service_id) override;
  void OnTransactionEnd() override;
  void OnServiceDisconnected(uint16_t service_id) override;

  void PublishEmergencyState();

  std::recursive_mutex state_mutex_{};

  const ros::Publisher& publisher;

  // keep track of high level emergency
  bool latched_emergency_ = true;
  bool active_low_level_emergency_ = true;
  bool active_high_level_emergency_ = true;
  std::string latest_emergency_reason_ = "NONE";
};

#endif  // EMERGENCYSERVICEINTERFACE_H
