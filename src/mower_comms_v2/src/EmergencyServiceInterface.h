#ifndef EMERGENCYSERVICEINTERFACE_H
#define EMERGENCYSERVICEINTERFACE_H

#include <ros/node_handle.h>

#include <EmergencyServiceInterfaceBase.hpp>

class EmergencyServiceInterface : public EmergencyServiceInterfaceBase {
 public:
  EmergencyServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx, const ros::Publisher& publisher)
      : EmergencyServiceInterfaceBase(service_id, ctx), publisher_(publisher) {
  }

  bool SetHighLevelEmergency(bool new_value);
  void Heartbeat();

 protected:
  void OnEmergencyReasonChanged(const uint16_t& new_value) override;

 private:
  void OnServiceDisconnected(uint16_t service_id) override;

  void SendHighLevelEmergencyHelper(uint16_t add, uint16_t clear = 0);
  void PublishEmergencyState();

  const ros::Publisher& publisher_;

  // keep track of high level emergency
  std::atomic<uint16_t> high_level_emergency_reason_{0};
  std::atomic<uint16_t> latest_emergency_reason_{EmergencyReason::TIMEOUT_HIGH_LEVEL};
};

#endif  // EMERGENCYSERVICEINTERFACE_H
