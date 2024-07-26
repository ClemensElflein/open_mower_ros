//
// Created by clemens on 25.07.24.
//

#include "EmergencyServiceInterface.h"

#include <mower_msgs/Emergency.h>

bool EmergencyServiceInterface::SetEmergency(bool new_value) {
  std::unique_lock<std::recursive_mutex> lk{state_mutex_};
  // Set the high level emergency
  active_high_level_emergency_ = new_value;

  if (new_value) {
    latest_emergency_reason_ = "High Level Emergency";
  }

  // Send the new emergency state to the service.
  // the LL emergency will be updated in the callback
  SendSetEmergency(new_value);

  // Instantly send the new emergency
  PublishEmergencyState();
  return true;
}

bool EmergencyServiceInterface::OnConfigurationRequested(const std::string& uid) {
  // No config needed
  return true;
}
void EmergencyServiceInterface::OnEmergencyActiveChanged(const uint8_t& new_value) {
  std::unique_lock<std::recursive_mutex> lk{state_mutex_};
  active_low_level_emergency_ = new_value;
}

void EmergencyServiceInterface::OnEmergencyLatchChanged(const uint8_t& new_value) {
  std::unique_lock<std::recursive_mutex> lk{state_mutex_};
  if (!new_value && !active_high_level_emergency_) {
    // Only clear latch, if we don't have a high level emergency going on
    latched_emergency_ = false;
  }
}
void EmergencyServiceInterface::OnEmergencyReasonChanged(const char* new_value, uint32_t length) {
  latest_emergency_reason_ = std::string{new_value, length};
}
void EmergencyServiceInterface::OnTransactionStart(uint64_t timestamp) {
}
void EmergencyServiceInterface::OnTransactionEnd() {
  // Send the packet
  PublishEmergencyState();
}

void EmergencyServiceInterface::OnServiceConnected(const std::string& uid) {
  std::unique_lock<std::recursive_mutex> lk{state_mutex_};
  latched_emergency_ = active_high_level_emergency_ = active_low_level_emergency_ = true;
  latest_emergency_reason_ = "Service Starting Up";
  PublishEmergencyState();
}
void EmergencyServiceInterface::OnServiceDisconnected(const std::string& uid) {
  std::unique_lock<std::recursive_mutex> lk{state_mutex_};
  latched_emergency_ = active_high_level_emergency_ = active_low_level_emergency_ = true;
  latest_emergency_reason_ = "Service Disconnected";
  PublishEmergencyState();
}
void EmergencyServiceInterface::PublishEmergencyState() {
  mower_msgs::Emergency emergency_{};
  std::unique_lock<std::recursive_mutex> lk{state_mutex_};
  emergency_.stamp = ros::Time::now();
  // Make sure the latch is set, if there's an active emergency
  latched_emergency_ |= active_high_level_emergency_ | active_low_level_emergency_;

  emergency_.latched_emergency = latched_emergency_;
  emergency_.active_emergency = active_high_level_emergency_ | active_low_level_emergency_;
  emergency_.reason = latest_emergency_reason_;
  publisher.publish(emergency_);
}
