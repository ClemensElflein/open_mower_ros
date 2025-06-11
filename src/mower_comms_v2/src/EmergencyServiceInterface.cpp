#include "EmergencyServiceInterface.h"

#include <mower_msgs/Emergency.h>

bool EmergencyServiceInterface::SetHighLevelEmergency(bool new_value) {
  high_level_emergency_reason_ = new_value ? EmergencyReason::HIGH_LEVEL | EmergencyReason::LATCH : 0;

  // Send the new emergency state to the service.
  if (new_value) {
    SendHighLevelEmergencyHelper(high_level_emergency_reason_);

    // Update cached state for immediate feedback.
    latest_emergency_reason_ |= high_level_emergency_reason_;
    PublishEmergencyState();
  } else {
    // TODO: Add a separate service call to clear the latch.
    SendHighLevelEmergencyHelper(0, EmergencyReason::HIGH_LEVEL | EmergencyReason::LATCH);
  }

  return true;
}

void EmergencyServiceInterface::Heartbeat() {
  SendHighLevelEmergencyHelper(high_level_emergency_reason_, EmergencyReason::HIGH_LEVEL);
}

void EmergencyServiceInterface::SendHighLevelEmergencyHelper(uint16_t add, uint16_t clear) {
  uint16_t payload[2] = {add, clear};
  SendHighLevelEmergency(payload, 2);
}

void EmergencyServiceInterface::OnEmergencyReasonChanged(const uint16_t& new_value) {
  latest_emergency_reason_ = new_value;
  PublishEmergencyState();
}

void EmergencyServiceInterface::OnServiceDisconnected(uint16_t service_id) {
  latest_emergency_reason_ = EmergencyReason::TIMEOUT_HIGH_LEVEL;
  PublishEmergencyState();
}

static std::string ReasonToString(uint16_t reason) {
#define CHECK_REASON(r)              \
  if (reason & EmergencyReason::r) { \
    if (!first) str << ", ";         \
    str << #r;                       \
    first = false;                   \
  }

  std::ostringstream str;
  bool first = true;
  CHECK_REASON(LATCH)
  CHECK_REASON(TIMEOUT_INPUTS)
  CHECK_REASON(STOP)
  CHECK_REASON(LIFT)
  CHECK_REASON(LIFT_MULTIPLE)
  CHECK_REASON(COLLISION)
  CHECK_REASON(TIMEOUT_HIGH_LEVEL)
  CHECK_REASON(HIGH_LEVEL)
  return str.str();
}

void EmergencyServiceInterface::PublishEmergencyState() {
  mower_msgs::Emergency emergency{};
  const uint16_t reason = latest_emergency_reason_;
  emergency.stamp = ros::Time::now();
  emergency.latched_emergency = reason != 0;
  emergency.active_emergency = reason != 0;
  emergency.reason = ReasonToString(reason);
  publisher_.publish(emergency);
}
