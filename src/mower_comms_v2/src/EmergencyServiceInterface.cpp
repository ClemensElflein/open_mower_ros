#include "EmergencyServiceInterface.h"

#include <mower_msgs/Emergency.h>

bool EmergencyServiceInterface::SetHighLevelEmergency(uint16_t reason) {
  // reason == 0 means clear the emergency. Otherwise latch the given reason(s).
  if (reason != 0) {
    reason |= EmergencyReason::HIGH_LEVEL | EmergencyReason::LATCH;
  }
  high_level_emergency_reason_ = reason;

  if (reason != 0) {
    SendHighLevelEmergencyHelper(reason);

    // Update cached state for immediate feedback.
    latest_emergency_reason_ |= high_level_emergency_reason_;
    PublishEmergencyState();
  } else {
    // Clear all previously set high-level emergency bits.
    SendHighLevelEmergencyHelper(0, high_level_emergency_reason_);
    high_level_emergency_reason_ = 0;
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
  CHECK_REASON(COLLISION_MULTIPLE)
  CHECK_REASON(TIMEOUT_HIGH_LEVEL)
  CHECK_REASON(HIGH_LEVEL)
  CHECK_REASON(SERVICE_NOT_READY)
  CHECK_REASON(MOWER_RPM_TIMEOUT)
  CHECK_REASON(MOWER_RPM_LIMIT)
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

  // Also check, if there was a change and log
  {
    static uint16_t old_reasons = 0;
    if (old_reasons != reason) {
      if (reason == 0) {
        ROS_INFO_STREAM("Emergency cleared");
      } else {
        ROS_WARN_STREAM("Emergency reason changed from: '" << ReasonToString(old_reasons) << "' to: '"
                                                           << emergency.reason << "'");
      }
      old_reasons = reason;
    }
  }
}
