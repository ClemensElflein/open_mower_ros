//
// Created by clemens on 26.07.24.
//

#include "emergency_service.hpp"

// How long we tolerate silence from the high level before latching TIMEOUT_HIGH_LEVEL.
// Matches the firmware's 1 s window.
static constexpr double HIGH_LEVEL_TIMEOUT_S = 1.0;

void EmergencyService::OnStop() {
  // We won't get further updates from the high level, so raise the timeout reason.
  robot_.ApplyEmergencyUpdate(EmergencyReason::TIMEOUT_HIGH_LEVEL, 0);
}

void EmergencyService::OnHighLevelEmergencyChanged(const uint16_t* new_value, uint32_t length) {
  last_high_level_emergency_message_ = ros::Time::now();
  if (length >= 2) {
    // The high level sends an (add, clear) pair. It can only clear the reasons it names,
    // so a latched reason it doesn't clear (e.g. LATCH) stays asserted - this is what
    // makes an emergency triggered from the sim RPC persist until it is explicitly reset.
    robot_.ApplyEmergencyUpdate(new_value[0], new_value[1]);
  } else if (length == 1) {
    // Legacy single-value form: just add.
    robot_.ApplyEmergencyUpdate(new_value[0], 0);
  }
}

void EmergencyService::tick() {
  // Raise or clear TIMEOUT_HIGH_LEVEL based on how recently we heard from the high level.
  const bool timed_out = last_high_level_emergency_message_.isZero() ||
                         (ros::Time::now() - last_high_level_emergency_message_).toSec() > HIGH_LEVEL_TIMEOUT_S;
  if (timed_out) {
    robot_.ApplyEmergencyUpdate(EmergencyReason::TIMEOUT_HIGH_LEVEL, 0);
  } else {
    robot_.ApplyEmergencyUpdate(0, EmergencyReason::TIMEOUT_HIGH_LEVEL);
  }

  bool emergency_active, emergency_latch;
  uint16_t emergency_reason;
  robot_.GetEmergencyState(emergency_active, emergency_latch, emergency_reason);

  StartTransaction();
  SendEmergencyReason(emergency_reason);
  CommitTransaction();
}
