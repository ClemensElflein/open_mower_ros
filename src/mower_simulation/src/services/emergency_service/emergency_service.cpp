//
// Created by clemens on 26.07.24.
//

#include "emergency_service.hpp"

bool EmergencyService::OnStart() {
  robot_.SetEmergency(true, EmergencyReason::LATCH);
  return true;
}

void EmergencyService::OnStop() {
  robot_.SetEmergency(true, EmergencyReason::STOP);
}

void EmergencyService::tick() {
  /*// Get the current emergency state
  chMtxLock(&mower_status_mutex);
  uint32_t status_copy = mower_status;
  chMtxUnlock(&mower_status_mutex);
  bool emergency_latch = (status_copy & MOWER_FLAG_EMERGENCY_LATCH) != 0;
  bool emergency_active = (status_copy & MOWER_FLAG_EMERGENCY_ACTIVE) != 0;
  // Check timeout, but only overwrite if no emergency is currently active
  // reasoning is that we want to keep the original reason and not overwrite
  // with "timeout"
  if (!emergency_latch &&
      chVTTimeElapsedSinceX(last_clear_emergency_message_) > TIME_S2I(1)) {
    emergency_reason = "Timeout";
    // set the emergency and notify services
    chMtxLock(&mower_status_mutex);
    mower_status |= MOWER_FLAG_EMERGENCY_LATCH;
    chMtxUnlock(&mower_status_mutex);
    chEvtBroadcastFlags(&mower_events, MOWER_EVT_EMERGENCY_CHANGED);
    // The last flags did not have emergency yet, so need to set it here as well
    emergency_latch = true;
  }*/

  bool emergency_active, emergency_latch;
  robot_.GetEmergencyState(emergency_active, emergency_latch, emergency_reason);

  StartTransaction();
  SendEmergencyReason(emergency_reason);
  CommitTransaction();
}
void EmergencyService::OnHighLevelEmergencyChanged(const uint16_t* new_value, uint32_t length) {
  if (length > 0 && new_value[0] > 0) {
    robot_.SetEmergency(true, EmergencyReason::HIGH_LEVEL);
  } else {
    robot_.ResetEmergency();
  }
}
