//
// SoundSync - push the sound definitions to the low level on ROS start.
//
// The sounds themselves live in sounds_*.yaml; the `soundctl` CLI validates them,
// uploads the MP3s and sends the definitions blob (the SoundService does not play
// anything before it received that blob).  This is the glue that runs the CLI once the
// low-level firmware is up - without it a freshly booted robot stays silent.
//
#pragma once

#include <ros/ros.h>

namespace sound_sync {

/// Reads ll/services/sound/{enabled,config_file,volume}. Stays a no-op when disabled.
void Init(const ros::NodeHandle& param_nh);

/// Runs `soundctl sync` in the background. Call it on every low-level (re)connect:
/// a sync that is already in progress is never started twice.
void Trigger();

}  // namespace sound_sync
