#include "SoundSync.h"

#include <sys/wait.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

namespace sound_sync {
namespace {

/// The CLI installed into the image (see docker/Dockerfile).
constexpr const char* kCommand = "soundctl";
/// The low level may still be bringing its services up (the SoundService starts only
/// after it got the definitions), so allow a few attempts with a pause in between.
constexpr int kAttempts = 3;
constexpr auto kRetryDelay = std::chrono::seconds(20);

struct Settings {
  bool enabled = true;
  std::string config_file;
  int volume = -1;  ///< < 0: keep the volume the firmware has stored
};

Settings settings;
std::atomic<bool> running{false};

/// POSIX single-quote escaping, so a path with spaces or quotes stays intact.
std::string ShellQuote(const std::string& value) {
  std::string quoted = "'";
  for (const char c : value) {
    if (c == '\'') {
      quoted += "'\\''";
    } else {
      quoted += c;
    }
  }
  quoted += "'";
  return quoted;
}

std::string BuildCommand() {
  const std::string config = ShellQuote(settings.config_file);
  // Render the `tts:` texts first; with up-to-date sidecars this is a no-op that only stats a few files.
  // The two steps are chained with ';' on purpose: a failed render must not skip the sync,
  // because then the sync's own stale/missing check reports exactly which file is broken,
  // and sounds that are already rendered still play.
  //
  // --wait 30s: an xbot service advertises itself every second only while it is unclaimed and
  // younger than 60 s; afterwards (and once claimed) only every 10 s (xbot/config.hpp). The sync
  // spends a few seconds probing the SoundService before it starts waiting for the FileService,
  // so a 10 s window could miss the LL forever - 30 s covers three full advertisement periods.
  std::string command = std::string(kCommand) + " tts --config " + config + "; " + std::string(kCommand) +
                        " sync --config " + config + " --wait 30s";
  if (settings.volume >= 0) {
    command += " --volume " + std::to_string(settings.volume);
  }
  return command;
}

/// One sync attempt. Every output line is forwarded to the ROS log.
bool RunOnce() {
  const std::string command = BuildCommand();
  ROS_INFO_STREAM("Sound sync: " << command);

  FILE* pipe = popen((command + " 2>&1").c_str(), "r");
  if (pipe == nullptr) {
    ROS_WARN_STREAM("Sound sync: cannot run '" << kCommand << "' (missing in the image?)");
    return false;
  }

  char buffer[512];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    std::string line(buffer);
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
      line.pop_back();
    }
    if (!line.empty()) {
      ROS_INFO("%s", line.c_str());  // as data, never as format string
    }
  }

  const int status = pclose(pipe);
  if (status == -1) {
    ROS_WARN("Sound sync: pclose() failed.");
    return false;
  }
  return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

void RunWithRetries() {
  for (int attempt = 1; attempt <= kAttempts; ++attempt) {
    if (RunOnce()) {
      ROS_INFO("Sound sync: done.");
      running.store(false);
      return;
    }
    if (attempt < kAttempts) {
      ROS_WARN_STREAM("Sound sync: attempt " << attempt << "/" << kAttempts << " failed, retrying in "
                                             << kRetryDelay.count() << " s");
      std::this_thread::sleep_for(kRetryDelay);
    }
  }
  ROS_WARN_STREAM("Sound sync: failed after " << kAttempts << " attempts (sounds may be missing/stale).");
  running.store(false);
}

}  // namespace

void Init(const ros::NodeHandle& param_nh) {
  param_nh.param("services/sound/enabled", settings.enabled, true);
  param_nh.param("services/sound/config_file", settings.config_file, std::string());
  param_nh.param("services/sound/volume", settings.volume, -1);

  if (!settings.enabled) {
    ROS_INFO("Sound sync: disabled (ll/services/sound/enabled=false).");
    return;
  }
  if (settings.config_file.empty()) {
    ROS_WARN("Sound sync: ll/services/sound/config_file is empty, disabled.");
    settings.enabled = false;
    return;
  }
  ROS_INFO_STREAM("Sound sync: will push " << settings.config_file << " on every low-level connect.");
}

void Trigger() {
  if (!settings.enabled || running.exchange(true)) {
    return;  // disabled, or a sync is already in flight
  }
  std::thread(RunWithRetries).detach();
}

}  // namespace sound_sync
