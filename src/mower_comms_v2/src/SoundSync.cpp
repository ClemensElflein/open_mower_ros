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

/// Log format for the piped runs: the ROS log already stamps every line
/// ("[INFO] [1793893786.077...]: ...") and names the node, so soundctl's own
/// date/time would be a duplicate. The prefix keeps a line attributable when the
/// output of the two commands is interleaved in that log.
///
/// --no-color: soundctl colours its output only on a terminal, and popen() gives it
/// a pipe, so this is belt and braces - but it means an escape sequence can never
/// reach the ROS log, not even if someone later runs the binary by hand.
constexpr const char* kLogFlags = "--log-time=false --no-color ";

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
  //
  // `timeout 300` bounds the whole chain: if a child ever hangs, the read loop would block
  // forever and RunWithRetries could never clear the running flag (no sync for the rest of the
  // uptime). A fresh install needs ~40 s on a CM4, so 300 s is generous.
  // --keep-going: try every variant instead of stopping at the first failure, so a
  // missing key or an API error leaves as much as possible rendered. The sync below
  // then skips whatever is still stale (the low level keeps its previous file).
  std::string command = "timeout 300 " + std::string(kCommand) + " tts " + kLogFlags +
                        "--log-prefix '[soundctl tts] ' --keep-going --config " + config + "; timeout 300 " +
                        std::string(kCommand) + " sync " + kLogFlags + "--log-prefix '[soundctl sync] ' --config " +
                        config + " --wait 30s";
  if (settings.volume >= 0) {
    command += " --volume " + std::to_string(settings.volume);
  }
  return command;
}

/// Forward one complete output line with the severity soundctl gave it.
///
/// stdout and stderr share one pipe (the ordering matters for reading the log), so the
/// severity has to come from the line itself. soundctl prefixes its slog lines with a header
/// ("[soundctl sync] INFO ...", "[soundctl tts] WARN ...") and cobra prints the fatal one as
/// "Error: ..." at the start of the line. Only that header is inspected: a file name or a
/// message that happens to contain "] ERROR " must not change the severity of the line.
static void LogSoundctlLine(std::string line) {
  while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
    line.pop_back();
  }
  if (line.empty()) {
    return;
  }

  bool is_error = line.rfind("Error: ", 0) == 0;
  bool is_warn = false;
  std::string text = line;
  const auto header_end = is_error ? std::string::npos : text.find("] ");
  if (header_end != std::string::npos) {
    const size_t level_at = header_end + 2;
    for (const char* level : {"ERROR", "WARN", "INFO", "DEBUG", "TRACE"}) {
      const std::string token = std::string(level) + " ";
      if (text.compare(level_at, token.size(), token) != 0) {
        continue;
      }
      is_error = std::string(level) == "ERROR";
      is_warn = std::string(level) == "WARN";
      // The ROS line already carries the severity, so the token is dropped from the text -
      // logging it as is read "INFO ... INFO ..." or "ERROR ... ERROR ...".
      text.erase(level_at, token.size());
      break;
    }
  }

  if (is_error) {
    ROS_ERROR("%s", text.c_str());  // as data, never as format string
  } else if (is_warn) {
    ROS_WARN("%s", text.c_str());
  } else {
    ROS_INFO("%s", text.c_str());
  }
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

  // fgets() cuts a line at sizeof(buffer), and only the complete line carries the severity:
  // a long error message would arrive in fragments and its tail would be logged as info.
  char buffer[512];
  std::string pending;
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    pending += buffer;
    if (pending.back() != '\n') {
      continue;  // the rest of this line is still in the pipe
    }
    LogSoundctlLine(std::move(pending));
    pending.clear();
  }
  LogSoundctlLine(std::move(pending));  // a last line without a newline

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
