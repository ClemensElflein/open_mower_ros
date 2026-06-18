#include <mower_analytics/sentry_guard.h>
#include <ros/ros.h>
#include <sentry.h>

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <shared_mutex>
#include <string>

namespace mower_analytics {

static std::atomic<bool> s_initialized{false};
static std::shared_mutex s_sentry_mutex;

static std::string getenv_or(const char* name, const std::string& fallback = "") {
  const char* v = std::getenv(name);
  return v ? std::string(v) : fallback;
}

static std::string toLower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  return s;
}

static bool isTelemetryEnabled(const std::string& val) {
  const std::string v = toLower(val);
  return v == "true" || v == "1" || v == "yes" || v == "on";
}

static bool isTelemetryDisabled(const std::string& val) {
  const std::string v = toLower(val);
  return v == "false" || v == "0" || v == "no" || v == "off";
}

static std::string findHandlerPath() {
  std::error_code ec;
  std::filesystem::path exe = std::filesystem::read_symlink("/proc/self/exe", ec);
  if (ec) return "";
  std::filesystem::path exe_dir = exe.parent_path();
  auto candidate = exe_dir / "crashpad_handler";
  if (std::filesystem::exists(candidate, ec) && !ec) return candidate.string();
  return (exe_dir.parent_path() / "mower_analytics" / "crashpad_handler").string();
}

static sentry_level_t toSentryLevel(Level level) {
  switch (level) {
    case Level::Warning: return SENTRY_LEVEL_WARNING;
    case Level::Error: return SENTRY_LEVEL_ERROR;
    default: return SENTRY_LEVEL_INFO;
  }
}

SentryGuard::SentryGuard(const std::string& node_name) {
  const std::string telemetry = getenv_or("OM_TELEMETRY_ENABLE");

  if (telemetry.empty() || (!isTelemetryEnabled(telemetry) && !isTelemetryDisabled(telemetry))) {
    ROS_WARN_STREAM(
        "[analytics] ============================================================\n"
        "[analytics] Anonymous telemetry is not yet enabled.\n"
        "[analytics]\n"
        "[analytics] By enabling it you help the OpenMower project:\n"
        "[analytics]   - Crashes and exceptions are reported automatically,\n"
        "[analytics]     so bugs get fixed faster.\n"
        "[analytics]   - We can see which firmware versions are running reliably\n"
        "[analytics]     in the field and use that to cut stable releases more often.\n"
        "[analytics]\n"
        "[analytics] NO location or GPS data is ever collected.\n"
        "[analytics] Only anonymous data: software version, hardware platform,\n"
        "[analytics] mow start/stop, docking events, and crash reports.\n"
        "[analytics]\n"
        "[analytics] Opt in:  export OM_TELEMETRY_ENABLE=true   (in mower_config.sh)\n"
        "[analytics] Opt out: export OM_TELEMETRY_ENABLE=false  (silences this message)\n"
        "[analytics] ============================================================");
    return;
  }

  if (!isTelemetryEnabled(telemetry)) return;

  const std::string dsn = getenv_or("OM_SENTRY_DSN");
  if (dsn.empty()) return;

  const std::string release = getenv_or("OM_SOFTWARE_VERSION", "unknown");
  const std::string hw_platform = getenv_or("HARDWARE_PLATFORM", "unknown");

  sentry_options_t* options = sentry_options_new();
  sentry_options_set_dsn(options, dsn.c_str());
  sentry_options_set_release(options, release.c_str());
  const std::string handler = findHandlerPath();
  if (handler.empty()) {
    ROS_WARN_STREAM("[analytics] crashpad_handler: could not resolve exe path — crash reports disabled");
  } else {
    std::error_code ec;
    bool found = std::filesystem::exists(handler, ec) && !ec;
    ROS_INFO_STREAM("[analytics] crashpad_handler: " << handler
                                                     << (found ? "" : " (NOT FOUND — crash reports disabled)"));
  }
  sentry_options_set_handler_path(options, handler.c_str());
  const std::string ros_home = getenv_or("ROS_HOME", getenv_or("HOME", "/root") + "/.ros");
  sentry_options_set_database_path(options, (ros_home + "/sentry").c_str());
  sentry_options_set_auto_session_tracking(options, 1);
  if (sentry_init(options) != 0) {
    ROS_ERROR_STREAM("[analytics] sentry_init failed — telemetry disabled for " << node_name);
    return;
  }

  sentry_set_tag("hardware_platform", hw_platform.c_str());
  sentry_set_tag("node", node_name.c_str());

  enabled_ = true;
  s_initialized = true;
  ROS_INFO_STREAM("[analytics] Sentry active for " << node_name << " (release=" << release << ")");
  captureEvent(Level::Info, "node.started", {{"node", node_name}, {"release", release}});
}

SentryGuard::~SentryGuard() {
  if (enabled_) {
    std::unique_lock lock(s_sentry_mutex);
    s_initialized = false;
    sentry_close();
  }
}

void SentryGuard::captureEvent(Level level, const std::string& message,
                               const std::map<std::string, std::string>& extra) {
  std::shared_lock lock(s_sentry_mutex);
  if (!s_initialized) return;
  sentry_value_t event = sentry_value_new_message_event(toSentryLevel(level), "openmower", message.c_str());

  if (!extra.empty()) {
    sentry_value_t extra_obj = sentry_value_new_object();
    for (const auto& [k, v] : extra) {
      sentry_value_set_by_key(extra_obj, k.c_str(), sentry_value_new_string(v.c_str()));
    }
    sentry_value_set_by_key(event, "extra", extra_obj);
  }

  sentry_capture_event(event);
}

void SentryGuard::addBreadcrumb(const std::string& message, const std::string& category) {
  std::shared_lock lock(s_sentry_mutex);
  if (!s_initialized) return;
  sentry_value_t crumb = sentry_value_new_breadcrumb("default", message.c_str());
  sentry_value_set_by_key(crumb, "category", sentry_value_new_string(category.c_str()));
  sentry_add_breadcrumb(crumb);
}

}  // namespace mower_analytics
