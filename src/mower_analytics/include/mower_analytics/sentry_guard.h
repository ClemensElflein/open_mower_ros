#pragma once
#include <map>
#include <string>

namespace mower_analytics {

enum class Level { Info, Warning, Error };

// RAII wrapper for sentry init/close. Construct once in main() after ros::init().
// Reads OM_TELEMETRY_ENABLE, OM_SENTRY_DSN, OM_SOFTWARE_VERSION, HARDWARE_PLATFORM from env.
// Prints a ROS_WARN when OM_TELEMETRY_ENABLE is unset so users know they must decide.
// No-op when telemetry is disabled or DSN is unset — safe to use unconditionally.
class SentryGuard {
 public:
  explicit SentryGuard(const std::string& node_name);
  ~SentryGuard();

  SentryGuard(const SentryGuard&) = delete;
  SentryGuard& operator=(const SentryGuard&) = delete;

  bool enabled() const {
    return enabled_;
  }

  // Call from anywhere after SentryGuard is constructed in main().
  // No-ops if telemetry is disabled.
  static void captureEvent(Level level, const std::string& message,
                           const std::map<std::string, std::string>& extra = {});

  static void addBreadcrumb(const std::string& message, const std::string& category = "openmower");

 private:
  bool enabled_{false};
};

}  // namespace mower_analytics
