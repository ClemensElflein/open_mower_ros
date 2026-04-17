// Blade Speed Adapter — dynamically adjusts mower travel speed based on blade load.
//
// Subscribes to blade motor current from /ll/mower_status and mowing state from
// mower_logic/current_state. Computes an adaptive speed_fast for the FTC local
// planner: slows down instantly when the blade bogs (current rises), recovers
// gradually when current drops.
//
// All tunables (including live enable/disable) are exposed via dynamic_reconfigure
// (BladeSpeedAdapterConfig). Flip ~enable false -> true to take over the planner
// at any time; flip true -> false to restore the planner's original speeds.

#include <dynamic_reconfigure/client.h>
#include <dynamic_reconfigure/server.h>
#include <ftc_local_planner/FTCPlannerConfig.h>
#include <mower_logic/BladeSpeedAdapterConfig.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/HighLevelStatus.h>
#include <mower_msgs/Status.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <deque>
#include <mutex>
#include <sstream>

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------

static std::mutex g_mutex;

// Latest sensor data
static mower_msgs::Status g_status;
static bool g_has_status = false;

// Latest high-level state
static mower_msgs::HighLevelStatus g_high_level;
static bool g_has_high_level = false;

// FTC planner dynamic_reconfigure client (downstream — we drive its speed_fast)
static dynamic_reconfigure::Client<ftc_local_planner::FTCPlannerConfig>* g_ftc_client = nullptr;
static ftc_local_planner::FTCPlannerConfig g_ftc_config;
static bool g_has_ftc_config = false;
static double g_original_speed_fast = -1.0;
static double g_original_speed_slow = -1.0;

// Our own dynamic_reconfigure config (upstream — operator drives us)
static mower_logic::BladeSpeedAdapterConfig g_cfg;
static bool g_has_cfg = false;

// Algorithm state
static std::deque<double> g_current_buffer;
static double g_actual_speed = 0.0;
static bool g_was_mowing = false;

// Startup-only param (timer rate cannot be reconfigured without recreating the timer)
static double p_sample_interval;

// Publisher
static ros::Publisher g_log_pub;

// ---------------------------------------------------------------------------
// Callbacks
// ---------------------------------------------------------------------------

static void statusCallback(const mower_msgs::Status::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(g_mutex);
  g_status = *msg;
  g_has_status = true;
}

static void highLevelCallback(const mower_msgs::HighLevelStatus::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(g_mutex);
  g_high_level = *msg;
  g_has_high_level = true;
}

static void ftcConfigCallback(const ftc_local_planner::FTCPlannerConfig& config) {
  std::lock_guard<std::mutex> lock(g_mutex);
  g_ftc_config = config;
  g_has_ftc_config = true;

  // Capture the original values on first callback (before we modify anything)
  if (g_original_speed_fast < 0.0) {
    g_original_speed_fast = config.speed_fast;
    g_original_speed_slow = config.speed_slow;
    ROS_INFO("blade_speed_adapter: captured original speed_fast=%.3f speed_slow=%.3f", g_original_speed_fast,
             g_original_speed_slow);
  }
}

// ---------------------------------------------------------------------------
// Restore original planner speeds
// ---------------------------------------------------------------------------

static void restoreOriginalSpeed() {
  // No-op if we never observed planner config (so never overrode anything).
  if (g_original_speed_fast < 0.0) return;

  ftc_local_planner::FTCPlannerConfig cfg;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    cfg = g_ftc_config;
  }
  cfg.speed_fast = g_original_speed_fast;
  cfg.speed_slow = g_original_speed_slow;
  g_ftc_client->setConfiguration(cfg);
  ROS_INFO("blade_speed_adapter: restored speed_fast=%.3f speed_slow=%.3f", g_original_speed_fast,
           g_original_speed_slow);
}

// Our dynamic_reconfigure server callback. Fires synchronously on setCallback
// with the initial params loaded from the parameter server.
static void reconfigureCB(mower_logic::BladeSpeedAdapterConfig& c, uint32_t /*level*/) {
  bool was_enabled;
  bool was_mowing;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    was_enabled = g_has_cfg && g_cfg.enable;
    was_mowing = g_was_mowing;
    g_cfg = c;
    g_has_cfg = true;
  }

  ROS_INFO(
      "blade_speed_adapter: config  enable=%s  current=[%.2f..%.2f]A  speed=[%.3f..%.3f]m/s  "
      "recovery_rate=%.3f  window=%d",
      c.enable ? "TRUE" : "FALSE", c.current_nominal, c.current_max_load, c.speed_min, c.speed_max, c.recovery_rate,
      c.current_window);

  // enable true -> false while actively controlling: restore planner immediately
  // so operators don't have to wait for the next "mowing stopped" event.
  if (was_enabled && !c.enable && was_mowing) {
    ROS_INFO("blade_speed_adapter: disabled while mowing — restoring original planner speeds");
    restoreOriginalSpeed();
  }
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------

static void controlLoop(const ros::TimerEvent&) {
  // Snapshot current state under lock
  mower_msgs::Status status;
  mower_msgs::HighLevelStatus high_level;
  mower_logic::BladeSpeedAdapterConfig cfg;
  bool has_data;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    status = g_status;
    high_level = g_high_level;
    cfg = g_cfg;
    has_data = g_has_status && g_has_high_level && g_has_ftc_config && g_has_cfg;
  }

  if (!has_data) {
    ROS_INFO_THROTTLE(10, "blade_speed_adapter: waiting for data (status=%d hl=%d ftc=%d cfg=%d)", g_has_status,
                      g_has_high_level, g_has_ftc_config, g_has_cfg);
    return;
  }

  bool blade_running = (status.mower_esc_status == mower_msgs::ESCStatus::ESC_STATUS_RUNNING);
  bool is_mowing = (high_level.state_name == "MOWING") && blade_running;

  // Handle mowing -> not-mowing transition
  if (!is_mowing) {
    if (g_was_mowing) {
      ROS_INFO("blade_speed_adapter: mowing stopped, restoring original speeds");
      if (cfg.enable) {
        restoreOriginalSpeed();
      }
      g_current_buffer.clear();
      g_actual_speed = cfg.speed_max;
      g_was_mowing = false;
    }
    return;
  }

  g_was_mowing = true;

  // Push blade current into moving average buffer
  double blade_current = static_cast<double>(status.mower_esc_current);
  g_current_buffer.push_back(blade_current);
  while (static_cast<int>(g_current_buffer.size()) > cfg.current_window) {
    g_current_buffer.pop_front();
  }

  // Compute smoothed current
  double current_sum = 0.0;
  for (double c : g_current_buffer) {
    current_sum += c;
  }
  double current_avg = current_sum / static_cast<double>(g_current_buffer.size());

  // Load ratio: 0.0 = no load (nominal current), 1.0 = max load.
  double load_ratio = 0.0;
  if (current_avg > cfg.current_nominal && cfg.current_max_load > cfg.current_nominal) {
    load_ratio = (current_avg - cfg.current_nominal) / (cfg.current_max_load - cfg.current_nominal);
    load_ratio = std::max(0.0, std::min(1.0, load_ratio));
  }
  // Invert: high load = low speed
  double target_speed = cfg.speed_max - load_ratio * (cfg.speed_max - cfg.speed_min);

  // Asymmetric ramp: instant slowdown, gradual recovery
  if (target_speed < g_actual_speed) {
    g_actual_speed = target_speed;
  } else {
    g_actual_speed += std::min(cfg.recovery_rate * p_sample_interval, target_speed - g_actual_speed);
  }

  // Publish diagnostic log (always, regardless of enable)
  {
    std_msgs::String log_msg;
    std::ostringstream ss;
    ss << "current=" << blade_current << ";current_avg=" << current_avg << ";load_ratio=" << load_ratio
       << ";target_speed=" << target_speed << ";actual_speed=" << g_actual_speed
       << ";mode=" << (cfg.enable ? "live" : "monitor") << ";state=" << high_level.state_name;
    log_msg.data = ss.str();
    g_log_pub.publish(log_msg);
  }

  ROS_INFO_THROTTLE(5,
                    "blade_speed_adapter [%s]: current=%.2fA avg=%.2fA load=%.2f "
                    "target=%.3f actual=%.3f m/s",
                    cfg.enable ? "live" : "monitor", blade_current, current_avg, load_ratio, target_speed,
                    g_actual_speed);

  // Push speed to FTC planner in live mode
  if (cfg.enable) {
    ftc_local_planner::FTCPlannerConfig ftc;
    {
      std::lock_guard<std::mutex> lock(g_mutex);
      ftc = g_ftc_config;
    }
    ftc.speed_fast = g_actual_speed;
    ftc.speed_slow = std::min(g_actual_speed, g_original_speed_slow);
    g_ftc_client->setConfiguration(ftc);
  }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
  ros::init(argc, argv, "blade_speed_adapter");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  // Startup-only (controls the ros::Timer rate; cannot be reconfigured live)
  pn.param("sample_interval", p_sample_interval, 0.5);
  ROS_INFO("blade_speed_adapter: starting  sample_interval=%.2fs", p_sample_interval);

  // Subscribers
  ros::Subscriber status_sub = n.subscribe("/ll/mower_status", 10, statusCallback);
  ros::Subscriber hl_sub = n.subscribe("mower_logic/current_state", 10, highLevelCallback);

  // Diagnostic publisher
  g_log_pub = pn.advertise<std_msgs::String>("log", 50);

  // FTC planner dynamic_reconfigure client (we drive its speed_fast)
  g_ftc_client = new dynamic_reconfigure::Client<ftc_local_planner::FTCPlannerConfig>("/move_base_flex/FTCPlanner",
                                                                                      ftcConfigCallback);

  // Our own dynamic_reconfigure server (operator controls enable + tuning).
  // setCallback() fires synchronously with the initial loaded config.
  dynamic_reconfigure::Server<mower_logic::BladeSpeedAdapterConfig> reconfig_server(pn);
  reconfig_server.setCallback(reconfigureCB);

  // Seed g_actual_speed now that we have the initial config
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_actual_speed = g_has_cfg ? g_cfg.speed_max : 0.4;
  }

  // Control loop timer
  ros::Timer timer = n.createTimer(ros::Duration(p_sample_interval), controlLoop);

  ros::spin();

  // Cleanup: restore original speed if we were actively controlling
  bool should_restore;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    should_restore = g_was_mowing && g_has_cfg && g_cfg.enable;
  }
  if (should_restore) {
    restoreOriginalSpeed();
  }
  delete g_ftc_client;

  return 0;
}
