// Blade Speed Adapter — dynamically adjusts mower travel speed based on blade load.
//
// Subscribes to blade motor current from /ll/mower_status and mowing state from
// mower_logic/current_state. Computes an adaptive speed_fast for the FTC local
// planner: slows down instantly when the blade bogs (current rises), recovers
// gradually when current drops.
//
// In dry-run mode (default), only logs computed values without touching the planner.
// In live mode (~dry_run:=false), pushes speed changes via dynamic_reconfigure.

#include <ros/ros.h>
#include <dynamic_reconfigure/client.h>
#include <ftc_local_planner/FTCPlannerConfig.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/HighLevelStatus.h>
#include <mower_msgs/Status.h>
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

// FTC planner config
static dynamic_reconfigure::Client<ftc_local_planner::FTCPlannerConfig>* g_ftc_client = nullptr;
static ftc_local_planner::FTCPlannerConfig g_ftc_config;
static bool g_has_ftc_config = false;
static double g_original_speed_fast = -1.0;
static double g_original_speed_slow = -1.0;

// Algorithm state
static std::deque<double> g_current_buffer;
static double g_actual_speed = 0.0;
static bool g_was_mowing = false;

// ROS parameters
static bool p_dry_run;
static double p_current_nominal;   // idle blade current (no load)
static double p_current_max_load;  // current at which blade is heavily loaded
static double p_speed_max;
static double p_speed_min;
static double p_recovery_rate;
static double p_sample_interval;
static int p_current_window;

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
    ROS_INFO("blade_speed_adapter: captured original speed_fast=%.3f speed_slow=%.3f",
             g_original_speed_fast, g_original_speed_slow);
  }
}

// ---------------------------------------------------------------------------
// Restore original planner speeds
// ---------------------------------------------------------------------------

static void restoreOriginalSpeed() {
  if (p_dry_run || g_original_speed_fast < 0.0) return;

  ftc_local_planner::FTCPlannerConfig cfg;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    cfg = g_ftc_config;
  }
  cfg.speed_fast = g_original_speed_fast;
  cfg.speed_slow = g_original_speed_slow;
  g_ftc_client->setConfiguration(cfg);
  ROS_INFO("blade_speed_adapter: restored speed_fast=%.3f speed_slow=%.3f",
           g_original_speed_fast, g_original_speed_slow);
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------

static void controlLoop(const ros::TimerEvent&) {
  // Snapshot current state under lock
  mower_msgs::Status status;
  mower_msgs::HighLevelStatus high_level;
  bool has_data;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    status = g_status;
    high_level = g_high_level;
    has_data = g_has_status && g_has_high_level && g_has_ftc_config;
  }

  if (!has_data) {
    ROS_INFO_THROTTLE(10, "blade_speed_adapter: waiting for data (status=%d hl=%d ftc=%d)",
                      g_has_status, g_has_high_level, g_has_ftc_config);
    return;
  }

  bool blade_running = (status.mow_esc_status.status == mower_msgs::ESCStatus::ESC_STATUS_RUNNING);
  bool is_mowing = (high_level.state_name == "MOWING") && blade_running;

  // Handle mowing -> not-mowing transition
  if (!is_mowing) {
    if (g_was_mowing) {
      ROS_INFO("blade_speed_adapter: mowing stopped, restoring original speeds");
      restoreOriginalSpeed();
      g_current_buffer.clear();
      g_actual_speed = p_speed_max;
      g_was_mowing = false;
    }
    return;
  }

  g_was_mowing = true;

  // Push blade current into moving average buffer
  double blade_current = static_cast<double>(status.mow_esc_status.current);
  g_current_buffer.push_back(blade_current);
  while (static_cast<int>(g_current_buffer.size()) > p_current_window) {
    g_current_buffer.pop_front();
  }

  // Compute smoothed current
  double current_sum = 0.0;
  for (double c : g_current_buffer) {
    current_sum += c;
  }
  double current_avg = current_sum / static_cast<double>(g_current_buffer.size());

  // Compute load ratio: 0.0 = no load (nominal current), 1.0 = max load
  // Higher current = more load = lower speed
  double load_ratio = 0.0;
  if (current_avg > p_current_nominal) {
    load_ratio = (current_avg - p_current_nominal) / (p_current_max_load - p_current_nominal);
    load_ratio = std::max(0.0, std::min(1.0, load_ratio));
  }
  // Invert: high load = low speed
  double target_speed = p_speed_max - load_ratio * (p_speed_max - p_speed_min);

  // Asymmetric ramp: instant slowdown, gradual recovery
  if (target_speed < g_actual_speed) {
    g_actual_speed = target_speed;
  } else {
    g_actual_speed += std::min(p_recovery_rate * p_sample_interval,
                               target_speed - g_actual_speed);
  }

  // Publish diagnostic log (always, regardless of dry_run)
  {
    std_msgs::String log_msg;
    std::ostringstream ss;
    ss << "current=" << blade_current
       << ";current_avg=" << current_avg
       << ";load_ratio=" << load_ratio
       << ";target_speed=" << target_speed
       << ";actual_speed=" << g_actual_speed
       << ";mode=" << (p_dry_run ? "dry_run" : "live")
       << ";state=" << high_level.state_name;
    log_msg.data = ss.str();
    g_log_pub.publish(log_msg);
  }

  ROS_INFO_THROTTLE(5, "blade_speed_adapter [%s]: current=%.2fA avg=%.2fA load=%.2f "
                       "target=%.3f actual=%.3f m/s",
                    p_dry_run ? "dry_run" : "live",
                    blade_current, current_avg, load_ratio,
                    target_speed, g_actual_speed);

  // Push speed to FTC planner in live mode
  if (!p_dry_run) {
    ftc_local_planner::FTCPlannerConfig cfg;
    {
      std::lock_guard<std::mutex> lock(g_mutex);
      cfg = g_ftc_config;
    }
    cfg.speed_fast = g_actual_speed;
    cfg.speed_slow = std::min(g_actual_speed, g_original_speed_slow);
    g_ftc_client->setConfiguration(cfg);
  }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
  ros::init(argc, argv, "blade_speed_adapter");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  // Load parameters
  pn.param("dry_run", p_dry_run, true);
  pn.param("current_nominal", p_current_nominal, 1.0);     // amps at no-load spinning
  pn.param("current_max_load", p_current_max_load, 5.0);   // amps at heavy grass load
  pn.param("speed_max", p_speed_max, 0.4);
  pn.param("speed_min", p_speed_min, 0.05);
  pn.param("recovery_rate", p_recovery_rate, 0.02);
  pn.param("sample_interval", p_sample_interval, 0.5);
  pn.param("current_window", p_current_window, 5);

  g_actual_speed = p_speed_max;

  ROS_INFO("blade_speed_adapter: starting in %s mode", p_dry_run ? "DRY-RUN" : "LIVE");
  ROS_INFO("  current_nominal=%.1fA  current_max_load=%.1fA", p_current_nominal, p_current_max_load);
  ROS_INFO("  speed=[%.3f, %.3f] m/s  recovery_rate=%.3f m/s/s", p_speed_min, p_speed_max, p_recovery_rate);
  ROS_INFO("  sample_interval=%.2fs  current_window=%d", p_sample_interval, p_current_window);

  // Subscribers
  ros::Subscriber status_sub = n.subscribe("/ll/mower_status", 10, statusCallback);
  ros::Subscriber hl_sub = n.subscribe("mower_logic/current_state", 10, highLevelCallback);

  // Diagnostic publisher
  g_log_pub = pn.advertise<std_msgs::String>("log", 50);

  // FTC planner dynamic_reconfigure client
  g_ftc_client = new dynamic_reconfigure::Client<ftc_local_planner::FTCPlannerConfig>(
      "/move_base_flex/FTCPlanner", ftcConfigCallback);

  // Control loop timer
  ros::Timer timer = n.createTimer(ros::Duration(p_sample_interval), controlLoop);

  ros::spin();

  // Cleanup: restore original speed if we were actively controlling
  if (g_was_mowing) {
    restoreOriginalSpeed();
  }
  delete g_ftc_client;

  return 0;
}
