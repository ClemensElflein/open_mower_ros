//
// Created by clemens on 29.11.24.
//

#ifndef SIMROBOT_H
#define SIMROBOT_H

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <xbot_positioning/GPSControlSrv.h>
#include <xbot_positioning/SetPoseSrv.h>

#include <mutex>
#include <random>
#include <thread>

#include "EmergencyServiceBase.hpp"

class SimRobot {
 public:
  explicit SimRobot(ros::NodeHandle& nh);
  void Start();

  void GetPosition(double& x, double& y);
  void GetTwist(double& vx, double& vr);

  // Emergency latch mirroring the real firmware: `reasons_` is a bitfield of
  // EmergencyReason flags; the robot is in emergency whenever any bit is set. The high
  // level drives it with an (add, clear) pair - it can only clear the bits it explicitly
  // names, so a latched reason (e.g. LATCH) persists until the high level clears it.
  void ApplyEmergencyUpdate(uint16_t add, uint16_t clear);
  // Latches an emergency from the sim-control RPC (LATCH stays until the high level clears
  // it, exactly like a physical stop button).
  void TriggerEmergency();
  // Test hook: clears every emergency reason directly.
  void ClearEmergency();

  void GetEmergencyState(bool& active, bool& latch, uint16_t& reason);
  void SetControlTwist(double linear, double angular);

  // Instantly teleports the robot by (dx, dy, dheading) to simulate a GPS jump.
  void Displace(double dx, double dy, double dheading);
  // Snaps the robot onto the docking pose and starts charging.
  void MoveToDock();
  void GetPosition(double& x, double& y, double& heading);
  void SetPosition(const double x, const double y, const double heading);

  void SetDockingPose(const double x, const double y, const double heading);

  void GetIsCharging(bool& charging, double& seconds_since_start, std::string& charging_status, double& charger_volts,
                     double& battery_volts, double& charging_current);

  // Simulation control hooks (driven by the ROS-layer RPCs in SimRpc).
  // When movement is disallowed the robot simulates being "stuck": the commanded twist
  // is still reported on the wheel odometry / gyro (wheels keep turning) but the
  // ground-truth position no longer integrates, so GPS reports no movement.
  void SetMovementAllowed(bool allowed);
  // When joy override is enabled, /joy_vel commands drive the robot directly and all
  // SetControlTwist() calls from the normal mower stack are ignored.
  void SetJoyOverride(bool enabled);
  // Good GPS = RTK fix, ~2 cm accuracy. Bad GPS = no RTK fix, ~1 m accuracy.
  void SetGpsGood(bool good);
  bool IsGpsGood();
  // Snap the battery to full or empty voltage.
  void SetBatteryFull(bool full);
  // Set the battery to an exact pack voltage. Values outside the normal
  // [BATTERY_VOLTS_MIN, BATTERY_VOLTS_MAX] range are allowed so the app can
  // simulate critically low / critically high (over-voltage) faults.
  void SetBatteryVolts(double volts);

  // Snapshot of the simulation control state, streamed to the app via MQTT.
  struct SimControlState {
    bool emergency_active;
    bool emergency_latch;
    uint16_t emergency_reason;
    bool movement_allowed;
    bool gps_good;
    double battery_voltage;
    double battery_percentage;
    bool charging;
    bool joy_override;
  };
  SimControlState GetSimControlState();

 private:
  // 7 cells
  static constexpr double BATTERY_VOLTS_MIN = 3.2 * 7;
  static constexpr double BATTERY_VOLTS_MAX = 4.18 * 7;
  static constexpr double CHARGE_CURRENT = 2.5;
  static constexpr double CHARGE_VOLTS = 32.0;

  // Lock for all getters and setters and the simulation step
  std::mutex state_mutex_;

  // Position of the simulated dock, used to simulate "docked" and charging voltages
  double docking_pos_x_ = 0;
  double docking_pos_y_ = 0;
  double docking_pos_heading_ = 0;

  bool started_ = false;
  // Speed X in m/s
  double vx_ = 0;
  // Speed rotating in rad/s
  double vr_ = 0;
  // position and heading. This can be used to simulate GPS
  double pos_x_ = 0;
  double pos_y_ = 0;
  double pos_heading_ = 0;

  // When false, the ground-truth position is frozen while the reported wheel odometry
  // still follows the commanded twist -> simulates a stuck robot.
  bool movement_allowed_ = true;

  // When true, /joy_vel drives the robot and SetControlTwist() calls are ignored.
  bool joy_override_ = false;

  void OnJoyVel(const geometry_msgs::Twist::ConstPtr& msg);

  // Last noisy twist values (for encoder/IMU readback)
  double last_noisy_vx_ = 0;
  double last_noisy_vr_ = 0;

  // Emergency reason bitfield (see ApplyEmergencyUpdate). Starts blocked on
  // TIMEOUT_HIGH_LEVEL so the robot won't move until the high level has connected, just
  // like the real hardware; the EmergencyService clears it once heartbeats arrive.
  uint16_t emergency_reasons_ = EmergencyReason::TIMEOUT_HIGH_LEVEL;

  ros::Time last_update_{0};
  ros::NodeHandle nh_;

  // Whether to broadcast the map->base_link transform ourselves. Standalone simulation
  // launch files (no real xbot_positioning running) need this; when a real
  // xbot_positioning runs alongside (broadcasting its own EKF-filtered estimate on the
  // exact same transform), it must be disabled here or the two race each other.
  bool publish_tf_ = true;

  bool is_charging_ = false;
  ros::Time charging_started_time;
  double charger_volts_ = 0;
  double battery_volts_ = BATTERY_VOLTS_MAX;
  double charge_current_ = 0;
  std::string charger_state_{"Unknown"};

  // Timer for simulation
  ros::Timer timer_;
  void SimulationStep(const ros::TimerEvent& te);
  void PublishPosition();

  /*
   * Generate some noise
   */
  std::default_random_engine generator{};
  std::normal_distribution<double> position_noise{0.0, 0.005};
  std::normal_distribution<double> heading_noise{0.0, 0.01};
  std::normal_distribution<double> linear_speed_noise{0.0, 0.02};
  std::normal_distribution<double> angular_speed_noise{0.0, 0.02};

  ros::Publisher odometry_pub_{};
  ros::Publisher xbot_absolute_pose_pub_{};
  ros::Subscriber joy_vel_sub_{};
  // GPS quality: true = RTK fix (~2 cm), false = no RTK fix (~1 m).
  bool gps_good_ = true;
};

#endif  // SIMROBOT_H
