//
// Created by clemens on 29.11.24.
//

#ifndef SIMROBOT_H
#define SIMROBOT_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <mutex>
#include <random>
#include <thread>

class SimRobot {
 public:
  explicit SimRobot(ros::NodeHandle &nh);
  void Start();

  void GetPosition(double &x, double &y);
  void GetTwist(double &vx, double &vr);

  void ResetEmergency();
  void SetEmergency(bool active, const std::string &reason);

  void GetEmergencyState(bool &active, bool &latch, std::string &reason);
  void SetControlTwist(double linear, double angular);
  void GetPosition(double &x, double &y, double &heading);

  void SetDockingPose(double x, double y, double heading);

  void GetIsCharging(bool &charging, double &seconds_since_start, std::string &charging_status, double &charger_volts,
                     double &battery_volts, double &charging_current);

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

  // Current Emergency State
  bool emergency_active_ = false;
  // Latched Emergency
  bool emergency_latch_ = false;
  std::string emergency_reason_{"Boot"};
  ros::Time last_update_{0};
  ros::NodeHandle nh_;

  bool is_charging_ = false;
  ros::Time charging_started_time;
  double charger_volts_ = 0;
  double battery_volts_ = BATTERY_VOLTS_MIN;
  double charge_current_ = 0;
  std::string charger_state_{"Unknown"};

  // Timer for simulation
  ros::Timer timer_;
  void SimulationStep(const ros::TimerEvent &te);

  /*
   * Generate some noise
   */
  std::default_random_engine generator{};
  std::normal_distribution<double> position_noise{0.0, 0.03};
  std::normal_distribution<double> heading_noise{0.0, 0.01};
  std::normal_distribution<double> linear_speed_noise{0.0, 0.02};
  std::normal_distribution<double> angular_speed_noise{0.0, 0.02};

  // Debugging
  nav_msgs::Odometry actual_position_{};
  ros::Publisher actual_position_publisher_{};
};

#endif  // SIMROBOT_H
