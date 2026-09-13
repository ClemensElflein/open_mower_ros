//
// Created by clemens on 25.07.24.
//

#ifndef DIFFDRIVESERVICEINTERFACE_H
#define DIFFDRIVESERVICEINTERFACE_H

#include <geometry_msgs/Twist.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/EmergencyStopSrv.h>
#include <ros/ros.h>

#include <DiffDriveServiceInterfaceBase.hpp>

class DiffDriveServiceInterface : public DiffDriveServiceInterfaceBase {
 public:
  DiffDriveServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                            const ros::Publisher& actual_twist_publisher,
                            const ros::Publisher& left_esc_status_publisher,
                            const ros::Publisher& right_esc_status_publisher, const ros::NodeHandle& param_nh)
      : DiffDriveServiceInterfaceBase(service_id, ctx),
        actual_twist_publisher_(actual_twist_publisher),
        left_esc_status_publisher_(left_esc_status_publisher),
        right_esc_status_publisher_(right_esc_status_publisher) {
    if (!param_nh.getParam("services/diff_drive/ticks_per_m", ticks_per_meter_)) {
      ROS_ERROR("Need to provide param services/diff_drive/ticks_per_m");
      param_error_ = 1;
      return;
    }
    if (!param_nh.getParam("services/diff_drive/wheel_distance_m", wheel_distance_)) {
      ROS_ERROR("Need to provide param services/diff_drive/wheel_distance_m");
      param_error_ = 1;
      return;
    }
    ROS_INFO_STREAM("Wheel ticks [1/m]: " << ticks_per_meter_);
    ROS_INFO_STREAM("Wheel distance [m]: " << wheel_distance_);
  }

  bool OnConfigurationRequested(uint16_t service_id) override;

  /**
   * Exit code for missing required parameters (0 = parameters valid).
   */
  int GetParamError() const {
    return param_error_;
  }

  /**
   * Convenience function to transmit the twist from a ROS message
   * @param msg The ROS message
   */
  void SendTwist(const geometry_msgs::TwistConstPtr& msg);

 protected:
  /**
   * Callback whenever an updated twist arrives
   * @param new_value the updated value
   * @param length length of array
   */
  void OnActualTwistChanged(const double* new_value, uint32_t length) override;
  void OnLeftESCTemperatureChanged(const float& new_value) override;
  void OnLeftESCCurrentChanged(const float& new_value) override;
  void OnRightESCTemperatureChanged(const float& new_value) override;
  void OnRightESCCurrentChanged(const float& new_value) override;
  void OnWheelTicksChanged(const uint32_t* new_value, uint32_t length) override;
  void OnLeftESCStatusChanged(const uint8_t& new_value) override;
  void OnRightESCStatusChanged(const uint8_t& new_value) override;

 private:
  void OnServiceConnected(uint16_t service_id) override;
  void OnTransactionEnd() override;
  void OnServiceDisconnected(uint16_t service_id) override;

 private:
  // Store the seq number for the actual twist message
  uint32_t seq = 0;
  std::mutex state_mutex_{};

 public:
  const ros::Publisher& actual_twist_publisher_;
  const ros::Publisher& left_esc_status_publisher_;
  const ros::Publisher& right_esc_status_publisher_;
  double wheel_distance_ = 0.0;
  double ticks_per_meter_ = 0.0;
  int param_error_ = 0;

  // Store the latest ESC state
  mower_msgs::ESCStatus left_esc_state_{};
  mower_msgs::ESCStatus right_esc_state_{};
};

#endif  // DIFFDRIVESERVICEINTERFACE_H
