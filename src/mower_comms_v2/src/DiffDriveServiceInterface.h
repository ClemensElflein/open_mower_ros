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
                            const ros::Publisher& right_esc_status_publisher, double ticks_per_meter,
                            double wheel_distance)
      : DiffDriveServiceInterfaceBase(service_id, ctx),
        actual_twist_publisher_(actual_twist_publisher),
        left_esc_status_publisher_(left_esc_status_publisher),
        right_esc_status_publisher_(right_esc_status_publisher),
        wheel_distance_(wheel_distance),
        ticks_per_meter_(ticks_per_meter) {
  }

  bool OnConfigurationRequested(uint16_t service_id) override;

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
  double wheel_distance_;
  double ticks_per_meter_;

  // Store the latest ESC state
  mower_msgs::ESCStatus left_esc_state_{};
  mower_msgs::ESCStatus right_esc_state_{};
};

#endif  // DIFFDRIVESERVICEINTERFACE_H
