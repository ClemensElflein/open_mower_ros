//
// Created by clemens on 25.07.24.
//

#ifndef DIFFDRIVESERVICEINTERFACE_H
#define DIFFDRIVESERVICEINTERFACE_H

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <DiffDriveServiceInterfaceBase.hpp>

class DiffDriveServiceInterface : public DiffDriveServiceInterfaceBase {
 public:
  DiffDriveServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                            const ros::Publisher& actual_twist_publisher, double ticks_per_meter, double wheel_distance)
      : DiffDriveServiceInterfaceBase(service_id, ctx),
        actual_twist_publisher_(actual_twist_publisher),
        wheel_distance_(wheel_distance),
        ticks_per_meter_(ticks_per_meter) {
  }

  bool OnConfigurationRequested(const std::string& uid) override;

  /**
   * Convenience function to transmit the twist from a ROS message
   * @param msg The ROS message
   */
  void SendTwist(const geometry_msgs::TwistConstPtr& msg);
  void GetEscInfo(bool& left_error, float& left_esc_temperature, float& left_esc_current, bool& right_error,
                  float& right_esc_temperature, float& right_esc_current);

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

 private:
  void OnServiceConnected(const std::string& uid) override;
  void OnTransactionStart(uint64_t timestamp) override;
  void OnTransactionEnd() override;
  void OnServiceDisconnected(const std::string& uid) override;

 private:
  // Store the seq number for the actual twist message
  uint32_t seq = 0;
  std::mutex state_mutex_{};

 public:
  const ros::Publisher& actual_twist_publisher_;
  double wheel_distance_;
  double ticks_per_meter_;

  // Store the latest ESC state
  struct ESCState {
    bool error = true;
    float temperature = 0.0;
    float current = 0.0;
  } left_esc_state_{}, right_esc_state_{};
};

#endif  // DIFFDRIVESERVICEINTERFACE_H
