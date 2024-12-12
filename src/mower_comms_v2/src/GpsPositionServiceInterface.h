//
// Created by clemens on 30.11.24.
//

#ifndef GPSPOSITIONSERVICEINTERFACE_H
#define GPSPOSITIONSERVICEINTERFACE_H
#include <ros/publisher.h>
#include <xbot_msgs/AbsolutePose.h>

#include <PositionServiceInterfaceBase.hpp>


class GpsPositionServiceInterface : public PositionServiceInterfaceBase {
public:
  GpsPositionServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx, const ros::Publisher& imu_publisher)
      : PositionServiceInterfaceBase(service_id, ctx), absolute_pose_publisher_(imu_publisher) {
  }
  bool OnConfigurationRequested(uint16_t service_id) override;

protected:
  void OnPositionXYZChanged(const double* new_value, uint32_t length) override;
  void OnPositionAccuracyChanged(const double& new_value) override;
  void OnFixTypeChanged(const char* new_value, uint32_t length) override;
  void OnMotionVectorXYZChanged(const double* new_value, uint32_t length) override;
  void OnMotionHeadingAndAccuracyChanged(const double* new_value, uint32_t length) override;
  void OnVehicleHeadingAndAccuracyChanged(const double* new_value, uint32_t length) override;
  void OnNMEAChanged(const char* new_value, uint32_t length) override;

 private:
  void OnServiceConnected(uint16_t service_id) override;
  void OnTransactionStart(uint64_t timestamp) override;
  void OnTransactionEnd() override;
  void OnServiceDisconnected(uint16_t service_id) override;
  const ros::Publisher& absolute_pose_publisher_;
  xbot_msgs::AbsolutePose pose_msg_;
};



#endif //GPSPOSITIONSERVICEINTERFACE_H
