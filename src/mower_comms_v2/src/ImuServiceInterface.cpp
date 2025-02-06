//
// Created by clemens on 26.07.24.
//

#include "ImuServiceInterface.h"

void ImuServiceInterface::OnAxesChanged(const double* new_value, uint32_t length) {
  if (length < 6) {
    return;
  }
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.seq++;
  imu_msg.header.frame_id = "base_link";
  imu_msg.linear_acceleration.x = new_value[0];
  imu_msg.linear_acceleration.y = new_value[1];
  imu_msg.linear_acceleration.z = new_value[2];
  imu_msg.angular_velocity.x = new_value[3];
  imu_msg.angular_velocity.y = new_value[4];
  imu_msg.angular_velocity.z = new_value[5];
  imu_publisher_.publish(imu_msg);
}
