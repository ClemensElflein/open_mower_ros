//
// Created by clemens on 09.09.24.
//

#include "DockingSensorServiceInterface.h"
bool DockingSensorServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  StartTransaction(true);
  SetRegisterSensorTimeoutMs(1000);
  CommitTransaction();
  return true;
}
void DockingSensorServiceInterface::OnDetectedSensorsLeftChanged(const uint8_t& new_value) {
  msg.detected_left = new_value;
}
void DockingSensorServiceInterface::OnDetectedSensorsRightChanged(const uint8_t& new_value) {
  msg.detected_right = new_value;
}
void DockingSensorServiceInterface::OnServiceConnected(uint16_t service_id) {
}
void DockingSensorServiceInterface::OnTransactionStart(uint64_t timestamp) {
  msg = {};
  msg.stamp = ros::Time::now();
}
void DockingSensorServiceInterface::OnTransactionEnd() {
  sensor_publisher_.publish(msg);
}
