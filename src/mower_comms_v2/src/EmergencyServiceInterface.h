//
// Created by clemens on 25.07.24.
//

#ifndef EMERGENCYSERVICEINTERFACE_H
#define EMERGENCYSERVICEINTERFACE_H

#include <ros/node_handle.h>

#include <EmergencyServiceInterfaceBase.hpp>

class EmergencyServiceInterface : public EmergencyServiceInterfaceBase {
 public:
  EmergencyServiceInterface(const ros::NodeHandle &nh, uint16_t service_id, const xbot::serviceif::Context &ctx)
      : EmergencyServiceInterfaceBase(service_id, ctx), nh(nh) {
  }

 private:
  const ros::NodeHandle &nh;
};

#endif  // EMERGENCYSERVICEINTERFACE_H
