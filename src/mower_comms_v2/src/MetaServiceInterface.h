//
// Created by openmower on 09.08.26.
//

#ifndef METASERVICEINTERFACE_H
#define METASERVICEINTERFACE_H

#include <ros/ros.h>

#include <MetaServiceInterfaceBase.hpp>
#include <string>

class MetaServiceInterface : public MetaServiceInterfaceBase {
 public:
  MetaServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx, const ros::NodeHandle& nh,
                       const std::string& firmware_name)
      : MetaServiceInterfaceBase(service_id, ctx), nh_(nh), firmware_name_(firmware_name) {
    // One-shot timer that runs LogFirmwareInfo() on the ROS (in separate spinner thread)
    post_config_timer_ =
        nh_.createTimer(ros::Duration(0.5), &MetaServiceInterface::OnPostConfigurationTimer, this, true, false);
  }

  /// Queries the firmware's major version and firmware version string and logs
  /// them. Blocking (RPC); must NOT be called from the xbot IO thread, as it
  /// would deadlock waiting for a response delivered on that same thread.
  void LogFirmwareInfo();

 private:
  bool OnConfigurationRequested(uint16_t service_id) override;
  void OnServiceDisconnected(uint16_t service_id) override;
  void OnPostConfigurationTimer(const ros::TimerEvent&);

  ros::NodeHandle nh_;
  ros::Timer post_config_timer_;
  std::string firmware_name_;
};

#endif  // METASERVICEINTERFACE_H
