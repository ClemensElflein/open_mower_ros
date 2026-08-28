//
// Created by openmower on 09.08.26.
//

#ifndef METASERVICEINTERFACE_H
#define METASERVICEINTERFACE_H

#include <ros/ros.h>

#include <MetaServiceInterfaceBase.hpp>
#include <atomic>
#include <functional>
#include <string>

/// Result of a firmware version query.
struct FirmwareInfo {
  uint16_t major = 0;      // 0 = Unknown/RPC-failure/Disconnect
  std::string version;     // full version string (empty if unknown)
  bool connected = false;  // true while the MetaService is connected (configured)
};

class MetaServiceInterface : public MetaServiceInterfaceBase {
 public:
  MetaServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx, const ros::NodeHandle& nh,
                       const std::string& firmware_name, std::function<void(const FirmwareInfo&)> version_callback)
      : MetaServiceInterfaceBase(service_id, ctx),
        nh_(nh),
        firmware_name_(firmware_name),
        version_callback_(std::move(version_callback)) {
    // Periodic timer that polls the firmware version. Runs on the ROS (separate
    // spinner) thread until StopFirmwareCheck() is called or the service disconnects.
    firmware_check_timer_ = nh_.createTimer(
        ros::Duration(1.0), [this](const ros::TimerEvent&) { CheckFirmwareVersion(); }, false, true);
  }

  /// Stops the periodic firmware version polling.
  void StopFirmwareCheck() {
    firmware_check_timer_.stop();
  }

 private:
  bool OnConfigurationRequested(uint16_t service_id) override;
  void OnServiceDisconnected(uint16_t service_id) override;

  /// Reads the firmware major version (+ version string) and reports it via the
  /// callback. Blocking (RPC); must NOT be called from the xbot IO thread, as it
  /// would deadlock waiting for a response delivered on that same thread.
  void CheckFirmwareVersion();

  ros::NodeHandle nh_;
  ros::Timer firmware_check_timer_;
  std::string firmware_name_;
  std::function<void(const FirmwareInfo&)> version_callback_;
  // True once the firmware's MetaService has connected and requested configuration.
  std::atomic<bool> configured_{false};
};

#endif  // METASERVICEINTERFACE_H
