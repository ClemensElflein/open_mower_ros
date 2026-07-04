//
// Registers ROS-layer RPCs (bridged to the app over MQTT by xbot_monitoring) that drive
// the simulation into interesting test states, and streams the current simulation
// control state to MQTT so the app can display it.
//

#ifndef SIM_RPC_H
#define SIM_RPC_H

#include <ros/ros.h>
#include <xbot_mqtt/provider.h>

#include "SimRobot.h"

class SimRpc {
 public:
  SimRpc(ros::NodeHandle& nh, SimRobot& robot);

  // Advertises the MQTT publisher, registers the RPC methods and starts the state stream.
  void Start();

 private:
  void PublishState();
  void PublishState(const ros::TimerEvent&) {
    PublishState();
  }

  SimRobot& robot_;
  ros::NodeHandle& nh_;

  ros::Publisher mqtt_publish_pub_;
  ros::Timer publish_timer_;
  xbot_mqtt::RpcProvider rpc_provider_;
};

#endif  // SIM_RPC_H
