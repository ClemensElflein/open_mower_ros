#ifndef INPUTSERVICEINTERFACE_H
#define INPUTSERVICEINTERFACE_H

#include <mower_msgs/HighLevelStatus.h>
#include <ros/ros.h>

#include <InputServiceInterfaceBase.hpp>

using json = nlohmann::ordered_json;

class InputServiceInterface : public InputServiceInterfaceBase {
 public:
  InputServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx, std::string config_file,
                        ros::Publisher action_pub)
      : InputServiceInterfaceBase(service_id, ctx),
        config_file_(config_file),
        action_pub_(action_pub),
        current_context_("idle") {
    ros::NodeHandle nh;
    high_level_status_sub_ =
        nh.subscribe("mower_logic/current_state", 1, &InputServiceInterface::highLevelStatusReceived, this);
  }

  bool OnConfigurationRequested(uint16_t service_id) override;
  void OnActiveInputsChanged(const uint64_t& new_value) override;
  void OnInputEventChanged(const uint8_t* new_value, uint32_t length) override;
  void OnAction(std::string raw_payload);

 private:
  std::string config_file_;
  std::vector<json> inputs_;
  ros::Publisher action_pub_;
  ros::Subscriber high_level_status_sub_;
  std::string current_context_;

  void highLevelStatusReceived(const mower_msgs::HighLevelStatus::ConstPtr& msg);
  bool TriggerActionForContext(json& actions, std::string context);
};

#endif  // INPUTSERVICEINTERFACE_H
