#include "InputServiceInterface.h"

#include <libfyaml.h>
#include <ros/console.h>
#include <std_msgs/String.h>

#include <xbot-service-interface/HeatshrinkEncode.hpp>

static void fy_error_handler(struct fy_diag *diag, void *user, const char *buf, size_t len) {
  // Error token is printed with a leading newline, which doesn'twork well with ROS logging.
  if (buf[0] == '\n') {
    buf++;
    len--;
  }
  if (buf[len - 1] == '\n') {
    len--;
  }
  ROS_ERROR("%.*s", static_cast<int>(len), buf);
}

static char *yaml_file_to_json_str(const char *yaml_file) {
  fy_diag_cfg diag_cfg;
  fy_diag_cfg_default(&diag_cfg);
  diag_cfg.fp = nullptr;
  diag_cfg.output_fn = fy_error_handler;
  diag_cfg.colorize = false;

  fy_diag *diag = fy_diag_create(&diag_cfg);
  fy_parse_cfg cfg{.flags = static_cast<fy_parse_cfg_flags>(FYPCF_DEFAULT_DOC | FYPCF_JSON_AUTO), .diag = diag};

  fy_document *fyd = fy_document_build_from_file(&cfg, yaml_file);
  if (fyd == nullptr) {
    fy_diag_destroy(diag);
    return nullptr;
  }

  char *json_str = fy_emit_document_to_string(fyd, FYECF_MODE_JSON_TP);
  fy_document_destroy(fyd);
  fy_diag_destroy(diag);
  return json_str;
}

bool InputServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  if (config_file_.empty()) {
    ROS_ERROR_STREAM("Input config file not specified");
    return true;
  }
  char *json_str = yaml_file_to_json_str(config_file_.c_str());
  if (!json_str) {
    return true;
  }

  // Parse the JSON to a DOM structure so we can access it later.
  json config = json::parse(json_str);
  free(json_str);

  // Build an index to access the config for a specific input more easily.
  size_t next_idx = 0;
  inputs_.clear();
  for (auto &drivers : config.items()) {
    for (auto input : drivers.value()) {
      input["_idx"] = next_idx++;
      input["_driver"] = drivers.key();
      inputs_.push_back(input);
    }
  }

  // We don't want and need to send the names and actions to the firmware, so create a copy and remove.
  json config_for_firmware(config);
  for (auto &drivers : config_for_firmware.items()) {
    for (auto &input : drivers.value()) {
      input.erase("name");
      input.erase("actions");
    }
  }
  std::string config_for_firmware_str = config_for_firmware.dump(2);

  std::vector<uint8_t> compressed = HeatshrinkEncode(
      reinterpret_cast<uint8_t *>(const_cast<char *>(config_for_firmware_str.c_str())), config_for_firmware_str.size());
  ROS_INFO_STREAM("Input config JSON has " << config_for_firmware_str.size() << " bytes, compressed to "
                                           << compressed.size() << " bytes");

  StartTransaction(true);
  SetRegisterInputConfigs(compressed.data(), compressed.size());
  CommitTransaction();

  return true;
}

void InputServiceInterface::OnActiveInputsChanged(const uint64_t &new_value) {
  static uint64_t prev_value = 0;
  if (new_value == prev_value) return;
  ROS_WARN_STREAM("Active Inputs bitmask: " << new_value);
  for (size_t idx = 0; idx < 64; ++idx) {
    if (new_value & (uint64_t(1) << idx)) {
      json &input = inputs_[idx];
      ROS_WARN_STREAM("\"" << std::string(input["name"]) << "\" is active");
    }
  }
  prev_value = new_value;
}

bool InputServiceInterface::TriggerActionForContext(json &actions, std::string context) {
  auto it_action = actions.find(context);
  if (it_action != actions.end()) {
    ROS_INFO_STREAM("Triggering " << *it_action);
    std_msgs::String action_msg;
    action_msg.data = *it_action;
    action_pub_.publish(action_msg);
    return true;
  } else {
    return false;
  }
}

void InputServiceInterface::OnInputEventChanged(const uint8_t *new_value, uint32_t length) {
  // TODO: Determine context from high level status.
  const std::string context{"area_recording"};

  // Check if we want to handle this event.
  const InputEventType type = static_cast<InputEventType>(new_value[1]);
  std::string duration;
  switch (type) {
    case InputEventType::SHORT: duration = "short"; break;
    case InputEventType::LONG: duration = "long"; break;
    default: return;
  }

  // Look up the actions for this input.
  const uint8_t idx = new_value[0];
  json &input = inputs_[idx];
  auto it_actions = input.find("actions");
  if (it_actions != input.end()) {
    if (TriggerActionForContext(*it_actions, context + "/" + duration)) return;
    if (TriggerActionForContext(*it_actions, context)) return;
  }
  ROS_WARN_STREAM("No action found for \"" << std::string(input["name"]) << "\", " << context + "/" << duration);
}

void InputServiceInterface::OnAction(std::string raw_payload) {
  const char *prefix = "mower_comms_v2/simulate_input||";
  if (raw_payload.rfind(prefix, 0) == std::string::npos) return;
  const std::string name = raw_payload.substr(strlen(prefix));
  for (const json &input : inputs_) {
    if (input["_driver"] == "simulated" && input.contains("name") && input["name"] == name) {
      uint8_t bit = input["bit"];
      uint64_t mask = 1 << bit;
      SendSimulatedInputs(mask);
      // TODO: Use a better way to delay, allow multiple keys at once.
      usleep(300'000);
      SendSimulatedInputs(0);
      return;
    }
  }
  ROS_ERROR_STREAM("No simulated input with name \"" << name << "\" found");
}
