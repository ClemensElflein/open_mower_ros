#include "InputServiceInterface.h"

#include <libfyaml.h>
#include <ros/console.h>

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
  config_ = json::parse(json_str);
  free(json_str);

  // Build an index to access the config for a specific input more easily.
  size_t next_idx = 0;
  inputs_.clear();
  for (auto &drivers : config_.items()) {
    for (auto &input : drivers.value()) {
      inputs_.push_back(input);
    }
  }

  // We don't want and need to send the actions to the firmware, so create a copy and remove.
  json config_without_actions(config_);
  for (auto &drivers : config_without_actions.items()) {
    for (auto &input : drivers.value()) {
      input.erase("actions");
    }
  }
  std::string config_without_actions_str = config_without_actions.dump(2);

  std::vector<uint8_t> compressed =
      HeatshrinkEncode(reinterpret_cast<uint8_t *>(const_cast<char *>(config_without_actions_str.c_str())),
                       config_without_actions_str.size());
  ROS_INFO_STREAM("Input config JSON has " << config_without_actions_str.size() << " bytes, compressed to "
                                           << compressed.size() << " bytes");

  StartTransaction(true);
  SetRegisterInputConfigs(compressed.data(), compressed.size());
  CommitTransaction();

  return true;
}

void InputServiceInterface::OnActiveInputsChanged(const uint64_t &new_value) {
  ROS_WARN_STREAM("Active Inputs bitmask: " << new_value);
  for (size_t idx = 0; idx < 64; ++idx) {
    if (new_value & (uint64_t(1) << idx)) {
      json &input = inputs_[idx];
      ROS_WARN_STREAM("\"" << std::string(input["name"]) << "\" is active");
    }
  }
}

std::string to_string(InputServiceInterface::InputEventType type) {
#define ENUM_TO_STRING_CASE(name) \
  case InputServiceInterface::InputEventType::name: return #name;
  switch (type) {
    ENUM_TO_STRING_CASE(ACTIVE)
    ENUM_TO_STRING_CASE(INACTIVE)
    ENUM_TO_STRING_CASE(SHORT)
    ENUM_TO_STRING_CASE(LONG)
    default: return "Unknown";
  }
}

void InputServiceInterface::OnInputEventChanged(const uint8_t *new_value, uint32_t length) {
  const uint8_t idx = new_value[0];
  const InputEventType type = static_cast<InputEventType>(new_value[1]);
  json &input = inputs_[idx];
  ROS_WARN_STREAM("Event for \"" << std::string(input["name"]) << "\": " << to_string(type));
  auto it_actions = input.find("actions");
  if (it_actions != input.end()) {
    ROS_WARN_STREAM("The following actions are available: " << it_actions->dump());
  }
}
