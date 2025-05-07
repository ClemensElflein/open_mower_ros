#include "InputServiceInterface.h"

#include <libfyaml.h>
#include <ros/console.h>

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

  StartTransaction(true);
  SetRegisterInputConfigs(json_str, strlen(json_str));
  CommitTransaction();

  free(json_str);
  return true;
}
