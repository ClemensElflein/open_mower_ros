#ifndef INPUTSERVICEINTERFACE_H
#define INPUTSERVICEINTERFACE_H

#include <InputServiceInterfaceBase.hpp>

using json = nlohmann::ordered_json;

class InputServiceInterface : public InputServiceInterfaceBase {
 public:
  InputServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx, std::string config_file)
      : InputServiceInterfaceBase(service_id, ctx), config_file_(config_file) {
  }

  bool OnConfigurationRequested(uint16_t service_id) override;
  void OnActiveInputsChanged(const uint64_t& new_value) override;
  void OnInputEventChanged(const uint8_t* new_value, uint32_t length) override;

 private:
  std::string config_file_;
  json config_;
  std::vector<std::reference_wrapper<json>> inputs_;
};

#endif  // INPUTSERVICEINTERFACE_H
