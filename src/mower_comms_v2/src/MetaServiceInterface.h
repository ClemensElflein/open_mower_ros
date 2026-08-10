//
// Created by openmower on 09.08.26.
//

#ifndef METASERVICEINTERFACE_H
#define METASERVICEINTERFACE_H

#include <MetaServiceInterfaceBase.hpp>
#include <string>

class MetaServiceInterface : public MetaServiceInterfaceBase {
 public:
  MetaServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx, const std::string& firmware_name)
      : MetaServiceInterfaceBase(service_id, ctx), firmware_name_(firmware_name) {
  }

 private:
  void OnServiceConnected(uint16_t service_id) override;

  std::string firmware_name_;
};

#endif  // METASERVICEINTERFACE_H
