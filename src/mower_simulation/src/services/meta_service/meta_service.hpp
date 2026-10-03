/*
 * Copyright (C) 2026 The OpenMower Contributors
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef META_SERVICE_HPP
#define META_SERVICE_HPP

#include <MetaServiceBase.hpp>

// Provides the firmware compatibility handshake for the simulated board.
class MetaService : public MetaServiceBase {
 public:
  explicit MetaService(uint16_t service_id) : MetaServiceBase(service_id) {
  }

 protected:
  void RPCGetFirmwareVersion(uint16_t call_id, char* data, uint16_t* response_length) override;
  void RPCGetMajorVersion(uint16_t call_id) override;
};

#endif  // META_SERVICE_HPP
