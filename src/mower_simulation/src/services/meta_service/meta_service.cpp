/*
 * Copyright (C) 2026 The OpenMower Contributors
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "meta_service.hpp"

#include <cstring>

void MetaService::RPCGetFirmwareVersion(uint16_t call_id, char* data, uint16_t* response_length) {
  static constexpr char version[] = "1.0.0-simulation";
  std::memcpy(data, version, sizeof(version));
  *response_length = sizeof(version);
  SendRpcResponse(call_id, xbot::datatypes::RpcStatus::SUCCESS, data, *response_length);
}

void MetaService::RPCGetMajorVersion(uint16_t call_id) {
  // The simulator implements the major-1 firmware service contract.
  const uint16_t major_version = 1;
  SendRpcResponse(call_id, xbot::datatypes::RpcStatus::SUCCESS, &major_version, sizeof(major_version));
}
