#pragma once

#include "nlohmann/json.hpp"

inline const nlohmann::ordered_json CAPABILITIES = {
    {"rpc", 1},
    {"map:json", 1},
};
