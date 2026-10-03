#pragma once

#include "nlohmann/json.hpp"

inline const nlohmann::ordered_json CAPABILITIES = {
    {"rpc", 1},
    {"map:json", 1},
    {"mqtt:params", 1},
    // params.settable / params.set in mower_logic, params/json follows a change
    {"params:set", 1},
    {"events", 1},
    {"position", 1},
};
