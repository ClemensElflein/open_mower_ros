//
// See SimRpc.h.
//

#include "SimRpc.h"

#include <xbot_mqtt/MqttPublish.h>
#include <xbot_mqtt/RpcError.h>
#include <xbot_mqtt/publish.h>

using json = nlohmann::ordered_json;

namespace {

// MQTT topic the app subscribes to for the simulation control state.
constexpr const char* SIM_STATE_TOPIC = "sim/state/json";

// Extracts a required boolean parameter, raising an RPC error for the app if it's absent
// or of the wrong type.
bool requireBool(const nlohmann::basic_json<>& params, const char* key) {
  if (!params.is_object() || !params.contains(key) || !params[key].is_boolean()) {
    throw xbot_mqtt::RpcException(xbot_mqtt::RpcError::ERROR_INVALID_PARAMS,
                                  std::string("missing or non-boolean parameter: ") + key);
  }
  return params[key].get<bool>();
}

// Extracts a required numeric parameter, raising an RPC error when absent or wrong type.
double requireNumber(const nlohmann::basic_json<>& params, const char* key) {
  if (!params.is_object() || !params.contains(key) || !params[key].is_number()) {
    throw xbot_mqtt::RpcException(xbot_mqtt::RpcError::ERROR_INVALID_PARAMS,
                                  std::string("missing or non-numeric parameter: ") + key);
  }
  return params[key].get<double>();
}

// Extracts an optional numeric parameter, falling back to `def` when absent.
double optionalNumber(const nlohmann::basic_json<>& params, const char* key, double def) {
  if (params.is_object() && params.contains(key)) {
    if (!params[key].is_number()) {
      throw xbot_mqtt::RpcException(xbot_mqtt::RpcError::ERROR_INVALID_PARAMS,
                                    std::string("non-numeric parameter: ") + key);
    }
    return params[key].get<double>();
  }
  return def;
}

// Serializes the current simulation control state.
json toJson(const SimRobot::SimControlState& s) {
  json state = json::object();
  state["emergency_active"] = s.emergency_active;
  state["emergency_latch"] = s.emergency_latch;
  state["emergency_reason"] = s.emergency_reason;
  state["movement_allowed"] = s.movement_allowed;
  state["gps_good"] = s.gps_good;
  state["battery_voltage"] = s.battery_voltage;
  state["battery_percentage"] = s.battery_percentage;
  state["charging"] = s.charging;
  state["joy_override"] = s.joy_override;
  return state;
}

}  // namespace

SimRpc::SimRpc(ros::NodeHandle& nh, SimRobot& robot) : robot_(robot), nh_(nh), rpc_provider_("mower_simulation") {
}

void SimRpc::Start() {
  mqtt_publish_pub_ = nh_.advertise<xbot_mqtt::MqttPublish>("/xbot_monitoring/mqtt_publish", 10);

  // Set / clear the emergency. Setting it latches the emergency in SimRobot; the latch
  // stays active (robot stopped) until it is explicitly cleared - either here with
  // active=false, or by the high level's reset. The high level's normal heartbeat does
  // NOT clear it.
  rpc_provider_.addMethod("sim.emergency.set",
                          [this](const std::string&, const nlohmann::basic_json<>& params) -> json {
                            if (requireBool(params, "active")) {
                              robot_.TriggerEmergency();
                            } else {
                              robot_.ClearEmergency();
                            }
                            PublishState();
                            return nullptr;
                          });

  // Allow / disallow movement. Disallowing simulates a stuck robot: wheels keep turning
  // (odometry reports motion) but the GPS position stays put.
  rpc_provider_.addMethod("sim.movement.set", [this](const std::string&, const nlohmann::basic_json<>& params) -> json {
    robot_.SetMovementAllowed(requireBool(params, "allowed"));
    PublishState();
    return nullptr;
  });

  // Set the simulated battery pack voltage.
  rpc_provider_.addMethod("sim.battery.set", [this](const std::string&, const nlohmann::basic_json<>& params) -> json {
    robot_.SetBatteryVolts(requireNumber(params, "voltage"));
    PublishState();
    return nullptr;
  });

  // Set GPS quality: good = RTK fix (~2 cm), bad = no RTK fix (~1 m).
  rpc_provider_.addMethod("sim.gps.set", [this](const std::string&, const nlohmann::basic_json<>& params) -> json {
    robot_.SetGpsGood(requireBool(params, "good"));
    PublishState();
    return nullptr;
  });

  // Drive the robot onto the dock and start charging.
  rpc_provider_.addMethod("sim.dock.move", [this](const std::string&, const nlohmann::basic_json<>&) -> json {
    robot_.MoveToDock();
    PublishState();
    return nullptr;
  });

  // Teleport the robot by (dx, dy) meters and dheading radians to simulate a GPS jump.
  rpc_provider_.addMethod("sim.displace", [this](const std::string&, const nlohmann::basic_json<>& params) -> json {
    const double dx = optionalNumber(params, "dx", 0.0);
    const double dy = optionalNumber(params, "dy", 0.0);
    const double dheading = optionalNumber(params, "dheading", 0.0);
    robot_.Displace(dx, dy, dheading);
    PublishState();
    return nullptr;
  });

  // Enable / disable joystick override. When enabled, /joy_vel drives the robot directly
  // and all twist commands from the normal mower stack are ignored.
  rpc_provider_.addMethod("sim.joy_override.set",
                          [this](const std::string&, const nlohmann::basic_json<>& params) -> json {
                            robot_.SetJoyOverride(requireBool(params, "enabled"));
                            PublishState();
                            return nullptr;
                          });

  // Read the current simulation control state on demand.
  rpc_provider_.addMethod("sim.state.get", [this](const std::string&, const nlohmann::basic_json<>&) -> json {
    return toJson(robot_.GetSimControlState());
  });

  rpc_provider_.init();

  // Stream the state to MQTT so the app can display it (retained so late subscribers get
  // the latest value immediately).
  publish_timer_ = nh_.createTimer(ros::Duration(1.0), &SimRpc::PublishState, this);
}

void SimRpc::PublishState() {
  xbot_mqtt::publish(mqtt_publish_pub_, SIM_STATE_TOPIC, toJson(robot_.GetSimControlState()), /*retain=*/true);
}
