//
// Created by Clemens Elflein on 22.11.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
#include <mqtt/async_client.h>

#include <algorithm>
#include <atomic>
#include <boost/regex.hpp>
#include <climits>
#include <cmath>
#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <vector>

#include "EventHistory.h"
#include "LogBuffer.h"
#include "PositionHistory.h"
#include "capabilities.h"
#include "dynamic_reconfigure/Config.h"
#include "dynamic_reconfigure/ConfigDescription.h"
#include "dynamic_reconfigure/Reconfigure.h"
#include "dynamic_reconfigure/config_tools.h"
#include "geometry_msgs/Twist.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "xbot_mqtt/RegisterMethodsSrv.h"
#include "xbot_mqtt/RpcError.h"
#include "xbot_mqtt/RpcRequest.h"
#include "xbot_mqtt/RpcResponse.h"
#include "xbot_mqtt/constants.h"
#include "xbot_mqtt/provider.h"
#include "xbot_mqtt/publish.h"
#include "xbot_msgs/AbsolutePose.h"
#include "xbot_msgs/ActionInfo.h"
#include "xbot_msgs/MapOverlay.h"
#include "xbot_msgs/RegisterActionsSrv.h"
#include "xbot_msgs/RobotState.h"
#include "xbot_msgs/SensorDataDouble.h"
#include "xbot_msgs/SensorDataString.h"
#include "xbot_msgs/SensorInfo.h"

const double MQTT_POSITION_PUBLISH_INTERVAL = 0.150;
const double POSITION_HISTORY_FLUSH_INTERVAL = 30.0;

using json = nlohmann::ordered_json;

void publish_capabilities();
void publish_sensor_metadata();
void publish_map();
void publish_map_overlay();
void publish_actions();
void publish_version();
void publish_params(bool only_if_changed = false);
void rpc_request_callback(const std::string &payload);

// Stores registered actions (prefix to vector<action>)
std::map<std::string, std::vector<xbot_msgs::ActionInfo>> registered_actions;
std::mutex registered_actions_mutex;

// Stores registered RPC methods
std::map<std::string, std::vector<std::string>> registered_methods;
std::mutex registered_methods_mutex;

std::map<std::string, xbot_msgs::SensorInfo> found_sensors;
std::mutex found_sensors_mutex;

// Parameter descriptions of the nodes with a dynamic_reconfigure server, by node namespace
std::map<std::string, dynamic_reconfigure::ConfigDescription::ConstPtr> param_descriptions;
std::mutex param_descriptions_mutex;
// Set when one of them publishes new values
std::atomic<bool> params_changed(false);
// Set on an MQTT connect, params/json is published again then. Only the main loop publishes it
std::atomic<bool> params_connected(false);

// The last params/json, many parameter updates don't change anything. Only touched by the main loop
std::string params_payload;

ros::NodeHandle *n;

// The MQTT Client
std::shared_ptr<mqtt::async_client> client_;
std::shared_ptr<mqtt::async_client> client_external_;


// Publisher for cmd_vel and commands
ros::Publisher cmd_vel_pub;
ros::Publisher action_pub;
ros::Publisher rpc_request_pub;

// properties for external mqtt
bool external_mqtt_enable = false;
std::string external_mqtt_username = "";
std::string external_mqtt_password = "";
std::string external_mqtt_hostname = "";
std::string external_mqtt_topic_prefix = "";
std::string external_mqtt_port = "";
std::string version_string = "";

class MqttCallback : public mqtt::callback {

    void connected(const mqtt::string &string) override {
        ROS_INFO_STREAM("MQTT Connected");
        publish_capabilities();
        publish_sensor_metadata();
        publish_map();
        publish_map_overlay();
        publish_actions();
        publish_version();
        params_connected = true;

        // BEGIN: Deprecated code (1/2)
        // Earlier implementations subscribed to "/action" and "prefix//action" topics, we do it to not break stuff as well.
        client_->subscribe(this->mqtt_topic_prefix + "/teleop", 0);
        client_->subscribe(this->mqtt_topic_prefix + "/command", 0);
        client_->subscribe(this->mqtt_topic_prefix + "/action", 0);
        // END: Deprecated code (1/2)

        client_->subscribe(this->mqtt_topic_prefix + "teleop", 0);
        client_->subscribe(this->mqtt_topic_prefix + "command", 0);
        client_->subscribe(this->mqtt_topic_prefix + "action", 0);
        client_->subscribe(this->mqtt_topic_prefix + "rpc/request", 0);
    }

public:
    void setMqttClient(std::shared_ptr<mqtt::async_client> c, const std::string &mqtt_topic_prefix) {
        this->client_ = std::move(c);
        this->mqtt_topic_prefix = mqtt_topic_prefix;
    }
    void message_arrived(mqtt::const_message_ptr ptr) override {
        if(ptr->get_topic() == this->mqtt_topic_prefix + "teleop") {
            // only vx and vz, some 30 bytes. a large nested document overflowed the stack while decoding
            if (ptr->get_payload().size() > 1024) {
                ROS_ERROR_STREAM("Ignoring teleop bson of " << ptr->get_payload().size() << " bytes");
                return;
            }
            try {
                json json = json::from_bson(ptr->get_payload().begin(), ptr->get_payload().end());
                geometry_msgs::Twist t;
                t.linear.x = json["vx"];
                t.angular.z = json["vz"];
                cmd_vel_pub.publish(t);
            } catch (const json::exception &e) {
                ROS_ERROR_STREAM("Error decoding teleop bson: " << e.what());
            }
        } else if(ptr->get_topic() == this->mqtt_topic_prefix + "action") {
            ROS_INFO_STREAM("Got action: " + ptr->get_payload());
            std_msgs::String action_msg;
            action_msg.data = ptr->get_payload_str();
            action_pub.publish(action_msg);
        } else if(ptr->get_topic() == this->mqtt_topic_prefix + "/action") {
            // BEGIN: Deprecated code (2/2)
            ROS_WARN_STREAM("Got action on deprecated topic! Change your topic names!: " + ptr->get_payload());
            std_msgs::String action_msg;
            action_msg.data = ptr->get_payload_str();
            action_pub.publish(action_msg);
            // END: Deprecated code (2/2)
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "rpc/request") {
          std::string payload = ptr->get_payload_str();
          rpc_request_callback(payload);
        }
    }
private:
    std::shared_ptr<mqtt::async_client> client_;
    std::string mqtt_topic_prefix = "";
};

MqttCallback mqtt_callback;
MqttCallback mqtt_callback_external;

json map;
std::mutex map_mutex;
json map_overlay;
std::mutex map_overlay_mutex;
bool has_map = false;
bool has_map_overlay = false;

EventHistory event_history;
PositionHistory position_history;
LogBuffer log_buffer(1000);

// clang-format off
xbot_mqtt::RpcProvider rpc_provider("xbot_monitoring", {{
    RPC_METHOD("rpc.ping", {
        return "pong";
    }),
    RPC_METHOD("rpc.methods", {
        std::lock_guard<std::mutex> lk(registered_methods_mutex);
        json methods = json::array();
        for (const auto& [_, method_ids] : registered_methods) {
            for (const auto& method_id : method_ids) {
                methods.push_back(method_id);
            }
        }
        std::sort(methods.begin(), methods.end());
        return methods;
    }),
    RPC_METHOD("events.history", {
        if (params.is_object() && params.contains("date")) {
            return event_history.getAll(params["date"].get<std::string>());
        } else {
            return event_history.getAll();
        }
    }),
    RPC_METHOD("events.history.list", {
        return event_history.listHistories();
    }),
    RPC_METHOD("events.history.delete", {
        if (params.is_object() && params.contains("date")) {
            return event_history.deleteHistory(params["date"].get<std::string>());
        } else {
            return event_history.deleteHistory(std::nullopt);
        }
    }),
    RPC_METHOD("logs.recent", {
        double since = 0;
        uint8_t level = rosgraph_msgs::Log::WARN;
        size_t limit = 200;
        if (params.is_object()) {
            if (params.contains("since") && params["since"].is_number()) since = params["since"].get<double>();
            if (params.contains("level") && params["level"].is_string()) level = LogBuffer::levelFromName(params["level"].get<std::string>());
            if (params.contains("limit") && params["limit"].is_number_unsigned()) limit = params["limit"].get<size_t>();
        }
        return log_buffer.get(since, level, limit);
    }),
    RPC_METHOD("position.history", {
        if (params.is_object() && params.contains("job_id")) {
            return position_history.getHistory(params["job_id"].get<std::string>());
        } else {
            return position_history.getHistory();
        }
    }),
    RPC_METHOD("position.history.list", {
        return position_history.listHistories();
    }),
    RPC_METHOD("position.history.delete", {
        if (params.is_object() && params.contains("job_id")) {
            return position_history.deleteHistory(params["job_id"].get<std::string>());
        } else {
            return position_history.deleteHistory(std::nullopt);
        }
    }),
}});
// clang-format on

void setupMqttClient() {
    // setup mqtt client for app use
    {
        // MQTT connection options
        mqtt::connect_options connect_options_;

        // basic client connection options
        connect_options_.set_automatic_reconnect(true);
        connect_options_.set_clean_session(true);
        connect_options_.set_keep_alive_interval(1000);
        connect_options_.set_max_inflight(10);

        // create MQTT client
        std::string uri = "tcp" + std::string("://") + "127.0.0.1" +
                          std::string(":") + std::to_string(1883);

        try {
            client_ = std::make_shared<mqtt::async_client>(
                    uri, "xbot_monitoring");
            mqtt_callback.setMqttClient(client_, "");
            client_->set_callback(mqtt_callback);

            client_->connect(connect_options_);

        } catch (const mqtt::exception &e) {
            ROS_ERROR("Client could not be initialized: %s", e.what());
            exit(EXIT_FAILURE);
        }
    }
    // setup external mqtt client
    if(external_mqtt_enable) {
        // MQTT connection options
        mqtt::connect_options connect_options_;

        // basic client connection options
        connect_options_.set_automatic_reconnect(true);
        connect_options_.set_clean_session(true);
        connect_options_.set_keep_alive_interval(1000);
        connect_options_.set_max_inflight(10);

        if(!external_mqtt_username.empty()) {
            connect_options_.set_user_name(external_mqtt_username);
            connect_options_.set_password(external_mqtt_password);
        }

        // create MQTT client
        std::string uri = "tcp" + std::string("://") + external_mqtt_hostname +
                          std::string(":") + external_mqtt_port;

        try {
            client_external_ = std::make_shared<mqtt::async_client>(
                    uri, "ext_xbot_monitoring");
            mqtt_callback_external.setMqttClient(client_external_, external_mqtt_topic_prefix);
            client_external_->set_callback(mqtt_callback_external);

            client_external_->connect(connect_options_);

        } catch (const mqtt::exception &e) {
            ROS_ERROR("External Client could not be initialized: %s", e.what());
            exit(EXIT_FAILURE);
        }
    }
}

void try_publish(const std::string &topic, const std::string &data, bool retain = false) {
    try {
        if (retain) {
            // QOS 1 so that the data actually arrives at the client at least once.
            client_->publish(topic, data, 1, true);
        } else {
            client_->publish(topic, data);
        }
    } catch (const mqtt::exception &e) {
        // client disconnected or something, we drop it.
    }
    // publish external
    if(external_mqtt_enable) {
        try {
            if (retain) {
                // QOS 1 so that the data actually arrives at the client at least once.
                client_external_->publish(external_mqtt_topic_prefix + topic, data, 1, true);
            } else {
                client_external_->publish(external_mqtt_topic_prefix + topic, data);
            }
        } catch (const mqtt::exception &e) {
            // client disconnected or something, we drop it.
        }
    }
}

void publish_event(const std::string& type, json details = nullptr) {
  const std::string payload_str = xbot_mqtt::buildEventPayload(type, details).dump();
  try_publish(xbot_mqtt::EVENTS_TOPIC, payload_str);
  event_history.add(payload_str);
}

void try_publish_binary(const std::string &topic, const void *data, size_t size, bool retain = false) {
    try {
        if (retain) {
            // QOS 1 so that the data actually arrives at the client at least once.
            client_->publish(topic, data, size, 1, true);
        } else {
            client_->publish(topic, data, size);
        }
    } catch (const mqtt::exception &e) {
        // client disconnected or something, we drop it.
    }
}

void publish_version() {
    json version = {
            {"version", version_string}
    };
    try_publish("version/json", version.dump(), true);
    if(external_mqtt_enable) {
        try {
            client_external_->publish(external_mqtt_topic_prefix + "version", version.dump(), 1, true);
            
        } catch (const mqtt::exception &e) {
            // client disconnected or something, we drop it.
        }
    }
    auto bson = json::to_bson(version);
    try_publish_binary("version", bson.data(), bson.size(), true);
}

void publish_capabilities() {
  try_publish("capabilities/json", CAPABILITIES.dump(2), true);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Wswitch-enum"
json xmlrpc_to_json(XmlRpc::XmlRpcValue value) {
    switch (value.getType()) {
        case XmlRpc::XmlRpcValue::TypeBoolean:
            return static_cast<bool>(value);
        case XmlRpc::XmlRpcValue::TypeInt:
            return static_cast<int>(value);
        case XmlRpc::XmlRpcValue::TypeDouble:
            return static_cast<double>(value);
        case XmlRpc::XmlRpcValue::TypeString:
            return static_cast<std::string>(value);
        case XmlRpc::XmlRpcValue::TypeArray: {
            json arr = json::array();
            for (int i = 0; i < value.size(); ++i)
                arr.push_back(xmlrpc_to_json(value[i]));
            return arr;
        }
        case XmlRpc::XmlRpcValue::TypeStruct: {
            json obj = json::object();
            for (auto it = value.begin(); it != value.end(); ++it)
                obj[it->first] = xmlrpc_to_json(it->second);
            return obj;
        }
        case XmlRpc::XmlRpcValue::TypeDateTime: {
            const struct tm& t = static_cast<const struct tm&>(value);
            char buf[32];
            std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &t);
            return std::string(buf);
        }
        case XmlRpc::XmlRpcValue::TypeBase64: {
            const XmlRpc::XmlRpcValue::BinaryData& data = static_cast<const XmlRpc::XmlRpcValue::BinaryData&>(value);
            static const char* b64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
            std::string out;
            out.reserve(((data.size() + 2) / 3) * 4);
            for (size_t i = 0; i < data.size(); i += 3) {
                unsigned int n = (static_cast<unsigned char>(data[i]) << 16)
                    | (i + 1 < data.size() ? static_cast<unsigned char>(data[i + 1]) << 8 : 0)
                    | (i + 2 < data.size() ? static_cast<unsigned char>(data[i + 2]) : 0);
                out += b64[(n >> 18) & 0x3F];
                out += b64[(n >> 12) & 0x3F];
                out += (i + 1 < data.size()) ? b64[(n >> 6) & 0x3F] : '=';
                out += (i + 2 < data.size()) ? b64[n & 0x3F] : '=';
            }
            return out;
        }
        case XmlRpc::XmlRpcValue::TypeInvalid:
            return nullptr;
    }
    return nullptr;
}
#pragma GCC diagnostic pop

void publish_params(bool only_if_changed) {
    std::vector<std::string> param_names;
    ros::param::getParamNames(param_names);
    std::sort(param_names.begin(), param_names.end());

    json params = json::object();
    for (const auto &name : param_names) {
        if (name.find("password") != std::string::npos) {
            params[name] = nullptr;
            continue;
        }
        XmlRpc::XmlRpcValue value;
        if (ros::param::get(name, value)) {
            params[name] = xmlrpc_to_json(value);
        }
    }
    const std::string payload = params.dump();
    if (only_if_changed && payload == params_payload) {
        return;
    }
    params_payload = payload;
    try_publish("params/json", payload, true);
}

// The value of a param in a dynamic_reconfigure config. A python node puts it in the list of its python type, so all
// of them are searched.
json reconfigure_value(const dynamic_reconfigure::Config &config, const std::string &name) {
    for (const auto &p : config.bools) {
        if (p.name == name) return static_cast<bool>(p.value);
    }
    for (const auto &p : config.ints) {
        if (p.name == name) return p.value;
    }
    for (const auto &p : config.doubles) {
        if (p.name == name) return p.value;
    }
    for (const auto &p : config.strs) {
        if (p.name == name) return p.value;
    }
    return nullptr;
}

// The node would ignore an unknown name or a value of the wrong type, it only logs an error and answers as if all went
// well. Values out of range are left to the node, it clamps them.
void add_reconfigure_value(dynamic_reconfigure::Config &config,
                           const dynamic_reconfigure::ConfigDescription &description, const std::string &name,
                           const nlohmann::basic_json<> &value) {
    using dynamic_reconfigure::ConfigTools;

    const std::string param_name = name.substr(name.rfind('/') + 1);
    const dynamic_reconfigure::ParamDescription *param = nullptr;
    for (const auto &group : description.groups) {
        for (const auto &p : group.parameters) {
            if (p.name == param_name) param = &p;
        }
    }
    if (param == nullptr) {
        throw xbot_mqtt::RpcException(xbot_mqtt::RpcError::ERROR_INVALID_PARAMS, "Unknown param " + name);
    }

    if (param->type == "bool" && value.is_boolean()) {
        ConfigTools::appendParameter(config, param_name, value.get<bool>());
    } else if (param->type == "int" && value.is_number() && std::floor(value.get<double>()) == value.get<double>() &&
               value.get<double>() >= INT_MIN && value.get<double>() <= INT_MAX) {
        ConfigTools::appendParameter(config, param_name, value.get<int>());
    } else if (param->type == "double" && value.is_number()) {
        ConfigTools::appendParameter(config, param_name, value.get<double>());
    } else if (param->type == "str" && value.is_string()) {
        ConfigTools::appendParameter(config, param_name, value.get<std::string>());
    } else {
        throw xbot_mqtt::RpcException(xbot_mqtt::RpcError::ERROR_INVALID_PARAMS,
                                      name + " must be of type " + param->type);
    }
}

// params.set: {"params": {"/mower_logic/rain_mode": 1, ...}}, full names like in params/json, for nodes with a
// dynamic_reconfigure server. Nothing is sent unless all values fit, then each node gets one set_parameters call. Like
// with dynparam, the values aren't stored anywhere. Returns what the nodes applied.
json set_params(const nlohmann::basic_json<> &request) {
    // no other options yet, one added later (like persisting) must not be ignored by this version
    if (!request.is_object() || request.size() != 1 || !request.contains("params") || !request["params"].is_object() ||
        request["params"].empty()) {
        throw xbot_mqtt::RpcException(xbot_mqtt::RpcError::ERROR_INVALID_PARAMS,
                                      "Expected {\"params\": {name: value, ...}}");
    }
    const auto &params = request["params"];

    std::map<std::string, dynamic_reconfigure::Reconfigure> requests;
    {
        std::lock_guard<std::mutex> lk(param_descriptions_mutex);
        for (const auto &item : params.items()) {
            const size_t slash = item.key().rfind('/');
            if (slash == std::string::npos || slash == 0) {
                throw xbot_mqtt::RpcException(xbot_mqtt::RpcError::ERROR_INVALID_PARAMS,
                                              "Expected a full name like /mower_logic/rain_mode: " + item.key());
            }
            const std::string ns = item.key().substr(0, slash);
            const auto it = param_descriptions.find(ns);
            if (it == param_descriptions.end()) {
                throw xbot_mqtt::RpcException(xbot_mqtt::RpcError::ERROR_INVALID_PARAMS,
                                              "No dynamic_reconfigure server found for " + item.key());
            }
            add_reconfigure_value(requests[ns].request.config, *it->second, item.key(), item.value());
        }
    }

    // a node that has gone away would only fail after the ones before it are set
    for (const auto &[ns, _] : requests) {
        if (!ros::service::exists(ns + "/set_parameters", false)) {
            throw xbot_mqtt::RpcException(xbot_mqtt::RpcError::ERROR_INVALID_PARAMS,
                                          "No dynamic_reconfigure server running for " + ns);
        }
    }

    for (auto &[ns, srv] : requests) {
        ROS_INFO_STREAM("Setting params of " << ns << " via RPC");
        if (!ros::service::call(ns + "/set_parameters", srv)) {
            throw xbot_mqtt::RpcException(xbot_mqtt::RpcError::ERROR_INTERNAL,
                                          "Calling " + ns + "/set_parameters failed");
        }
    }

    json result = json::object();
    for (const auto &item : params.items()) {
        const size_t slash = item.key().rfind('/');
        const auto &applied = requests[item.key().substr(0, slash)].response.config;
        result[item.key()] = reconfigure_value(applied, item.key().substr(slash + 1));
    }
    return result;
}

// Runs on its own queue and thread, see main()
// clang-format off
xbot_mqtt::RpcProvider params_rpc_provider("xbot_monitoring_params", {{
    RPC_METHOD("params.set", {
        return set_params(params);
    }),
}});
// clang-format on

void publish_sensor_metadata() {
    json sensor_info;
    {
        std::unique_lock<std::mutex> lk(found_sensors_mutex);

        if(found_sensors.empty())
            return;

        for (const auto &kv: found_sensors) {
            json info;
            info["sensor_id"] = kv.second.sensor_id;
            info["sensor_name"] = kv.second.sensor_name;

            switch (kv.second.value_type) {
                case xbot_msgs::SensorInfo::TYPE_STRING: {
                    info["value_type"] = "STRING";
                    break;
                }
                case xbot_msgs::SensorInfo::TYPE_DOUBLE: {
                    info["value_type"] = "DOUBLE";
                    break;
                }
                default: {
                    info["value_type"] = "UNKNOWN";
                    break;
                }


            }

            switch (kv.second.value_description) {
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE: {
                    info["value_description"] = "TEMPERATURE";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VELOCITY: {
                    info["value_description"] = "VELOCITY";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_ACCELERATION: {
                    info["value_description"] = "ACCELERATION";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VOLTAGE: {
                    info["value_description"] = "VOLTAGE";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_CURRENT: {
                    info["value_description"] = "CURRENT";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_PERCENT: {
                    info["value_description"] = "PERCENT";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_RPM: {
                    info["value_description"] = "REVOLUTIONS";
                    break;
                }
                default: {
                    info["value_description"] = "UNKNOWN";
                    break;
                }
            }

            info["unit"] = kv.second.unit;
            info["has_min_max"] = kv.second.has_min_max;
            info["min_value"] = kv.second.min_value;
            info["max_value"] = kv.second.max_value;
            info["has_critical_low"] = kv.second.has_critical_low;
            info["lower_critical_value"] = kv.second.lower_critical_value;
            info["has_critical_high"] = kv.second.has_critical_high;
            info["upper_critical_value"] = kv.second.upper_critical_value;
            sensor_info.push_back(info);
        }
    }
    try_publish("sensor_infos/json", sensor_info.dump(), true);
    json data;
    data["d"] = sensor_info;
    auto bson = json::to_bson(data);
    try_publish_binary("sensor_infos/bson", bson.data(), bson.size(), true);
}

void subscribe_to_sensor(std::string topic, std::vector<ros::Subscriber> &sensor_data_subscribers) {
    xbot_msgs::SensorInfo sensor;
    {
        std::unique_lock<std::mutex> lk(found_sensors_mutex);
        sensor = found_sensors[topic];
    }

    ROS_INFO_STREAM("Subscribing to sensor data for sensor with name: " << sensor.sensor_name);

    std::string data_topic = "xbot_monitoring/sensors/" + sensor.sensor_id + "/data";

    switch (sensor.value_type) {
        case xbot_msgs::SensorInfo::TYPE_DOUBLE: {
            ros::Subscriber s = n->subscribe<xbot_msgs::SensorDataDouble>(data_topic, 10, [info = sensor](
                    const xbot_msgs::SensorDataDouble::ConstPtr &msg) {
                try_publish("sensors/" + info.sensor_id + "/data", std::to_string(msg->data));

                json data;
                data["d"] = msg->data;
                auto bson = json::to_bson(data);
                try_publish_binary("sensors/" + info.sensor_id + "/bson", bson.data(), bson.size());
            });
            sensor_data_subscribers.push_back(s);
            break;
        }
        case xbot_msgs::SensorInfo::TYPE_STRING: {
            ros::Subscriber s = n->subscribe<xbot_msgs::SensorDataString>(data_topic, 10, [info = sensor](
                    const xbot_msgs::SensorDataString::ConstPtr &msg) {
                try_publish("sensors/" + info.sensor_id + "/data", msg->data);

                json data;
                data["d"] = msg->data;
                auto bson = json::to_bson(data);
                try_publish_binary("sensors/" + info.sensor_id + "/bson", bson.data(), bson.size());
            });
            sensor_data_subscribers.push_back(s);
            break;
        }
        default: {
            ROS_ERROR_STREAM("Invalid Sensor Data Type: " << (int) sensor.value_type);
        }
    }
}

void robot_state_callback(const xbot_msgs::RobotState::ConstPtr &msg) {
    // Build a JSON and publish it
    json j;

    j["battery_percentage"] = msg->battery_percentage;
    j["gps_percentage"] = msg->gps_percentage;
    j["current_action_progress"] = msg->current_action_progress;
    j["current_state"] = msg->current_state;
    j["current_sub_state"] = msg->current_sub_state;
    j["current_area"] = msg->current_area;
    j["current_path"] = msg->current_path;
    j["current_path_index"] = msg->current_path_index;
    j["emergency"] = msg->emergency;
    j["is_charging"] = msg->is_charging;
    j["rain_detected"] = msg->rain_detected;
    j["pose"]["x"] = msg->robot_pose.pose.pose.position.x;
    j["pose"]["y"] = msg->robot_pose.pose.pose.position.y;
    j["pose"]["heading"] = msg->robot_pose.vehicle_heading;
    j["pose"]["pos_accuracy"] = msg->robot_pose.position_accuracy;
    j["pose"]["heading_accuracy"] = msg->robot_pose.orientation_accuracy;
    j["pose"]["heading_valid"] = msg->robot_pose.orientation_valid;

    try_publish("robot_state/json", j.dump());
    json data;
    data["d"] = j;
    auto bson = json::to_bson(data);
    try_publish_binary("robot_state/bson", bson.data(), bson.size());
}

struct PoseSample {
    double x, y, heading;
};
std::vector<PoseSample> pose_buffer;
std::mutex pose_buffer_mutex;

void pose_callback(const xbot_msgs::AbsolutePose::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk(pose_buffer_mutex);
  pose_buffer.push_back({msg->pose.pose.position.x, msg->pose.pose.position.y, msg->vehicle_heading});
}

void pose_publish_timer_callback(const ros::TimerEvent&) {
    std::vector<PoseSample> buf;
    {
        std::lock_guard<std::mutex> lk(pose_buffer_mutex);
        if (pose_buffer.empty()) return;
        buf.swap(pose_buffer);
    }

    const size_t n = buf.size();
    const size_t mid = n / 2;

    std::vector<double> xs(n), ys(n), hs(n);
    for (size_t i = 0; i < n; ++i) {
        xs[i] = buf[i].x;
        ys[i] = buf[i].y;
        hs[i] = buf[i].heading;
    }
    std::nth_element(xs.begin(), xs.begin() + mid, xs.end());
    std::nth_element(ys.begin(), ys.begin() + mid, ys.end());
    std::nth_element(hs.begin(), hs.begin() + mid, hs.end());

    position_history.addPoint(xs[mid], ys[mid]);

    const auto attrs = position_history.getAttributes();
    const json j = {
        {"x", xs[mid]},
        {"y", ys[mid]},
        {"heading", hs[mid]},
        {"attributes", attrs},
    };
    try_publish("position/json", j.dump());
}

void position_history_flush_timer_callback(const ros::TimerEvent&) {
  position_history.periodicFlush();
}

void mqtt_publish_callback(const xbot_mqtt::MqttPublish::ConstPtr& msg) {
    try_publish(msg->topic, msg->payload, msg->retain);

    if (xbot_mqtt::isEvent(msg->topic)) {
        event_history.add(msg->payload);
        try {
          const json event = json::parse(msg->payload);
          position_history.onEvent(event);
        } catch (const json::exception& e) {
            ROS_WARN_STREAM("mqtt_publish_callback: failed to parse event JSON: " << e.what());
        }
    }
}

void publish_actions() {
    json actions = json::array();
    {
        std::lock_guard<std::mutex> lk(registered_actions_mutex);
        for(const auto &kv : registered_actions) {
            for(const auto &action : kv.second) {
                json action_info;
                action_info["action_id"] = kv.first + "/" + action.action_id;
                action_info["action_name"] = action.action_name;
                action_info["enabled"] = action.enabled;
                actions.push_back(action_info);
            }
        }
    }

    try_publish("actions/json", actions.dump(), true);
    json data;
    data["d"] = actions;

    auto bson = json::to_bson(data);
    try_publish_binary("actions/bson", bson.data(), bson.size(), true);
}

void publish_map() {
    json m;
    {
        std::lock_guard<std::mutex> lk(map_mutex);
        if(!has_map)
            return;
        m = map;
    }
    try_publish("map/json", m.dump(2), true);
    json data;
    data["d"] = m;
    auto bson = json::to_bson(data);
    try_publish_binary("map/bson", bson.data(), bson.size(), true);
}

void publish_map_overlay() {
    json m;
    {
        std::lock_guard<std::mutex> lk(map_overlay_mutex);
        if(!has_map_overlay)
            return;
        m = map_overlay;
    }
    try_publish("map_overlay/json", m.dump(), true);
    json data;
    data["d"] = m;
    auto bson = json::to_bson(data);
    try_publish_binary("map_overlay/bson", bson.data(), bson.size(), true);
}

void map_callback(const std_msgs::String::ConstPtr &msg) {
    try {
        json m = json::parse(msg->data);
        {
            std::lock_guard<std::mutex> lk(map_mutex);
            map = m;
            has_map = true;
        }
        publish_map();
    } catch (const json::exception &e) {
        ROS_ERROR_STREAM("Error processing map JSON: " << e.what());
    }
}


void map_overlay_callback(const xbot_msgs::MapOverlay::ConstPtr &msg) {
    // Build a JSON and publish it

    json polys;
    for(const auto &poly : msg->polygons) {
        if(poly.polygon.points.size() < 2)
            continue;
        json poly_j;
        {
            json outline_poly_j;
            for (const auto &pt: poly.polygon.points) {
                json p_j;
                p_j["x"] = pt.x;
                p_j["y"] = pt.y;
                outline_poly_j.push_back(p_j);
            }
            poly_j["poly"] = outline_poly_j;
            poly_j["is_closed"] = poly.closed;
            poly_j["line_width"] = poly.line_width;
            poly_j["color"] = poly.color;
        }
        polys.push_back(poly_j);
    }

    json j;
    j["polygons"] = polys;
    {
        std::lock_guard<std::mutex> lk(map_overlay_mutex);
        map_overlay = j;
        has_map_overlay = true;
    }

    publish_map_overlay();
}


bool registerActions(xbot_msgs::RegisterActionsSrvRequest &req, xbot_msgs::RegisterActionsSrvResponse &res) {

    ROS_INFO_STREAM("new actions registered: " << req.node_prefix << " registered " << req.actions.size() << " actions.");

    {
        std::lock_guard<std::mutex> lk(registered_actions_mutex);
        registered_actions[req.node_prefix] = req.actions;
    }

    publish_actions();
    return true;
}

void rpc_publish_error(const int16_t code, const std::string &message, const nlohmann::basic_json<> &id = nullptr) {
    json err_resp = {{"jsonrpc", "2.0"},
                       {"error", {{"code", code}, {"message", message}}},
                       {"id", id}};
    try_publish("rpc/response", err_resp.dump(2));
}

// deepest nesting of [ and { in a json text, strings skipped. only exact for valid json, anything else doesn't get
// past the parser anyway
size_t json_nesting(const std::string &text) {
    size_t depth = 0, deepest = 0;
    bool in_string = false;
    for (size_t i = 0; i < text.size(); i++) {
        const char c = text[i];
        if (in_string) {
            if (c == '\\') i++;
            else if (c == '"') in_string = false;
        } else if (c == '"') {
            in_string = true;
        } else if (c == '[' || c == '{') {
            deepest = std::max(deepest, ++depth);
        } else if ((c == ']' || c == '}') && depth > 0) {
            depth--;
        }
    }
    return deepest;
}

void rpc_request_callback(const std::string &payload) {
    // a request nested some 100k levels deep overflowed the stack when it was copied or written again, no rpc needs
    // more than a few
    if (json_nesting(payload) > 100) {
        return rpc_publish_error(xbot_mqtt::RpcError::ERROR_INVALID_REQUEST, "Request is nested too deep");
    }

    // Parse
    json req;
    try {
      req = json::parse(payload);
    } catch (const json::exception &e) {
      // not only parse_error, e.g. a number too large for a double (1e400) is an out_of_range and would end the node
      return rpc_publish_error(xbot_mqtt::RpcError::ERROR_INVALID_JSON, "Could not parse request JSON");
    }

    // Validate
    if (!req.is_object()) {
        return rpc_publish_error(xbot_mqtt::RpcError::ERROR_INVALID_REQUEST, "Request is not a JSON object");
    }
    json id = req.contains("id") ? req["id"] : nullptr;
    if (id != nullptr && !id.is_string()) {
        return rpc_publish_error(xbot_mqtt::RpcError::ERROR_INVALID_REQUEST, "ID is not a string", id);
    } else if (!req.contains("jsonrpc") || !req["jsonrpc"].is_string() || req["jsonrpc"] != "2.0") {
        return rpc_publish_error(xbot_mqtt::RpcError::ERROR_INVALID_REQUEST, "Invalid JSON-RPC version");
    } else if (!req.contains("method") || !req["method"].is_string()) {
        return rpc_publish_error(xbot_mqtt::RpcError::ERROR_INVALID_REQUEST, "Method is not a string", req["id"]);
    }

    // Check if the method is registered
    const std::string method = req["method"];
    if (method.compare(0, 5, "meta.") == 0 || method.compare(0, 4, "ext.") == 0) {
      // Silently ignore methods that are handled by other services.
      return;
    }
    bool is_registered = false;
    {
        std::lock_guard<std::mutex> lk(registered_methods_mutex);
        for (const auto& [_, method_ids] : registered_methods) {
            if (std::find(method_ids.begin(), method_ids.end(), method) != method_ids.end()) {
                is_registered = true;
                break;
            }
        }
    }
    if (!is_registered) {
        return rpc_publish_error(xbot_mqtt::RpcError::ERROR_METHOD_NOT_FOUND, "Method \"" + method + "\" not found", req["id"]);
    }

    // Forward to the providers as ROS message
    xbot_mqtt::RpcRequest msg;
    msg.method = method;
    msg.params = req.contains("params") ? req["params"].dump() : "";
    msg.id = id != nullptr ? id : "";
    rpc_request_pub.publish(msg);
}

void rpc_response_callback(const xbot_mqtt::RpcResponse::ConstPtr &msg) {
    json result;
    try {
        result = json::parse(msg->result);
    } catch (const json::exception &e) {
        return rpc_publish_error(xbot_mqtt::RpcError::ERROR_INTERNAL, "Internal error while parsing result JSON: " + std::string(e.what()), msg->id);
    }

    json j = {{"jsonrpc", "2.0"}, {"result", result}, {"id", msg->id}};
    try_publish("rpc/response", j.dump(2));
}

void rpc_error_callback(const xbot_mqtt::RpcError::ConstPtr &msg) {
    rpc_publish_error(msg->code, msg->message, msg->id);
}

bool register_methods(xbot_mqtt::RegisterMethodsSrvRequest &req, xbot_mqtt::RegisterMethodsSrvResponse &res) {
    std::lock_guard<std::mutex> lk(registered_methods_mutex);
    registered_methods[req.node_id] = req.methods;
    ROS_INFO_STREAM("new methods registered: " << req.node_id << " registered " << req.methods.size() << " methods.");
    return true;
}

void rosout_callback(const rosgraph_msgs::Log::ConstPtr &msg) {
    log_buffer.add(*msg);
}

void parameter_updates_callback(const dynamic_reconfigure::Config::ConstPtr &) {
    // params/json takes a call to the master for every param, it's built in the main loop and not on the spinner
    params_changed = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xbot_monitoring");
    has_map = false;
    has_map_overlay = false;


    n = new ros::NodeHandle();
    ros::NodeHandle paramNh("~");

    version_string = paramNh.param("software_version", std::string("UNKNOWN VERSION"));
    if(version_string.empty()) {
        version_string = "UNKNOWN VERSION";
    }

    event_history.init();
    position_history.init();

    external_mqtt_enable = paramNh.param("external_mqtt_enable", false);
    external_mqtt_topic_prefix = paramNh.param("external_mqtt_topic_prefix", std::string(""));
    if(!external_mqtt_topic_prefix.empty() && external_mqtt_topic_prefix.back() != '/') {
        // append the /
        external_mqtt_topic_prefix = external_mqtt_topic_prefix+"/";
    }

    external_mqtt_hostname = paramNh.param("external_mqtt_hostname", std::string(""));
    external_mqtt_port = std::to_string(paramNh.param("external_mqtt_port", 1883));
    external_mqtt_username = paramNh.param("external_mqtt_username", std::string(""));
    external_mqtt_password = paramNh.param("external_mqtt_password", std::string(""));

    if(external_mqtt_enable) {
        ROS_INFO_STREAM("Using external MQTT broker: " << external_mqtt_hostname << ":" << external_mqtt_port << " with topic prefix: " + external_mqtt_topic_prefix);
    }

    // First setup MQTT
    setupMqttClient();

    ros::ServiceServer register_action_service = n->advertiseService("xbot/register_actions", registerActions);

    ros::Subscriber robotStateSubscriber = n->subscribe("xbot_monitoring/robot_state", 10, robot_state_callback);
    ros::Subscriber mapSubscriber = n->subscribe("mower_map_service/json_map", 10, map_callback);
    ros::Subscriber mapOverlaySubscriber = n->subscribe("xbot_monitoring/map_overlay", 10, map_overlay_callback);
    ros::Subscriber poseSubscriber = n->subscribe("/xbot_positioning/xb_pose", 10, pose_callback);
    ros::Timer posePublishTimer = n->createTimer(ros::Duration(MQTT_POSITION_PUBLISH_INTERVAL), pose_publish_timer_callback);
    ros::Timer positionHistoryFlushTimer =
        n->createTimer(ros::Duration(POSITION_HISTORY_FLUSH_INTERVAL), position_history_flush_timer_callback);
    ros::Subscriber mqttPublishSubscriber = n->subscribe("/xbot_monitoring/mqtt_publish", 50, mqtt_publish_callback);
    ros::Subscriber rosoutSubscriber = n->subscribe("/rosout_agg", 100, rosout_callback);

    cmd_vel_pub = n->advertise<geometry_msgs::Twist>("xbot_monitoring/remote_cmd_vel", 1);
    action_pub = n->advertise<std_msgs::String>("xbot/action", 1);

    rpc_request_pub = n->advertise<xbot_mqtt::RpcRequest>(xbot_mqtt::TOPIC_REQUEST, 100);
    ros::Subscriber rpc_response_sub = n->subscribe(xbot_mqtt::TOPIC_RESPONSE, 100, rpc_response_callback);
    ros::Subscriber rpc_error_sub = n->subscribe(xbot_mqtt::TOPIC_ERROR, 100, rpc_error_callback);
    ros::ServiceServer register_methods_service = n->advertiseService(xbot_mqtt::SERVICE_REGISTER_METHODS, register_methods);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    rpc_provider.init();

    // params.set waits for the node to answer, and mower_logic may be calling xbot/register_actions (global queue)
    // before it gets to the request. Its own queue avoids that, and a node that doesn't answer holds up nothing else.
    ros::CallbackQueue params_rpc_queue;
    ros::NodeHandle params_rpc_nh;
    params_rpc_nh.setCallbackQueue(&params_rpc_queue);
    ros::AsyncSpinner params_rpc_spinner(1, &params_rpc_queue);
    params_rpc_spinner.start();
    params_rpc_provider.init(params_rpc_nh);

    publish_event("BOOTED");

    ros::Rate sensor_check_rate(10.0);

    boost::regex topic_regex("/xbot_monitoring/sensors/.*/info");

    // Maps a sensor info or dynamic_reconfigure topic to its subscriber. Only touched by this thread.
    std::map<std::string, ros::Subscriber> active_subscribers;
    std::vector<ros::Subscriber> sensor_data_subscribers;

    // When the last parameter update came in. params/json is published again once they stop for a second, after the
    // start every node sends one.
    ros::SteadyTime params_changed_at;

    while (ros::ok()) {
        // Read the topics in /xbot_monitoring/sensors/.*/info and the parameter descriptions of dynamic_reconfigure
        // servers and subscribe to them.
        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);
        std::for_each(topics.begin(), topics.end(), [&](const ros::master::TopicInfo &item) {
            if (item.datatype == "dynamic_reconfigure/ConfigDescription" && active_subscribers.count(item.name) == 0) {
                const std::string ns = item.name.substr(0, item.name.rfind('/'));
                ROS_INFO_STREAM("Found dynamic_reconfigure server " << ns);
                active_subscribers[item.name] = n->subscribe<dynamic_reconfigure::ConfigDescription>(
                    item.name, 1, [ns](const dynamic_reconfigure::ConfigDescription::ConstPtr &msg) {
                        std::lock_guard<std::mutex> lk(param_descriptions_mutex);
                        param_descriptions[ns] = msg;
                    });
                active_subscribers[ns + "/parameter_updates"] =
                    n->subscribe(ns + "/parameter_updates", 1, parameter_updates_callback);
                return;
            }

            if (!boost::regex_match(item.name, topic_regex) || active_subscribers.count(item.name) != 0)
                return;

            ROS_INFO_STREAM("Found new sensor topic " << item.name);
            active_subscribers[item.name] = n->subscribe<xbot_msgs::SensorInfo>(
                item.name, 1, [topic = item.name, &sensor_data_subscribers](const xbot_msgs::SensorInfo::ConstPtr &msg) {
                    ROS_INFO_STREAM("Got sensor info for sensor on topic " << msg->sensor_name << " on topic " << topic);

                    bool is_new = false;
                    {
                        std::unique_lock<std::mutex> lk(found_sensors_mutex);
                        is_new = found_sensors.count(topic) == 0;

                        // Sensor already known and sensor-info equals?
                        if (!is_new && found_sensors[topic] == *msg) return;

                        found_sensors[topic] = *msg;  // Save the (new|changed) sensor info
                    }

                    // Let the info subscription alive for dynamic threshold changes
                    //active_subscribers.erase(topic);  // Stop subscribing to infos

                    if (is_new) {
                        subscribe_to_sensor(topic, sensor_data_subscribers);  // Subscribe for data
                    }

                    // Republish (new|changed) sensor info
                    // NOTE: If a sensor name or id changes, the related data topic wouldn't change!
                    //       But do we dynamically change a sensor name or id?
                    publish_sensor_metadata();
                }
            );
        });

        if (params_changed.exchange(false)) {
            params_changed_at = ros::SteadyTime::now();
        }
        if (params_connected.exchange(false)) {
            params_changed_at = ros::SteadyTime();
            publish_params(false);
        } else if (!params_changed_at.isZero() && ros::SteadyTime::now() - params_changed_at > ros::WallDuration(1.0)) {
            params_changed_at = ros::SteadyTime();
            publish_params(true);
        }
        sensor_check_rate.sleep();
    }
    publish_event("SHUTDOWN");
    position_history.flush();
    return 0;
}
