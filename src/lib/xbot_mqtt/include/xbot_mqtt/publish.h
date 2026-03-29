#pragma once

#include <algorithm>
#include <random>
#include <string>

#include <ros/ros.h>
#include <nlohmann/json.hpp>
#include <xbot_mqtt/MqttPublish.h>

// Declares and advertises the MQTT publish publisher in the enclosing scope.
// Usage (once in main()): MQTT_PUBLISHER(*n);
// Creates variable: ros::Publisher mqtt_publish_pub
#define MQTT_PUBLISHER(nh) \
    ros::Publisher mqtt_publish_pub = \
        (nh).advertise<xbot_mqtt::MqttPublish>("/xbot_monitoring/mqtt_publish", 10)

namespace xbot_mqtt {

static const std::string EVENTS_TOPIC = "events/json";

// Same algorithm as mower_map_service generateNanoId()
inline std::string generateNanoId(size_t length = 32) {
    static const char alphabet[] =
        "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
    thread_local std::mt19937 rng{std::random_device{}()};
    thread_local std::uniform_int_distribution<> dist(0, sizeof(alphabet) - 2);
    std::string id(length, '\0');
    std::generate_n(id.begin(), length, [&]() { return alphabet[dist(rng)]; });
    return id;
}

// Generic publish to any MQTT topic. Payload is forwarded as-is.
inline void publish(ros::Publisher& pub,
                    const std::string& mqtt_topic,
                    const nlohmann::json& payload_json,
                    bool retain = false) {
    xbot_mqtt::MqttPublish msg;
    msg.topic   = mqtt_topic;
    msg.payload = payload_json.dump();
    msg.retain  = retain;
    pub.publish(msg);
}

// Convenience wrapper to publish events.
// Always injects "id" (NanoId), "t" (current ROS time, float seconds), and "type".
// Optional details object is merged into the payload for event-specific fields.
inline void publishEvent(ros::Publisher& pub,
                         const std::string& type,
                         nlohmann::json details = nullptr,
                         bool retain = false) {
    nlohmann::json payload = {
        {"id",   generateNanoId()},
        {"t",    ros::Time::now().toSec()},
        {"type", type},
    };
    if (details.is_object()) {
        payload.update(details);
    }
    publish(pub, EVENTS_TOPIC, payload, retain);
}

inline bool isEvent(const std::string& topic) {
    return topic == EVENTS_TOPIC;
}

}  // namespace xbot_mqtt
