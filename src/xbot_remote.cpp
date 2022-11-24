//
// Created by Clemens Elflein on 22.11.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
#include <filesystem>

#include "ros/ros.h"
#include <memory>
#include <boost/regex.hpp>
#include "xbot_msgs/SensorInfo.h"
#include "xbot_msgs/Map.h"
#include "xbot_msgs/SensorDataString.h"
#include "xbot_msgs/SensorDataDouble.h"
#include "xbot_msgs/RobotState.h"
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "websocketpp/server.hpp"
#include <websocketpp/config/asio_no_tls.hpp>

typedef websocketpp::server<websocketpp::config::asio> server;
typedef server::message_ptr message_ptr;

using json = nlohmann::json;

ros::NodeHandle *n;

// Publisher for cmd_vel
ros::Publisher cmd_vel_pub;

// Create a server endpoint
server echo_server;

// Define a callback to handle incoming messages
void on_message(server* s, websocketpp::connection_hdl hdl, message_ptr msg) {
    try {
        json json = json::from_bson(msg->get_payload());

        ROS_INFO_STREAM_THROTTLE(0.5, "vx:" << json["vx"] << " vr: " << json["vz"]);
        geometry_msgs::Twist t;
        t.linear.x = json["vx"];
        t.angular.z = json["vz"];
        cmd_vel_pub.publish(t);
    } catch (std::exception &e) {
        ROS_ERROR_STREAM("Exception during remote decoding: " << e.what());
    }
}

void* server_thread(void* arg) {
    try {
        // Set logging settings
        echo_server.set_access_channels(websocketpp::log::alevel::all);
        echo_server.clear_access_channels(websocketpp::log::alevel::frame_payload);

        // Initialize Asio
        echo_server.init_asio();

        // Register our message handler
        echo_server.set_message_handler(bind(&on_message,&echo_server,::_1,::_2));

        echo_server.set_reuse_addr(true);

        // Listen on port 9002
        echo_server.listen(9002);

        // Start the server accept loop
        echo_server.start_accept();

        // Start the ASIO io_service run loop
        while(ros::ok()) {
            echo_server.run_one();
        }
    } catch (websocketpp::exception const & e) {
        std::cout << e.what() << std::endl;
        exit(1);
    } catch (...) {
        std::cout << "other exception" << std::endl;
        exit(1);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xbot_remote");

    n = new ros::NodeHandle();
    ros::NodeHandle paramNh("~");

    cmd_vel_pub = n->advertise<geometry_msgs::Twist>("xbot_remote/cmd_vel", 1);

    pthread_t server_thread_handle;
    pthread_create(&server_thread_handle, nullptr, &server_thread, nullptr);

    ros::spin();
    ROS_INFO_STREAM("Stopping websocket server");
    echo_server.stop();

    ROS_INFO_STREAM("Waiting for server thread to shutdown");
    pthread_join(server_thread_handle, nullptr);
    ROS_INFO_STREAM("Server shut down");

    return 0;
}
