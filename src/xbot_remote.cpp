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

// Publisher for cmd_vel and commands
ros::Publisher cmd_vel_pub;
ros::Publisher command_pub;

// Create a server endpoint
server echo_server;

// Define a callback to handle incoming messages
void on_message(server* s, websocketpp::connection_hdl hdl, message_ptr msg) {
    std::cout << "on_message called with hdl: " << hdl.lock().get()
              << " and message: " << msg->get_payload()
              << std::endl;
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
    } catch (...) {
        std::cout << "other exception" << std::endl;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xbot_remote");

    n = new ros::NodeHandle();
    ros::NodeHandle paramNh("~");

    cmd_vel_pub = n->advertise<geometry_msgs::Twist>("xbot_remote/remote_cmd_vel", 1);
    command_pub = n->advertise<std_msgs::String>("xbot_remote/remote_command", 1);


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
