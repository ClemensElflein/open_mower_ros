//
// Created by Clemens Elflein on 22.11.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "ros/ros.h"
#include "xbot_msgs/SensorInfo.h"
#include "xbot_msgs/SensorDataDouble.h"

xbot_msgs::SensorInfo my_info;

// Maps a topic to a subscriber.
std::map<std::string, ros::Subscriber> active_subscribers;
std::map<std::string, xbot_msgs::SensorInfo> found_sensors;


int main(int argc, char **argv) {
    ros::init(argc, argv, "xbot_sensor_example");

    my_info.has_critical_high = false;
    my_info.has_critical_low = false;
    my_info.has_min_max = true;
    my_info.min_value = -5;
    my_info.max_value = 105.0;
    my_info.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    my_info.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE;
    my_info.unit = "deg C";
    my_info.sensor_id = "some_temperature";
    my_info.sensor_name = "Some Temperature";


    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher sensor_info_publisher = n.advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + my_info.sensor_id + "/info", 1, true);
    ros::Publisher sensor_data_publisher = n.advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + my_info.sensor_id + "/data", 1, false);
    sensor_info_publisher.publish(my_info);

    xbot_msgs::SensorDataDouble data;

    ros::Rate sensorRate(1);
    int i = 0;
    while(ros::ok()) {
        // Generate some data here
        data.stamp = ros::Time::now();
        data.data = (sin((i++) / 10.0) + 0.5) * 50.0;

        // Publish the data
        sensor_data_publisher.publish(data);
        sensorRate.sleep();
    }

    return 0;
}
