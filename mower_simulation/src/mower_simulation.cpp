// Created by Clemens Elflein on 2/18/22, 5:37 PM.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
#include "ros/ros.h"

// Include messages for mower control
#include "mower_msgs/Status.h"
#include "mower_msgs/MowerControlSrv.h"
#include "mower_msgs/GPSControlSrv.h"
#include "mower_msgs/EmergencyStopSrv.h"
#include "geometry_msgs/Twist.h"

#include "dynamic_reconfigure/server.h"
#include "mower_simulation/MowerSimulationConfig.h"


ros::Publisher status_pub;
ros::Publisher cmd_vel_pub;

mower_msgs::Status fake_mow_status;
mower_simulation::MowerSimulationConfig config;
dynamic_reconfigure::Server<mower_simulation::MowerSimulationConfig> *reconfig_server;

geometry_msgs::Twist last_cmd_vel;


bool setGpsState(mower_msgs::GPSControlSrvRequest &req, mower_msgs::GPSControlSrvResponse &res) {
    return true;
}

bool setMowEnabled(mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvResponse &res) {
    config.mower_running = req.mow_enabled;
    reconfig_server->updateConfig(config);
    return true;
}

bool setEmergencyStop(mower_msgs::EmergencyStopSrvRequest &req, mower_msgs::EmergencyStopSrvResponse &res) {
    config.emergency_stop = req.emergency;
    reconfig_server->updateConfig(config);
    return true;
}

void publishStatus(const ros::TimerEvent &timer_event) {

    if (config.emergency_stop) {
        geometry_msgs::Twist emergency_twist;
        emergency_twist.linear.x = 0.0;
        emergency_twist.linear.y = 0.0;
        emergency_twist.linear.z = 0.0;
        emergency_twist.angular.x = 0.0;
        emergency_twist.angular.y = 0.0;
        emergency_twist.angular.z = 0.0;

        cmd_vel_pub.publish(emergency_twist);
    } else {
        cmd_vel_pub.publish(last_cmd_vel);
    }

    fake_mow_status.mow_esc_status = mower_msgs::Status::ESC_STATUS_OK;
    if (config.mower_error) {
        fake_mow_status.mow_esc_status = mower_msgs::Status::ESC_STATUS_ERROR;;
    }
    if (config.mower_running) {
        fake_mow_status.mow_esc_status = mower_msgs::Status::ESC_STATUS_RUNNING;
    }
    fake_mow_status.v_battery = config.battery_voltage;
    fake_mow_status.v_charge = config.is_charging ? config.battery_voltage + 0.2 : 0.0;
    if (config.wheels_stalled) {
        fake_mow_status.left_esc_status = mower_msgs::Status::ESC_STATUS_STALLED;
        fake_mow_status.right_esc_status = mower_msgs::Status::ESC_STATUS_STALLED;
    } else {
        fake_mow_status.left_esc_status = mower_msgs::Status::ESC_STATUS_OK;
        fake_mow_status.right_esc_status = mower_msgs::Status::ESC_STATUS_OK;
    }
    fake_mow_status.temperature_mow = config.temperature_mower * 10.0;


    status_pub.publish(fake_mow_status);
}

void reconfigureCB(mower_simulation::MowerSimulationConfig &c, uint32_t level) {
    config = c;
}

void velReceived(const geometry_msgs::Twist::ConstPtr &msg) {
    last_cmd_vel = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mower_simulation");


    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    reconfig_server = new dynamic_reconfigure::Server<mower_simulation::MowerSimulationConfig>(paramNh);
    reconfig_server->setCallback(reconfigureCB);

    fake_mow_status.mower_status = mower_msgs::Status::MOWER_STATUS_OK;
    fake_mow_status.v_charge = 0.0;
    fake_mow_status.v_battery = 29.0;
    fake_mow_status.stamp = ros::Time::now();
    fake_mow_status.left_esc_status = fake_mow_status.right_esc_status = fake_mow_status.mow_esc_status = mower_msgs::Status::ESC_STATUS_OK;
    fake_mow_status.temperature_mow = 500;

    status_pub = n.advertise<mower_msgs::Status>("mower/status", 1);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_out", 1);

    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 0, velReceived, ros::TransportHints().tcpNoDelay(true));
    ros::ServiceServer mow_service = n.advertiseService("mower_service/mow_enabled", setMowEnabled);
    ros::ServiceServer gps_service = n.advertiseService("mower_service/set_gps_state", setGpsState);
    ros::ServiceServer emergency_service = n.advertiseService("mower_service/emergency", setEmergencyStop);

    ros::Timer publish_timer = n.createTimer(ros::Duration(0.02), publishStatus);

    ros::spin();
    delete (reconfig_server);
    return 0;
}
