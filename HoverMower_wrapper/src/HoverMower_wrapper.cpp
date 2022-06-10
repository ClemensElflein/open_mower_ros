//
// Created by Patrick Weber on 10.06.22.
// Copyright (c) 2022 Patrick Weber. All rights reserved.
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

#include <mower_msgs/Status.h>
#include "rosmower_msgs/Battery.h"
#include "rosmower_msgs/MowMotor.h"
//#include "<mower_msgs/HighLevelControlSrv.h>"



ros::Publisher status_pub;
ros::Subscriber battery_sub;
ros::Subscribermow_motor_sub;


// True, if ROS thinks there sould be an emergency
bool emergency_high_level = false;
// True, if the LL board thinks there should be an emergency
bool emergency_low_level = false;

// True, if the LL emergency should be cleared in the next request
bool ll_clear_emergency = false;


// Current speeds 
float speed_mow = 0;


ros::Time last_cmd_vel(0.0);

ros::ServiceClient highLevelClient;


bool is_emergency() {
    return emergency_high_level || emergency_low_level;
}

void batteryCallback(const rosmower_msgs::Battery::ConstPtr &msg)
{

}

void publishStatus() {
    mower_msgs::Status status_msg;
    status_msg.stamp = ros::Time::now();

    if (last_ll_status.status_bitmask & 1) {
        // LL OK, fill the message
        status_msg.mower_status = mower_msgs::Status::MOWER_STATUS_OK;
    } else {
        // LL initializing
        status_msg.mower_status = mower_msgs::Status::MOWER_STATUS_INITIALIZING;
    }

    status_msg.raspberry_pi_power = (last_ll_status.status_bitmask & 0b00000010) != 0;
    status_msg.gps_power = (last_ll_status.status_bitmask & 0b00000100) != 0;
    status_msg.esc_power = (last_ll_status.status_bitmask & 0b00001000) != 0;
    status_msg.rain_detected = (last_ll_status.status_bitmask & 0b00010000) != 0;
    status_msg.sound_module_available = (last_ll_status.status_bitmask & 0b00100000) != 0;
    status_msg.sound_module_busy = (last_ll_status.status_bitmask & 0b01000000) != 0;
    status_msg.ui_board_available = (last_ll_status.status_bitmask & 0b10000000) != 0;

    for (uint8_t i = 0; i < 5; i++) {
        status_msg.ultrasonic_ranges[i] = last_ll_status.uss_ranges_m[i];
    }

    // overwrite emergency with the LL value.
    emergency_low_level = last_ll_status.emergency_bitmask > 0;
    if (!emergency_low_level) {
        // it obviously worked, reset the request
        ll_clear_emergency = false;
    } else {
        ROS_ERROR_STREAM_THROTTLE(1, "Low Level Emergency. Bitmask was: " << (int)last_ll_status.emergency_bitmask);
    }

    // True, if high or low level emergency condition is present
    status_msg.emergency = is_emergency();

    status_msg.v_battery = last_ll_status.v_system;
    status_msg.v_charge = last_ll_status.v_charge;
    status_msg.charge_current = last_ll_status.charging_current;


    vesc_driver::VescStatusStruct mow_status;
    mow_vesc_interface->get_status(&mow_status);

    convertStatus(mow_status, status_msg.mow_esc_status);


    status_pub.publish(status_msg);
}

void publishActuatorsTimerTask(const ros::TimerEvent &timer_event) {
    publishStatus();
}

bool setMowEnabled(mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvResponse &res) {
    if (req.mow_enabled && !is_emergency()) {
        speed_mow = 1;
    } else {
        speed_mow = 0;
    }
    ROS_INFO_STREAM("Setting mow enabled to " << speed_mow);
    return true;
}

bool setEmergencyStop(mower_msgs::EmergencyStopSrvRequest &req, mower_msgs::EmergencyStopSrvResponse &res) {
    if (req.emergency) {
        ROS_ERROR_STREAM("Setting emergency!!");
        ll_clear_emergency = false;
    } else {
        ll_clear_emergency = true;
    }
    // Set the high level emergency instantly. Low level value will be set on next update.
    emergency_high_level = req.emergency;
    publishActuators();
    return true;
}


// void handleLowLevelUIEvent(struct ui_command *ui_command) {
//     ROS_INFO_STREAM("Got UI button with code:" << ui_command->cmd1);

//     mower_msgs::HighLevelControlSrv srv;

//     switch(ui_command->cmd1) {
//         case 2:
//             // Home
//             srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_HOME;
//             break;
//         case 3:
//             // Play
//             srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_START;
//             break;
//         case 4:
//             // S1
//             srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_S1;
//             break;
//         case 5:
//             // S2
//             srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_S2;
//             break;
//         default:
//             // Return, don't call the service.
//             return;
//     }

//     if(!highLevelClient.call(srv)) {
//         ROS_ERROR_STREAM("Error calling high level control service");
//     }

// }


void mowVescError(const std::string &error) {
    ROS_ERROR_STREAM("Mower VESC error: " << error);
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "HoverMower_wrapper");

    
    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    // highLevelClient = n.serviceClient<mower_msgs::HighLevelControlSrv>(
    //         "mower_service/high_level_control");

    speed_mow = 0;

    
    status_pub = n.advertise<mower_msgs::Status>("mower/status", 1);
    battery_sub = nh.subscribe("hovermower/sensors/Battery", 1000, batteryCallback);
    ros::ServiceServer mow_service = n.advertiseService("mower_service/mow_enabled", setMowEnabled);
    ros::ServiceServer emergency_service = n.advertiseService("mower_service/emergency", setEmergencyStop);
    ros::Timer publish_timer = n.createTimer(ros::Duration(0.02), publishActuatorsTimerTask);


    ros::AsyncSpinner spinner(1);
    spinner.start();
    while (ros::ok()) {
    
   
    }

    spinner.stop();
    return 0;
}
