//
// Created by Clemens Elflein on 15.03.22.
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

#include "boost/crc.hpp"
#include "std_msgs/Empty.h"
#include <mower_msgs/Status.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <serial/serial.h>
#include "ll_datatypes.h"
#include "COBS.h"
#include "std_msgs/Bool.h"
#include "mower_msgs/MowerControlSrv.h"
#include "mower_msgs/EmergencyStopSrv.h"
#include "mower_msgs/ImuRaw.h"

#include "vesc_driver/vesc_interface.h"


ros::Publisher status_pub;
ros::Publisher imu_pub;

COBS cobs;


// True, if ROS thinks there sould be an emergency
bool emergency_high_level = false;
// True, if the LL board thinks there should be an emergency
bool emergency_low_level = false;

// True, if the LL emergency should be cleared in the next request
bool ll_clear_emergency = false;

// True, if we can send to the low level board
bool allow_send = false;

// Current speeds (duty cycle) for the three ESCs
float speed_l = 0, speed_r = 0, speed_mow = 0;

// Serial port and buffer for the low level connection
serial::Serial serial_port;
uint8_t out_buf[1000];
ros::Time last_cmd_vel(0.0);

boost::crc_ccitt_type crc;


vesc_driver::VescInterface *mow_vesc_interface;
vesc_driver::VescInterface *left_vesc_interface;
vesc_driver::VescInterface *right_vesc_interface;

std::mutex ll_status_mutex;
struct ll_status last_ll_status = {0};

bool is_emergency() {
    return emergency_high_level || emergency_low_level;
}

void publishActuators() {
// emergency or timeout -> send 0 speeds
    if (is_emergency()) {
        speed_l = 0;
        speed_r = 0;
        speed_mow = 0;
    }
    if (ros::Time::now() - last_cmd_vel > ros::Duration(1.0)) {
        speed_l = 0;
        speed_r = 0;

    }
    if (ros::Time::now() - last_cmd_vel > ros::Duration(25.0)) {
        speed_l = 0;
        speed_r = 0;
        speed_mow = 0;
    }

    mow_vesc_interface->setDutyCycle(speed_mow);
    left_vesc_interface->setDutyCycle(speed_l);
    // We need to invert the speed, because the ESC has the same config as the left one, so the motor is running in the "wrong" direction
    right_vesc_interface->setDutyCycle(-speed_r);

    struct ll_heartbeat heartbeat = {
            .type = PACKET_ID_LL_HEARTBEAT,
            // If high level has emergency and LL does not know yet, we set it
            .emergency_requested = (!emergency_low_level && emergency_high_level),
            .emergency_release_requested = ll_clear_emergency
    };


    crc.reset();
    crc.process_bytes(&heartbeat, sizeof(struct ll_heartbeat) - 2);
    heartbeat.crc = crc.checksum();

    size_t encoded_size = cobs.encode((uint8_t *) &heartbeat, sizeof(struct ll_heartbeat), out_buf);
    out_buf[encoded_size] = 0;
    encoded_size++;

    if (serial_port.isOpen() && allow_send) {
        try {
            serial_port.write(out_buf, encoded_size);
        } catch (std::exception &e) {
            ROS_ERROR_STREAM("Error writing to serial port");
        }
    }
}


void convertStatus(vesc_driver::VescStatusStruct &vesc_status, mower_msgs::ESCStatus &ros_esc_status) {
    if (vesc_status.connection_state != vesc_driver::VESC_CONNECTION_STATE::CONNECTED &&
        vesc_status.connection_state != vesc_driver::VESC_CONNECTION_STATE::CONNECTED_INCOMPATIBLE_FW) {
        // ESC is disconnected
        ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
    } else if(vesc_status.fault_code) {
        // ESC has a fault
        ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_ERROR;
    } else if(abs(vesc_status.speed_erpm) > 500) {
        // ESC is running
        ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_RUNNING;
    } else {
        // ESC is OK but standing still
        ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    }
    ros_esc_status.tacho = vesc_status.tacho;
    ros_esc_status.current = vesc_status.current_motor;
    ros_esc_status.temperature_motor = vesc_status.temperature_motor;
    ros_esc_status.temperature_pcb = vesc_status.temperature_pcb;
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
    }

    // True, if high or low level emergency condition is present
    status_msg.emergency = is_emergency();

    status_msg.v_battery = last_ll_status.v_system;
    status_msg.v_charge = last_ll_status.v_charge;
    status_msg.charge_current = last_ll_status.charging_current;


    vesc_driver::VescStatusStruct mow_status, left_status, right_status;
    mow_vesc_interface->get_status(&mow_status);
    left_vesc_interface->get_status(&left_status);
    right_vesc_interface->get_status(&right_status);

    convertStatus(mow_status, status_msg.mow_esc_status);
    convertStatus(left_status, status_msg.left_esc_status);
    convertStatus(right_status, status_msg.right_esc_status);

    status_pub.publish(status_msg);
}

void publishActuatorsTimerTask(const ros::TimerEvent &timer_event) {
    publishActuators();
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

void velReceived(const geometry_msgs::Twist::ConstPtr &msg) {
    // TODO: update this to rad/s values and implement xESC speed control
    last_cmd_vel = ros::Time::now();
    speed_l = msg->linear.x - msg->angular.z;
    speed_r = msg->linear.x + msg->angular.z;

    if (speed_l >= 1.0) {
        speed_l = 1.0;
    } else if (speed_l <= -1.0) {
        speed_l = -1.0;
    }
    if (speed_r >= 1.0) {
        speed_r = 1.0;
    } else if (speed_r <= -1.0) {
        speed_r = -1.0;
    }
}


void handleLowLevelStatus(struct ll_status *status) {
    std::unique_lock<std::mutex> lk(ll_status_mutex);
    last_ll_status = *status;
}

void handleLowLevelIMU(struct ll_imu *imu) {
    mower_msgs::ImuRaw imu_msg;
    imu_msg.dt = imu->dt_millis;
    imu_msg.ax = imu->acceleration_mss[0];
    imu_msg.ay = imu->acceleration_mss[1];
    imu_msg.az = imu->acceleration_mss[2];
    imu_msg.gx = imu->gyro_rads[0];
    imu_msg.gy = imu->gyro_rads[1];
    imu_msg.gz = imu->gyro_rads[2];
    imu_msg.mx = imu->mag_uT[0];
    imu_msg.my = imu->mag_uT[1];
    imu_msg.mz = imu->mag_uT[2];

    imu_pub.publish(imu_msg);
}

void mowVescError(const std::string &error) {
    ROS_ERROR_STREAM("Mower VESC error: " << error);
}

void leftVescError(const std::string &error) {
    ROS_ERROR_STREAM("LEFT VESC error: " << error);
}

void rightVescError(const std::string &error) {
    ROS_ERROR_STREAM("RIGHT VESC error: " << error);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "mower_comms");


    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    std::string ll_serial_port_name, left_esc_port_name, right_esc_port_name, mow_esc_port_name;
    if (!paramNh.getParam("ll_serial_port", ll_serial_port_name)) {
        ROS_ERROR_STREAM("Error getting low level serial port parameter. Quitting.");
        return 1;
    }
    if (!paramNh.getParam("left_esc_serial_port", left_esc_port_name)) {
        ROS_ERROR_STREAM("Error getting left ESC serial port parameter. Quitting.");
        return 1;
    }
    if (!paramNh.getParam("right_esc_serial_port", right_esc_port_name)) {
        ROS_ERROR_STREAM("Error getting right ESC serial port parameter. Quitting.");
        return 1;
    }
    if (!paramNh.getParam("mow_esc_serial_port", mow_esc_port_name)) {
        ROS_ERROR_STREAM("Error getting mow ESC serial port parameter. Quitting.");
        return 1;
    }

    speed_l = speed_r = speed_mow = 0;


    // Setup VESC interfaces
    mow_vesc_interface = new vesc_driver::VescInterface(mowVescError);
    left_vesc_interface = new vesc_driver::VescInterface(leftVescError);
    right_vesc_interface = new vesc_driver::VescInterface(rightVescError);

    mow_vesc_interface->start(mow_esc_port_name);
    left_vesc_interface->start(left_esc_port_name);
    right_vesc_interface->start(right_esc_port_name);

    status_pub = n.advertise<mower_msgs::Status>("mower/status", 1);
    imu_pub = n.advertise<mower_msgs::ImuRaw>("mower/imu", 1);
    ros::ServiceServer mow_service = n.advertiseService("mower_service/mow_enabled", setMowEnabled);
    ros::ServiceServer emergency_service = n.advertiseService("mower_service/emergency", setEmergencyStop);
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 0, velReceived, ros::TransportHints().tcpNoDelay(true));
    ros::Timer publish_timer = n.createTimer(ros::Duration(0.02), publishActuatorsTimerTask);


    size_t buflen = 1000;
    uint8_t buffer[buflen];
    uint8_t buffer_decoded[buflen];
    size_t read = 0;
    // don't change, we need to wait for arduino to boot before actually sending stuff
    ros::Duration retryDelay(5, 0);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    while (ros::ok()) {
        if (!serial_port.isOpen()) {
            ROS_INFO_STREAM("connecting serial interface: " << ll_serial_port_name);
            allow_send = false;
            try {
                serial_port.setPort(ll_serial_port_name);
                serial_port.setBaudrate(115200);
                auto to = serial::Timeout::simpleTimeout(100);
                serial_port.setTimeout(to);
                serial_port.open();

                // wait for controller to boot
                retryDelay.sleep();
                // this will only be set if no error was set

                allow_send = true;
            } catch (std::exception &e) {
                retryDelay.sleep();
                ROS_ERROR_STREAM("Error during reconnect.");
            }
        }
        size_t bytes_read = 0;
        try {
            bytes_read = serial_port.read(buffer + read, 1);
        } catch (std::exception &e) {
            ROS_ERROR_STREAM("Error reading serial_port. Closing Connection.");
            serial_port.close();
            retryDelay.sleep();
        }
        if (read + bytes_read >= buflen) {
            read = 0;
            bytes_read = 0;
            ROS_ERROR_STREAM("Prevented buffer overflow. There is a problem with the serial comms.");
        }
        if (bytes_read) {
            if (buffer[read] == 0) {
                // end of packet found
                size_t data_size = cobs.decode(buffer, read, buffer_decoded);

                // first, check the CRC
                if (data_size < 3) {
                    // We don't even have one byte of data
                    // (type + crc = 3 bytes already)
                    ROS_INFO_STREAM("Got empty packet from Low Level Board");
                } else {
                    // We have at least 1 byte of data, check the CRC
                    crc.reset();
                    // We start at the second byte (ignore the type) and process (data_size- byte for type - 2 bytes for CRC) bytes.
                    crc.process_bytes(buffer_decoded, data_size - 2);
                    uint16_t checksum = crc.checksum();
                    uint16_t received_checksum = *(uint16_t *) (buffer_decoded + data_size - 2);
                    if (checksum == received_checksum) {
                        // Packet checksum is OK, process it
                        switch (buffer_decoded[0]) {
                            case PACKET_ID_LL_STATUS:
                                if (data_size == sizeof(struct ll_status)) {
                                    handleLowLevelStatus((struct ll_status *) buffer_decoded);
                                } else {
                                    ROS_INFO_STREAM(
                                            "Low Level Board sent a valid packet with the wrong size. Type was STATUS");
                                }
                                break;
                            case PACKET_ID_LL_IMU:
                                if (data_size == sizeof(struct ll_imu)) {
                                    handleLowLevelIMU((struct ll_imu *) buffer_decoded);
                                } else {
                                    ROS_INFO_STREAM(
                                            "Low Level Board sent a valid packet with the wrong size. Type was IMU");
                                }
                                break;
                            default:
                                ROS_INFO_STREAM("Got unknown packet from Low Level Board");
                                break;
                        }
                    } else {
                        ROS_INFO_STREAM("Got invalid checksum from Low Level Board");
                    }

                }

                read = 0;
            } else {
                read += bytes_read;
            }
        }
    }

    spinner.stop();

    mow_vesc_interface->setDutyCycle(0.0);
    left_vesc_interface->setDutyCycle(0.0);
    right_vesc_interface->setDutyCycle(0.0);

    mow_vesc_interface->stop();
    left_vesc_interface->stop();
    right_vesc_interface->stop();

    delete mow_vesc_interface;
    delete left_vesc_interface;
    delete right_vesc_interface;

    return 0;
}
