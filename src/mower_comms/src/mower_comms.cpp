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
#include "mower_msgs/HighLevelControlSrv.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#include <xesc_driver/xesc_driver.h>
#include <xesc_msgs/XescStateStamped.h>
#include <xbot_msgs/WheelTick.h>
#include "mower_msgs/HighLevelStatus.h"

#include <bitset>

ros::Publisher status_pub;
ros::Publisher wheel_tick_pub;

ros::Publisher sensor_imu_pub;
ros::Publisher sensor_mag_pub;

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

// Ticks / m and wheel distance for this robot
double wheel_ticks_per_m = 0.0;
double wheel_distance_m = 0.0;

bool dfp_is_5v = false;       // DFP is set to 5V Vcc
std::string language = "en";  // ISO-639-1 (2 char) language code
int volume = -1;              // -1 = don't change, 0-100 = volume (%)

// Serial port and buffer for the low level connection
serial::Serial serial_port;
uint8_t out_buf[1000];
ros::Time last_cmd_vel(0.0);

boost::crc_ccitt_type crc;

mower_msgs::HighLevelStatus last_high_level_status;

xesc_driver::XescDriver *mow_xesc_interface;
xesc_driver::XescDriver *left_xesc_interface;
xesc_driver::XescDriver *right_xesc_interface;

std::mutex ll_status_mutex;
struct ll_status last_ll_status = {0};

sensor_msgs::MagneticField sensor_mag_msg;
sensor_msgs::Imu sensor_imu_msg;

ros::ServiceClient highLevelClient;


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

    if(mow_xesc_interface) {
        mow_xesc_interface->setDutyCycle(speed_mow);
    }
    // We need to invert the speed, because the ESC has the same config as the left one, so the motor is running in the "wrong" direction
    left_xesc_interface->setDutyCycle(speed_l);
    right_xesc_interface->setDutyCycle(-speed_r);

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


void convertStatus(xesc_msgs::XescStateStamped &vesc_status, mower_msgs::ESCStatus &ros_esc_status) {
    if (vesc_status.state.connection_state != xesc_msgs::XescState::XESC_CONNECTION_STATE_CONNECTED &&
            vesc_status.state.connection_state != xesc_msgs::XescState::XESC_CONNECTION_STATE_CONNECTED_INCOMPATIBLE_FW) {
        // ESC is disconnected
        ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_DISCONNECTED;
    } else if(vesc_status.state.fault_code) {
        ROS_ERROR_STREAM_THROTTLE(1, "Motor controller fault code: " << vesc_status.state.fault_code);
        // ESC has a fault
        ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_ERROR;
    } else {
        // ESC is OK but standing still
        ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    }
    ros_esc_status.tacho = vesc_status.state.tacho;
    ros_esc_status.current = vesc_status.state.current_input;
    ros_esc_status.temperature_motor = vesc_status.state.temperature_motor;
    ros_esc_status.temperature_pcb = vesc_status.state.temperature_pcb;
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


    xesc_msgs::XescStateStamped mow_status, left_status, right_status;
    if(mow_xesc_interface) {
        mow_xesc_interface->getStatus(mow_status);
    } else {
        mow_status.state.connection_state = xesc_msgs::XescState::XESC_CONNECTION_STATE_DISCONNECTED;
    }
    left_xesc_interface->getStatus(left_status);
    right_xesc_interface->getStatus(right_status);

    convertStatus(mow_status, status_msg.mow_esc_status);
    convertStatus(left_status, status_msg.left_esc_status);
    convertStatus(right_status, status_msg.right_esc_status);

    status_pub.publish(status_msg);

    xbot_msgs::WheelTick wheel_tick_msg;
    wheel_tick_msg.wheel_tick_factor = static_cast<unsigned int>(wheel_ticks_per_m);
    wheel_tick_msg.stamp = status_msg.stamp;
    wheel_tick_msg.wheel_ticks_rl = left_status.state.tacho_absolute;
    wheel_tick_msg.wheel_direction_rl = left_status.state.direction && abs(left_status.state.duty_cycle) > 0;
    wheel_tick_msg.wheel_ticks_rr = right_status.state.tacho_absolute;
    wheel_tick_msg.wheel_direction_rr = !right_status.state.direction && abs(right_status.state.duty_cycle) > 0;

    wheel_tick_pub.publish(wheel_tick_msg);
}

void publishLowLevelConfig() {
    if (!serial_port.isOpen() || !allow_send)
        return;

    struct ll_high_level_config config_pkt;

    config_pkt.volume = volume;
    for (unsigned int i = 0; i < sizeof(config_pkt.language) / sizeof(char); i++) {
        config_pkt.language[i] = language[i];
    }
    // Set config_bitmask flags
    (dfp_is_5v) ? config_pkt.config_bitmask |= LL_HIGH_LEVEL_CONFIG_BIT_DFPIS5V : config_pkt.config_bitmask &= ~LL_HIGH_LEVEL_CONFIG_BIT_DFPIS5V;

    crc.reset();
    crc.process_bytes(&config_pkt, sizeof(struct ll_high_level_config) - 2);
    config_pkt.crc = crc.checksum();

    size_t encoded_size = cobs.encode((uint8_t *)&config_pkt, sizeof(struct ll_high_level_config), out_buf);
    out_buf[encoded_size] = 0;
    encoded_size++;

    try {
        ROS_INFO_STREAM("Send ll_high_level_config packet 0x" << std::hex << +config_pkt.type << " with comms_version=" << +config_pkt.comms_version
                                                              << ", config_bitmask=0b" << std::bitset<8>(config_pkt.config_bitmask)
                                                              << ", volume=" << std::dec << +config_pkt.volume << ", language='" << config_pkt.language << "'");
        serial_port.write(out_buf, encoded_size);
    } catch (std::exception &e) {
        ROS_ERROR_STREAM("Error writing to serial port");
    }
}

void publishActuatorsTimerTask(const ros::TimerEvent &timer_event) {
    publishActuators();
    publishStatus();
}

bool setMowEnabled(mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvResponse &res) {
    if (req.mow_enabled && !is_emergency()) {
        speed_mow = req.mow_direction ? 1 : -1;
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

void highLevelStatusReceived(const mower_msgs::HighLevelStatus::ConstPtr &msg) {
    struct ll_high_level_state hl_state = {
            .type = PACKET_ID_LL_HIGH_LEVEL_STATE,
            .current_mode = msg->state,
            .gps_quality = static_cast<uint8_t>(msg->gps_quality_percent*100.0)
    };


    crc.reset();
    crc.process_bytes(&hl_state, sizeof(struct ll_high_level_state) - 2);
    hl_state.crc = crc.checksum();

    size_t encoded_size = cobs.encode((uint8_t *) &hl_state, sizeof(struct ll_high_level_state), out_buf);
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

void velReceived(const geometry_msgs::Twist::ConstPtr &msg) {
    // TODO: update this to rad/s values and implement xESC speed control
    last_cmd_vel = ros::Time::now();
    speed_r = msg->linear.x + 0.5*wheel_distance_m*msg->angular.z;
    speed_l = msg->linear.x - 0.5*wheel_distance_m*msg->angular.z;

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

void handleLowLevelUIEvent(struct ll_ui_event *ui_event) {
    ROS_INFO_STREAM("Got UI button with code:" << +ui_event->button_id << " and duration: " << +ui_event->press_duration);

    mower_msgs::HighLevelControlSrv srv;

    switch(ui_event->button_id) {
        case 2:
            // Home
            srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_HOME;
            break;
        case 3:
            // Play
            srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_START;
            break;
        case 4:
            // S1
            srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_S1;
            break;
        case 5:
            // S2
            if(ui_event->press_duration == 2) {
                srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_DELETE_MAPS;
            } else {
                srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_S2;
            }
            break;
        case 6:
            //LOCK
            if(ui_event->press_duration == 2) {
                // very long press on lock
                srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_RESET_EMERGENCY;
            }
            break;
        default:
            // Return, don't call the service.
            return;
    }

    if(!highLevelClient.call(srv)) {
        ROS_ERROR_STREAM("Error calling high level control service");
    }

}

void handleLowLevelConfig(struct ll_high_level_config *config_pkt) {
    ROS_INFO_STREAM("Received ll_high_level_config packet 0x" << std::hex << +config_pkt->type
                                                              << " with comms_version=" << +config_pkt->comms_version
                                                              << ", config_bitmask=0b" << std::bitset<8>(config_pkt->config_bitmask)
                                                              << ", volume=" << std::dec << +config_pkt->volume << ", language='" << config_pkt->language << "'");

    // TODO: Handle announced comms_version once required

    // We're not interested in the received langauge setting (yet)
    
    // We're not interested in the received volume setting (yet)

    if (config_pkt->type == PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ ||                      // Config requested
        config_pkt->config_bitmask & LL_HIGH_LEVEL_CONFIG_BIT_DFPIS5V != dfp_is_5v) {  // Our DFP_IS_5V setting is leading
        publishLowLevelConfig();
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


    sensor_mag_msg.header.stamp = ros::Time::now();
    sensor_mag_msg.header.seq++;
    sensor_mag_msg.header.frame_id = "base_link";
    sensor_mag_msg.magnetic_field.x = imu_msg.mx/1000.0;
    sensor_mag_msg.magnetic_field.y = imu_msg.my/1000.0;
    sensor_mag_msg.magnetic_field.z = imu_msg.mz/1000.0;

    sensor_imu_msg.header.stamp = ros::Time::now();
    sensor_imu_msg.header.seq++;
    sensor_imu_msg.header.frame_id = "base_link";
    sensor_imu_msg.linear_acceleration.x = imu_msg.ax;
    sensor_imu_msg.linear_acceleration.y = imu_msg.ay;
    sensor_imu_msg.linear_acceleration.z = imu_msg.az;
    sensor_imu_msg.angular_velocity.x = imu_msg.gx;
    sensor_imu_msg.angular_velocity.y = imu_msg.gy;
    sensor_imu_msg.angular_velocity.z = imu_msg.gz;
 
    sensor_imu_pub.publish(sensor_imu_msg);
    sensor_mag_pub.publish(sensor_mag_msg);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "mower_comms");

    sensor_mag_msg.header.seq = 0;
    sensor_imu_msg.header.seq = 0;

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");
    ros::NodeHandle leftParamNh("~/left_xesc");
    ros::NodeHandle mowerParamNh("~/mower_xesc");
    ros::NodeHandle rightParamNh("~/right_xesc");



    highLevelClient = n.serviceClient<mower_msgs::HighLevelControlSrv>(
            "mower_service/high_level_control");


    std::string ll_serial_port_name;
    if (!paramNh.getParam("ll_serial_port", ll_serial_port_name)) {
        ROS_ERROR_STREAM("Error getting low level serial port parameter. Quitting.");
        return 1;
    }

    paramNh.getParam("wheel_ticks_per_m",wheel_ticks_per_m);
    paramNh.getParam("wheel_distance_m",wheel_distance_m);

    ROS_INFO_STREAM("Wheel ticks [1/m]: " << wheel_ticks_per_m);
    ROS_INFO_STREAM("Wheel distance [m]: " << wheel_distance_m);

    speed_l = speed_r = speed_mow = 0;

    paramNh.getParam("dfp_is_5v", dfp_is_5v);
    paramNh.getParam("language", language);
    paramNh.getParam("volume", volume);
    ROS_INFO_STREAM("DFP is set to 5V [boolean]: " << dfp_is_5v << ", language: '" << language << "', volume: " << volume);

    // Setup XESC interfaces
    if(mowerParamNh.hasParam("xesc_type")) {
        mow_xesc_interface = new xesc_driver::XescDriver(n, mowerParamNh);
    } else {
        mow_xesc_interface = nullptr;
    }

    left_xesc_interface = new xesc_driver::XescDriver(n, leftParamNh);
    right_xesc_interface = new xesc_driver::XescDriver(n, rightParamNh);


    status_pub = n.advertise<mower_msgs::Status>("mower/status", 1);
    wheel_tick_pub = n.advertise<xbot_msgs::WheelTick>("mower/wheel_ticks", 1);

    sensor_imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
    sensor_mag_pub = n.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
    ros::ServiceServer mow_service = n.advertiseService("mower_service/mow_enabled", setMowEnabled);
    ros::ServiceServer emergency_service = n.advertiseService("mower_service/emergency", setEmergencyStop);
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 0, velReceived, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber high_level_status_sub = n.subscribe("/mower_logic/current_state", 0, highLevelStatusReceived);
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
                            case PACKET_ID_LL_UI_EVENT:
                                if(data_size == sizeof(struct ll_ui_event)) {
                                    handleLowLevelUIEvent((struct ll_ui_event*) buffer_decoded);
                                } else {
                                    ROS_INFO_STREAM(
                                            "Low Level Board sent a valid packet with the wrong size. Type was UI_EVENT");
                                }
                                break;
                            case PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ:
                            case PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP:
                                if (data_size == sizeof(struct ll_high_level_config)) {
                                    handleLowLevelConfig((struct ll_high_level_config *)buffer_decoded);
                                } else {
                                    ROS_INFO_STREAM(
                                        "Low Level Board sent a valid packet with the wrong size. Type was CONFIG_* (0x" << std::hex << buffer_decoded[0] << ")");
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

    if(mow_xesc_interface) {
        mow_xesc_interface->setDutyCycle(0.0);
        mow_xesc_interface->stop();
    }
    left_xesc_interface->setDutyCycle(0.0);
    right_xesc_interface->setDutyCycle(0.0);
    left_xesc_interface->stop();
    right_xesc_interface->stop();

    if(mow_xesc_interface) {
        delete mow_xesc_interface;
    }
    delete left_xesc_interface;
    delete right_xesc_interface;

    return 0;
}
