//
// Created by Clemens Elflein on 15.03.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based
// on it without getting my consent first.
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
#include <dynamic_reconfigure/client.h>
#include <geometry_msgs/Twist.h>
#include <mower_msgs/Status.h>
#include <sensor_msgs/Joy.h>
#include <serial/serial.h>
#include <xbot_msgs/WheelTick.h>
#include <xesc_driver/xesc_driver.h>
#include <xesc_msgs/XescStateStamped.h>

#include <algorithm>
#include <bitset>

#include "COBS.h"
#include "boost/crc.hpp"
#include "ll_datatypes.h"
#include "mower_logic/MowerLogicConfig.h"
#include "mower_msgs/EmergencyStopSrv.h"
#include "mower_msgs/HighLevelControlSrv.h"
#include "mower_msgs/HighLevelStatus.h"
#include "mower_msgs/ImuRaw.h"
#include "mower_msgs/MowerControlSrv.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"

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
float speed_l = 0, speed_r = 0, speed_mow = 0, target_speed_mow = 0;

// Ticks / m and wheel distance for this robot
double wheel_ticks_per_m = 0.0;
double wheel_distance_m = 0.0;

// LL/HL configuration
struct ll_high_level_config llhl_config;

dynamic_reconfigure::Client<mower_logic::MowerLogicConfig> *reconfigClient;
mower_logic::MowerLogicConfig mower_logic_config;

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
  speed_mow = target_speed_mow;

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

  if (mow_xesc_interface) {
    mow_xesc_interface->setDutyCycle(speed_mow);
  }
  // We need to invert the speed, because the ESC has the same config as the left one, so the motor is running in the
  // "wrong" direction
  left_xesc_interface->setDutyCycle(speed_l);
  right_xesc_interface->setDutyCycle(-speed_r);

  struct ll_heartbeat heartbeat = {.type = PACKET_ID_LL_HEARTBEAT,
                                   // If high level has emergency and LL does not know yet, we set it
                                   .emergency_requested = (!emergency_low_level && emergency_high_level),
                                   .emergency_release_requested = ll_clear_emergency};

  crc.reset();
  crc.process_bytes(&heartbeat, sizeof(struct ll_heartbeat) - 2);
  heartbeat.crc = crc.checksum();

  size_t encoded_size = cobs.encode((uint8_t *)&heartbeat, sizeof(struct ll_heartbeat), out_buf);
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
  } else if (vesc_status.state.fault_code) {
    ROS_ERROR_STREAM_THROTTLE(1, "Motor controller fault code: " << vesc_status.state.fault_code);
    // ESC has a fault
    ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_ERROR;
  } else {
    // ESC is OK but standing still
    ros_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
  }
  ros_esc_status.tacho = vesc_status.state.tacho;
  ros_esc_status.rpm = vesc_status.state.rpm;
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
  status_msg.mow_enabled = !(target_speed_mow == 0);

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
  if (mow_xesc_interface) {
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

std::string getHallConfigsString(const HallConfig *hall_configs, const size_t size) {
  std::string str;

  // Parse hall_configs and build a readable string
  for (size_t i = 0; i < size; i++) {
    if (str.length()) str.append(", ");
    if (hall_configs->active_low) str.append("!");
    switch (hall_configs->mode) {
      case HallMode::OFF: str.append("I"); break;
      case HallMode::LIFT_TILT: str.append("L"); break;
      case HallMode::STOP: str.append("S"); break;
      case HallMode::UNDEFINED: str.append("U"); break;
      default: break;
    }
    hall_configs++;
  }

  return str;
}

void publishLowLevelConfig(const uint8_t pkt_type) {
  if (!serial_port.isOpen() || !allow_send) return;

  // Prepare the pkt
  size_t size = sizeof(struct ll_high_level_config) + 3;  // +1 type, +2 crc
  uint8_t buf[size];

  // Send config and request a config answer
  buf[0] = pkt_type;

  // Copy our live config into the message (behind type)
  memcpy(&buf[1], &llhl_config, sizeof(struct ll_high_level_config));

  // Member access to buffer
  struct ll_high_level_config *buf_config = (struct ll_high_level_config *)&buf[1];

  // CRC
  crc.reset();
  crc.process_bytes(buf, sizeof(struct ll_high_level_config) + 1);  // + type
  buf[size - 1] = (crc.checksum() >> 8) & 0xFF;
  buf[size - 2] = crc.checksum() & 0xFF;

  // COBS
  size_t encoded_size = cobs.encode(buf, size, out_buf);
  out_buf[encoded_size] = 0;
  encoded_size++;

  // Send
  try {
    // Let's be verbose for easier follow-up
    ROS_INFO(
        "Send ll_high_level_config packet %#04x\n"
        "\t options{dfp_is_5v=%d, background_sounds=%d, ignore_charging_current=%d},\n"
        "\t v_charge_cutoff=%f, i_charge_cutoff=%f,\n"
        "\t v_battery_cutoff=%f, v_battery_empty=%f, v_battery_full=%f,\n"
        "\t lift_period=%d, tilt_period=%d,\n"
        "\t shutdown_esc_max_pitch=%d,\n"
        "\t language=\"%.2s\", volume=%d\n"
        "\t hall_configs=\"%s\"",
        buf[0], (int)buf_config->options.dfp_is_5v, (int)buf_config->options.background_sounds,
        (int)buf_config->options.ignore_charging_current, buf_config->v_charge_cutoff, buf_config->i_charge_cutoff,
        buf_config->v_battery_cutoff, buf_config->v_battery_empty, buf_config->v_battery_full, buf_config->lift_period,
        buf_config->tilt_period, buf_config->shutdown_esc_max_pitch, buf_config->language, buf_config->volume,
        getHallConfigsString(buf_config->hall_configs, MAX_HALL_INPUTS).c_str());

    serial_port.write(out_buf, encoded_size);
  } catch (std::exception &e) {
    ROS_ERROR_STREAM("Error writing to serial port");
  }
}

/**
 * @brief A simple config tracker (struct-class) for managing lost response packets as well as simpler handling of
 * LowLevel reboots or flash period.
 */
struct {
  ros::Time last_config_req;    // Time when last config request was sent
  unsigned int tries_left = 0;  // Remaining request tries before giving up

  void ackResponse() {  // Call this on receive of a response packet to stop monitoring
    tries_left = 0;
  };
  void setDirty() {  // Call this for indicating that config packet need to be resend, i.e. die to LL-reboot
    tries_left = 5;
  };
  void check() {
    if (!tries_left ||                                            // No request tries left (probably old LL-FW)
        !serial_port.isOpen() || !allow_send ||                   // Serial not ready
        ros::Time::now() - last_config_req < ros::Duration(0.5))  // Timeout waiting for response not reached
      return;
    publishLowLevelConfig(PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ);
    last_config_req = ros::Time::now();
    tries_left--;
    ROS_WARN_STREAM_COND(
        !tries_left, "Didn't received a config packet from LowLevel in time. Is your LowLevel firmware up-to-date?");
  };
} configTracker;

void publishActuatorsTimerTask(const ros::TimerEvent &timer_event) {
  publishActuators();
  publishStatus();
  configTracker.check();
}

bool setMowEnabled(mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvResponse &res) {
  if (req.mow_enabled && !is_emergency()) {
    target_speed_mow = req.mow_direction ? 1 : -1;
  } else {
    target_speed_mow = 0;
  }
  ROS_INFO_STREAM("Setting mow enabled to " << target_speed_mow);
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
  struct ll_high_level_state hl_state = {.type = PACKET_ID_LL_HIGH_LEVEL_STATE,
                                         .current_mode = msg->state,
                                         .gps_quality = static_cast<uint8_t>(msg->gps_quality_percent * 100.0)};

  crc.reset();
  crc.process_bytes(&hl_state, sizeof(struct ll_high_level_state) - 2);
  hl_state.crc = crc.checksum();

  size_t encoded_size = cobs.encode((uint8_t *)&hl_state, sizeof(struct ll_high_level_state), out_buf);
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
  speed_r = msg->linear.x + 0.5 * wheel_distance_m * msg->angular.z;
  speed_l = msg->linear.x - 0.5 * wheel_distance_m * msg->angular.z;

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

  switch (ui_event->button_id) {
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
      if (ui_event->press_duration == 2) {
        srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_DELETE_MAPS;
      } else {
        srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_S2;
      }
      break;
    case 6:
      // LOCK
      if (ui_event->press_duration == 2) {
        // very long press on lock
        srv.request.command = mower_msgs::HighLevelControlSrvRequest::COMMAND_RESET_EMERGENCY;
      }
      break;
    default:
      // Return, don't call the service.
      return;
  }

  if (!highLevelClient.call(srv)) {
    ROS_ERROR_STREAM("Error calling high level control service");
  }
}

/**
 * @brief getNewSetChanged return t_new and checks if the value changed in comparison to t_cur.
 * t_new can't be a reference because the same function is also used for packed structures.
 * @param t_cur source value
 * @param t_new reference
 * @return &bool get set to true if t_cur and t_new differ, otherwise changed doesn't get touched
 */
template <typename T>
T getNewSetChanged(const T t_cur, const T t_new, bool &changed) {
  bool equal;
  if (std::is_floating_point<T>::value)
    equal = fabs(t_cur - t_new) < std::numeric_limits<T>::epsilon();
  else
    equal = t_cur == t_new;

  if (!equal) changed = true;

  //ROS_INFO_STREAM("DEBUG mower_comms comp. member: cur " << t_cur << " ?= " << t_new << " == equal " << equal << ", changed " << changed);

  return t_new;
}

/**
 * Handle config packet on receive from LL (LL->HL config packet response)
 */
void handleLowLevelConfig(const uint8_t *buffer, const size_t size) {
  // This is a flexible length packet where the size may vary when ll_high_level_config struct got enhanced only on one
  // side. If payload size is larger than our struct size, ensure that we only copy those we know of = our struct size.
  // If payload size is smaller than our struct size, copy only the payload we got, but ensure that the unsent member(s)
  // have reasonable defaults.
  size_t payload_size = std::min(sizeof(ll_high_level_config), size - 3);  // exclude type & crc

  // Copy payload to separated ll_config
  memcpy(&llhl_config, buffer + 1, payload_size);

  // Let's be verbose for easier follow-up
  ROS_INFO(
      "Received ll_high_level_config packet %#04x\n"
      "\t options{dfp_is_5v=%d, background_sounds=%d, ignore_charging_current=%d},\n"
      "\t v_charge_cutoff=%f, i_charge_cutoff=%f,\n"
      "\t v_battery_cutoff=%f, v_battery_empty=%f, v_battery_full=%f,\n"
      "\t lift_period=%d, tilt_period=%d,\n"
      "\t shutdown_esc_max_pitch=%d,\n"
      "\t language=\"%.2s\", volume=%d\n"
      "\t hall_configs=\"%s\"",
      *buffer, (int)llhl_config.options.dfp_is_5v, (int)llhl_config.options.background_sounds,
      (int)llhl_config.options.ignore_charging_current, llhl_config.v_charge_cutoff, llhl_config.i_charge_cutoff,
      llhl_config.v_battery_cutoff, llhl_config.v_battery_empty, llhl_config.v_battery_full, llhl_config.lift_period,
      llhl_config.tilt_period, llhl_config.shutdown_esc_max_pitch, llhl_config.language, llhl_config.volume,
      getHallConfigsString(llhl_config.hall_configs, MAX_HALL_INPUTS).c_str());

  // Inform config packet tracker about the response
  configTracker.ackResponse();

  // Copy received config values from LL to mower_logic's related dynamic reconfigure variables and
  // decide if mower_logic's dynamic reconfigure need to be updated with probably changed values
  bool dirty = false;
  // clang-format off
  mower_logic_config.cu_rain_threshold = getNewSetChanged<int>(mower_logic_config.cu_rain_threshold, llhl_config.rain_threshold, dirty);
  mower_logic_config.charge_critical_high_voltage = getNewSetChanged<double>(mower_logic_config.charge_critical_high_voltage, llhl_config.v_charge_cutoff, dirty);
  mower_logic_config.charge_critical_high_current = getNewSetChanged<double>(mower_logic_config.charge_critical_high_current, llhl_config.i_charge_cutoff, dirty);
  mower_logic_config.battery_critical_high_voltage = getNewSetChanged<double>(mower_logic_config.battery_critical_high_voltage, llhl_config.v_battery_cutoff, dirty);
  mower_logic_config.battery_empty_voltage = getNewSetChanged<double>(mower_logic_config.battery_empty_voltage, llhl_config.v_battery_empty, dirty);
  mower_logic_config.battery_full_voltage = getNewSetChanged<double>(mower_logic_config.battery_full_voltage, llhl_config.v_battery_full, dirty);
  mower_logic_config.emergency_lift_period = getNewSetChanged<int>(mower_logic_config.emergency_lift_period, llhl_config.lift_period, dirty);
  mower_logic_config.emergency_tilt_period = getNewSetChanged<int>(mower_logic_config.emergency_tilt_period, llhl_config.tilt_period, dirty);
  mower_logic_config.shutdown_esc_max_pitch = getNewSetChanged<int>(mower_logic_config.shutdown_esc_max_pitch, llhl_config.shutdown_esc_max_pitch, dirty);
  // clang-format on

  if (dirty) reconfigClient->setConfiguration(mower_logic_config);
}

void handleLowLevelStatus(struct ll_status *status) {
  static ros::Time last_ll_status_update(ros::Time::now());

  std::unique_lock<std::mutex> lk(ll_status_mutex);
  last_ll_status = *status;

  // LL status get send at 100ms cycle. If we miss 10 packets, we can assume that it got restarted or flashed with a
  // new FW. In either case we should ensure that it has the right config and update/re-align with us.
  if (ros::Time::now() - last_ll_status_update > ros::Duration(1.0)) configTracker.setDirty();
  last_ll_status_update = ros::Time::now();
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
  sensor_mag_msg.magnetic_field.x = imu_msg.mx / 1000.0;
  sensor_mag_msg.magnetic_field.y = imu_msg.my / 1000.0;
  sensor_mag_msg.magnetic_field.z = imu_msg.mz / 1000.0;

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

void reconfigCB(const mower_logic::MowerLogicConfig &config) {
  ROS_INFO_STREAM("mower_comms received new mower_logic config");

  mower_logic_config = config;

  // Copy changed mower_config's values to the related llhl_config values and
  // decide if LL need to be informed with a new config packet
  bool dirty = false;

  // clang-format off
  llhl_config.rain_threshold = getNewSetChanged<int>(llhl_config.rain_threshold, mower_logic_config.cu_rain_threshold, dirty);
  llhl_config.v_charge_cutoff = getNewSetChanged<double>(llhl_config.v_charge_cutoff, mower_logic_config.charge_critical_high_voltage, dirty);
  llhl_config.i_charge_cutoff = getNewSetChanged<double>(llhl_config.i_charge_cutoff, mower_logic_config.charge_critical_high_current, dirty);
  llhl_config.v_battery_cutoff = getNewSetChanged<double>(llhl_config.v_battery_cutoff, mower_logic_config.battery_critical_high_voltage, dirty);
  llhl_config.v_battery_empty = getNewSetChanged<double>(llhl_config.v_battery_empty, mower_logic_config.battery_empty_voltage, dirty);
  llhl_config.v_battery_full = getNewSetChanged<double>(llhl_config.v_battery_full, mower_logic_config.battery_full_voltage, dirty);
  llhl_config.lift_period = getNewSetChanged<int>(llhl_config.lift_period, mower_logic_config.emergency_lift_period, dirty);
  llhl_config.tilt_period = getNewSetChanged<int>(llhl_config.tilt_period, mower_logic_config.emergency_tilt_period, dirty);
  llhl_config.shutdown_esc_max_pitch = getNewSetChanged<int>(llhl_config.shutdown_esc_max_pitch, mower_logic_config.shutdown_esc_max_pitch, dirty);
  // clang-format on

  // Parse emergency_input_config and set hall_configs
  char *token = strtok(strdup(mower_logic_config.emergency_input_config.c_str()), ",");
  bool low_active;
  unsigned int hall_idx = 0;
  while (token != NULL) {
    low_active = false;
    while (*token != 0) {
      switch (std::toupper(*token)) {
        case '!': low_active = true; break;
        case 'I': llhl_config.hall_configs[hall_idx] = {HallMode::OFF, low_active}; break;
        case 'L': llhl_config.hall_configs[hall_idx] = {HallMode::LIFT_TILT, low_active}; break;
        case 'S': llhl_config.hall_configs[hall_idx] = {HallMode::STOP, low_active}; break;
        case 'U': llhl_config.hall_configs[hall_idx] = {HallMode::UNDEFINED, low_active}; break;
        default: break;
      }
      token++;
    }
    token = strtok(NULL, ",");
    hall_idx++;
  }

  if (dirty) configTracker.setDirty();
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

  highLevelClient = n.serviceClient<mower_msgs::HighLevelControlSrv>("mower_service/high_level_control");

  mower_logic_config = mower_logic::MowerLogicConfig::__getDefault__();
  reconfigClient = new dynamic_reconfigure::Client<mower_logic::MowerLogicConfig>("/mower_logic", reconfigCB);

  std::string ll_serial_port_name;
  if (!paramNh.getParam("ll_serial_port", ll_serial_port_name)) {
    ROS_ERROR_STREAM("Error getting low level serial port parameter. Quitting.");
    return 1;
  }

  paramNh.getParam("wheel_ticks_per_m", wheel_ticks_per_m);
  paramNh.getParam("wheel_distance_m", wheel_distance_m);

  ROS_INFO_STREAM("Wheel ticks [1/m]: " << wheel_ticks_per_m);
  ROS_INFO_STREAM("Wheel distance [m]: " << wheel_distance_m);

  speed_l = speed_r = speed_mow = target_speed_mow = 0;

  // Some generic settings from param server (non- dynamic)
  llhl_config.options.ignore_charging_current =
      paramNh.param("/mower_logic/ignore_charging_current", false) ? OptionState::ON : OptionState::OFF;
  llhl_config.options.dfp_is_5v = paramNh.param("dfp_is_5v", false) ? OptionState::ON : OptionState::OFF;
  llhl_config.volume = paramNh.param("volume", -1);
  llhl_config.options.background_sounds =
      paramNh.param("background_sounds", false) ? OptionState::ON : OptionState::OFF;
  // ISO-639-1 (2 char) language code
  strncpy(llhl_config.language, paramNh.param<std::string>("language", "en").c_str(), 2);

  // Setup XESC interfaces
  if (mowerParamNh.hasParam("xesc_type")) {
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
          // We start at the second byte (ignore the type) and process (data_size- byte for type - 2 bytes for CRC)
          // bytes.
          crc.process_bytes(buffer_decoded, data_size - 2);
          uint16_t checksum = crc.checksum();
          uint16_t received_checksum = *(uint16_t *)(buffer_decoded + data_size - 2);
          if (checksum == received_checksum) {
            // Packet checksum is OK, process it
            switch (buffer_decoded[0]) {
              case PACKET_ID_LL_STATUS:
                if (data_size == sizeof(struct ll_status)) {
                  handleLowLevelStatus((struct ll_status *)buffer_decoded);
                } else {
                  ROS_INFO_STREAM("Low Level Board sent a valid packet with the wrong size. Type was STATUS");
                }
                break;
              case PACKET_ID_LL_IMU:
                if (data_size == sizeof(struct ll_imu)) {
                  handleLowLevelIMU((struct ll_imu *)buffer_decoded);
                } else {
                  ROS_INFO_STREAM("Low Level Board sent a valid packet with the wrong size. Type was IMU");
                }
                break;
              case PACKET_ID_LL_UI_EVENT:
                if (data_size == sizeof(struct ll_ui_event)) {
                  handleLowLevelUIEvent((struct ll_ui_event *)buffer_decoded);
                } else {
                  ROS_INFO_STREAM("Low Level Board sent a valid packet with the wrong size. Type was UI_EVENT");
                }
                break;
              case PACKET_ID_LL_HIGH_LEVEL_CONFIG_REQ:
              case PACKET_ID_LL_HIGH_LEVEL_CONFIG_RSP:
                handleLowLevelConfig(buffer_decoded, data_size);
                break;
              default: ROS_INFO_STREAM("Got unknown packet from Low Level Board"); break;
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

  if (mow_xesc_interface) {
    mow_xesc_interface->setDutyCycle(0.0);
    mow_xesc_interface->stop();
  }
  left_xesc_interface->setDutyCycle(0.0);
  right_xesc_interface->setDutyCycle(0.0);
  left_xesc_interface->stop();
  right_xesc_interface->stop();

  if (mow_xesc_interface) {
    delete mow_xesc_interface;
  }
  delete left_xesc_interface;
  delete right_xesc_interface;

  return 0;
}
