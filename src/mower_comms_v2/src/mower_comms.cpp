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
#include <geometry_msgs/Twist.h>
#include <mower_msgs/Status.h>
#include <sensor_msgs/Joy.h>
#include <serial/serial.h>
#include <xbot_msgs/WheelTick.h>
#include <xesc_driver/xesc_driver.h>
#include <xesc_msgs/XescStateStamped.h>

#include <EmergencyServiceInterfaceBase.hpp>
#include <bitset>

#include "boost/crc.hpp"
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

ros::ServiceClient highLevelClient;

int main(int argc, char **argv) {
  ros::init(argc, argv, "mower_comms_v2");

  ros::NodeHandle n;
  ros::NodeHandle paramNh("~");

  highLevelClient = n.serviceClient<mower_msgs::HighLevelControlSrv>("mower_service/high_level_control");

  paramNh.getParam("wheel_ticks_per_m", wheel_ticks_per_m);
  paramNh.getParam("wheel_distance_m", wheel_distance_m);

  ROS_INFO_STREAM("Wheel ticks [1/m]: " << wheel_ticks_per_m);
  ROS_INFO_STREAM("Wheel distance [m]: " << wheel_distance_m);

  speed_l = speed_r = speed_mow = target_speed_mow = 0;

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
                if (data_size == sizeof(struct ll_high_level_config)) {
                  handleLowLevelConfig((struct ll_high_level_config *)buffer_decoded);
                } else {
                  ROS_INFO_STREAM("Low Level Board sent a valid packet with the wrong size. Type was CONFIG_* (0x"
                                  << std::hex << buffer_decoded[0] << ")");
                }
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
