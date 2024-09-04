// Created by Clemens Elflein on 3/28/22.
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

#include "mower_msgs/HighLevelStatus.h"
#include "mower_msgs/Status.h"
#include "ros/ros.h"
#include "xbot_msgs/AbsolutePose.h"
#include "xbot_msgs/RobotState.h"
#include "xbot_msgs/SensorDataDouble.h"
#include "xbot_msgs/SensorInfo.h"

ros::Publisher state_pub;
xbot_msgs::RobotState state;

// Sensor struct which hold all sensor relevant data
struct Sensor {
  std::string name;  // Speaking name, used in sensor widget
  std::string unit;  // Unit like A, V, ...
  uint8_t value_type;
  uint8_t value_desc;
  std::string param_path = "";  // Path to parameters
  xbot_msgs::SensorInfo si;     // SensorInfo Msg
  ros::Publisher si_pub;        // SensorInfo publisher
  ros::Publisher data_pub;      // Sensor-data publisher
};
// Place all sensors into a map, keyed by sensor.id
std::map<std::string, Sensor> sensors{
    {"om_v_charge", {"V Charge", "V", xbot_msgs::SensorInfo::TYPE_DOUBLE, xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VOLTAGE}},
    {"om_v_battery", {"V Battery", "V", xbot_msgs::SensorInfo::TYPE_DOUBLE, xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VOLTAGE}},
    {"om_charge_current", {"Charge Current", "A", xbot_msgs::SensorInfo::TYPE_DOUBLE, xbot_msgs::SensorInfo::VALUE_DESCRIPTION_CURRENT}},
    {"om_left_esc_temp", {"Left ESC Temp", "deg.C", xbot_msgs::SensorInfo::TYPE_DOUBLE, xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE, "left_xesc"}},
    {"om_right_esc_temp", {"Right ESC Temp", "deg.C", xbot_msgs::SensorInfo::TYPE_DOUBLE, xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE, "right_xesc"}},
    {"om_mow_esc_temp", {"Mow ESC Temp", "deg.C", xbot_msgs::SensorInfo::TYPE_DOUBLE, xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE, "mower_xesc"}},
    {"om_mow_motor_temp", {"Mow Motor Temp", "deg.C", xbot_msgs::SensorInfo::TYPE_DOUBLE, xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE, "mower_xesc"}},
    {"om_mow_motor_current", {"Mow Motor Current", "A", xbot_msgs::SensorInfo::TYPE_DOUBLE, xbot_msgs::SensorInfo::VALUE_DESCRIPTION_CURRENT, "mower_xesc"}},
    {"om_mow_motor_rpm", {"Mow Motor Revolutions", "rpm", xbot_msgs::SensorInfo::TYPE_DOUBLE, xbot_msgs::SensorInfo::VALUE_DESCRIPTION_RPM, "mower_xesc"}},
    {"om_gps_accuracy", {"GPS Accuracy", "m", xbot_msgs::SensorInfo::TYPE_DOUBLE, xbot_msgs::SensorInfo::VALUE_DESCRIPTION_DISTANCE}},
};

ros::NodeHandle *n;

ros::Time last_status_update(0);
ros::Time last_pose_update(0);

ros::NodeHandle *paramNh;

void status(const mower_msgs::Status::ConstPtr &msg) {
  // Rate limit to 2Hz
  if ((msg->stamp - last_status_update).toSec() < 0.5) return;
  last_status_update = msg->stamp;

  xbot_msgs::SensorDataDouble sensor_data;
  sensor_data.stamp = msg->stamp;

  const std::map<std::string, float> sensor_values{
      {"om_v_charge", msg->v_charge},
      {"om_v_battery", msg->v_battery},
      {"om_charge_current", msg->charge_current},
      {"om_left_esc_temp", msg->left_esc_status.temperature_pcb},
      {"om_right_esc_temp", msg->right_esc_status.temperature_pcb},
      {"om_mow_esc_temp", msg->mow_esc_status.temperature_pcb},
      {"om_mow_motor_temp", msg->mow_esc_status.temperature_motor},
      {"om_mow_motor_current", msg->mow_esc_status.current},
      {"om_mow_motor_rpm", msg->mow_esc_status.rpm},
  };
  for (auto &sensor_value : sensor_values) {
    Sensor sensor = sensors.at(sensor_value.first);
    sensor_data.data = sensor_value.second;
    sensor.data_pub.publish(sensor_data);
  }
}

void high_level_status(const mower_msgs::HighLevelStatus::ConstPtr &msg) {
  state.gps_percentage = msg->gps_quality_percent;
  state.current_state = msg->state_name;
  state.current_sub_state = msg->sub_state_name;
  state.current_area = msg->current_area;
  state.current_path = msg->current_path;
  state.current_path_index = msg->current_path_index;
  state.battery_percentage = msg->battery_percent;
  state.emergency = msg->emergency;
  state.is_charging = msg->is_charging;

  state_pub.publish(state);
}

void pose_received(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
  state.robot_pose = *msg;

  // Rate limit to 2Hz
  if ((msg->header.stamp - last_pose_update).toSec() < 0.5) return;
  last_pose_update = msg->header.stamp;

  xbot_msgs::SensorDataDouble sensor_data;
  sensor_data.stamp = msg->header.stamp;
  sensor_data.data = msg->position_accuracy;
  auto sensor = sensors.at("om_gps_accuracy");
  sensor.data_pub.publish(sensor_data);
}

void set_sensor_limits(Sensor &sensor) {
    // At the moment we process only those with a params parameter path
    if(!strlen(sensor.param_path.c_str())) {
        return;
    }
    switch (sensor.value_desc) {
      case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_CURRENT:
        sensor.si.max_value = paramNh->param(sensor.param_path + "/motor_current_limit", 0.0f);
        break;
      case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE:
        if (sensor.si.sensor_id.find("_motor_") != std::string::npos)
          sensor.si.max_value = paramNh->param(sensor.param_path + "/max_motor_temp", 0);
        else // i.e. _esc_
          sensor.si.max_value = paramNh->param(sensor.param_path + "/max_pcb_temp", 0);
        break;
      case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_RPM:
        sensor.si.min_value = paramNh->param(sensor.param_path + "/min_motor_rpm", 0);
        sensor.si.max_value = paramNh->param(sensor.param_path + "/max_motor_rpm", 0);
        sensor.si.lower_critical_value = paramNh->param(sensor.param_path + "/min_motor_rpm_critical", 0);
        break;
    }
    // Set has* sensor infos
    if (sensor.si.min_value || sensor.si.max_value)
        sensor.si.has_min_max = true;
    if (sensor.si.lower_critical_value)
        sensor.si.has_critical_low = true;
    if (sensor.si.upper_critical_value)
        sensor.si.has_critical_high = true;
}

void registerSensors() {
  for (auto &sensor : sensors) {
    sensor.second.si.sensor_id = sensor.first;
    sensor.second.si.sensor_name = sensor.second.name;

    sensor.second.si.unit = sensor.second.unit;
    sensor.second.si.value_type = sensor.second.value_type;
    sensor.second.si.value_description = sensor.second.value_desc;

    // Set sensor threshold values
    set_sensor_limits(sensor.second);

    sensor.second.si_pub =
        n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + sensor.first + "/info", 1, true);
    sensor.second.data_pub =
        n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + sensor.first + "/data", 10);
    sensor.second.si_pub.publish(sensor.second.si);
          }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "monitoring");

  n = new ros::NodeHandle();
  paramNh = new ros::NodeHandle("~");

  registerSensors();

  ros::Subscriber pose_sub = n->subscribe("xbot_positioning/xb_pose", 10, pose_received);
  ros::Subscriber state_sub = n->subscribe("mower_logic/current_state", 10, high_level_status);
  ros::Subscriber status_sub = n->subscribe("mower/status", 10, status);

  state_pub = n->advertise<xbot_msgs::RobotState>("xbot_monitoring/robot_state", 10);

  ros::spin();

  return 0;
}
