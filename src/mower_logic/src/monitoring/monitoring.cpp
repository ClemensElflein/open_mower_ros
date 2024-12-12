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

#include <dynamic_reconfigure/client.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/Power.h>
#include <xbot_msgs/SensorDataString.h>

#include "mower_logic/MowerLogicConfig.h"
#include "mower_msgs/HighLevelStatus.h"
#include "mower_msgs/Status.h"
#include "ros/ros.h"
#include "xbot_msgs/AbsolutePose.h"
#include "xbot_msgs/RobotState.h"
#include "xbot_msgs/SensorDataDouble.h"
#include "xbot_msgs/SensorInfo.h"

ros::Publisher state_pub;
xbot_msgs::RobotState state;

ros::NodeHandle *n;

ros::NodeHandle *paramNh;

dynamic_reconfigure::Client<mower_logic::MowerLogicConfig> *reconfigClient;
mower_logic::MowerLogicConfig mower_logic_config;
bool mower_logic_config_valid = false;

typedef const mower_msgs::Status::ConstPtr StatusPtr;

// Sensor configuration
struct SensorConfig {
  std::string name;     // Speaking name, used in sensor widget
  std::string unit;     // Unit like A, V, ...
  uint8_t value_desc;   // Voltage, Current, RPM, ...
  uint8_t sensor_type;  // Double, String, ...
  std::function<double(StatusPtr)> getStatusSensorValueCB = nullptr;
  std::function<void(SensorConfig &sensor_config)> setSensorLimitsCB = nullptr;
  std::string param_path = "";              // Path to parameters
  std::function<bool()> existCB = nullptr;  // nullptr = no callback for exist check = enabled
  xbot_msgs::SensorInfo si;                 // SensorInfo Msg
  ros::Publisher si_pub;                    // SensorInfo publisher
  ros::Publisher data_pub;                  // Sensor-data publisher
};

// Forward declare set_limits_* callback functions
void set_limits_battery_v(SensorConfig &sensor_config);
void set_limits_charge_current(SensorConfig &sensor_config);
void set_limits_charge_v(SensorConfig &sensor_config);
void set_limits_esc_temp(SensorConfig &sensor_config);
void set_limits_mow_motor_current(SensorConfig &sensor_config);
void set_limits_mow_motor_rpm(SensorConfig &sensor_config);
void set_limits_mow_motor_temp(SensorConfig &sensor_config);

// Place all sensors in a key=sensor.id -> SensorConfig map
// clang-format off
std::map<std::string, SensorConfig> sensor_configs{
  {"om_v_charge", {"V Charge", "V", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VOLTAGE, xbot_msgs::SensorInfo::TYPE_DOUBLE, nullptr, &set_limits_charge_v}},
  {"om_v_battery", {"V Battery", "V", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VOLTAGE, xbot_msgs::SensorInfo::TYPE_DOUBLE, nullptr, &set_limits_battery_v}},
  {"om_charge_current", {"Charge Current", "A", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_CURRENT, xbot_msgs::SensorInfo::TYPE_DOUBLE, nullptr, &set_limits_charge_current, "", [](){ return !paramNh->param("/mower_logic/ignore_charging_current", false); }}},
  {"om_charge_state", {"Charge State", "", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, xbot_msgs::SensorInfo::TYPE_STRING, nullptr}},
  {"om_left_esc_temp", {"Left ESC Temp", "deg.C", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE, xbot_msgs::SensorInfo::TYPE_DOUBLE, nullptr, &set_limits_esc_temp, "left_xesc"}},
  {"om_right_esc_temp", {"Right ESC Temp", "deg.C", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE, xbot_msgs::SensorInfo::TYPE_DOUBLE, nullptr, &set_limits_esc_temp, "right_xesc"}},
  {"om_mow_esc_temp", {"Mow ESC Temp", "deg.C", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE, xbot_msgs::SensorInfo::TYPE_DOUBLE, [](StatusPtr msg) { return msg->mower_esc_temperature; }, &set_limits_esc_temp, "mower_xesc"}},
  {"om_mow_motor_temp", {"Mow Motor Temp", "deg.C", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE, xbot_msgs::SensorInfo::TYPE_DOUBLE, [](StatusPtr msg) { return msg->mower_motor_temperature; }, &set_limits_mow_motor_temp, "mower_xesc", [](){ return paramNh->param("mower_xesc/has_motor_temp", true); }}},
  {"om_mow_motor_current", {"Mow Motor Current", "A", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_CURRENT, xbot_msgs::SensorInfo::TYPE_DOUBLE, [](StatusPtr msg) { return msg->mower_esc_current; }, &set_limits_mow_motor_current, "mower_xesc"}},
  {"om_mow_motor_rpm", {"Mow Motor Revolutions", "rpm", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_RPM, xbot_msgs::SensorInfo::TYPE_DOUBLE, [](StatusPtr msg) { return msg->mower_motor_rpm; }, &set_limits_mow_motor_rpm, "mower_xesc"}},
  {"om_gps_accuracy", {"GPS Accuracy", "m", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_DISTANCE, xbot_msgs::SensorInfo::TYPE_DOUBLE}},
};
// clang-format on

void status(StatusPtr &msg) {
  // Rate limit to 2Hz
  static ros::Time last_update{0};
  if ((msg->stamp - last_update).toSec() < 0.5) return;
  last_update = msg->stamp;

  xbot_msgs::SensorDataDouble sensor_data;
  sensor_data.stamp = msg->stamp;

  for (auto &sc_pair : sensor_configs) {
    // Skip if sensor doesn't exists or is disabled
    if (sc_pair.second.existCB && !sc_pair.second.existCB()) continue;

    if (sc_pair.second.getStatusSensorValueCB) {
      sensor_data.data = sc_pair.second.getStatusSensorValueCB(msg);
      sc_pair.second.data_pub.publish(sensor_data);
    }
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
  static ros::Time last_update{0};
  if ((msg->header.stamp - last_update).toSec() < 0.5) return;
  last_update = msg->header.stamp;

  xbot_msgs::SensorDataDouble sensor_data;
  sensor_data.stamp = msg->header.stamp;
  sensor_data.data = msg->position_accuracy;

  auto sc_it = sensor_configs.find("om_gps_accuracy");
  if (sc_it != std::end(sensor_configs)) {
    sc_it->second.data_pub.publish(sensor_data);
  }
}
void power_received(const mower_msgs::Power::ConstPtr &msg) {
  // Rate limit to 2Hz
  static ros::Time last_update{0};
  if ((msg->stamp - last_update).toSec() < 0.5) return;
  last_update = msg->stamp;

  {
    xbot_msgs::SensorDataDouble sensor_data;
    sensor_data.stamp = msg->stamp;
    sensor_data.data = msg->v_charge;

    auto sc_it = sensor_configs.find("om_v_charge");
    if (sc_it != std::end(sensor_configs)) {
      sc_it->second.data_pub.publish(sensor_data);
    }
  }
  {
    xbot_msgs::SensorDataDouble sensor_data;
    sensor_data.stamp = msg->stamp;
    sensor_data.data = msg->v_battery;

    auto sc_it = sensor_configs.find("om_v_battery");
    if (sc_it != std::end(sensor_configs)) {
      sc_it->second.data_pub.publish(sensor_data);
    }
  }
  {
    xbot_msgs::SensorDataDouble sensor_data;
    sensor_data.stamp = msg->stamp;
    sensor_data.data = msg->charge_current;

    auto sc_it = sensor_configs.find("om_charge_current");
    if (sc_it != std::end(sensor_configs)) {
      sc_it->second.data_pub.publish(sensor_data);
    }
  }
  {
    xbot_msgs::SensorDataString sensor_data;
    sensor_data.stamp = msg->stamp;
    sensor_data.data = msg->charger_status;

    auto sc_it = sensor_configs.find("om_charge_state");
    if (sc_it != std::end(sensor_configs)) {
      sc_it->second.data_pub.publish(sensor_data);
    }
  }
}

void set_limits_battery_v(SensorConfig &sensor_config) {
  sensor_config.si.lower_critical_value = mower_logic_config.battery_critical_voltage;
  sensor_config.si.min_value = mower_logic_config.battery_empty_voltage;
  sensor_config.si.max_value = mower_logic_config.battery_full_voltage;
  sensor_config.si.upper_critical_value = mower_logic_config.battery_critical_high_voltage;
}

void set_limits_charge_v(SensorConfig &sensor_config) {
  sensor_config.si.upper_critical_value = mower_logic_config.charge_critical_high_voltage;
}

void set_limits_charge_current(SensorConfig &sensor_config) {
  sensor_config.si.upper_critical_value = mower_logic_config.charge_critical_high_current;
}

void set_limits_esc_temp(SensorConfig &sensor_config) {
  sensor_config.si.max_value = paramNh->param(sensor_config.param_path + "/max_pcb_temp", 0);
}

void set_limits_mow_motor_current(SensorConfig &sensor_config) {
  sensor_config.si.upper_critical_value = paramNh->param(sensor_config.param_path + "/motor_current_limit", 0.0f);
}

void set_limits_mow_motor_rpm(SensorConfig &sensor_config) {
  // Use the labeled YF-C500 mow motor value (3800 rpm) for default threshold estimations...
  sensor_config.si.lower_critical_value = paramNh->param(sensor_config.param_path + "/min_motor_rpm_critical", 2300);
  sensor_config.si.min_value = paramNh->param(sensor_config.param_path + "/min_motor_rpm", 2800);
  sensor_config.si.max_value = paramNh->param(sensor_config.param_path + "/max_motor_rpm", 3800);
}

void set_limits_mow_motor_temp(SensorConfig &sensor_config) {
  // mower_config settings have precedence before xesc param file because user editable
  sensor_config.si.max_value =
      (mower_logic_config_valid ? mower_logic_config.motor_hot_temperature
                                : paramNh->param(sensor_config.param_path + "/max_motor_temp", 0.0f));
  sensor_config.si.min_value =
      (mower_logic_config_valid ? mower_logic_config.motor_cold_temperature
                                : paramNh->param(sensor_config.param_path + "/min_motor_temp", 0.0f));
}

void registerSensors() {
  for (auto &sc_pair : sensor_configs) {
    if (sc_pair.second.existCB && !sc_pair.second.existCB()) {
      ROS_INFO_STREAM("Skipped monitoring of sensor " << sc_pair.first);
      continue;
    }

    sc_pair.second.si.sensor_id = sc_pair.first;
    sc_pair.second.si.sensor_name = sc_pair.second.name;

    sc_pair.second.si.unit = sc_pair.second.unit;
    sc_pair.second.si.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    sc_pair.second.si.value_description = sc_pair.second.value_desc;

    // Set sensor threshold values
    if (sc_pair.second.setSensorLimitsCB) sc_pair.second.setSensorLimitsCB(sc_pair.second);

    // Set has* sensor infos
    // FIXME: "has_min_max" is somehow confusing/misstakeable.
    // From the logic point of view, has_min_max should only be set if it has min as well as max limits,
    // but then it's somehow useless because i.e. for temperatures, we don't have a reasonable min value.
    // At least it doesn't make much sense to show i.e. a gauge from -15 to 80°C.
    // If has_min_max get set if min OR max is set, then it's useless again, because then we need to check
    // min as well as max for a limit value.
    // In my opinion, we can drop all has_* settings (except has_motor_temp) and let decide the view logic how to handle
    // the limits
    if (sc_pair.second.si.min_value && sc_pair.second.si.max_value) sc_pair.second.si.has_min_max = true;
    if (sc_pair.second.si.lower_critical_value) sc_pair.second.si.has_critical_low = true;
    if (sc_pair.second.si.upper_critical_value) sc_pair.second.si.has_critical_high = true;

    sc_pair.second.si_pub =
        n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + sc_pair.first + "/info", 1, true);
    sc_pair.second.data_pub =
        n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + sc_pair.first + "/data", 10);
    sc_pair.second.si_pub.publish(sc_pair.second.si);
  }
}

void reconfigCB(const mower_logic::MowerLogicConfig &config) {
  ROS_INFO_STREAM("Monitoring received new mower_logic config");
  mower_logic_config = config;
  mower_logic_config_valid = true;

  registerSensors();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "monitoring");

  n = new ros::NodeHandle();
  paramNh = new ros::NodeHandle("/mower_comms");

  mower_logic_config = mower_logic::MowerLogicConfig::__getDefault__();
  reconfigClient = new dynamic_reconfigure::Client<mower_logic::MowerLogicConfig>("/mower_logic", reconfigCB);

  registerSensors();

  ros::Subscriber pose_sub = n->subscribe("xbot_positioning/xb_pose", 10, pose_received);
  ros::Subscriber state_sub = n->subscribe("mower_logic/current_state", 10, high_level_status);
  ros::Subscriber status_sub = n->subscribe("mower/status", 10, status);

  state_pub = n->advertise<xbot_msgs::RobotState>("xbot_monitoring/robot_state", 10);

  ros::spin();

  return 0;
}
