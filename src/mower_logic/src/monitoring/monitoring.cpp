// Created by Clemens Elflein on 3/28/22.
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
#include "xbot_msgs/RobotState.h"
#include "xbot_msgs/AbsolutePose.h"
#include "xbot_msgs/SensorInfo.h"
#include "xbot_msgs/SensorDataDouble.h"
#include "mower_msgs/HighLevelStatus.h"
#include "mower_msgs/Status.h"

ros::Publisher state_pub;
xbot_msgs::RobotState state;

xbot_msgs::SensorInfo si_v_charge;
ros::Publisher si_v_charge_pub;
ros::Publisher v_charge_data_pub;

xbot_msgs::SensorInfo si_v_battery;
ros::Publisher si_v_battery_pub;
ros::Publisher v_battery_data_pub;

xbot_msgs::SensorInfo si_charge_current;
ros::Publisher si_charge_current_pub;
ros::Publisher charge_current_data_pub;

xbot_msgs::SensorInfo si_left_esc_temp;
ros::Publisher si_left_esc_temp_pub;
ros::Publisher left_esc_temp_data_pub;

xbot_msgs::SensorInfo si_right_esc_temp;
ros::Publisher si_right_esc_temp_pub;
ros::Publisher right_esc_temp_data_pub;

xbot_msgs::SensorInfo si_mow_esc_temp;
ros::Publisher si_mow_esc_temp_pub;
ros::Publisher mow_esc_temp_data_pub;

xbot_msgs::SensorInfo si_mow_motor_temp;
ros::Publisher si_mow_motor_temp_pub;
ros::Publisher mow_motor_temp_data_pub;

ros::NodeHandle *n;

ros::Time last_status_update(0);

void status(const mower_msgs::Status::ConstPtr &msg) {
    // Rate limit to 1Hz
    if((msg->stamp - last_status_update).toSec() < 1)
        return;
    last_status_update = msg->stamp;

    xbot_msgs::SensorDataDouble sensor_data;
    sensor_data.stamp = msg->stamp;

    sensor_data.data = msg->v_charge;
    v_charge_data_pub.publish(sensor_data);

    sensor_data.data = msg->v_battery;
    v_battery_data_pub.publish(sensor_data);

    sensor_data.data = msg->charge_current;
    charge_current_data_pub.publish(sensor_data);

    sensor_data.data = msg->left_esc_status.temperature_pcb;
    left_esc_temp_data_pub.publish(sensor_data);

    sensor_data.data = msg->right_esc_status.temperature_pcb;
    right_esc_temp_data_pub.publish(sensor_data);

    sensor_data.data = msg->mow_esc_status.temperature_pcb;
    mow_esc_temp_data_pub.publish(sensor_data);

    sensor_data.data = msg->mow_esc_status.temperature_motor;
    mow_motor_temp_data_pub.publish(sensor_data);
}

void high_level_status(const mower_msgs::HighLevelStatus::ConstPtr &msg) {
    state.gps_percentage = msg->gps_quality_percent;
    state.current_state = msg->state_name;
    state.current_sub_state = msg->sub_state_name;
    state.battery_percentage = msg->battery_percent;
    state.emergency = msg->emergency;
    state.is_charging = msg->is_charging;

    state_pub.publish(state);
}

void pose_received(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
    state.robot_pose = *msg;
}

void registerSensors() {
    si_v_charge.sensor_id = "om_v_charge";
    si_v_charge.sensor_name = "V Charge";
    si_v_charge.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_v_charge.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VOLTAGE;
    si_v_charge.unit = "V";
    si_v_charge_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_v_charge.sensor_id + "/info", 1, true);
    v_charge_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_v_charge.sensor_id + "/data",10);
    si_v_charge_pub.publish(si_v_charge);

    si_v_battery.sensor_id = "om_v_battery";
    si_v_battery.sensor_name = "V Battery";
    si_v_battery.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_v_battery.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VOLTAGE;
    si_v_battery.unit = "V";
    si_v_battery_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_v_battery.sensor_id + "/info", 1, true);
    v_battery_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_v_battery.sensor_id + "/data",10);
    si_v_battery_pub.publish(si_v_battery);

    si_charge_current.sensor_id = "om_charge_current";
    si_charge_current.sensor_name = "Charge Current";
    si_charge_current.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_charge_current.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_CURRENT;
    si_charge_current.unit = "A";
    si_charge_current_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_charge_current.sensor_id + "/info", 1, true);
    charge_current_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_charge_current.sensor_id + "/data",10);
    si_charge_current_pub.publish(si_charge_current);

    si_left_esc_temp.sensor_id = "om_left_esc_temp";
    si_left_esc_temp.sensor_name = "Left ESC Temp";
    si_left_esc_temp.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_left_esc_temp.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE;
    si_left_esc_temp.unit = "deg.C";
    si_left_esc_temp_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_left_esc_temp.sensor_id + "/info", 1, true);
    left_esc_temp_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_left_esc_temp.sensor_id + "/data",10);
    si_left_esc_temp_pub.publish(si_left_esc_temp);

    si_right_esc_temp.sensor_id = "om_right_esc_temp";
    si_right_esc_temp.sensor_name = "Right ESC Temp";
    si_right_esc_temp.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_right_esc_temp.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE;
    si_right_esc_temp.unit = "deg.C";
    si_right_esc_temp_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_right_esc_temp.sensor_id + "/info", 1, true);
    right_esc_temp_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_right_esc_temp.sensor_id + "/data",10);
    si_right_esc_temp_pub.publish(si_right_esc_temp);

    si_mow_esc_temp.sensor_id = "om_mow_esc_temp";
    si_mow_esc_temp.sensor_name = "Mow ESC Temp";
    si_mow_esc_temp.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_mow_esc_temp.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE;
    si_mow_esc_temp.unit = "deg.C";
    si_mow_esc_temp_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_mow_esc_temp.sensor_id + "/info", 1, true);
    mow_esc_temp_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_mow_esc_temp.sensor_id + "/data",10);
    si_mow_esc_temp_pub.publish(si_mow_esc_temp);

    si_mow_motor_temp.sensor_id = "om_mow_motor_temp";
    si_mow_motor_temp.sensor_name = "Mow Motor Temp";
    si_mow_motor_temp.value_type = xbot_msgs::SensorInfo::TYPE_DOUBLE;
    si_mow_motor_temp.value_description = xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE;
    si_mow_motor_temp.unit = "deg.C";
    si_mow_motor_temp_pub = n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + si_mow_motor_temp.sensor_id + "/info", 1, true);
    mow_motor_temp_data_pub = n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + si_mow_motor_temp.sensor_id + "/data",10);
    si_mow_motor_temp_pub.publish(si_mow_motor_temp);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "monitoring");

    n = new ros::NodeHandle();

    registerSensors();

    ros::Subscriber pose_sub = n->subscribe("xbot_positioning/xb_pose", 10, pose_received);
    ros::Subscriber state_sub = n->subscribe("mower_logic/current_state", 10, high_level_status);
    ros::Subscriber status_sub = n->subscribe("mower/status", 10, status);


    state_pub = n->advertise<xbot_msgs::RobotState>("xbot_monitoring/robot_state", 10);


    ros::spin();

    return 0;
}
