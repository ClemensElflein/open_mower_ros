// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_driver.h"

#include <cassert>
#include <cmath>

#include <boost/bind.hpp>
#include <vesc_msgs/VescState.h>

namespace vesc_driver
{

VescDriver::VescDriver(ros::NodeHandle nh,
                       ros::NodeHandle private_nh) :
  vesc_(std::string(),
        boost::bind(&VescDriver::vescPacketCallback, this, _1),
        boost::bind(&VescDriver::vescErrorCallback, this, _1)),
  driver_mode_(MODE_INITIALIZING), fw_version_major_(-1), fw_version_minor_(-1)
{
  // get vesc serial port address
  std::string port;
  if (!private_nh.getParam("port", port)) {
    ROS_FATAL("VESC communication port parameter required.");
    ros::shutdown();
    return;
  }

  // attempt to connect to the serial port
  try {
    vesc_.connect(port);
  }
  catch (SerialException e) {
    ROS_FATAL("Failed to connect to the VESC, %s.", e.what());
    ros::shutdown();
    return;
  }

  // create vesc state (telemetry) publisher
  state_pub_ = private_nh.advertise<vesc_msgs::VescState>("sensors/core", 10);

  // subscribe to motor and servo command topics
  duty_cycle_sub_ = private_nh.subscribe("commands/motor/duty_cycle",
                                         10, &VescDriver::dutyCycleCallback, this);
  current_sub_ = private_nh.subscribe("commands/motor/current", 10,
                                      &VescDriver::currentCallback, this);
  brake_sub_ = private_nh.subscribe("commands/motor/brake", 10, &VescDriver::brakeCallback, this);
  speed_sub_ = private_nh.subscribe("commands/motor/speed", 10, &VescDriver::speedCallback, this);
  position_sub_ = private_nh.subscribe("commands/motor/position", 10,
                                       &VescDriver::positionCallback, this);
  servo_sub_ = private_nh.subscribe("commands/servo/position", 10,
                                    &VescDriver::servoCallback, this);

  // create a 50Hz timer, used for state machine & polling VESC telemetry
  timer_ = nh.createTimer(ros::Duration(1.0/50.0), &VescDriver::timerCallback, this);
}

  /* TODO or TO-THINKABOUT LIST
    - what should we do on startup? send brake or zero command?
    - what to do if the vesc interface gives an error?
    - check version number against know compatable?
    - should we wait until we receive telemetry before sending commands?
    - should we track the last motor command
    - what to do if no motor command received recently?
    - what to do if no servo command received recently?
    - what is the motor safe off state (0 current?)
    - what to do if a command parameter is out of range, ignore?
    - try to predict vesc bounds (from vesc config) and command detect bounds errors
  */

void VescDriver::timerCallback(const ros::TimerEvent& event)
{
  // VESC interface should not unexpectedly disconnect, but test for it anyway
  if (!vesc_.isConnected()) {
    ROS_FATAL("Unexpectedly disconnected from serial port.");
    timer_.stop();
    ros::shutdown();
    return;
  }

  /*
   * Driver state machine, modes:
   *  INITIALIZING - request and wait for vesc version
   *  OPERATING - receiving commands from subscriber topics
   */
  if (driver_mode_ == MODE_INITIALIZING) {
    // request version number, return packet will update the internal version numbers
    vesc_.requestFWVersion();
    if (fw_version_major_ >= 0 && fw_version_minor_ >= 0) {
      ROS_INFO("Connected to VESC with firmware version %d.%d",
               fw_version_major_, fw_version_minor_);
      driver_mode_ = MODE_OPERATING;
    }
  }
  else if (driver_mode_ == MODE_OPERATING) {
    // poll for vesc state (telemetry)
    vesc_.requestState();
  }
  else {
    // unknown mode, how did that happen?
    assert(false && "unknown driver mode");
  }
}

void VescDriver::vescPacketCallback(const boost::shared_ptr<VescPacket const>& packet)
{
  if (packet->name() == "Values") {
    boost::shared_ptr<VescPacketValues const> values =
      boost::dynamic_pointer_cast<VescPacketValues const>(packet);

    vesc_msgs::VescState state_msg;
    state_msg.header.stamp = ros::Time::now();
    state_msg.voltage_input = values->v_in();
    state_msg.temperature_pcb = values->temp_pcb();
    state_msg.current_motor = values->current_motor();
    state_msg.current_input = values->current_in();
    state_msg.speed = values->rpm();
    state_msg.duty_cycle = values->duty_now();
    state_msg.charge_drawn = values->amp_hours();
    state_msg.charge_regen = values->amp_hours_charged();
    state_msg.energy_drawn = values->watt_hours();
    state_msg.energy_regen = values->watt_hours_charged();
    state_msg.displacement = values->tachometer();
    state_msg.distance_traveled = values->tachometer_abs();
    state_msg.fault_code = values->fault_code();

    state_pub_.publish(state_msg);
  }
  else if (packet->name() == "FWVersion") {
    boost::shared_ptr<VescPacketFWVersion const> fw_version =
      boost::dynamic_pointer_cast<VescPacketFWVersion const>(packet);
    // todo: might need lock here
    fw_version_major_ = fw_version->fwMajor();
    fw_version_minor_ = fw_version->fwMinor();
  }
}

void VescDriver::vescErrorCallback(const std::string& error)
{
  ROS_ERROR("%s", error.c_str());
}

/**
 * @param duty_cycle Commanded VESC duty cycle. Valid range for this driver is -1 to +1. However,
 *                   note that the VESC may impose a more restrictive bounds on the range depending
 *                   on its configuration, e.g. absolute value is between 0.05 and 0.95.
 */
void VescDriver::dutyCycleCallback(const std_msgs::Float64::ConstPtr& duty_cycle)
{
  if (duty_cycle->data < -1.0 || duty_cycle->data > 1.0) {
    ROS_ERROR("VESC duty cycle should be between -1 and 1, ignoring.");
    return;
  }
  if (driver_mode_ = MODE_OPERATING) {
    vesc_.setDutyCycle(duty_cycle->data);
  }
}

/**
 * @param current Commanded VESC current in Amps. Any value is accepted by this driver. However,
 *                note that the VESC may impose a more restrictive bounds on the range depending on
 *                its configuration.
 */
void VescDriver::currentCallback(const std_msgs::Float64::ConstPtr& current)
{
  if (driver_mode_ = MODE_OPERATING) {
    vesc_.setCurrent(current->data);
  }
}

/**
 * @param brake Commanded VESC braking current in Amps. Any value is accepted by this driver.
 *              However, note that the VESC may impose a more restrictive bounds on the range
 *              depending on its configuration.
 */
void VescDriver::brakeCallback(const std_msgs::Float64::ConstPtr& brake)
{
  if (driver_mode_ = MODE_OPERATING) {
    vesc_.setBrake(brake->data);
  }
}

/**
 * @param speed Commanded VESC speed in electrical RPM. Electrical RPM is the mechanical RPM
 *              multiplied by the number of motor poles. Any value is accepted by this
 *              driver. However, note that the VESC may impose a more restrictive bounds on the
 *              range depending on its configuration.
 */
void VescDriver::speedCallback(const std_msgs::Float64::ConstPtr& speed)
{
  if (driver_mode_ = MODE_OPERATING) {
    vesc_.setSpeed(speed->data);
  }
}

/**
 * @param position Commanded VESC motor position in radians. Any value is accepted by this driver.
 *                 Note that the VESC must be in encoder mode for this command to have an effect.
 */
void VescDriver::positionCallback(const std_msgs::Float64::ConstPtr& position)
{
  if (driver_mode_ = MODE_OPERATING) {
    // ROS uses radians but VESC seems to use degrees. Convert to degrees.
    double position_deg = position->data * M_PI / 180.0;
    vesc_.setPosition(position_deg);
  }
}

/**
 * @param servo Commanded VESC servo output position. Valid range is 0 to 1.
 */
void VescDriver::servoCallback(const std_msgs::Float64::ConstPtr& servo)
{
  if (servo->data < 0 || servo->data > 1.0) {
    ROS_ERROR("VESC servo position should be between 0 and 1, ignoring.");
    return;
  }

  if (driver_mode_ = MODE_OPERATING) {
    vesc_.setServo(servo->data);
  }
}

} // namespace vesc_driver
