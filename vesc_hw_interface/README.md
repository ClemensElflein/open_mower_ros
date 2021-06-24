# Hardware Interface for VESC
This package provides us with hardware interfaces of VESC open source motor drivers running on ROS (Robot Operating System).
`vesc_hw_interface` libraries enable your robot to drive motors with VESC with [ros_control](http://wiki.ros.org/ros_control) framework.

## Features
- A simple controller drives a BLDC motor with VESC.
- Position (PID), velocity, current, and duty-cycle control are supported.
  - Calibration and joint limitation are implemented for position control.
  - You can give torque constant and gear ratio of your geared motor

### Limitations
* To prevent motors from breaking machines, the joint limitation should be implemented.
* Safety functions of this packages are NOT PERFECT. Please notice us if you find some mistakes in the implementation.

## Installation
Before build this package, you should install following packages additionally:
```bash
ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-hardware-interface ros-$ROS_DISTRO-controller-manager
```

Then, clone this repository in `src` directory of your catkin workspace, and build with `catkin build` command. This package will be built with `vesc_driver` package, placed in the same repository.

## Usage
### Parameters
All of following parameters are in `${VESC_HW_INTERFACE_NODE_NAME}/` namespace.

#### Common
- `port` (string, **required**): port name connecting to your VESC, *e.g.* `/dev/ttyUSB0`.
- `command_mode` (string, **required**): control mode you want to use. Enter one of following parameters: `position`, `velocity`, `effort` and `effort_duty`.
- `joint_name` (string, *default*: `joint_vesc`): corresponding joint name in your robot URDF.
- `num_motor_pole_pairs` (double, *default*: 1.0): the number of motor pole pairs.
- `gear_ratio` (double, *default*: 1.0): ratio of reduction calculated by `joint velocity/motor velocity`.
- `torque_const` (double, *default*: 1.0): motor torque constant (unit: Nm/A).
- `robot_description_name` (string, *default*: /robot_description): name of the robot description parameters for loading joint limits

**NOTE**: `gear_ratio` and `torque_const` are used to calculate joint states because VESC generally senses just motor position and motor current, neither joint position nor motor torque.
If your motor unit has other structures, you should implement your own controller.

#### For PID Position Control
- `servo/Kp` (double, *default*: 50.0): proportional gain of the controller.
- `servo/Ki` (double, *default*: 0.0): integral gain of the controller.
- `servo/Kd` (double, *default*: 1.0): derivative gain of the controller.
- `servo/calibration_current` (double, *default*: 6.0): maximum current used in origin calibration.
- `servo/calibration_position` (double, *default*: 0.0): the position on which the robot calibrates.

### Public Functions
- `void read()` sends request to get current states, but DOES NOT update immediately. After a return packet comes, the callback function will update private variables.
- `void write()` sends a command with specified mode.

### Samples

**position control**

```bash
roslaunch vesc_hw_interface position_control_sample.launch
```

**velocity control**

```bash
roslaunch vesc_hw_interface velocity_control_sample.launch
```

**effort control**

```bash
roslaunch vesc_hw_interface effort_control_sample.launch
```

## License
`vesc_hw_interface` is licensed under the [Apache 2.0 license](https://www.apache.org/licenses/LICENSE-2.0.html).
