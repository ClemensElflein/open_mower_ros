# VESC Driving Library
`vesc_driver` is a basic library to drive motors with a VESC.

## Features
- communicates with a VESC.
- Position (PID), velocity, current, and duty-cycle control are supported.
  - Calibration and joint limitation are implemented for position control.
  - You can give torque constant and gear ratio of your geared motor

## Package Structures
#### Dependency Diagram
```
vesc_driver
 -> vesc_interface
    |-> vesc_packet_factory
    |-> vesc_packet
```

#### Description
- `vesc_packet` defines communication packets and the lowest interfaces via USB serial. 
- `vesc_packet_factory` creates packets defines in `vesc_packet`.
- `vesc_interface` provides us with basic APIs to drive VESC moto drivers: API functions send commands including duty, reference current, brake, and reference velocity.
- `vesc_driver` publishes all states using `vesc_msgs` and sends commands with callback functions.


## Installation
This package depends on the following ROS packages.

- nodelet
- pluginlib
- roscpp
- std_msgs
- serial

Before building this, you have to install them (e.g. with `apt` command).

## Usage
will be prepared ...

## License
`vesc_driver` is licensed under the [Apache 2.0 license](https://www.apache.org/licenses/LICENSE-2.0.html).
