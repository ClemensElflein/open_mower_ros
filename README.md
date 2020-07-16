# VESC
## ROS Interfaces for VESC Motor Drivers

<!-- TODO: CI setting -->

## Composition
This repository is composed as follows:

- `vesc` is a meta package to manage the other packages.
- `vesc_driver` is the main library to drive VESCs.
- `vesc_hw_interface` wraps `vesc_driver` as a hardware interface to use `ros_control` systems.
- `vesc_msgs` defines messages to publish VESC status.

## License
This repositry is licensed under the [Apache 2.0 License](https://www.apache.org/licenses/LICENSE-2.0.html).

## Note
A part of these packages had been developed by Michael T. Boulet at MIT under the BSD 3-clause License until Dec. 2016 in the repository named [mit-racecar/vesc](https://github.com/mit-racecar/vesc). Since Nov. 2019, Softbank Corp. takes over development as new packages.
